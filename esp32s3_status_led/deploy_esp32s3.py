#!/usr/bin/env python3
"""
Deploy & Flash Lonely Binary ESP32-S3 N16R8 Status LED Firmware + RPi5 Systemd Service.

Supports two explicit deployment modes:
  1. USB Bootstrap / Recovery (`--usb`, default):
     - Verifies USB VID:PID is 303a:1001 (Espressif USB JTAG/serial debug unit) on
       /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:71:6A:98-if00.
     - Flashes bootloader, default_16MB.csv partition table, boot_app0, and firmware.bin.
  2. Secure Wi-Fi OTA Update (`--ota`):
     - Works with USB completely unplugged.
     - Queries the ESP32-S3 over RPi5 GPIO UART0 (`/dev/ttyAMA0`) to verify target identity,
       current firmware version, active partition, and live LAN IPv4 address.
     - Uploads `firmware.bin` to RPi5 and executes authenticated ArduinoOTA push over LAN
       (reading the OTA password from a chmod-600 file on RPi5 so secrets never appear in
       process `argv` or console logs).

Safety Invariants:
  1. Refuses to touch /dev/rover-esp32 or /dev/ttyUSB0 (1a86:7523 Maker-ESP32-Pro).
  2. Never stops or restarts rover-server.service.
  3. Never prints Wi-Fi passwords or OTA secrets to stdout, stderr, or build logs.
"""

import argparse
import hashlib
import json
import os
import re
import secrets
import subprocess
import sys
import time
import paramiko

ESP32S3_PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:71:6A:98-if00"
EXPECTED_VID = "303a"
EXPECTED_PID = "1001"
EXPECTED_OTA_HOSTNAME = "rover1-esp32s3"
RPI_HOST = "10.0.0.246"
RPI_USER = "ron"
REMOTE_DIR = "/home/ron/esp32s3_status_led"


def find_esp32s3_port(client: paramiko.SSHClient) -> str:
    """Find active ESP32-S3 USB port by symlink or fallback to ttyACM0."""
    rc, out, _ = run_ssh(
        client,
        "ls -1 /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00 2>/dev/null",
        check=False,
    )
    if rc == 0 and out.strip():
        return out.splitlines()[0].strip()
    return "/dev/ttyACM0"


def run_ssh(client: paramiko.SSHClient, cmd: str, check: bool = True) -> tuple[int, str, str]:
    stdin, stdout, stderr = client.exec_command(cmd)
    rc = stdout.channel.recv_exit_status()
    out = stdout.read().decode("utf-8", errors="replace").strip()
    err = stderr.read().decode("utf-8", errors="replace").strip()
    if check and rc != 0:
        raise RuntimeError(f"Remote command failed (rc={rc}): {cmd}\nSTDOUT: {out}\nSTDERR: {err}")
    return rc, out, err


def ensure_local_ota_secrets(base_dir: str, client: paramiko.SSHClient) -> dict:
    """
    Ensure ignored local credentials exist in:
      - esp32s3_status_led/.ota_secrets.json
      - esp32s3_status_led/include/ota_secrets.h
    If missing, reads the active home Wi-Fi SSID/PSK directly from RPi5 NetworkManager over SSH
    (never printing secrets to stdout/logs) and generates a 256-bit random OTA password.
    """
    json_path = os.path.join(base_dir, ".ota_secrets.json")
    header_path = os.path.join(base_dir, "include", "ota_secrets.h")

    data = {}
    if os.path.exists(json_path):
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

    if not data.get("wifi_ssid") or not data.get("wifi_password"):
        print("  Provisioning Wi-Fi station credentials from RPi5 NetworkManager (redacted)...")
        py_extract = (
            "import configparser, glob\n"
            "for p in glob.glob('/etc/NetworkManager/system-connections/*.nmconnection'):\n"
            "    c = configparser.ConfigParser()\n"
            "    c.read(p)\n"
            "    if 'wifi' in c and 'wifi-security' in c:\n"
            "        print(c['wifi'].get('ssid',''))\n"
            "        print(c['wifi-security'].get('psk',''))\n"
            "        break\n"
        )
        _, raw_out, _ = run_ssh(client, f"sudo -n python3 -c \"{py_extract}\"")
        lines = [ln.strip() for ln in raw_out.splitlines() if ln.strip()]
        if len(lines) < 2:
            raise RuntimeError("Could not automatically read Wi-Fi SSID/PSK from RPi5 NetworkManager.")
        data["wifi_ssid"] = lines[0]
        data["wifi_password"] = lines[1]

    if not data.get("ota_password"):
        data["ota_password"] = secrets.token_hex(16)
    if not data.get("ota_hostname"):
        data["ota_hostname"] = EXPECTED_OTA_HOSTNAME

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    os.makedirs(os.path.dirname(header_path), exist_ok=True)

    def c_escape(val: str) -> str:
        return val.replace("\\", "\\\\").replace('"', '\\"')

    header_content = (
        "#pragma once\n"
        "// Auto-generated local git-ignored credentials. DO NOT COMMIT.\n"
        f"#define ESP32S3_WIFI_SSID      \"{c_escape(data['wifi_ssid'])}\"\n"
        f"#define ESP32S3_WIFI_PASSWORD  \"{c_escape(data['wifi_password'])}\"\n"
        f"#define ESP32S3_OTA_HOSTNAME   \"{c_escape(data['ota_hostname'])}\"\n"
        f"#define ESP32S3_OTA_PASSWORD   \"{c_escape(data['ota_password'])}\"\n"
    )
    # Only rewrite header if changed to avoid unnecessary rebuilds
    write_header = True
    if os.path.exists(header_path):
        with open(header_path, "r", encoding="utf-8") as f:
            if f.read() == header_content:
                write_header = False
    if write_header:
        with open(header_path, "w", encoding="utf-8") as f:
            f.write(header_content)

    pw_hash_prefix = hashlib.sha256(data["ota_password"].encode("utf-8")).hexdigest()[:8]
    print(
        f"  Verified local ignored OTA config: SSID='{data['wifi_ssid']}', "
        f"HOST='{data['ota_hostname']}', OTA_KEY_SHA256_PREFIX={pw_hash_prefix} (secrets redacted)"
    )
    return data


def compile_firmware(base_dir: str, build_dir: str, fw_version: str | None) -> str:
    print("=== 2. Compiling ESP32-S3 Firmware Locally ===")
    env = os.environ.copy()
    if fw_version:
        env["PLATFORMIO_BUILD_FLAGS"] = f'-DESP32S3_FW_VERSION=\\"{fw_version}\\"'
        print(f"  Target Firmware Version Override: {fw_version}")
    subprocess.run(["pio", "run", "-d", base_dir], env=env, check=True)
    fw_bin = os.path.join(build_dir, "firmware.bin")
    if not os.path.exists(fw_bin):
        raise FileNotFoundError(f"Missing compiled firmware binary: {fw_bin}")
    fw_sha = hashlib.sha256(open(fw_bin, "rb").read()).hexdigest()[:12]
    print(f"  Compiled firmware.bin ({os.path.getsize(fw_bin)} bytes, sha256={fw_sha})")
    return fw_bin


def upload_and_update_daemon(client: paramiko.SSHClient, base_dir: str) -> str:
    run_ssh(client, f"mkdir -p {REMOTE_DIR}")
    daemon_local = os.path.join(base_dir, "rpi5_status_led_daemon.py")
    daemon_remote = f"{REMOTE_DIR}/rpi5_status_led_daemon.py"
    sftp = client.open_sftp()
    sftp.put(daemon_local, daemon_remote)
    sftp.close()
    run_ssh(client, f"chmod +x {daemon_remote}")
    return daemon_remote


def install_and_start_systemd_service(client: paramiko.SSHClient, daemon_remote: str):
    service_unit = f"""[Unit]
Description=Rover 1 ESP32-S3 Status LED Daemon (Green=RPi5 On, Blue=Moving, Red=Off)
After=network.target rover-server.service

[Service]
Type=simple
User=ron
ExecStart=/usr/bin/python3 {daemon_remote}
Restart=always
RestartSec=1
KillSignal=SIGTERM
TimeoutStopSec=3

[Install]
WantedBy=multi-user.target
"""
    run_ssh(
        client,
        f"cat <<'EOF' | sudo -n tee /etc/systemd/system/esp32s3-status-led.service >/dev/null\n{service_unit}EOF",
    )
    run_ssh(client, "sudo -n systemctl daemon-reload")
    run_ssh(client, "sudo -n systemctl enable --now esp32s3-status-led.service")


def ensure_rpi5_gpio_uart0(client: paramiko.SSHClient):
    print("\n=== Enabling RPi5 GPIO UART0 (/dev/ttyAMA0 on GPIO14/15) & Disabling Serial Console ===")
    run_ssh(client, "sudo -n systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true", check=False)
    run_ssh(client, "sudo -n systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true", check=False)
    run_ssh(client, "sudo -n systemctl mask serial-getty@ttyAMA0.service 2>/dev/null || true", check=False)
    run_ssh(client, "test -e /dev/ttyAMA0 || sudo -n dtoverlay uart0-pi5", check=True)
    run_ssh(
        client,
        "grep -q '^dtparam=uart0=on' /boot/firmware/config.txt || echo 'dtparam=uart0=on' | sudo -n tee -a /boot/firmware/config.txt >/dev/null",
    )
    run_ssh(
        client,
        "grep -q '^dtoverlay=uart0-pi5' /boot/firmware/config.txt || echo 'dtoverlay=uart0-pi5' | sudo -n tee -a /boot/firmware/config.txt >/dev/null",
    )
    run_ssh(
        client,
        "sudo -n sed -i 's/console=serial0,[0-9]* //g; s/console=ttyAMA0,[0-9]* //g' /boot/firmware/cmdline.txt",
    )


def query_esp32s3_uart_status(client: paramiko.SSHClient, daemon_remote: str) -> tuple[str, dict]:
    """Pause daemon briefly, query OTA and STATUS lines over UART, and parse key=value fields."""
    run_ssh(client, "sudo -n systemctl stop esp32s3-status-led.service 2>/dev/null || true", check=False)
    try:
        _, out, _ = run_ssh(
            client,
            f"python3 {daemon_remote} --ota-status && python3 {daemon_remote} --status",
            check=True,
        )
    finally:
        run_ssh(client, "sudo -n systemctl start esp32s3-status-led.service 2>/dev/null || true", check=False)

    parsed = {}
    for line in out.splitlines():
        if line.startswith("[ESP32S3_OTA]") or line.startswith("[ESP32S3_LED]"):
            for token in line.split()[1:]:
                if "=" in token:
                    k, v = token.split("=", 1)
                    parsed[k.strip()] = v.strip()
    return out, parsed


def deploy_via_usb(client: paramiko.SSHClient, base_dir: str, build_dir: str, daemon_remote: str):
    print("\n=== 3. Verifying ESP32-S3 USB Identity (303a:1001) ===")
    port = find_esp32s3_port(client)
    rc, udev_out, _ = run_ssh(client, f"udevadm info -q property -n '{port}'", check=False)
    if rc != 0:
        raise RuntimeError(
            f"ESP32-S3 USB port {port} is not connected! "
            "Reconnect the USB cable for --usb bootstrap/recovery or use --ota for wireless update."
        )
    vid, pid, devname = "", "", ""
    for line in udev_out.splitlines():
        if line.startswith("ID_VENDOR_ID="):
            vid = line.split("=", 1)[1].strip().lower()
        elif line.startswith("ID_MODEL_ID="):
            pid = line.split("=", 1)[1].strip().lower()
        elif line.startswith("DEVNAME="):
            devname = line.split("=", 1)[1].strip()

    if vid != EXPECTED_VID or pid != EXPECTED_PID or "ttyUSB" in devname:
        raise RuntimeError(
            f"SAFETY ABORT: Port {port} resolved to {devname} ({vid}:{pid}), "
            f"expected {EXPECTED_VID}:{EXPECTED_PID} on ttyACM!"
        )
    print(f"  PASS: {port} -> {devname} (VID:PID {vid}:{pid})")

    home_dir = os.path.expanduser("~")
    boot_app0 = os.path.join(
        home_dir,
        ".platformio",
        "packages",
        "framework-arduinoespressif32",
        "tools",
        "partitions",
        "boot_app0.bin",
    )
    images = [
        ("0x0000", os.path.join(build_dir, "bootloader.bin")),
        ("0x8000", os.path.join(build_dir, "partitions.bin")),
        ("0xe000", boot_app0),
        ("0x10000", os.path.join(build_dir, "firmware.bin")),
    ]
    for offset, path in images:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Missing build artifact for offset {offset}: {path}")

    run_ssh(client, "sudo -n systemctl stop esp32s3-status-led.service 2>/dev/null || true", check=False)

    sftp = client.open_sftp()
    flash_pairs = []
    for offset, local_path in images:
        fname = os.path.basename(local_path)
        rpath = f"{REMOTE_DIR}/{fname}"
        sftp.put(local_path, rpath)
        flash_pairs.append(f"{offset} '{rpath}'")
    sftp.close()

    print(f"\n=== 4. Flashing ESP32-S3 N16R8 ({port}) ===")
    flash_cmd = (
        f"python3 -m esptool --chip esp32s3 --port '{port}' --baud 460800 "
        f"--before default-reset --after hard-reset write-flash {' '.join(flash_pairs)}"
    )
    rc, flash_out, flash_err = run_ssh(client, flash_cmd, check=False)
    print(flash_out)
    if rc != 0:
        if flash_out.count("Hash of data verified.") >= 4 and "No such device" in flash_err:
            print("  Note: Native USB-Serial/JTAG re-enumerated on hard-reset after all 4 hashes verified.")
            time.sleep(2.0)
        else:
            raise RuntimeError(f"Flashing failed (rc={rc}):\n{flash_err}")

    ensure_rpi5_gpio_uart0(client)

    print("\n=== 5. Waiting for ESP32-S3 Boot & Wi-Fi Station Association ===")
    for _ in range(10):
        time.sleep(1.5)
        raw_status, parsed = query_esp32s3_uart_status(client, daemon_remote)
        if parsed.get("WIFI") == "CONNECTED" and parsed.get("IP", "0.0.0.0") != "0.0.0.0":
            print(raw_status)
            break
    else:
        print(raw_status)

    print("\n=== 6. Verifying I2C Multiplexer (0x70) over GPIO UART (/dev/ttyAMA0) ===")
    run_ssh(client, "sudo -n systemctl stop esp32s3-status-led.service 2>/dev/null || true", check=False)
    _, mux_scan_out, _ = run_ssh(client, f"python3 {daemon_remote} --scan-mux")
    print(mux_scan_out)

    print("\n=== 7. Installing & Starting esp32s3-status-led.service on RPi5 ===")
    install_and_start_systemd_service(client, daemon_remote)
    time.sleep(2.0)


def deploy_via_ota(
    client: paramiko.SSHClient,
    base_dir: str,
    build_dir: str,
    daemon_remote: str,
    secrets_cfg: dict,
    target_ip_override: str | None = None,
):
    print("\n=== 3. Verifying ESP32-S3 Target Identity & Wi-Fi IP over RPi5 GPIO UART (/dev/ttyAMA0) ===")
    raw_status, parsed = query_esp32s3_uart_status(client, daemon_remote)
    print(raw_status)

    host_reported = parsed.get("HOST", "")
    if host_reported != EXPECTED_OTA_HOSTNAME:
        raise RuntimeError(
            f"SAFETY ABORT: Target reported HOST='{host_reported}', expected '{EXPECTED_OTA_HOSTNAME}'!"
        )

    target_ip = target_ip_override or parsed.get("IP", "0.0.0.0")
    if not target_ip or target_ip == "0.0.0.0" or parsed.get("WIFI") != "CONNECTED":
        raise RuntimeError(
            f"ESP32-S3 Wi-Fi is not connected (WIFI={parsed.get('WIFI')}, IP={target_ip}). "
            "Cannot perform OTA update."
        )

    prev_fw = parsed.get("FW", "unknown")
    prev_part = parsed.get("PART", "unknown")
    print(
        f"  PASS: Verified target '{host_reported}' at {target_ip}:3232 "
        f"(current FW={prev_fw}, partition={prev_part})"
    )

    print("\n=== 4. Uploading Firmware & Executing Authenticated ArduinoOTA Update ===")
    local_fw = os.path.join(build_dir, "firmware.bin")
    remote_fw = f"{REMOTE_DIR}/firmware_ota.bin"
    remote_espota = f"{REMOTE_DIR}/espota.py"
    remote_secret_file = f"{REMOTE_DIR}/.ota_secret_key"
    remote_runner = f"{REMOTE_DIR}/run_ota_push.py"

    home_dir = os.path.expanduser("~")
    local_espota = os.path.join(
        home_dir,
        ".platformio",
        "packages",
        "framework-arduinoespressif32",
        "tools",
        "espota.py",
    )
    if not os.path.exists(local_espota):
        raise FileNotFoundError(f"Missing espota.py tool at {local_espota}")

    sftp = client.open_sftp()
    sftp.put(local_fw, remote_fw)
    sftp.put(local_espota, remote_espota)
    # Write secret key directly via SFTP with 0600 permissions so it never appears in shell commands
    with sftp.file(remote_secret_file, "w") as sf:
        sf.write(secrets_cfg["ota_password"] + "\n")
    sftp.chmod(remote_secret_file, 0o600)

    # Create a helper script on RPi5 that imports espota in-process so the password is never passed in sys.argv
    runner_code = f"""#!/usr/bin/env python3
import hashlib
import importlib.util
import logging
import os
import random
import sys

logging.basicConfig(level=logging.INFO, format="%(message)s")

secret_path = {remote_secret_file!r}
with open(secret_path, "r", encoding="utf-8") as f:
    ota_password = f.read().strip()

spec = importlib.util.spec_from_file_location("espota", {remote_espota!r})
espota = importlib.util.module_from_spec(spec)
spec.loader.exec_module(espota)
espota.PROGRESS = True
espota.TIMEOUT = 15

sys.exit(
    espota.serve(
        {target_ip!r},
        "0.0.0.0",
        3232,
        random.randint(10000, 60000),
        ota_password,
        {remote_fw!r},
        espota.FLASH,
    )
)
"""
    with sftp.file(remote_runner, "w") as rf:
        rf.write(runner_code)
    sftp.chmod(remote_runner, 0o700)
    sftp.close()

    # Ensure systemd daemon stays active DURING OTA so it coordinates/logs the maintenance window live
    install_and_start_systemd_service(client, daemon_remote)
    time.sleep(0.5)

    try:
        rc, ota_out, ota_err = run_ssh(client, f"python3 {remote_runner}", check=False)
        if ota_out:
            print(ota_out)
        if ota_err:
            print(ota_err)
        if rc != 0:
            raise RuntimeError(f"OTA push failed with exit code {rc}")
    finally:
        run_ssh(client, f"rm -f {remote_secret_file} {remote_runner}", check=False)

    print("\n=== 5. Waiting for ESP32-S3 Post-OTA Reboot & Reconnection ===")
    new_parsed = {}
    raw_after = ""
    for _ in range(12):
        time.sleep(1.5)
        raw_after, new_parsed = query_esp32s3_uart_status(client, daemon_remote)
        if new_parsed.get("WIFI") == "CONNECTED" and new_parsed.get("IP", "0.0.0.0") != "0.0.0.0":
            break
    print(raw_after.encode("ascii", "replace").decode("ascii"))

    print(
        f"  Post-OTA Transition: FW {prev_fw} ({prev_part}) -> "
        f"FW {new_parsed.get('FW', 'unknown')} ({new_parsed.get('PART', 'unknown')})"
    )

    print("\n=== 6. Verifying I2C Multiplexer (0x70) Post-OTA ===")
    run_ssh(client, "sudo -n systemctl stop esp32s3-status-led.service 2>/dev/null || true", check=False)
    _, mux_scan_out, _ = run_ssh(client, f"python3 {daemon_remote} --scan-mux")
    print(mux_scan_out.encode("ascii", "replace").decode("ascii"))
    run_ssh(client, "sudo -n systemctl start esp32s3-status-led.service")
    time.sleep(1.5)


def main():
    parser = argparse.ArgumentParser(description="Deploy Lonely Binary ESP32-S3 Status LED & Mux Firmware")
    mode_grp = parser.add_mutually_exclusive_group()
    mode_grp.add_argument("--usb", action="store_true", help="Flash via USB JTAG/serial (303a:1001) [default]")
    mode_grp.add_argument("--ota", action="store_true", help="Update wirelessly via authenticated ArduinoOTA")
    parser.add_argument("--provision-secrets", action="store_true", help="Provision local ignored Wi-Fi/OTA secrets and exit")
    parser.add_argument("--fw-version", type=str, default=None, help="Override ESP32S3_FW_VERSION string")
    parser.add_argument("--ota-ip", type=str, default=None, help="Optional explicit ESP32-S3 LAN IP override")
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.abspath(__file__))
    build_dir = os.path.join(base_dir, ".pio", "build", "esp32-s3-devkitc-1")

    print(f"=== 1. Connecting to RPi5 ({RPI_USER}@{RPI_HOST}) & Verifying Local Ignored Secrets ===")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(RPI_HOST, username=RPI_USER, timeout=10)

    try:
        secrets_cfg = ensure_local_ota_secrets(base_dir, client)
        if args.provision_secrets:
            print("Provisioning complete.")
            return

        compile_firmware(base_dir, build_dir, args.fw_version)
        daemon_remote = upload_and_update_daemon(client, base_dir)

        if args.ota:
            deploy_via_ota(client, base_dir, build_dir, daemon_remote, secrets_cfg, args.ota_ip)
        else:
            deploy_via_usb(client, base_dir, build_dir, daemon_remote)

        print("\n=== 8. Verifying esp32s3-status-led.service Journal & Maker-ESP32-Pro Untouched ===")
        _, status_out, _ = run_ssh(
            client,
            "systemctl is-active esp32s3-status-led.service && journalctl -u esp32s3-status-led.service -n 18 --no-pager",
        )
        print(status_out)

        _, rover_srv, _ = run_ssh(
            client,
            "systemctl is-active rover-server.service && curl -s http://127.0.0.1:3000/api/status",
        )
        print(rover_srv)
    finally:
        client.close()


if __name__ == "__main__":
    main()
