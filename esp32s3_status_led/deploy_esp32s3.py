#!/usr/bin/env python3
"""
Deploy & Flash Lonely Binary ESP32-S3 N16R8 Status LED Firmware + RPi5 Systemd Service.

Safety Invariants:
  1. Verifies USB VID:PID is 303a:1001 (Espressif USB JTAG/serial debug unit) on
     /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:72:3F:FC-if00.
  2. Refuses to touch /dev/rover-esp32 or /dev/ttyUSB0 (1a86:7523 Maker-ESP32-Pro).
  3. Never stops or restarts rover-server.service.
"""

import os
import subprocess
import sys
import time
import paramiko

ESP32S3_PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:72:3F:FC-if00"
EXPECTED_VID = "303a"
EXPECTED_PID = "1001"
RPI_HOST = "10.0.0.246"
RPI_USER = "ron"


def run_ssh(client: paramiko.SSHClient, cmd: str, check: bool = True) -> tuple[int, str, str]:
    stdin, stdout, stderr = client.exec_command(cmd)
    rc = stdout.channel.recv_exit_status()
    out = stdout.read().decode("utf-8", errors="replace").strip()
    err = stderr.read().decode("utf-8", errors="replace").strip()
    if check and rc != 0:
        raise RuntimeError(f"Remote command failed (rc={rc}): {cmd}\nSTDOUT: {out}\nSTDERR: {err}")
    return rc, out, err


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    build_dir = os.path.join(base_dir, ".pio", "build", "esp32-s3-devkitc-1")

    print("=== 1. Compiling ESP32-S3 Firmware Locally ===")
    subprocess.run(["pio", "run", "-d", base_dir], check=True)

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
        print(f"  Verified local artifact {offset}: {os.path.basename(path)}")

    print(f"\n=== 2. Connecting to RPi5 ({RPI_USER}@{RPI_HOST}) ===")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(RPI_HOST, username=RPI_USER, timeout=10)

    try:
        print("\n=== 3. Verifying ESP32-S3 USB Identity (303a:1001) ===")
        _, udev_out, _ = run_ssh(client, f"udevadm info -q property -n '{ESP32S3_PORT}'")
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
                f"SAFETY ABORT: Port {ESP32S3_PORT} resolved to {devname} ({vid}:{pid}), "
                f"expected {EXPECTED_VID}:{EXPECTED_PID} on ttyACM!"
            )
        print(f"  PASS: {ESP32S3_PORT} -> {devname} (VID:PID {vid}:{pid})")

        # Stop esp32s3-status-led.service if already running so port is free
        run_ssh(client, "sudo -n systemctl stop esp32s3-status-led.service 2>/dev/null || true", check=False)

        remote_dir = "/home/ron/esp32s3_status_led"
        run_ssh(client, f"mkdir -p {remote_dir}")

        sftp = client.open_sftp()
        flash_pairs = []
        for offset, local_path in images:
            fname = os.path.basename(local_path)
            rpath = f"{remote_dir}/{fname}"
            sftp.put(local_path, rpath)
            flash_pairs.append(f"{offset} '{rpath}'")

        # Upload daemon script
        daemon_local = os.path.join(base_dir, "rpi5_status_led_daemon.py")
        daemon_remote = f"{remote_dir}/rpi5_status_led_daemon.py"
        sftp.put(daemon_local, daemon_remote)
        sftp.close()
        run_ssh(client, f"chmod +x {daemon_remote}")

        print("\n=== 4. Flashing ESP32-S3 N16R8 (/dev/ttyACM0) ===")
        flash_cmd = (
            f"python3 -m esptool --chip esp32s3 --port '{ESP32S3_PORT}' --baud 460800 "
            f"--before default-reset --after hard-reset write-flash {' '.join(flash_pairs)}"
        )
        rc, flash_out, flash_err = run_ssh(client, flash_cmd, check=False)
        print(flash_out)
        if rc != 0:
            # Native USB-Serial/JTAG (303a:1001) re-enumerates immediately on RTS reset,
            # which can cause pyserial's final setRTS(False) to raise Errno 19 after all hashes verified.
            if flash_out.count("Hash of data verified.") >= 4 and "No such device" in flash_err:
                print("  Note: Native USB-Serial/JTAG re-enumerated on hard-reset after all 4 hashes verified.")
                time.sleep(2.0)
            else:
                raise RuntimeError(f"Flashing failed (rc={rc}):\n{flash_err}")

        print("\n=== 5. Installing & Starting esp32s3-status-led.service on RPi5 ===")
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
        time.sleep(2.0)

        _, status_out, _ = run_ssh(client, "systemctl is-active esp32s3-status-led.service && journalctl -u esp32s3-status-led.service -n 10 --no-pager")
        print(status_out)

        print("\n=== 6. Verifying Maker-ESP32-Pro Untouched ===")
        _, rover_srv, _ = run_ssh(client, "systemctl is-active rover-server.service && curl -s http://127.0.0.1:3000/api/status")
        print(rover_srv)

    finally:
        client.close()


if __name__ == "__main__":
    main()
