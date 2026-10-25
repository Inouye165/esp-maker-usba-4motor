#!/usr/bin/env python3
"""
Raspberry Pi 5 -> Lonely Binary ESP32-S3 N16R8 Status LED & ToF Multiplexer Daemon.

Supports both:
  1. Primary: RPi5 40-pin header GPIO UART0 (/dev/ttyAMA0, Pi GPIO14 TX -> ESP GPIO18 RX,
     Pi GPIO15 RX <- ESP GPIO17 TX, 115200 baud) when USB is unplugged.
  2. Fallback: ESP32-S3 native USB JTAG/serial debug unit
     (/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:72:3F:FC-if00,
     VID:PID 303a:1001).

Never opens /dev/rover-esp32 or /dev/ttyUSB0 (Maker-ESP32-Pro motor controller).

LED Policy:
  - Solid GREEN ("G\\n"): RPi5 is ON and rover is stationary/idle.
  - Solid BLUE  ("B\\n"): RPi5 is ON and rover is actively moving.
  - Solid RED   ("R\\n"): Sent immediately on daemon stop / RPi5 shutdown (and enforced
                          by ESP32-S3's 1.5s heartbeat watchdog timeout, which also puts
                          ESP32-S3 GPIO17 TX into high-Z INPUT to prevent backfeeding an
                          unpowered RPi5).

ToF Multiplexer Telemetry Bridge:
  - Parses `[ESP32S3_TOF] {...}` JSON lines from the ESP32-S3 (8-channel TCA9548A @ 0x70
    hosting VL53L7CX 8x8 and VL53L4CD narrow-field distance sensors) and writes the
    latest state atomically to `/tmp/esp32s3_tof_state.json` for the Cockpit server.
  - Watches `/tmp/esp32s3_tof_cmd.txt` for on-demand commands (e.g., `TOF_RESCAN`).
"""

import json
import os
import signal
import subprocess
import sys
import time
import urllib.request
import serial

GPIO_UART_PORT = "/dev/ttyAMA0"
ESP32S3_BY_ID = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:72:3F:FC-if00"
EXPECTED_VID = "303a"
EXPECTED_PID = "1001"
PID_TELEMETRY_URL = "http://127.0.0.1:3000/api/pid-telemetry"
TOF_STATE_FILE = "/tmp/esp32s3_tof_state.json"
TOF_CMD_FILE = "/tmp/esp32s3_tof_cmd.txt"
POLL_INTERVAL_S = 0.20

running = True


def handle_stop(signum, _frame):
    global running
    running = False


signal.signal(signal.SIGTERM, handle_stop)
signal.signal(signal.SIGINT, handle_stop)


def verify_gpio_uart_port(port_path: str) -> bool:
    """Verify port_path is the RPi5 RP1 hardware UART0 (/dev/ttyAMA0) and never a USB serial adapter."""
    if not os.path.exists(port_path):
        return False
    real_dev = os.path.realpath(port_path)
    if "ttyUSB" in real_dev or "rover-esp32" in port_path or "ttyACM" in real_dev:
        return False
    return real_dev == "/dev/ttyAMA0"


def verify_esp32s3_usb_port(port_path: str) -> bool:
    """Verify via udevadm that port_path is the 303a:1001 ESP32-S3 and never the Maker-ESP32."""
    if not os.path.exists(port_path):
        return False
    real_dev = os.path.realpath(port_path)
    if "ttyUSB" in real_dev or "rover-esp32" in port_path:
        return False
    try:
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", real_dev],
            text=True,
            timeout=2.0,
        )
    except Exception:
        return False

    vid, pid = "", ""
    for line in out.splitlines():
        if line.startswith("ID_VENDOR_ID="):
            vid = line.split("=", 1)[1].strip().lower()
        elif line.startswith("ID_MODEL_ID="):
            pid = line.split("=", 1)[1].strip().lower()
    return vid == EXPECTED_VID and pid == EXPECTED_PID


def select_target_port() -> str | None:
    """Prefer GPIO UART (/dev/ttyAMA0); fall back to verified ESP32-S3 USB JTAG if UART0 is absent."""
    if verify_gpio_uart_port(GPIO_UART_PORT):
        return GPIO_UART_PORT
    if verify_esp32s3_usb_port(ESP32S3_BY_ID):
        return ESP32S3_BY_ID
    return None


def is_rover_moving() -> bool:
    """Query Cockpit /api/pid-telemetry to check if any wheel is commanded or moving."""
    try:
        with urllib.request.urlopen(PID_TELEMETRY_URL, timeout=0.20) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        if not data.get("ok") or not data.get("valid") or data.get("stale"):
            return False
        telem = data.get("telemetry", {})
        outer = telem.get("outerYaw", {})
        if outer.get("actuationState") == "DRIVE":
            return True
        for key in ("m1", "m2", "m3", "m4"):
            w = telem.get(key, {})
            if (
                abs(float(w.get("targetRadps", 0.0))) > 0.05
                or abs(float(w.get("measuredRadps", 0.0))) > 0.15
                or abs(int(w.get("finalPwm", 0))) > 0
            ):
                return True
    except Exception:
        pass
    return False


def write_tof_state_file(payload: dict) -> None:
    """Atomically write latest ToF snapshot to /tmp/esp32s3_tof_state.json."""
    tmp_path = f"{TOF_STATE_FILE}.tmp"
    try:
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(payload, f)
        os.replace(tmp_path, TOF_STATE_FILE)
    except Exception:
        pass


def consume_pending_cmd() -> str | None:
    """Check if the Cockpit server requested an on-demand command (e.g. TOF_RESCAN)."""
    if not os.path.exists(TOF_CMD_FILE):
        return None
    try:
        with open(TOF_CMD_FILE, "r", encoding="utf-8") as f:
            cmd = f.read().strip()
        os.remove(TOF_CMD_FILE)
        return cmd if cmd else None
    except Exception:
        return None


def run_oneshot_query(query_cmd: str, wait_sec: float = 3.0) -> int:
    """Send a one-shot query (e.g. MUX, STATUS, OTA, TOF, TOF_RESCAN) over the verified ESP32-S3 port."""
    target_port = select_target_port()
    if not target_port:
        print("[esp32s3-led-daemon] ERROR: No verified ESP32-S3 serial port found.", file=sys.stderr)
        return 1
    print(f"[esp32s3-led-daemon] Querying '{query_cmd}' on verified port {target_port}...")
    with serial.Serial(
        port=target_port,
        baudrate=115200,
        timeout=0.5,
        write_timeout=0.5,
        dsrdtr=False,
        rtscts=False,
    ) as ser:
        ser.reset_input_buffer()
        # Ensure heartbeat is active so GPIO17 TX is driven before reading response
        ser.write(b"G\n")
        ser.flush()
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(f"{query_cmd}\n".encode("ascii"))
        ser.flush()
        deadline = time.time() + wait_sec
        while time.time() < deadline:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(line)
                if query_cmd == "MUX" and "SUMMARY" in line:
                    break
                if query_cmd == "STATUS" and "STATUS=" in line:
                    break
                if query_cmd == "OTA" and "[ESP32S3_OTA] FW=" in line:
                    break
                if query_cmd in ("TOF", "TOF_RESCAN") and line.startswith("[ESP32S3_TOF] "):
                    break
                if query_cmd.startswith("WIFI_OFF_TEST") and "EVENT=WIFI_TEST_START" in line:
                    break
                if query_cmd == "WIFI_ON" and "EVENT=WIFI_RECONNECT_TRIGGERED" in line:
                    break
    return 0


def main():
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg == "--scan-mux":
            sys.exit(run_oneshot_query("MUX"))
        elif arg == "--status":
            sys.exit(run_oneshot_query("STATUS"))
        elif arg == "--ota-status":
            sys.exit(run_oneshot_query("OTA"))
        elif arg == "--tof":
            sys.exit(run_oneshot_query("TOF"))
        elif arg == "--tof-rescan":
            sys.exit(run_oneshot_query("TOF_RESCAN", wait_sec=5.0))
        elif arg == "--wifi-off-test":
            dur = int(sys.argv[2]) if len(sys.argv) > 2 else 8
            sys.exit(run_oneshot_query(f"WIFI_OFF_TEST {dur}"))
        elif arg == "--wifi-on":
            sys.exit(run_oneshot_query("WIFI_ON"))

    ser = None
    last_state = None
    did_initial_mux_log = False
    ota_maintenance_active = False
    ota_reboot_deadline = 0.0
    last_rx_time = time.time()
    latest_led_status = {}
    latest_ota_status = {}

    print("[esp32s3-led-daemon] Starting RPi5 -> ESP32-S3 status LED & ToF daemon...", flush=True)

    try:
        while running:
            now = time.time()
            if ser is None:
                target_port = select_target_port()
                if not target_port:
                    time.sleep(1.0)
                    continue
                try:
                    ser = serial.Serial(
                        port=target_port,
                        baudrate=115200,
                        timeout=0.05,
                        write_timeout=0.5,
                        dsrdtr=False,
                        rtscts=False,
                    )
                    print(f"[esp32s3-led-daemon] Connected to {target_port}", flush=True)
                    last_state = None
                    did_initial_mux_log = False
                    last_rx_time = now
                except Exception as exc:
                    print(f"[esp32s3-led-daemon] Open failed on {target_port}: {exc}", flush=True)
                    time.sleep(1.0)
                    continue

            moving = is_rover_moving()
            cmd = b"B\n" if moving else b"G\n"
            state_label = "BLUE (MOVING)" if moving else "GREEN (RPI5 ON / IDLE)"

            try:
                ser.write(cmd)
                if not did_initial_mux_log:
                    ser.write(b"MUX\nSTATUS\nOTA\nTOF\n")
                    did_initial_mux_log = True

                pending_cmd = consume_pending_cmd()
                if pending_cmd:
                    print(f"[esp32s3-led-daemon] Executing requested command: {pending_cmd}", flush=True)
                    ser.write(f"{pending_cmd}\n".encode("ascii"))

                ser.flush()
                while ser.in_waiting > 0:
                    rx_line = ser.readline().decode("utf-8", errors="replace").strip()
                    if not rx_line:
                        continue
                    last_rx_time = time.time()

                    if rx_line.startswith("[ESP32S3_TOF] "):
                        raw_json = rx_line[len("[ESP32S3_TOF] ") :].strip()
                        try:
                            tof_data = json.loads(raw_json)
                            tof_data["host_timestamp_ms"] = int(time.time() * 1000)
                            tof_data["uart_port"] = ser.port
                            tof_data["led_state"] = state_label
                            if latest_led_status:
                                tof_data["esp_status"] = latest_led_status
                            if latest_ota_status:
                                tof_data["ota_status"] = latest_ota_status
                            write_tof_state_file(tof_data)
                        except Exception as parse_err:
                            print(f"[esp32s3-uart] ToF JSON parse error: {parse_err}", flush=True)
                        continue

                    print(f"[esp32s3-uart] {rx_line}", flush=True)
                    if rx_line.startswith("[ESP32S3_LED] STATUS="):
                        for tok in rx_line.split()[1:]:
                            if "=" in tok:
                                k, v = tok.split("=", 1)
                                latest_led_status[k] = v
                    elif rx_line.startswith("[ESP32S3_OTA] FW="):
                        for tok in rx_line.split()[1:]:
                            if "=" in tok:
                                k, v = tok.split("=", 1)
                                latest_ota_status[k] = v

                    if "[ESP32S3_OTA] EVENT=START" in rx_line:
                        ota_maintenance_active = True
                        ota_reboot_deadline = time.time() + 45.0
                        print(
                            "[esp32s3-led-daemon] MAINTENANCE: ESP32-S3 OTA firmware update started; "
                            "expecting temporary unavailability during flash & reboot.",
                            flush=True,
                        )
                    elif "[ESP32S3_OTA] EVENT=END STATUS=SUCCESS REBOOTING=1" in rx_line:
                        ota_maintenance_active = True
                        ota_reboot_deadline = time.time() + 15.0
                        print(
                            "[esp32s3-led-daemon] MAINTENANCE: ESP32-S3 OTA flash complete; "
                            "device is rebooting into new partition...",
                            flush=True,
                        )
                        did_initial_mux_log = False
                    elif "[ESP32S3_OTA] EVENT=ERROR" in rx_line:
                        ota_maintenance_active = False
                        print(
                            f"[esp32s3-led-daemon] WARNING: ESP32-S3 OTA reported error: {rx_line}",
                            flush=True,
                        )
                    elif ota_maintenance_active and (
                        "[ESP32S3_LED] State ->" in rx_line or "[ESP32S3_OTA] FW=" in rx_line
                    ):
                        ota_maintenance_active = False
                        print(
                            "[esp32s3-led-daemon] MAINTENANCE COMPLETE: ESP32-S3 reconnected over UART after OTA reboot.",
                            flush=True,
                        )

                if ota_maintenance_active and time.time() > ota_reboot_deadline:
                    ota_maintenance_active = False
                    print(
                        "[esp32s3-led-daemon] WARNING: OTA maintenance window expired; resuming normal monitoring.",
                        flush=True,
                    )

                if state_label != last_state:
                    print(f"[esp32s3-led-daemon] LED -> {state_label}", flush=True)
                    last_state = state_label
            except Exception as exc:
                if ota_maintenance_active:
                    print(
                        f"[esp32s3-led-daemon] MAINTENANCE: Serial link temporarily unavailable during OTA reboot ({exc})",
                        flush=True,
                    )
                else:
                    print(f"[esp32s3-led-daemon] Serial write error ({exc}), reconnecting...", flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(0.5)
                continue

            time.sleep(POLL_INTERVAL_S)
    finally:
        if ser is not None:
            try:
                print("[esp32s3-led-daemon] Stopping: setting ESP32-S3 LED to RED (OFF)...", flush=True)
                ser.write(b"R\n")
                ser.flush()
                time.sleep(0.05)
                ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
