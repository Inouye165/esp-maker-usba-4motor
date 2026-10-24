#!/usr/bin/env python3
"""
Raspberry Pi 5 -> Lonely Binary ESP32-S3 N16R8 Status LED Daemon.

Strictly isolated to the ESP32-S3 USB JTAG/serial debug unit:
  - By-ID path: /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:72:3F:FC-if00
  - USB VID:PID: 303a:1001

Never opens /dev/rover-esp32 or /dev/ttyUSB0 (Maker-ESP32-Pro motor controller).

LED Policy:
  - Solid GREEN ("G\\n"): RPi5 is ON and rover is stationary/idle.
  - Solid BLUE  ("B\\n"): RPi5 is ON and rover is actively moving.
  - Solid RED   ("R\\n"): Sent immediately on daemon stop / RPi5 shutdown (and enforced
                          by ESP32-S3's 1.5s heartbeat watchdog timeout).
"""

import json
import os
import signal
import subprocess
import sys
import time
import urllib.request
import serial

ESP32S3_BY_ID = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:72:3F:FC-if00"
EXPECTED_VID = "303a"
EXPECTED_PID = "1001"
PID_TELEMETRY_URL = "http://127.0.0.1:3000/api/pid-telemetry"
POLL_INTERVAL_S = 0.25

running = True


def handle_Stop(signum, _frame):
    global running
    running = False


signal.signal(signal.SIGTERM, handle_Stop)
signal.signal(signal.SIGINT, handle_Stop)


def verify_esp32s3_port(port_path: str) -> bool:
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


def is_rover_moving() -> bool:
    """Query Cockpit /api/pid-telemetry to check if any wheel is commanded or moving."""
    try:
        with urllib.request.urlopen(PID_TELEMETRY_URL, timeout=0.25) as resp:
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


def main():
    ser = None
    last_state = None
    print("[esp32s3-led-daemon] Starting RPi5 -> ESP32-S3 status LED daemon...", flush=True)

    try:
        while running:
            if ser is None:
                if not verify_esp32s3_port(ESP32S3_BY_ID):
                    time.sleep(1.0)
                    continue
                try:
                    ser = serial.Serial(
                        port=ESP32S3_BY_ID,
                        baudrate=115200,
                        timeout=0.1,
                        write_timeout=0.5,
                        dsrdtr=False,
                        rtscts=False,
                    )
                    print(f"[esp32s3-led-daemon] Connected to {ESP32S3_BY_ID}", flush=True)
                    last_state = None
                except Exception as exc:
                    print(f"[esp32s3-led-daemon] Open failed: {exc}", flush=True)
                    time.sleep(1.0)
                    continue

            moving = is_rover_moving()
            cmd = b"B\n" if moving else b"G\n"
            state_label = "BLUE (MOVING)" if moving else "GREEN (RPI5 ON / IDLE)"

            try:
                ser.write(cmd)
                ser.flush()
                if state_label != last_state:
                    print(f"[esp32s3-led-daemon] LED -> {state_label}", flush=True)
                    last_state = state_label
            except Exception as exc:
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
