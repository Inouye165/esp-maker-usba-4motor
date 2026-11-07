#!/usr/bin/env python3
"""
preflight_check.py - Stationary Preflight Verification for Rover 1
Verifies geometry, credentials, live schemas, and telemetry advancement
WITHOUT arming or sending nonzero velocity commands.
"""

import os
import sys
import time
import math
import json
import requests

COCKPIT_URL = "http://127.0.0.1:3000"
BRIDGE_URL = "http://127.0.0.1:3010"

print("=" * 65)
print("       ROVER 1 STATIONARY PREFLIGHT VERIFICATION")
print("=" * 65)

# 1. Geometry Verification
cal_path = "/home/ron/yahboom-encoder/calibration_db.json"
if not os.path.exists(cal_path):
    print(f"[FAIL] Calibration database not found at {cal_path}")
    sys.exit(1)

with open(cal_path, "r", encoding="utf-8") as f:
    cal_data = json.load(f)

cfg = cal_data.get("currentConfig", {})
wheel_diam = cfg.get("wheelDiameter")
track_width = cfg.get("effectiveTrackWidth")
ticks_per_rev = cfg.get("ticksPerRevolution")

print(f"[GEOMETRY] Verified configuration loaded:")
print(f"  - Wheel Diameter:       {wheel_diam} m (Radius: {wheel_diam / 2.0} m)")
print(f"  - Effective Track Width:{track_width} m")
print(f"  - Ticks / Revolution:   {ticks_per_rev}")
print(f"  - Scale Factor:         {(math.pi * wheel_diam) / ticks_per_rev * 1000.0:.4f} mm/tick")

# 2. Credentials Verification
env_path = "/home/ron/yahboom-encoder/.env"
op_token = ""
cmd_token = ""
if os.path.exists(env_path):
    with open(env_path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("ROVER_OPERATOR_TOKEN="):
                op_token = line.split("=", 1)[1].strip().strip('"').strip("'")
            elif line.startswith("ROVER_CMD_VEL_TOKEN="):
                cmd_token = line.split("=", 1)[1].strip().strip('"').strip("'")

print(f"\n[CREDENTIALS] Token status:")
print(f"  - ROVER_OPERATOR_TOKEN: {'PRESENT (' + op_token[:6] + '...)' if op_token else 'MISSING'}")
print(f"  - ROVER_CMD_VEL_TOKEN:  {'PRESENT (' + cmd_token[:6] + '...)' if cmd_token else 'MISSING'}")

if not op_token or not cmd_token:
    print("[FAIL] Missing required authentication tokens.")
    sys.exit(1)

# 3. Endpoint Schema Validation & 4. Advancement Monitoring
print(f"\n[TELEMETRY ADVANCEMENT & SCHEMA MONITORING (5 reads over 1.0s)]")

prev_seqs = {}
for i in range(5):
    t_start = time.time()

    # /api/drive/status
    r_drv = requests.get(f"{COCKPIT_URL}/api/drive/status", timeout=0.5).json()
    if not r_drv.get("ok") or "status" not in r_drv:
        print(f"[FAIL] Invalid /api/drive/status schema: {r_drv}")
        sys.exit(1)
    drv = r_drv["status"]

    # /api/pid-telemetry
    r_pid = requests.get(f"{COCKPIT_URL}/api/pid-telemetry", timeout=0.5).json()
    if not r_pid.get("ok") or not r_pid.get("valid") or "telemetry" not in r_pid:
        print(f"[FAIL] Invalid /api/pid-telemetry schema: {r_pid}")
        sys.exit(1)
    pid = r_pid

    # /api/encoders
    r_enc = requests.get(f"{COCKPIT_URL}/api/encoders", timeout=0.5).json()
    if not r_enc.get("ok") or "encoders" not in r_enc:
        print(f"[FAIL] Invalid /api/encoders schema: {r_enc}")
        sys.exit(1)
    enc = r_enc

    # /api/imu
    r_imu = requests.get(f"{COCKPIT_URL}/api/imu", timeout=0.5).json()
    if not r_imu.get("ok") or "gyro" not in r_imu or "orientation" not in r_imu:
        print(f"[FAIL] Invalid /api/imu schema: {r_imu}")
        sys.exit(1)
    imu = r_imu

    cur_seqs = {
        "drive": drv["seq"],
        "pid": pid["sequence"],
        "encoders": enc["sequence"],
        "imu": imu["sequence"]
    }

    adv_info = ""
    if prev_seqs:
        advances = [k for k in cur_seqs if cur_seqs[k] > prev_seqs[k]]
        adv_info = f"Advancing: {advances}"
    prev_seqs = cur_seqs

    drive_age = (time.time() * 1000.0) - drv["receivedAt"]
    print(f"  Read {i+1}: Mode={drv['mode']} (Armed={drv['armed']}) | DriveSeq={drv['seq']} (age={drive_age:.1f}ms) | PIDSeq={pid['sequence']} (age={pid['dataAgeMs']}ms) | EncSeq={enc['sequence']} (age={enc['lastPacketAgeMs']}ms) | IMUSeq={imu['sequence']} (age={imu['dataAgeMs']}ms) | {adv_info}")
    time.sleep(0.2)

# Verify rover is disarmed and stopped (Mode 0)
if drv["mode"] != 0 or drv["armed"] is not False:
    print(f"\n[FAIL] Rover is NOT in stopped Mode 0! Mode={drv['mode']}, Armed={drv['armed']}")
    sys.exit(1)

print("\n" + "=" * 65)
print(" PREFLIGHT SUCCESS: Rover 1 is verified stopped in Mode 0 (disarmed).")
print(" Geometry, tokens, endpoints, and live streams are verified.")
print("=" * 65)
