#!/usr/bin/env python3
"""
characterize_floor_speeds.py - Empirical Floor Speed & Stiction Characterization for Rover 1

Safety & Production Invariants:
1. Commissioning limits strictly enforced: linear <= 0.20 m/s, angular <= 0.50 rad/s.
2. Operator runs ONE selected case per invocation. Default is reviewed baseline failed arc.
3. Full fail-closed fault latch: Any recovery exhaustion, rejected command, arm failure,
   stale/missing telemetry, non-advancing stream, unexpected disarm, or failed stop
   verification latches FAULT and permanently blocks all subsequent arming attempts.
4. Independent cleanup: Zero velocity and disarm are attempted independently and verified.
   Cleanup failure prevents SUCCESS and marks CLEANUP_FAILED.
5. Deployed geometry is verified from configuration interface (calibration_db.json / drive status).
   Refuses execution if verified configuration cannot be loaded.
6. Continuous signed yaw tracking with modulo-2pi unwrap.
7. Motion bounds track accumulated absolute wheel travel and absolute yaw change to prevent
   opposing wheels or reversals from evading limits.
8. Telemetry validation: Every required wheel field (measuredRadps, targetRadps, finalPwm)
   must be numeric and finite. Missing or non-finite fields cause immediate abort.
9. Stop confirmation requires 200 ms of uninterrupted near-zero speeds and gyro readings
   with advancing post-zero PID and IMU sequence evidence. Timer resets if motion resumes.
10. Telemetry receipt time is recorded immediately after fetch; loop-start times are never
    used to label measurements or qualify the stationary interval.
11. Every zero-velocity command response is checked, including stopping-loop and motion-bound calls.
    Rejected zero command or latched fault terminates case with failure and prevents SUCCESS.
12. Drive status freshness is strictly validated using dataAgeMs and receivedAt epoch timestamps.
    Stale status refuses arming and disarm verification.
13. Stall duration is calculated from actual sample timestamps, not an assumed rate.
14. Exceptions preserve a partial report and exit with a nonzero status.

Canonical Wheel Mapping:
  M1 = LF (Left Front)
  M2 = RF (Right Front)
  M3 = LR (Left Rear)
  M4 = RR (Right Rear)
  Left Track = {M1, M3}, Right Track = {M2, M4}
"""

import os
import sys
import time
import math
import json
import argparse
from datetime import datetime
from typing import Dict, Any, List, Optional, Tuple
import requests

COCKPIT_URL = os.environ.get("COCKPIT_URL", "http://127.0.0.1:3000").rstrip("/")
BRIDGE_URL = os.environ.get("BRIDGE_URL", "http://127.0.0.1:3010").rstrip("/")

MAX_COMMISSIONING_LINEAR_MPS = 0.20
MAX_COMMISSIONING_ANGULAR_RADPS = 0.50

MAX_STEP_DURATION_S = 2.5
MAX_DIAGNOSTIC_DURATION_S = 6.5
MAX_STRAIGHT_DISTANCE_M = 0.30
MAX_SPIN_ROTATION_DEG = 45.0
MAX_ARC_DISTANCE_M = 0.15
MAX_ARC_ROTATION_DEG = 30.0

TELEMETRY_MAX_AGE_MS = 250.0
COMMAND_RATE_HZ = 20.0
STOP_SETTLE_TIMEOUT_S = 1.0
STOP_CONFIRM_WINDOW_S = 0.200

REQUEST_TIMEOUT_S = 0.150
CMD_TIMEOUT_S = 0.100


def get_token(var_name: str) -> str:
    val = os.environ.get(var_name, "").strip()
    if val:
        return val
    candidates = [
        "/home/ron/yahboom-encoder/.env",
        r"c:\Users\Ron\electronic_projects\yahboom-encoder\.env"
    ]
    for p in candidates:
        if os.path.exists(p):
            try:
                with open(p, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith(f"{var_name}="):
                            return line.split("=", 1)[1].strip().strip('"').strip("'")
            except Exception:
                pass
    return ""


class ContinuousYawTracker:
    def __init__(self):
        self.initial_yaw_rad: Optional[float] = None
        self.last_raw_yaw_rad: Optional[float] = None
        self.accumulated_yaw_rad: float = 0.0

    def reset(self):
        self.initial_yaw_rad = None
        self.last_raw_yaw_rad = None
        self.accumulated_yaw_rad = 0.0

    def update(self, raw_yaw_rad: float) -> Tuple[float, float]:
        """
        Updates continuous yaw. Returns (continuous_yaw_rad, signed_delta_rad).
        """
        if self.initial_yaw_rad is None:
            self.initial_yaw_rad = raw_yaw_rad
            self.last_raw_yaw_rad = raw_yaw_rad
            self.accumulated_yaw_rad = 0.0
            return 0.0, 0.0

        diff = raw_yaw_rad - self.last_raw_yaw_rad
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi

        self.accumulated_yaw_rad += diff
        self.last_raw_yaw_rad = raw_yaw_rad
        return self.accumulated_yaw_rad, diff

    @property
    def relative_yaw_deg(self) -> float:
        return math.degrees(self.accumulated_yaw_rad)


class FloorSpeedCharacterizer:
    def __init__(
        self,
        cockpit_url: str = COCKPIT_URL,
        bridge_url: str = BRIDGE_URL,
        geometry_override: Optional[Dict[str, float]] = None
    ):
        self.cockpit_url = cockpit_url.rstrip("/")
        self.bridge_url = bridge_url.rstrip("/")

        self.session = requests.Session()
        adapter = requests.adapters.HTTPAdapter(pool_connections=10, pool_maxsize=10)
        self.session.mount("http://", adapter)

        self.op_token = get_token("ROVER_OPERATOR_TOKEN")
        self.cmd_token = get_token("ROVER_CMD_VEL_TOKEN")

        # Safety latch: Once faulted, all arming is permanently blocked for this run
        self.fault_latched: bool = False
        self.fault_reason: str = ""

        # Geometry fields
        self.wheel_diameter_m: float = 0.0
        self.wheel_radius_m: float = 0.0
        self.effective_track_width_m: float = 0.0
        self.ticks_per_rev: float = 0.0
        self.meters_per_tick: float = 0.0

        # Monotonic sequence & freeze trackers
        self.stream_seq: Dict[str, Optional[int]] = {
            "drive": None, "pid": None, "encoders": None, "imu": None
        }
        self.stream_last_adv_mono: Dict[str, Optional[float]] = {
            "drive": None, "pid": None, "encoders": None, "imu": None
        }

        self.yaw_tracker = ContinuousYawTracker()
        self.active_report: Optional[Dict[str, Any]] = None

        # Command dispatch & acknowledgement timestamp tracking
        self.last_cmd_dispatch_mono: Optional[float] = None
        self.last_cmd_ack_mono: Optional[float] = None
        self.last_zero_dispatch_mono: Optional[float] = None
        self.last_zero_ack_mono: Optional[float] = None

        self.load_and_verify_geometry(geometry_override)

    def load_and_verify_geometry(self, geometry_override: Optional[Dict[str, float]] = None):
        """Loads and verifies deployed geometry from calibration interface."""
        if geometry_override:
            cfg = geometry_override
            if "wheelDiameter" not in cfg or "effectiveTrackWidth" not in cfg or "ticksPerRevolution" not in cfg:
                raise RuntimeError("Geometry override missing required keys")
            self.wheel_diameter_m = float(cfg["wheelDiameter"])
            self.wheel_radius_m = self.wheel_diameter_m / 2.0
            self.effective_track_width_m = float(cfg["effectiveTrackWidth"])
            self.ticks_per_rev = float(cfg["ticksPerRevolution"])
        else:
            cal_paths = [
                "/home/ron/yahboom-encoder/calibration_db.json",
                r"c:\Users\Ron\electronic_projects\yahboom-encoder\calibration_db.json"
            ]
            cal_loaded = False
            for p in cal_paths:
                if os.path.exists(p):
                    try:
                        with open(p, "r", encoding="utf-8") as f:
                            data = json.load(f)
                        cfg = data.get("currentConfig", {})
                        if cfg.get("wheelDiameter") and cfg.get("effectiveTrackWidth") and cfg.get("ticksPerRevolution"):
                            self.wheel_diameter_m = float(cfg["wheelDiameter"])
                            self.wheel_radius_m = self.wheel_diameter_m / 2.0
                            self.effective_track_width_m = float(cfg["effectiveTrackWidth"])
                            self.ticks_per_rev = float(cfg["ticksPerRevolution"])
                            cal_loaded = True
                            break
                    except Exception as e:
                        raise RuntimeError(f"Failed to parse calibration_db.json: {e}")

            if not cal_loaded:
                raise RuntimeError("Refusing to start: Verified deployed geometry could not be loaded from configuration interface.")

        if self.wheel_diameter_m <= 0 or not math.isfinite(self.wheel_diameter_m):
            raise RuntimeError(f"Invalid wheel diameter: {self.wheel_diameter_m}")
        if self.effective_track_width_m <= 0 or not math.isfinite(self.effective_track_width_m):
            raise RuntimeError(f"Invalid effective track width: {self.effective_track_width_m}")
        if self.ticks_per_rev <= 0 or not math.isfinite(self.ticks_per_rev):
            raise RuntimeError(f"Invalid ticks per revolution: {self.ticks_per_rev}")

        self.meters_per_tick = (math.pi * self.wheel_diameter_m) / self.ticks_per_rev

    def trigger_fault(self, reason: str):
        """Latches fault state and initiates fail-closed stop."""
        self.fault_latched = True
        self.fault_reason = reason
        print(f"\n[CRITICAL FAULT LATCHED] {reason}")
        self.fail_closed_cleanup()

    def check_stream_advancement(self, name: str, seq: int, now_mono: float):
        """Validates sequence monotonicity and detects streams that stop advancing."""
        last_s = self.stream_seq[name]
        if last_s is None:
            self.stream_seq[name] = seq
            self.stream_last_adv_mono[name] = now_mono
            return

        if seq < last_s:
            raise RuntimeError(f"Stream '{name}' sequence regressed ({seq} < {last_s})")
        elif seq > last_s:
            self.stream_seq[name] = seq
            self.stream_last_adv_mono[name] = now_mono
        else:
            time_frozen_ms = (now_mono - self.stream_last_adv_mono[name]) * 1000.0
            if time_frozen_ms > TELEMETRY_MAX_AGE_MS:
                raise RuntimeError(
                    f"Stream '{name}' stopped advancing (sequence {seq} frozen for {time_frozen_ms:.1f} ms > {TELEMETRY_MAX_AGE_MS} ms)"
                )

    def fetch_drive_status(self) -> Dict[str, Any]:
        """Queries /api/drive/status and strictly unpacks nested 'status', rejecting stale status."""
        try:
            r = self.session.get(f"{self.cockpit_url}/api/drive/status", timeout=REQUEST_TIMEOUT_S)
            if r.status_code != 200:
                raise RuntimeError(f"HTTP {r.status_code} from /api/drive/status")
            data = r.json()
            if not data.get("ok"):
                raise RuntimeError(f"/api/drive/status returned ok=false: {data}")
            status = data.get("status")
            if not status or not isinstance(status, dict):
                raise RuntimeError(f"Missing or invalid nested 'status' in /api/drive/status: {data}")

            required_keys = ["armed", "mode", "uptimeMs", "seq", "receivedAt", "noMotionBudgetMs", "spinFailedExhausted"]
            for k in required_keys:
                if k not in status:
                    raise RuntimeError(f"Missing required drive status key '{k}'")

            if status.get("stale"):
                raise RuntimeError("Drive status reports stale=true")

            rx_at = status.get("receivedAt")
            if rx_at is None:
                raise RuntimeError("Missing required 'receivedAt' timestamp in drive status")

            # Check explicit dataAgeMs first if server reported it
            age_ms = status.get("dataAgeMs")
            if age_ms is not None:
                if age_ms > TELEMETRY_MAX_AGE_MS:
                    raise RuntimeError(f"Drive status stale (dataAgeMs={age_ms} ms > {TELEMETRY_MAX_AGE_MS} ms)")
            else:
                # Calculate age using actual receivedAt timestamp (epoch ms)
                calc_age_ms = (time.time() * 1000.0) - float(rx_at)
                if calc_age_ms > TELEMETRY_MAX_AGE_MS:
                    raise RuntimeError(f"Drive status stale (age={calc_age_ms:.1f} ms > {TELEMETRY_MAX_AGE_MS} ms)")
                elif calc_age_ms < -2000.0:
                    raise RuntimeError(f"Drive status receivedAt in future (skew={calc_age_ms:.1f} ms)")

            return status
        except Exception as e:
            raise RuntimeError(f"Drive status query failure: {e}")

    def fetch_pid_telemetry(self) -> Dict[str, Any]:
        """Queries actual /api/pid-telemetry endpoint, validating numeric and finite fields."""
        try:
            r = self.session.get(f"{self.cockpit_url}/api/pid-telemetry", timeout=REQUEST_TIMEOUT_S)
            if r.status_code != 200:
                raise RuntimeError(f"HTTP {r.status_code} from /api/pid-telemetry")
            data = r.json()
            if not data.get("ok") or not data.get("valid"):
                raise RuntimeError(f"/api/pid-telemetry invalid: {data}")
            if data.get("stale"):
                raise RuntimeError(f"/api/pid-telemetry reports stale=true")

            age_ms = data.get("dataAgeMs", 9999)
            if age_ms > TELEMETRY_MAX_AGE_MS:
                raise RuntimeError(f"PID telemetry stale ({age_ms} ms > {TELEMETRY_MAX_AGE_MS} ms)")

            telem = data.get("telemetry")
            if not telem or not isinstance(telem, dict):
                raise RuntimeError(f"Missing 'telemetry' object in /api/pid-telemetry")

            required_wheel_fields = ["measuredRadps", "targetRadps", "finalPwm"]
            for m in ["m1", "m2", "m3", "m4"]:
                if m not in telem or not isinstance(telem[m], dict):
                    raise RuntimeError(f"Missing wheel telemetry '{m}' in /api/pid-telemetry")
                w = telem[m]
                for f in required_wheel_fields:
                    if f not in w or w[f] is None:
                        raise RuntimeError(f"Missing required field '{f}' in wheel '{m}' telemetry")
                    val = w[f]
                    if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
                        raise RuntimeError(f"Non-numeric or non-finite '{f}'={val} in wheel '{m}' telemetry")

            return {
                "sequence": data.get("sequence"),
                "timestamp": data.get("timestamp"),
                "dataAgeMs": age_ms,
                "telemetry": telem
            }
        except Exception as e:
            raise RuntimeError(f"PID telemetry query failure: {e}")

    def fetch_encoders(self) -> Dict[str, Any]:
        """Queries /api/encoders."""
        try:
            r = self.session.get(f"{self.cockpit_url}/api/encoders", timeout=REQUEST_TIMEOUT_S)
            if r.status_code != 200:
                raise RuntimeError(f"HTTP {r.status_code} from /api/encoders")
            data = r.json()
            if not data.get("ok"):
                raise RuntimeError(f"/api/encoders returned ok=false: {data}")

            age_ms = data.get("lastPacketAgeMs", 9999)
            if age_ms > TELEMETRY_MAX_AGE_MS:
                raise RuntimeError(f"Encoder telemetry stale ({age_ms} ms > {TELEMETRY_MAX_AGE_MS} ms)")

            encs = data.get("encoders")
            if not encs or not isinstance(encs, dict):
                raise RuntimeError(f"Missing 'encoders' object in /api/encoders")

            for m in ["m1", "m2", "m3", "m4"]:
                if m not in encs or encs[m] is None:
                    raise RuntimeError(f"Missing encoder tick '{m}' in /api/encoders")
                val = encs[m]
                if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
                    raise RuntimeError(f"Non-numeric or non-finite encoder tick '{m}'={val}")

            return {
                "sequence": data.get("sequence"),
                "timestamp": data.get("timestamp"),
                "lastPacketAgeMs": age_ms,
                "encoders": encs
            }
        except Exception as e:
            raise RuntimeError(f"Encoder query failure: {e}")

    def fetch_imu(self) -> Dict[str, Any]:
        """Queries /api/imu and calculates continuous yaw."""
        try:
            r = self.session.get(f"{self.cockpit_url}/api/imu", timeout=REQUEST_TIMEOUT_S)
            if r.status_code != 200:
                raise RuntimeError(f"HTTP {r.status_code} from /api/imu")
            data = r.json()
            if not data.get("ok"):
                raise RuntimeError(f"/api/imu returned ok=false: {data}")
            if data.get("stale"):
                raise RuntimeError(f"/api/imu reports stale=true")

            age_ms = data.get("dataAgeMs", 9999)
            if age_ms > TELEMETRY_MAX_AGE_MS:
                raise RuntimeError(f"IMU data stale ({age_ms} ms > {TELEMETRY_MAX_AGE_MS} ms)")

            gyro = data.get("gyro", {})
            if "z" not in gyro or gyro["z"] is None:
                raise RuntimeError(f"Missing gyro.z in /api/imu")
            gyro_z = float(gyro["z"])
            if not math.isfinite(gyro_z):
                raise RuntimeError(f"Non-finite gyro_z in /api/imu: {gyro_z}")

            q = data.get("orientation", {})
            for axis in ["w", "x", "y", "z"]:
                if axis not in q or q[axis] is None:
                    raise RuntimeError(f"Missing orientation.{axis} in /api/imu")
                val = float(q[axis])
                if not math.isfinite(val):
                    raise RuntimeError(f"Non-finite orientation.{axis} in /api/imu: {val}")

            qw, qx, qy, qz = float(q["w"]), float(q["x"]), float(q["y"]), float(q["z"])
            raw_yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

            return {
                "sequence": data.get("sequence"),
                "timestamp": data.get("timestamp"),
                "dataAgeMs": age_ms,
                "gyro_z": gyro_z,
                "raw_yaw_rad": raw_yaw_rad,
                "orientation": q
            }
        except Exception as e:
            raise RuntimeError(f"IMU query failure: {e}")

    def fetch_all_telemetry(self, deadline_checker=None) -> Dict[str, Any]:
        """
        Fetches all 4 telemetry streams, validating freshness and sequence advancement.
        Records exact receipt timestamp immediately upon completion.
        """
        if deadline_checker:
            deadline_checker()
        drv = self.fetch_drive_status()

        if deadline_checker:
            deadline_checker()
        pid = self.fetch_pid_telemetry()

        if deadline_checker:
            deadline_checker()
        enc = self.fetch_encoders()

        if deadline_checker:
            deadline_checker()
        imu = self.fetch_imu()

        fetch_receipt_mono = time.monotonic()

        self.check_stream_advancement("drive", drv["seq"], fetch_receipt_mono)
        self.check_stream_advancement("pid", pid["sequence"], fetch_receipt_mono)
        self.check_stream_advancement("encoders", enc["sequence"], fetch_receipt_mono)
        self.check_stream_advancement("imu", imu["sequence"], fetch_receipt_mono)

        cont_yaw_rad, d_yaw_signed_rad = self.yaw_tracker.update(imu["raw_yaw_rad"])

        return {
            "fetch_mono": fetch_receipt_mono,
            "drive": drv,
            "pid": pid,
            "encoders": enc,
            "imu": imu,
            "continuous_yaw_deg": math.degrees(cont_yaw_rad),
            "delta_yaw_deg": math.degrees(d_yaw_signed_rad)
        }

    def zero_handshake(self) -> bool:
        """Sends 3 zero commands to bridge /api/cmd_vel to complete zero handshake."""
        headers = {"X-Rover-Bridge-Token": self.cmd_token} if self.cmd_token else {}
        payload = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            "source": "CALIBRATION_TEST"
        }
        for i in range(3):
            try:
                r = self.session.post(f"{self.bridge_url}/api/cmd_vel", json=payload, headers=headers, timeout=CMD_TIMEOUT_S)
                if r.status_code != 200:
                    print(f"[ERROR] Zero handshake {i+1}/3 rejected: HTTP {r.status_code}")
                    return False
                if not r.json().get("ok", False):
                    print(f"[ERROR] Zero handshake {i+1}/3 returned ok=false: {r.text}")
                    return False
            except Exception as e:
                print(f"[ERROR] Zero handshake exception: {e}")
                return False
            time.sleep(0.04)
        return True

    def verify_stopped_and_disarmed(self) -> bool:
        """Verifies rover is in Mode 0 (LOCKED) and disarmed with fresh telemetry."""
        try:
            status = self.fetch_drive_status()
            if status.get("armed") is not False or status.get("mode") != 0:
                return False
            return True
        except Exception as e:
            print(f"[PREFLIGHT/ARM REFUSAL] verify_stopped_and_disarmed rejected: {e}")
            return False

    def arm(self) -> bool:
        """Arms normal drive (Mode 3). Refuses if fault latched or drive status stale/not Mode 0."""
        if self.fault_latched:
            raise RuntimeError(f"Arming refused: Fault is latched ({self.fault_reason})")

        if not self.verify_stopped_and_disarmed():
            raise RuntimeError("Arming refused: Rover is not confirmed stopped, disarmed, and fresh (Mode 0).")

        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        try:
            r = self.session.post(f"{self.cockpit_url}/api/drive/arm", headers=headers, timeout=2.0)
            if r.status_code != 200:
                raise RuntimeError(f"Arm request rejected: HTTP {r.status_code} - {r.text}")
            res = r.json()
            if not res.get("ok"):
                raise RuntimeError(f"Arm request returned ok=false: {res}")
        except Exception as e:
            raise RuntimeError(f"Exception calling /api/drive/arm: {e}")

        # Confirm Mode 3 in fresh telemetry
        time.sleep(0.05)
        status = self.fetch_drive_status()
        if not status.get("armed") or status.get("mode") != 3:
            raise RuntimeError(f"Drive did not confirm Mode 3: armed={status.get('armed')}, mode={status.get('mode')}")

        return True

    def send_velocity(self, vx: float, wz: float) -> bool:
        """Sends velocity command to velocity bridge."""
        if self.fault_latched:
            return False

        vx_clamped = max(-MAX_COMMISSIONING_LINEAR_MPS, min(MAX_COMMISSIONING_LINEAR_MPS, vx))
        wz_clamped = max(-MAX_COMMISSIONING_ANGULAR_RADPS, min(MAX_COMMISSIONING_ANGULAR_RADPS, wz))

        payload = {
            "linear": {"x": float(vx_clamped), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(wz_clamped)},
            "source": "CALIBRATION_TEST"
        }
        headers = {"X-Rover-Bridge-Token": self.cmd_token} if self.cmd_token else {}
        t_dispatch = time.monotonic()
        self.last_cmd_dispatch_mono = t_dispatch
        try:
            r = self.session.post(f"{self.bridge_url}/api/cmd_vel", json=payload, headers=headers, timeout=CMD_TIMEOUT_S)
            t_ack = time.monotonic()
            self.last_cmd_ack_mono = t_ack
            if r.status_code != 200 or not r.json().get("ok"):
                self.trigger_fault(f"Velocity command rejected (HTTP {r.status_code}): {r.text}")
                return False
            return True
        except Exception as e:
            self.trigger_fault(f"Velocity command exception: {e}")
            return False

    def send_zero_velocity(self) -> bool:
        """Sends zero velocity command to bridge and strictly verifies response."""
        payload = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            "source": "CALIBRATION_TEST"
        }
        headers = {"X-Rover-Bridge-Token": self.cmd_token} if self.cmd_token else {}
        t_dispatch = time.monotonic()
        self.last_zero_dispatch_mono = t_dispatch
        try:
            r = self.session.post(f"{self.bridge_url}/api/cmd_vel", json=payload, headers=headers, timeout=CMD_TIMEOUT_S)
            t_ack = time.monotonic()
            self.last_zero_ack_mono = t_ack
            if r.status_code != 200:
                self.trigger_fault(f"Zero velocity command rejected (HTTP {r.status_code}): {r.text}")
                return False
            res = r.json()
            if not res.get("ok"):
                self.trigger_fault(f"Zero velocity command returned ok=false: {res}")
                return False
            return True
        except Exception as e:
            self.trigger_fault(f"Zero velocity command exception: {e}")
            return False

    def fail_closed_cleanup(self) -> Tuple[bool, bool]:
        """
        Independently attempts zero command and disarm, returning confirmation for each.
        Requires fresh drive status for disarm confirmation.
        """
        zero_confirmed = False
        disarm_confirmed = False

        # 1. Independent Zero Dispatch (check every response)
        headers_bridge = {"X-Rover-Bridge-Token": self.cmd_token} if self.cmd_token else {}
        zero_payload = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            "source": "CALIBRATION_TEST"
        }
        zero_attempts_ok = 0
        for i in range(3):
            try:
                r = self.session.post(f"{self.bridge_url}/api/cmd_vel", json=zero_payload, headers=headers_bridge, timeout=CMD_TIMEOUT_S)
                if r.status_code == 200 and r.json().get("ok"):
                    zero_attempts_ok += 1
                else:
                    print(f"[CLEANUP WARNING] Zero command attempt {i+1}/3 failed: HTTP {r.status_code}")
            except Exception as e:
                print(f"[CLEANUP WARNING] Zero command attempt {i+1}/3 exception: {e}")
            time.sleep(0.02)
        zero_confirmed = (zero_attempts_ok > 0)

        # 2. Independent Disarm Dispatch
        headers_op = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        try:
            r = self.session.post(f"{self.cockpit_url}/api/drive/disarm", headers=headers_op, timeout=1.0)
            if r.status_code == 200 and r.json().get("ok"):
                pass
            else:
                print(f"[CLEANUP WARNING] Disarm post failed: HTTP {r.status_code}")
        except Exception as e:
            print(f"[CLEANUP WARNING] Disarm post exception: {e}")

        # 3. Verify Mode 0 confirmed in fresh, advancing telemetry (bounded polling)
        disarm_deadline = time.monotonic() + 0.60
        last_disarm_err = None
        while time.monotonic() < disarm_deadline:
            try:
                status = self.fetch_drive_status() # Strictly validates schema, required keys, freshness
                if status.get("armed") is False and status.get("mode") == 0:
                    disarm_confirmed = True
                    break
                else:
                    last_disarm_err = f"armed={status.get('armed')}, mode={status.get('mode')}"
            except Exception as e:
                last_disarm_err = str(e)
            time.sleep(0.03)

        if not disarm_confirmed:
            print(f"[CLEANUP WARNING] Drive status disarm verification failed within deadline: {last_disarm_err}")

        if not zero_confirmed or not disarm_confirmed:
            print(f"[SAFETY WARNING] Cleanup status: zero_confirmed={zero_confirmed}, disarm_confirmed={disarm_confirmed}")

        return zero_confirmed, disarm_confirmed

    def run_profile_step(
        self,
        name: str,
        segments: List[Tuple[float, float, float]], # [(vx, wz, duration_s), ...]
        max_dist_m: float,
        max_yaw_deg: float,
        max_speed_mps: float = 0.0
    ) -> Dict[str, Any]:
        """
        Executes one characterization profile step, tracking per-segment performance,
        breakaway latency, stalls, wrong-direction motion, and stop confirmation.
        """
        if self.fault_latched:
            raise RuntimeError(f"Step '{name}' aborted: System in latched fault state.")

        is_diagnostic = ("Diagnostic" in name) or name.startswith("Diagnostic_")
        total_duration = sum(dur for _, _, dur in segments)
        allowed_duration_s = MAX_DIAGNOSTIC_DURATION_S if is_diagnostic else MAX_STEP_DURATION_S
        if total_duration > allowed_duration_s:
            raise RuntimeError(
                f"Step '{name}' total duration {total_duration:.2f} s exceeds limit of {allowed_duration_s:.2f} s"
            )

        print(f"\n==================================================================")
        print(f" Executing Case: {name}")
        print(f" Segments: {segments}")
        print(f" Bounds: max_dist={max_dist_m:.3f} m, max_yaw={max_yaw_deg:.1f} deg")
        print(f"==================================================================")

        # Preflight zero handshake and initial telemetry
        if not self.zero_handshake():
            self.trigger_fault("Zero handshake failed during pre-step check.")
            raise RuntimeError("Zero handshake failed.")

        self.yaw_tracker.reset()
        t0 = self.fetch_all_telemetry()
        start_ticks = {k: t0["encoders"]["encoders"][k] for k in ["m1", "m2", "m3", "m4"]}
        prev_ticks = dict(start_ticks)
        accum_abs_wheel_ticks = {k: 0.0 for k in ["m1", "m2", "m3", "m4"]}

        start_yaw_deg = t0["continuous_yaw_deg"]
        accum_abs_yaw_deg = 0.0

        report: Dict[str, Any] = {
            "name": name,
            "segments": segments,
            "geometry": {
                "wheel_diameter_m": self.wheel_diameter_m,
                "wheel_radius_m": self.wheel_radius_m,
                "effective_track_width_m": self.effective_track_width_m,
                "ticks_per_rev": self.ticks_per_rev,
                "meters_per_tick": self.meters_per_tick
            },
            "samples": [],
            "breakaway": {},
            "segment_metrics": [],
            "stopping": {},
            "status": "IN_PROGRESS"
        }
        self.active_report = report

        # Arm
        self.arm()

        step_start_time = time.monotonic()
        overall_deadline = step_start_time + total_duration + STOP_SETTLE_TIMEOUT_S + 1.0

        seg_idx = 0
        seg_start_time = step_start_time
        active_vx, active_wz, active_dur = segments[0]
        seg_cutoff_time = seg_start_time + active_dur
        command_active = True
        cutoff_time = None

        first_cmd_dispatch_mono: Optional[float] = None
        first_cmd_ack_mono: Optional[float] = None
        last_cmd_dispatch_mono: Optional[float] = None
        last_cmd_ack_mono: Optional[float] = None
        first_zero_dispatch_mono: Optional[float] = None
        first_zero_ack_mono: Optional[float] = None

        def record_cmd_sent(ok: bool):
            nonlocal first_cmd_dispatch_mono, first_cmd_ack_mono, last_cmd_dispatch_mono, last_cmd_ack_mono
            if ok:
                if first_cmd_dispatch_mono is None:
                    first_cmd_dispatch_mono = self.last_cmd_dispatch_mono
                    first_cmd_ack_mono = self.last_cmd_ack_mono
                last_cmd_dispatch_mono = self.last_cmd_dispatch_mono
                last_cmd_ack_mono = self.last_cmd_ack_mono
            else:
                if first_cmd_dispatch_mono is None:
                    first_cmd_dispatch_mono = self.last_cmd_dispatch_mono

        def record_zero_sent(ok: bool):
            nonlocal first_zero_dispatch_mono, first_zero_ack_mono
            if first_zero_dispatch_mono is None:
                first_zero_dispatch_mono = self.last_zero_dispatch_mono
            if ok and first_zero_ack_mono is None:
                first_zero_ack_mono = self.last_zero_ack_mono

        stopped_start_time: Optional[float] = None
        stop_confirm_initial_pid_seq: Optional[int] = None
        stop_confirm_initial_imu_seq: Optional[int] = None
        first_tick_time = None
        breakout_pwm = {}

        period = 1.0 / COMMAND_RATE_HZ
        prev_sample_receipt_mono = step_start_time

        def check_deadlines_and_segment_transition():
            nonlocal command_active, cutoff_time, seg_idx, active_vx, active_wz, active_dur, seg_start_time, seg_cutoff_time
            now_t = time.monotonic()
            if now_t > overall_deadline:
                self.trigger_fault("Hard step overall deadline exceeded.")
                report["status"] = "DEADLINE_EXCEEDED"
                return

            if command_active and now_t >= seg_cutoff_time:
                seg_idx += 1
                if seg_idx < len(segments):
                    active_vx, active_wz, active_dur = segments[seg_idx]
                    seg_start_time = now_t
                    seg_cutoff_time = seg_start_time + active_dur
                    cmd_ok = self.send_velocity(active_vx, active_wz)
                    record_cmd_sent(cmd_ok)
                    if not cmd_ok:
                        report["status"] = "COMMAND_REJECTED"
                        return
                else:
                    command_active = False
                    cutoff_time = now_t
                    zero_ok = self.send_zero_velocity()
                    record_zero_sent(zero_ok)
                    if not zero_ok:
                        report["status"] = "COMMAND_REJECTED"
                        return
                    active_vx, active_wz = 0.0, 0.0

        try:
            while True:
                now_loop = time.monotonic()
                check_deadlines_and_segment_transition()
                if self.fault_latched or report.get("status") in ("COMMAND_REJECTED", "DEADLINE_EXCEEDED"):
                    break

                # Send command if active, or zero velocity if stopping
                if command_active:
                    cmd_ok = self.send_velocity(active_vx, active_wz)
                    record_cmd_sent(cmd_ok)
                    if not cmd_ok:
                        report["status"] = "COMMAND_REJECTED"
                        break
                else:
                    zero_ok = self.send_zero_velocity()
                    record_zero_sent(zero_ok)
                    if not zero_ok:
                        report["status"] = "COMMAND_REJECTED"
                        break

                # Fetch all telemetry, checking deadlines between each blocking call
                try:
                    telem = self.fetch_all_telemetry(deadline_checker=check_deadlines_and_segment_transition)
                except Exception as e:
                    if self.fault_latched:
                        break
                    raise
                # Exact receipt monotonic time
                fetch_receipt_mono = telem["fetch_mono"]

                drv = telem["drive"]
                pid = telem["pid"]["telemetry"]
                enc = telem["encoders"]["encoders"]
                imu = telem["imu"]

                # Elapsed time and sample dt based on actual receipt time
                elapsed = fetch_receipt_mono - step_start_time
                sample_dt = fetch_receipt_mono - prev_sample_receipt_mono
                prev_sample_receipt_mono = fetch_receipt_mono

                # Check safety fault conditions
                if drv.get("spinFailedExhausted"):
                    self.trigger_fault("Drivetrain recovery exhausted (spinFailedExhausted=true).")
                    report["status"] = "RECOVERY_EXHAUSTED"
                    break

                if drv.get("armed") is not True or drv.get("mode") != 3:
                    if command_active:
                        self.trigger_fault(f"Unexpected disarm: armed={drv.get('armed')}, mode={drv.get('mode')}")
                        report["status"] = "UNEXPECTED_DISARM"
                        break

                # Update displacements & accumulated absolute motion
                delta_ticks = {}
                for m in ["m1", "m2", "m3", "m4"]:
                    delta_tick = enc[m] - prev_ticks[m]
                    delta_ticks[m] = delta_tick
                    accum_abs_wheel_ticks[m] += abs(delta_tick)
                    prev_ticks[m] = enc[m]

                accum_abs_wheel_travel_m = {m: accum_abs_wheel_ticks[m] * self.meters_per_tick for m in ["m1", "m2", "m3", "m4"]}
                max_accum_travel_m = max(accum_abs_wheel_travel_m.values())

                net_ticks = {m: enc[m] - start_ticks[m] for m in ["m1", "m2", "m3", "m4"]}
                avg_net_ticks = sum(net_ticks.values()) / 4.0
                net_disp_m = avg_net_ticks * self.meters_per_tick

                accum_abs_yaw_deg += abs(telem["delta_yaw_deg"])
                net_yaw_deg = abs(telem["continuous_yaw_deg"] - start_yaw_deg)

                m_radps_list = [pid[m].get("measuredRadps") for m in ["m1", "m2", "m3", "m4"]]
                if all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in m_radps_list):
                    signed_chassis_mps = (sum(m_radps_list) / 4.0) * self.wheel_radius_m
                    mean_abs_wheel_speed_mps = (sum(abs(v) for v in m_radps_list) / 4.0) * self.wheel_radius_m
                else:
                    signed_chassis_mps = 0.0
                    mean_abs_wheel_speed_mps = 0.0

                # Breakaway detection (>= 2 ticks on any wheel)
                if first_tick_time is None and any(abs(net_ticks[m]) >= 2 for m in ["m1", "m2", "m3", "m4"]):
                    first_tick_time = elapsed
                    breakout_pwm = {m: pid[m].get("finalPwm") for m in ["m1", "m2", "m3", "m4"]}

                # Record detailed sample
                sample = {
                    "t_rel": round(elapsed, 4),
                    "dt_s": round(sample_dt, 4),
                    "receipt_mono": fetch_receipt_mono,
                    "seg_idx": seg_idx if command_active else -1,
                    "cmd": {"vx": active_vx, "wz": active_wz},
                    "disp_net_m": round(net_disp_m, 4),
                    "disp_accum_m": round(max_accum_travel_m, 4),
                    "yaw_net_deg": round(net_yaw_deg, 2),
                    "yaw_accum_deg": round(accum_abs_yaw_deg, 2),
                    "gyro_z": round(imu["gyro_z"], 4),
                    "ticks": dict(net_ticks),
                    "delta_ticks": dict(delta_ticks),
                    "raw_ticks": dict(enc),
                    "measured_radps": {m: pid[m].get("measuredRadps") for m in ["m1", "m2", "m3", "m4"]},
                    "measured_mps": {m: round(pid[m]["measuredRadps"] * self.wheel_radius_m, 4) if pid[m].get("measuredRadps") is not None else None for m in ["m1", "m2", "m3", "m4"]},
                    "signed_chassis_mps": round(signed_chassis_mps, 4),
                    "mean_abs_wheel_speed_mps": round(mean_abs_wheel_speed_mps, 4),
                    "measured_chassis_mps": round(abs(signed_chassis_mps), 4),
                    "target_radps": {m: pid[m].get("targetRadps") for m in ["m1", "m2", "m3", "m4"]},
                    "pwm": {m: pid[m].get("finalPwm") for m in ["m1", "m2", "m3", "m4"]},
                    "pid_terms": {
                        m: {
                            "pTerm": pid[m].get("pTerm"),
                            "iTerm": pid[m].get("iTerm"),
                            "dTerm": pid[m].get("dTerm"),
                            "feedforward": pid[m].get("feedforward"),
                            "basePwm": pid[m].get("basePwm"),
                            "spinSyncTrim": pid[m].get("spinSyncTrim"),
                            "recoveryAssist": pid[m].get("recoveryAssist", 0.0),
                            "candidateEnabled": pid[m].get("candidateEnabled", False),
                            "finalPwm": pid[m].get("finalPwm")
                        } for m in ["m1", "m2", "m3", "m4"]
                    },
                    "stiction": {m: pid[m].get("stictionState", "") for m in ["m1", "m2", "m3", "m4"]},
                    "recovery": {
                        "noMotionBudgetMs": drv.get("noMotionBudgetMs"),
                        "spinFailedExhausted": drv.get("spinFailedExhausted"),
                        "spinRecoveryCode": drv.get("spinRecoveryCode"),
                        "spinRecoveryState": drv.get("spinRecoveryState"),
                        "chassisRebreakoutConsumed": drv.get("chassisRebreakoutConsumed"),
                        "recoveryAssist": {m: pid[m].get("recoveryAssist", 0.0) for m in ["m1", "m2", "m3", "m4"]},
                        "candidateEnabled": {m: pid[m].get("candidateEnabled", False) for m in ["m1", "m2", "m3", "m4"]}
                    },
                    "timestamps": {
                        "receipt_mono": fetch_receipt_mono,
                        "drive_rx": drv.get("receivedAt"),
                        "pid": telem["pid"].get("timestamp"),
                        "pid_data_age_ms": telem["pid"].get("dataAgeMs"),
                        "encoders": telem["encoders"].get("timestamp"),
                        "imu": imu.get("timestamp")
                    },
                    "sequences": {
                        "drive": drv.get("seq"),
                        "pid": telem["pid"].get("sequence"),
                        "encoders": telem["encoders"].get("sequence"),
                        "imu": imu.get("sequence")
                    }
                }
                report["samples"].append(sample)

                # Motion bounds: Use mean absolute wheel speed guard, accumulated absolute motion, and absolute yaw
                if max_speed_mps > 0.0 and mean_abs_wheel_speed_mps >= max_speed_mps:
                    print(f"[LIMIT] Mean absolute wheel speed bound reached ({mean_abs_wheel_speed_mps:.3f} m/s >= {max_speed_mps:.3f} m/s)")
                    if command_active:
                        command_active = False
                        cutoff_time = fetch_receipt_mono
                        report["abort_reason"] = f"SPEED_BOUND_EXCEEDED: {mean_abs_wheel_speed_mps:.3f} m/s >= {max_speed_mps:.3f} m/s"
                        zero_ok = self.send_zero_velocity()
                        record_zero_sent(zero_ok)
                        if not zero_ok:
                            report["status"] = "COMMAND_REJECTED"
                            break
                        active_vx, active_wz = 0.0, 0.0

                if max_accum_travel_m >= max_dist_m:
                    print(f"[LIMIT] Accumulated wheel travel bound reached ({max_accum_travel_m:.3f} m >= {max_dist_m:.3f} m)")
                    if command_active:
                        command_active = False
                        cutoff_time = fetch_receipt_mono
                        if not report.get("abort_reason"):
                            report["abort_reason"] = f"TRAVEL_BOUND_EXCEEDED: {max_accum_travel_m:.3f} m >= {max_dist_m:.3f} m"
                        zero_ok = self.send_zero_velocity()
                        record_zero_sent(zero_ok)
                        if not zero_ok:
                            report["status"] = "COMMAND_REJECTED"
                            break
                        active_vx, active_wz = 0.0, 0.0

                if accum_abs_yaw_deg >= max_yaw_deg:
                    print(f"[LIMIT] Accumulated yaw rotation bound reached ({accum_abs_yaw_deg:.1f} deg >= {max_yaw_deg:.1f} deg)")
                    if command_active:
                        command_active = False
                        cutoff_time = fetch_receipt_mono
                        if not report.get("abort_reason"):
                            report["abort_reason"] = f"YAW_BOUND_EXCEEDED: {accum_abs_yaw_deg:.1f} deg >= {max_yaw_deg:.1f} deg"
                        zero_ok = self.send_zero_velocity()
                        record_zero_sent(zero_ok)
                        if not zero_ok:
                            report["status"] = "COMMAND_REJECTED"
                            break
                        active_vx, active_wz = 0.0, 0.0

                # Stopping verification: Requires 200 ms uninterrupted near-zero speeds and gyro,
                # with advancing post-zero PID and IMU sequence evidence
                if not command_active:
                    if cutoff_time is None:
                        cutoff_time = fetch_receipt_mono

                    stop_elapsed = fetch_receipt_mono - cutoff_time
                    speeds = [abs(sample["measured_radps"][m]) for m in ["m1", "m2", "m3", "m4"]]
                    gyro_abs = abs(imu["gyro_z"])

                    is_near_zero = all(s < 0.05 for s in speeds) and (gyro_abs < 0.03)
                    is_post_zero = (fetch_receipt_mono >= cutoff_time)

                    pid_seq = telem["pid"]["sequence"]
                    imu_seq = telem["imu"]["sequence"]

                    if is_near_zero and is_post_zero:
                        if stopped_start_time is None:
                            stopped_start_time = fetch_receipt_mono
                            stop_confirm_initial_pid_seq = pid_seq
                            stop_confirm_initial_imu_seq = imu_seq
                        else:
                            confirm_duration = fetch_receipt_mono - stopped_start_time
                            pid_advanced = (pid_seq > stop_confirm_initial_pid_seq)
                            imu_advanced = (imu_seq > stop_confirm_initial_imu_seq)

                            if confirm_duration >= STOP_CONFIRM_WINDOW_S:
                                if pid_advanced and imu_advanced:
                                    if not self.fault_latched and report.get("status") == "IN_PROGRESS":
                                        report["status"] = "SUCCESS"
                                    break
                                else:
                                    # 200ms elapsed but advancing evidence not yet confirmed
                                    pass
                    else:
                        # Motion resumed, speed/gyro above threshold, or before cutoff: reset
                        if stopped_start_time is not None:
                            print(f"[STOP QUALIFICATION] Motion resumed (speeds={speeds}, gyro={gyro_abs}) or unconfirmed; resetting confirmation timer.")
                        stopped_start_time = None
                        stop_confirm_initial_pid_seq = None
                        stop_confirm_initial_imu_seq = None

                    if stop_elapsed >= STOP_SETTLE_TIMEOUT_S:
                        report["status"] = "UNCONFIRMED_STOP"
                        self.trigger_fault("Failed to confirm 200 ms of uninterrupted stopped motion with advancing PID/IMU evidence within settle timeout.")
                        break

                loop_elapsed = time.monotonic() - now_loop
                time.sleep(max(0.005, period - loop_elapsed))

        finally:
            zero_ok, disarm_ok = self.fail_closed_cleanup()

            # Active command duration calculation (strictly from dispatch, never substituting cleanup duration):
            if first_zero_dispatch_mono is not None and first_cmd_dispatch_mono is not None:
                cmd_duration_s = max(0.0, first_zero_dispatch_mono - first_cmd_dispatch_mono)
            elif last_cmd_ack_mono is not None and first_cmd_dispatch_mono is not None:
                cmd_duration_s = max(0.0, last_cmd_ack_mono - first_cmd_dispatch_mono)
            elif last_cmd_dispatch_mono is not None and first_cmd_dispatch_mono is not None:
                cmd_duration_s = max(0.0, last_cmd_dispatch_mono - first_cmd_dispatch_mono)
            else:
                cmd_duration_s = 0.0

            report["command_boundaries"] = {
                "first_cmd_dispatch_mono": round(first_cmd_dispatch_mono, 4) if first_cmd_dispatch_mono else None,
                "first_cmd_ack_mono": round(first_cmd_ack_mono, 4) if first_cmd_ack_mono else None,
                "last_cmd_dispatch_mono": round(last_cmd_dispatch_mono, 4) if last_cmd_dispatch_mono else None,
                "last_cmd_ack_mono": round(last_cmd_ack_mono, 4) if last_cmd_ack_mono else None,
                "first_zero_dispatch_mono": round(first_zero_dispatch_mono, 4) if first_zero_dispatch_mono else None,
                "first_zero_ack_mono": round(first_zero_ack_mono, 4) if first_zero_ack_mono else None,
                "telemetry_cutoff_mono": round(cutoff_time, 4) if cutoff_time else None,
                "command_duration_s": round(cmd_duration_s, 4),
                "command_start_mono": round(first_cmd_dispatch_mono or step_start_time, 4),
                "command_cutoff_mono": round(cutoff_time, 4) if cutoff_time else None,
                "first_zero_command_mono": round(first_zero_dispatch_mono, 4) if first_zero_dispatch_mono else None
            }
            report["cleanup_verification"] = {
                "zero_command_confirmed": zero_ok,
                "disarm_mode0_confirmed": disarm_ok
            }
            if not zero_ok or not disarm_ok:
                report["status"] = "CLEANUP_FAILED"
                self.trigger_fault("Cleanup failed to confirm zero velocity and/or Mode 0 disarm.")
            elif self.fault_latched:
                if report.get("status") in ("IN_PROGRESS", "SUCCESS"):
                    report["status"] = "FAULT_LATCHED"

        report["breakaway"] = {
            "first_tick_time_s": first_tick_time,
            "breakout_pwm": breakout_pwm
        }

        # Calculate per-segment metrics (using actual sample timestamps)
        for s_idx, (s_vx, s_wz, s_dur) in enumerate(segments):
            seg_samples = [s for s in report["samples"] if s["seg_idx"] == s_idx]
            if not seg_samples:
                continue

            # Exclude uninitialized startup samples and deduplicate repeated 10Hz PID packets in 20Hz loop:
            valid_pid_samples = []
            seen_pid_seqs = set()
            for s in seg_samples:
                pid_seq = s.get("sequences", {}).get("pid")
                if pid_seq is not None and pid_seq > 0:
                    if pid_seq not in seen_pid_seqs:
                        seen_pid_seqs.add(pid_seq)
                        valid_pid_samples.append(s)

            eval_samples = valid_pid_samples if valid_pid_samples else seg_samples

            seg_metric: Dict[str, Any] = {
                "segment_index": s_idx,
                "command": {"vx": s_vx, "wz": s_wz, "duration_s": s_dur},
                "sample_count": len(seg_samples),
                "unique_pid_packets": len(eval_samples),
                "wheels": {}
            }

            for m in ["m1", "m2", "m3", "m4"]:
                meass = [s["measured_radps"][m] for s in eval_samples]
                tgts = [s["target_radps"][m] for s in eval_samples]
                pwms = [s["pwm"][m] for s in eval_samples]

                mean_spd = sum(meass) / len(meass) if meass else 0.0
                mean_err = sum(t - m for t, m in zip(tgts, meass)) / len(meass) if meass else 0.0
                var_spd = sum((x - mean_spd) ** 2 for x in meass) / len(meass) if meass else 0.0
                std_spd = math.sqrt(var_spd)

                # Stall tracking based on sample timestamps
                stall_start_t = None
                longest_stall_s = 0.0
                stall_intervals_count = 0
                for s in seg_samples:
                    v_m = s["measured_radps"][m]
                    v_t = s["target_radps"][m]
                    t_curr = s["t_rel"]
                    is_stalled = (abs(v_m) < 0.05) and (abs(v_t) >= 0.05)
                    if is_stalled:
                        if stall_start_t is None:
                            stall_start_t = t_curr
                            stall_intervals_count += 1
                        dur = t_curr - stall_start_t
                        if dur > longest_stall_s:
                            longest_stall_s = dur
                    else:
                        stall_start_t = None

                longest_stall_ms = round(longest_stall_s * 1000.0, 1)

                wrong_dir_count = 0
                for t_val, m_val in zip(tgts, meass):
                    if (t_val > 0.05 and m_val < -0.05) or (t_val < -0.05 and m_val > 0.05):
                        wrong_dir_count += 1

                nonzero_tgts = [t for t in tgts if abs(t) > 0.01]
                effective_tgt = (sum(nonzero_tgts) / len(nonzero_tgts)) if nonzero_tgts else (tgts[0] if tgts else 0.0)

                seg_metric["wheels"][m] = {
                    "target_radps": round(effective_tgt, 3),
                    "mean_measured_radps": round(mean_spd, 3),
                    "mean_error_radps": round(mean_err, 3),
                    "std_dev_radps": round(std_spd, 3),
                    "mean_pwm": round(sum(pwms) / len(pwms), 1) if pwms else 0.0,
                    "stall_intervals_count": stall_intervals_count,
                    "longest_stall_ms": longest_stall_ms,
                    "wrong_direction_samples": wrong_dir_count
                }
            report["segment_metrics"].append(seg_metric)

        # Stopping metrics
        if cutoff_time:
            stopping_samples = [s for s in report["samples"] if s["seg_idx"] == -1]
            if stopping_samples:
                disp_cut = stopping_samples[0]["disp_net_m"]
                disp_end = stopping_samples[-1]["disp_net_m"]
                yaw_cut = stopping_samples[0]["yaw_net_deg"]
                yaw_end = stopping_samples[-1]["yaw_net_deg"]
                report["stopping"] = {
                    "overrun_distance_mm": round((disp_end - disp_cut) * 1000.0, 1),
                    "overrun_yaw_deg": round(yaw_end - yaw_cut, 2),
                    "settle_time_s": round(stopping_samples[-1]["t_rel"] - stopping_samples[0]["t_rel"], 3)
                }

        # -------------------------------------------------------------
        # Motion Tracking vs Lifecycle Completion Evaluation
        # -------------------------------------------------------------
        lifecycle_status = "COMPLETED" if report.get("status") in ("SUCCESS", "IN_PROGRESS") else report.get("status", "UNKNOWN")
        report["lifecycle_completion"] = {
            "lifecycle_status": lifecycle_status,
            "stop_confirmed": (stopped_start_time is not None),
            "zero_confirmed": report.get("cleanup_verification", {}).get("zero_command_confirmed", False),
            "disarm_confirmed": report.get("cleanup_verification", {}).get("disarm_mode0_confirmed", False),
            "settle_time_s": report.get("stopping", {}).get("settle_time_s", 0.0)
        }

        # Motion tracking evaluation:
        self.evaluate_motion_tracking(name, segments, report)

        # Communication evaluation:
        report["communication_status"] = {
            "total_telemetry_samples": len(report["samples"]),
            "communication_healthy": (len(report["samples"]) > 0 and not self.fault_latched)
        }

        # Recovery evaluation:
        last_rec = report["samples"][-1]["recovery"] if report["samples"] else {}
        cand_wheels = last_rec.get("candidateEnabled", {})
        cand_all_enabled = bool(cand_wheels and all(cand_wheels.get(m, False) for m in ["m1", "m2", "m3", "m4"]))
        report["recovery_status"] = {
            "final_no_motion_budget_ms": last_rec.get("noMotionBudgetMs"),
            "spin_failed_exhausted": last_rec.get("spinFailedExhausted"),
            "chassis_rebreakout_consumed": last_rec.get("chassisRebreakoutConsumed"),
            "recovery_exhausted": report.get("status") == "RECOVERY_EXHAUSTED",
            "candidate_recovery_assist_enabled": cand_all_enabled,
            "candidate_recovery_assist_wheels": cand_wheels,
            "final_recovery_assist": last_rec.get("recoveryAssist", {})
        }

        print(f"\n--- Step Result Summary ---")
        print(f"  Lifecycle Completion:  {report['lifecycle_completion']['lifecycle_status']}")
        print(f"  Motion Tracking:       {report['motion_tracking']['tracking_status']}")
        print(f"    - Nominal Requested: {report['motion_tracking']['nominal_requested_distance_m']*1000.0:.1f} mm")
        print(f"    - Script Cmd Profile:{report['motion_tracking']['script_command_profile_distance_m']*1000.0:.1f} mm")
        print(f"    - FW Wheel Target:   {report['motion_tracking']['firmware_wheel_target_distance_m']*1000.0:.1f} mm")
        print(f"    - Last Active Sample:{report['motion_tracking']['last_active_sample_distance_m']*1000.0:.1f} mm")
        print(f"    - Final Stopped Odom:{report['motion_tracking']['final_stopped_encoder_distance_m']*1000.0:.1f} mm ({report['motion_tracking']['distance_ratio_pct']}%)")
        print(f"    - Post-Cmd Overrun:  {report['motion_tracking']['post_command_overrun_distance_m']*1000.0:.1f} mm")
        print(f"    - Signed Net Disp:   {report['motion_tracking']['final_displacement_signed_m']*1000.0:.1f} mm")
        print(f"  Cleanup Verification:  zero={report['cleanup_verification']['zero_command_confirmed']}, "
              f"disarm={report['cleanup_verification']['disarm_mode0_confirmed']}")
        if "diagnostic_evaluation" in report:
            diag_info = report["diagnostic_evaluation"]
            print(f"  Diagnostic Eval:       {diag_info.get('equilibrium_status')}")
            print(f"    - Valid Duration:    {diag_info.get('valid_duration_s', 0.0):.2f} s")
            print(f"    - Diagnostic Reason: {diag_info.get('reason')}")
            if diag_info.get("assessment_window"):
                w_info = diag_info["assessment_window"]
                print(f"    - Assessment Window: [{w_info['start_t_rel']:.2f}s - {w_info['end_t_rel']:.2f}s] ({w_info['duration_s']:.2f}s, {w_info['sample_count']} samples, max gap {w_info['max_interpacket_gap_s']*1000:.1f}ms)")
            if diag_info.get("transient_metrics") and diag_info["transient_metrics"].get("sample_count", 0) > 0:
                tr = diag_info["transient_metrics"]
                print(f"    - Transient Window:  [{tr['start_t_rel']:.2f}s - {tr['end_t_rel']:.2f}s] ({tr['duration_s']:.2f}s, {tr['sample_count']} samples)")
        print(f"  Overall Step Result:   {report['status']}\n")
        return report

    def assess_diagnostic_tracking(self, report: Dict[str, Any]) -> Dict[str, Any]:
        """
        Assesses sustained reverse tracking and equilibrium for diagnostic profiles (Case 14).
        Invariants:
        1. Cleanup failure unconditionally takes precedence over equilibrium classification.
        2. Strictly exclude invalid, unidentified (missing or <= 0 sequence), or stale PID packets.
           Missing pid_data_age_ms must NOT default to fresh. Missing per-wheel iTerm must NOT default to zero.
           Require finite, valid timestamps, age, targets, measured speeds, PWM and integral terms for every assessed wheel.
           Reject non-increasing timestamps and report why samples were excluded.
        3. Within the last contiguous valid block (no inter-packet gaps > 250ms), select a trailing window
           ending at the last active packet and spanning at least 1.0 second, including the preceding boundary packet as needed.
        4. Report earlier valid samples separately as transient data. Do not average the entire post-startup run.
        5. Require per-wheel target agreement, speed/PWM/integral trends, AND variation / RMS-error criteria
           so oscillation cannot pass merely because its average and regression slope are small.
        """
        cleanup_verif = report.get("cleanup_verification", {})
        cleanup_failed = (
            report.get("status") == "CLEANUP_FAILED" or
            cleanup_verif.get("zero_command_confirmed") is False or
            cleanup_verif.get("disarm_mode0_confirmed") is False
        )
        if cleanup_failed:
            return {
                "tracking_assessed": False,
                "assessment_window": None,
                "valid_duration_s": 0.0,
                "window_sample_count": 0,
                "abort_reason": report.get("abort_reason"),
                "equilibrium_status": "CLEANUP_FAILED",
                "reason": "Cleanup failure took precedence over equilibrium classification (zero or Mode 0 unconfirmed).",
                "sample_exclusions": {},
                "transient_metrics": {},
                "wheels": {}
            }

        samples = report.get("samples", [])
        active_samples = [s for s in samples if s.get("seg_idx", -1) != -1]

        # Strictly validate active samples:
        # - PID sequence: must be integer > 0 (missing, None, <= 0 or bool excluded)
        # - Strictly increasing sequence (deduplicates repeated packets and rejects sequence regression)
        # - Timestamps: t_rel must be finite numeric and strictly increasing (reject non-increasing timestamps)
        # - Freshness: pid_data_age_ms must be finite numeric >= 0.0 and <= TELEMETRY_MAX_AGE_MS (missing age must NOT default to fresh)
        # - Wheel telemetry: every assessed wheel (m1..m4) must have finite numeric targets, measured speeds, PWM, and iTerm (missing iTerm must NOT default to zero)
        valid_pid_samples = []
        last_pid_seq = None
        last_t_rel = -1.0

        sample_exclusions: Dict[str, int] = {
            "missing_or_invalid_sequence": 0,
            "sequence_not_monotonic": 0,
            "missing_or_invalid_timestamp": 0,
            "timestamp_not_increasing": 0,
            "missing_or_stale_pid_data_age": 0,
            "missing_or_invalid_wheel_telemetry": 0
        }
        exclusion_details: List[str] = []

        for idx, s in enumerate(active_samples):
            # 1. Sequence validation
            seqs = s.get("sequences")
            p_seq = seqs.get("pid") if isinstance(seqs, dict) else None
            if p_seq is None or not isinstance(p_seq, int) or isinstance(p_seq, bool) or p_seq <= 0:
                sample_exclusions["missing_or_invalid_sequence"] += 1
                if len(exclusion_details) < 10:
                    exclusion_details.append(f"Sample {idx}: invalid sequence {p_seq}")
                continue
            if last_pid_seq is not None and p_seq <= last_pid_seq:
                sample_exclusions["sequence_not_monotonic"] += 1
                if len(exclusion_details) < 10:
                    exclusion_details.append(f"Sample {idx}: sequence {p_seq} <= last {last_pid_seq}")
                continue

            # 2. Timestamp validation
            t_rel = s.get("t_rel")
            if t_rel is None or not isinstance(t_rel, (int, float)) or isinstance(t_rel, bool) or not math.isfinite(t_rel):
                sample_exclusions["missing_or_invalid_timestamp"] += 1
                if len(exclusion_details) < 10:
                    exclusion_details.append(f"Sample {idx}: invalid t_rel {t_rel}")
                continue
            if t_rel <= last_t_rel:
                sample_exclusions["timestamp_not_increasing"] += 1
                if len(exclusion_details) < 10:
                    exclusion_details.append(f"Sample {idx}: non-increasing t_rel {t_rel} <= last {last_t_rel}")
                continue

            # 3. Telemetry age validation (missing pid_data_age_ms must NOT default to fresh)
            ts = s.get("timestamps")
            if not isinstance(ts, dict):
                sample_exclusions["missing_or_stale_pid_data_age"] += 1
                if len(exclusion_details) < 10:
                    exclusion_details.append(f"Sample {idx}: missing timestamps dict")
                continue
            age_ms = ts.get("pid_data_age_ms")
            if (age_ms is None or not isinstance(age_ms, (int, float)) or isinstance(age_ms, bool)
                    or not math.isfinite(age_ms) or age_ms < 0.0 or age_ms > TELEMETRY_MAX_AGE_MS):
                sample_exclusions["missing_or_stale_pid_data_age"] += 1
                if len(exclusion_details) < 10:
                    exclusion_details.append(f"Sample {idx}: missing/invalid/stale pid_data_age_ms={age_ms}")
                continue

            # 4. Wheel telemetry validation for all four wheels
            meas_dict = s.get("measured_radps")
            tgt_dict = s.get("target_radps")
            pwm_dict = s.get("pwm")
            pid_terms_dict = s.get("pid_terms")

            wheel_err = False
            for m in ["m1", "m2", "m3", "m4"]:
                v = meas_dict.get(m) if isinstance(meas_dict, dict) else None
                if v is None or not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v):
                    wheel_err = True
                    break

                vt = tgt_dict.get(m) if isinstance(tgt_dict, dict) else None
                if vt is None or not isinstance(vt, (int, float)) or isinstance(vt, bool) or not math.isfinite(vt):
                    wheel_err = True
                    break

                p = pwm_dict.get(m) if isinstance(pwm_dict, dict) else None
                if p is None or not isinstance(p, (int, float)) or isinstance(p, bool) or not math.isfinite(p):
                    wheel_err = True
                    break

                w_terms = pid_terms_dict.get(m) if isinstance(pid_terms_dict, dict) else None
                if not isinstance(w_terms, dict):
                    wheel_err = True
                    break
                iterm = w_terms.get("iTerm")
                if iterm is None or not isinstance(iterm, (int, float)) or isinstance(iterm, bool) or not math.isfinite(iterm):
                    wheel_err = True
                    break

            if wheel_err:
                sample_exclusions["missing_or_invalid_wheel_telemetry"] += 1
                if len(exclusion_details) < 10:
                    exclusion_details.append(f"Sample {idx}: incomplete wheel telemetry (missing/non-finite meas/tgt/pwm/iTerm)")
                continue

            # Valid sample accepted
            last_pid_seq = p_seq
            last_t_rel = t_rel
            valid_pid_samples.append(s)

        abort_reason = report.get("abort_reason")
        total_active_samples = len(active_samples)
        total_valid_samples = len(valid_pid_samples)

        if total_valid_samples < 2 or abort_reason is not None:
            if abort_reason:
                reason = f"Diagnostic run aborted before settling: {abort_reason}"
            else:
                reason = (
                    f"Insufficient valid PID samples ({total_valid_samples}/{total_active_samples} valid; "
                    f"excluded: {sample_exclusions})"
                )
            return {
                "tracking_assessed": False,
                "assessment_window": None,
                "valid_duration_s": 0.0,
                "window_sample_count": total_valid_samples,
                "abort_reason": abort_reason,
                "equilibrium_status": "INCONCLUSIVE_EQUILIBRIUM",
                "reason": reason,
                "sample_exclusions": sample_exclusions,
                "transient_metrics": {},
                "wheels": {}
            }

        # Identify the last contiguous valid block ending at the last valid sample (gap <= 0.250s)
        max_allowed_gap_s = TELEMETRY_MAX_AGE_MS / 1000.0  # 0.250 s
        n_valid = len(valid_pid_samples)
        contig_start_idx = n_valid - 1

        for i in range(n_valid - 1, 0, -1):
            gap = valid_pid_samples[i]["t_rel"] - valid_pid_samples[i - 1]["t_rel"]
            if gap > max_allowed_gap_s:
                contig_start_idx = i
                break
            contig_start_idx = i - 1

        contig_block = valid_pid_samples[contig_start_idx:]
        contig_duration_s = contig_block[-1]["t_rel"] - contig_block[0]["t_rel"]

        if contig_duration_s < 1.0 or len(contig_block) < 5:
            reason = (
                f"Insufficient final contiguous valid window duration ({contig_duration_s:.2f} s < 1.00 s; "
                f"gap > {max_allowed_gap_s*1000:.0f}ms or short block; {total_valid_samples}/{total_active_samples} valid)"
            )
            return {
                "tracking_assessed": False,
                "assessment_window": None,
                "valid_duration_s": round(contig_duration_s, 3),
                "window_sample_count": len(contig_block),
                "abort_reason": None,
                "equilibrium_status": "INCONCLUSIVE_EQUILIBRIUM",
                "reason": reason,
                "sample_exclusions": sample_exclusions,
                "transient_metrics": {},
                "wheels": {}
            }

        # Within the last contiguous block, select the trailing window ending at the last active packet
        # and spanning at least 1.0 second, including the preceding boundary packet as needed.
        t_end = contig_block[-1]["t_rel"]
        w_idx = len(contig_block) - 1
        while w_idx > 0 and (t_end - contig_block[w_idx]["t_rel"]) < 1.0:
            w_idx -= 1

        window_samples = contig_block[w_idx:]
        window_dur = window_samples[-1]["t_rel"] - window_samples[0]["t_rel"]

        # Earlier transient includes all valid active samples preceding the trailing window:
        window_start_t = window_samples[0]["t_rel"]
        transient_samples = [s for s in valid_pid_samples if s["t_rel"] < window_start_t]

        transient_metrics: Dict[str, Any] = {
            "duration_s": round(transient_samples[-1]["t_rel"] - transient_samples[0]["t_rel"], 3) if len(transient_samples) >= 2 else 0.0,
            "start_t_rel": round(transient_samples[0]["t_rel"], 3) if transient_samples else 0.0,
            "end_t_rel": round(transient_samples[-1]["t_rel"], 3) if transient_samples else 0.0,
            "sample_count": len(transient_samples),
            "wheels": {}
        }
        if transient_samples:
            for m in ["m1", "m2", "m3", "m4"]:
                t_meass = [s["measured_radps"][m] for s in transient_samples]
                t_tgts = [s["target_radps"][m] for s in transient_samples]
                t_pwms = [s["pwm"][m] for s in transient_samples]
                t_iterms = [s["pid_terms"][m]["iTerm"] for s in transient_samples]
                t_errs = [tgt - meas for tgt, meas in zip(t_tgts, t_meass)]
                transient_metrics["wheels"][m] = {
                    "mean_measured_radps": round(sum(t_meass) / len(t_meass), 3),
                    "mean_target_radps": round(sum(t_tgts) / len(t_tgts), 3),
                    "mean_error_radps": round(sum(t_errs) / len(t_errs), 3),
                    "mean_pwm": round(sum(t_pwms) / len(t_pwms), 1),
                    "mean_iterm": round(sum(t_iterms) / len(t_iterms), 2),
                    "min_measured_radps": round(min(t_meass), 3),
                    "max_measured_radps": round(max(t_meass), 3)
                }

        # Compute max inter-packet gap in the assessment window
        max_gap = 0.0
        for i in range(1, len(window_samples)):
            g = window_samples[i]["t_rel"] - window_samples[i - 1]["t_rel"]
            if g > max_gap:
                max_gap = g

        t_vals = [s["t_rel"] for s in window_samples]
        t_mean = sum(t_vals) / len(t_vals)
        t_denom = sum((t - t_mean) ** 2 for t in t_vals)
        if t_denom < 1e-6:
            t_denom = 1e-6

        expected_wheel_radps = -0.015 / self.wheel_radius_m  # approx -0.4615 rad/s
        wheels_eval = {}
        all_settled = True
        unsettled_reasons = []

        for m in ["m1", "m2", "m3", "m4"]:
            meas_vals = [s["measured_radps"][m] for s in window_samples]
            tgt_vals = [s["target_radps"][m] for s in window_samples]
            i_vals = [s["pid_terms"][m]["iTerm"] for s in window_samples]
            pwm_vals = [s["pwm"][m] for s in window_samples]

            n_w = len(meas_vals)
            tgt_mean = sum(tgt_vals) / n_w if tgt_vals else expected_wheel_radps
            meas_mean = sum(meas_vals) / n_w
            i_mean = sum(i_vals) / n_w
            pwm_mean = sum(pwm_vals) / n_w

            speed_slope = sum((t - t_mean) * (v - meas_mean) for t, v in zip(t_vals, meas_vals)) / t_denom
            i_slope = sum((t - t_mean) * (iv - i_mean) for t, iv in zip(t_vals, i_vals)) / t_denom
            pwm_slope = sum((t - t_mean) * (pv - pwm_mean) for t, pv in zip(t_vals, pwm_vals)) / t_denom

            speed_error = abs(meas_mean - tgt_mean)
            target_discrepancy = abs(tgt_mean - expected_wheel_radps)
            target_agrees = target_discrepancy <= 0.050

            # RMS error to target
            rms_error = math.sqrt(sum((v - tgt) ** 2 for v, tgt in zip(meas_vals, tgt_vals)) / n_w)

            # Speed variation / standard deviation and peak-to-peak ripple
            var_spd = sum((v - meas_mean) ** 2 for v in meas_vals) / n_w
            std_spd = math.sqrt(var_spd)
            peak_to_peak = max(meas_vals) - min(meas_vals)

            # Settling criteria:
            # - per-wheel target agreement with requested crawl speed: |tgt_mean - expected| <= 0.050 rad/s
            # - speed error within 0.12 rad/s
            # - speed trend |d_omega/dt| <= 0.15 rad/s^2
            # - integral trend |dI/dt| <= 1.0 PWM/s
            # - total PWM trend |dPWM/dt| <= 1.5 PWM/s
            # - RMS error to target <= 0.15 rad/s
            # - speed variation std_dev <= 0.08 rad/s
            # - peak-to-peak speed ripple <= 0.20 rad/s
            speed_approaching = speed_error <= 0.12
            speed_trend_settled = abs(speed_slope) <= 0.15
            integral_settled = abs(i_slope) <= 1.0
            pwm_settled = abs(pwm_slope) <= 1.5
            variation_settled = (rms_error <= 0.15) and (std_spd <= 0.08) and (peak_to_peak <= 0.20)

            wheel_settled = (
                target_agrees and speed_approaching and speed_trend_settled and
                integral_settled and pwm_settled and variation_settled
            )
            if not wheel_settled:
                all_settled = False
                unsettled_details = []
                if not target_agrees:
                    unsettled_details.append(f"target discrepancy |{tgt_mean:.3f} - ({expected_wheel_radps:.3f})| = {target_discrepancy:.3f} > 0.050 rad/s")
                if not speed_approaching:
                    unsettled_details.append(f"speed error {speed_error:.3f} > 0.12 rad/s")
                if not speed_trend_settled:
                    unsettled_details.append(f"speed slope {speed_slope:+.3f} rad/s^2")
                if not integral_settled:
                    unsettled_details.append(f"integral slope {i_slope:+.3f} PWM/s")
                if not pwm_settled:
                    unsettled_details.append(f"PWM slope {pwm_slope:+.3f} PWM/s")
                if not variation_settled:
                    unsettled_details.append(f"oscillation/variation (rms={rms_error:.3f}, std={std_spd:.3f}, p-p={peak_to_peak:.3f})")
                unsettled_reasons.append(f"Wheel {m}: {', '.join(unsettled_details)}")

            wheels_eval[m] = {
                "expected_target_radps": round(expected_wheel_radps, 3),
                "target_radps": round(tgt_mean, 3),
                "target_agrees": target_agrees,
                "mean_measured_radps": round(meas_mean, 3),
                "speed_error_radps": round(speed_error, 3),
                "speed_slope_radps2": round(speed_slope, 4),
                "integral_slope_pwps": round(i_slope, 4),
                "pwm_slope_pwps": round(pwm_slope, 4),
                "rms_error_radps": round(rms_error, 4),
                "std_dev_radps": round(std_spd, 4),
                "peak_to_peak_radps": round(peak_to_peak, 4),
                "mean_i_term": round(i_mean, 2),
                "mean_pwm": round(pwm_mean, 1),
                "speed_approaching_target": speed_approaching,
                "speed_trend_settled": speed_trend_settled,
                "integral_trend_settled": integral_settled,
                "pwm_settled": pwm_settled,
                "variation_settled": variation_settled,
                "wheel_settled": wheel_settled
            }

        eq_status = "SETTLED_EQUILIBRIUM" if all_settled else "INCONCLUSIVE_EQUILIBRIUM"
        reason = (
            "All wheels reached settled speed and integral equilibrium in final contiguous window."
            if all_settled else f"Unsettled equilibrium: {'; '.join(unsettled_reasons)}"
        )

        return {
            "tracking_assessed": True,
            "assessment_window": {
                "start_t_rel": round(window_samples[0]["t_rel"], 3),
                "end_t_rel": round(window_samples[-1]["t_rel"], 3),
                "duration_s": round(window_dur, 3),
                "sample_count": len(window_samples),
                "max_interpacket_gap_s": round(max_gap, 4)
            },
            "valid_duration_s": round(window_dur, 3),
            "rolling_sample_count": len(window_samples),
            "abort_reason": None,
            "equilibrium_status": eq_status,
            "reason": reason,
            "sample_exclusions": sample_exclusions,
            "transient_metrics": transient_metrics,
            "wheels": wheels_eval
        }

    def evaluate_motion_tracking(self, name: str, segments: List[Tuple[float, float, float]], report: Dict[str, Any]) -> Dict[str, Any]:
        """Evaluate nominal vs achieved travel, directional alignment, and tracking deficit threshold."""
        expected_dist_m = sum(abs(s[0]) * s[2] for s in segments)
        expected_signed_dist_m = sum(s[0] * s[2] for s in segments)
        cmd_samples = [s for s in report["samples"] if s.get("seg_idx", -1) != -1]
        
        last_active_sample_signed_m = cmd_samples[-1].get("disp_net_m", 0.0) if cmd_samples else (report["samples"][-1].get("disp_net_m", 0.0) if report.get("samples") else 0.0)
        final_stopped_signed_m = report["samples"][-1].get("disp_net_m", 0.0) if report.get("samples") else last_active_sample_signed_m
        overrun_signed_m = final_stopped_signed_m - last_active_sample_signed_m

        # Direction-aware distance evaluation:
        # If reverse motion was commanded (expected_signed_dist_m < -1e-4),
        # only negative displacement represents achieved backward travel.
        if expected_signed_dist_m < -1e-4:
            last_active_dist_m = abs(last_active_sample_signed_m) if last_active_sample_signed_m <= 0.0 else 0.0
            final_stopped_dist_m = abs(final_stopped_signed_m) if final_stopped_signed_m <= 0.0 else 0.0
        elif expected_signed_dist_m > 1e-4:
            last_active_dist_m = last_active_sample_signed_m if last_active_sample_signed_m >= 0.0 else 0.0
            final_stopped_dist_m = final_stopped_signed_m if final_stopped_signed_m >= 0.0 else 0.0
        else:
            last_active_dist_m = abs(last_active_sample_signed_m)
            final_stopped_dist_m = abs(final_stopped_signed_m)

        achieved_dist_m = final_stopped_dist_m
        dist_ratio = (achieved_dist_m / expected_dist_m) if expected_dist_m > 1e-4 else 1.0

        # Calculate script commanded-profile distance:
        script_cmd_profile_dist_m = 0.0
        for i in range(len(cmd_samples) - 1):
            s_cur = cmd_samples[i]
            s_nxt = cmd_samples[i+1]
            dt_step = s_nxt.get("t_rel", 0.0) - s_cur.get("t_rel", 0.0)
            tgt_v = abs(s_cur.get("cmd", {}).get("vx", 0.0))
            script_cmd_profile_dist_m += tgt_v * dt_step

        # Calculate firmware wheel-target telemetry distance:
        fw_wheel_target_dist_m = 0.0
        for i in range(len(cmd_samples) - 1):
            s_cur = cmd_samples[i]
            s_nxt = cmd_samples[i+1]
            dt_step = s_nxt.get("t_rel", 0.0) - s_cur.get("t_rel", 0.0)
            tgts = s_cur.get("target_radps", {})
            if tgts and any(abs(tgts.get(m, 0.0)) > 0.001 for m in ["m1", "m2", "m3", "m4"]):
                avg_radps = sum(abs(tgts.get(m, 0.0)) for m in ["m1", "m2", "m3", "m4"]) / 4.0
                fw_wheel_target_dist_m += (avg_radps * self.wheel_radius_m) * dt_step
            else:
                tgt_v = abs(s_cur.get("cmd", {}).get("vx", 0.0))
                fw_wheel_target_dist_m += tgt_v * dt_step

        # Check if diagnostic profile
        is_diagnostic = ("Diagnostic" in name) or name.startswith("Diagnostic_")
        diag_eval = None
        if is_diagnostic:
            diag_eval = self.assess_diagnostic_tracking(report)
            report["diagnostic_evaluation"] = diag_eval

        # Cleanup failure precedence:
        cleanup_failed = (
            report.get("status") == "CLEANUP_FAILED" or
            report.get("cleanup_verification", {}).get("zero_command_confirmed") is False or
            report.get("cleanup_verification", {}).get("disarm_mode0_confirmed") is False
        )

        # Grading thresholds:
        # 0.015 m/s crawl gate (Forward Case 2 or Reverse Case 13):
        # Identified by case name or by kinematic parameters (|vx| = 0.015 m/s, expected = 0.027 m = 27 mm).
        crawl_015_names = (
            "Straight_Crawl_0.015mps", "Reverse_Crawl_0.015mps",
            "Straight Crawl (vx=0.015 m/s)", "Reverse Crawl (vx=-0.015 m/s)"
        )
        is_crawl_015 = not is_diagnostic and (any(cid in name for cid in crawl_015_names) or (
            abs(expected_dist_m - 0.027) < 0.001
            and len(segments) == 1
            and abs(abs(segments[0][0]) - 0.015) < 0.001
        ))
        if is_diagnostic:
            tracking_status = diag_eval["equilibrium_status"]
            is_tracking_deficit = False
            is_excessive_travel = False
        elif is_crawl_015:
            # 0.015 m/s crawl acceptance window: 24 to 30 mm magnitude in commanded direction.
            # Forward commands require positive displacement; reverse commands require negative displacement.
            # Wrong-direction travel or zero travel fails as tracking deficit.
            wrong_direction = (
                (expected_signed_dist_m < -1e-4 and final_stopped_signed_m > 0.0) or
                (expected_signed_dist_m > 1e-4 and final_stopped_signed_m < 0.0)
            )
            is_tracking_deficit = (achieved_dist_m < 0.024) or wrong_direction
            is_excessive_travel = (achieved_dist_m > 0.030) and not wrong_direction
            if is_tracking_deficit:
                tracking_status = "TRACKING_DEFICIT"
            elif is_excessive_travel:
                tracking_status = "EXCESSIVE_TRAVEL"
            else:
                tracking_status = "TRACKING_NOMINAL"
        else:
            crawl_case_identifiers = (
                "Straight_Crawl", "Straight_UltraSlow", "Reverse_Crawl",
                "Straight Crawl", "Straight Ultra-Slow", "Reverse Crawl"
            )
            is_crawl_case = any(cid in name for cid in crawl_case_identifiers)
            is_tracking_deficit = (expected_dist_m > 1e-4 and dist_ratio < 0.70) if is_crawl_case else (expected_dist_m >= 0.030 and dist_ratio < 0.70)
            is_excessive_travel = (expected_dist_m > 1e-4 and dist_ratio > 1.25)
            if is_tracking_deficit:
                tracking_status = "TRACKING_DEFICIT"
            elif is_excessive_travel:
                tracking_status = "EXCESSIVE_TRAVEL"
            else:
                tracking_status = "TRACKING_NOMINAL"

        report["motion_tracking"] = {
            "nominal_requested_distance_m": round(expected_dist_m, 4),
            "script_command_profile_distance_m": round(script_cmd_profile_dist_m, 4),
            "firmware_wheel_target_distance_m": round(fw_wheel_target_dist_m, 4),
            "telemetry_observed_profile_distance_m": round(script_cmd_profile_dist_m, 4),
            "last_active_sample_distance_m": round(last_active_dist_m, 4),
            "final_stopped_encoder_distance_m": round(final_stopped_dist_m, 4),
            "post_command_overrun_distance_m": round(abs(overrun_signed_m), 4),
            "operator_physical_measurement_mm": report.get("operator_physical_measurement_mm", None),
            "expected_distance_m": round(expected_dist_m, 4),
            "achieved_distance_m": round(achieved_dist_m, 4),
            "final_displacement_signed_m": round(final_stopped_signed_m, 4),
            "last_active_displacement_signed_m": round(last_active_sample_signed_m, 4),
            "distance_ratio": round(dist_ratio, 3),
            "distance_ratio_pct": round(dist_ratio * 100.0, 1),
            "tracking_status": tracking_status,
            "tracking_deficit_threshold_pct": 70.0
        }

        # Final overall status classification:
        if cleanup_failed:
            report["status"] = "CLEANUP_FAILED"
            if is_diagnostic and diag_eval is not None:
                diag_eval["equilibrium_status"] = "CLEANUP_FAILED"
                diag_eval["reason"] = "Cleanup failure took precedence over equilibrium classification."
        elif report.get("status") in ("SUCCESS", "IN_PROGRESS"):
            if report.get("abort_reason"):
                report["status"] = report["abort_reason"].split(":")[0]
            elif is_diagnostic and diag_eval is not None:
                report["status"] = diag_eval["equilibrium_status"]
            elif is_tracking_deficit:
                report["status"] = "TRACKING_DEFICIT"
            elif is_excessive_travel:
                report["status"] = "EXCESSIVE_TRAVEL"
            else:
                report["status"] = "SUCCESS"

        return report["motion_tracking"]


CASES: Dict[str, Dict[str, Any]] = {
    "1": {
        "id": "FailedArc_Baseline",
        "name": "Failed Arc Baseline (vx=0.014 m/s, wz=-0.132 rad/s)",
        "desc": "Baseline response to exact failed Nav2 command (Right target = -0.26 rad/s)",
        "segments": [(0.014, -0.132, 1.8)],
        "max_dist_m": 0.10,
        "max_yaw_deg": 25.0
    },
    "2": {
        "id": "Straight_Crawl_0.015mps",
        "name": "Straight Crawl (vx=0.015 m/s)",
        "desc": "Rolling speed near 0.45 rad/s wheel speed floor",
        "segments": [(0.015, 0.0, 1.8)],
        "max_dist_m": 0.08,
        "max_yaw_deg": 10.0
    },
    "3": {
        "id": "Straight_UltraSlow_0.008mps",
        "name": "Straight Ultra-Slow (vx=0.008 m/s)",
        "desc": "Rolling speed below 0.35 rad/s (all wheels target ~0.24 rad/s)",
        "segments": [(0.008, 0.0, 1.8)],
        "max_dist_m": 0.05,
        "max_yaw_deg": 10.0
    },
    "4": {
        "id": "PureSpin_Slow_CCW",
        "name": "Pure Spin Slow CCW (wz=+0.15 rad/s)",
        "desc": "Low-rate in-place spin counterclockwise",
        "segments": [(0.0, +0.15, 1.5)],
        "max_dist_m": 0.05,
        "max_yaw_deg": 30.0
    },
    "5": {
        "id": "PureSpin_Slow_CW",
        "name": "Pure Spin Slow CW (wz=-0.15 rad/s)",
        "desc": "Low-rate in-place spin clockwise",
        "segments": [(0.0, -0.15, 1.5)],
        "max_dist_m": 0.05,
        "max_yaw_deg": 30.0
    },
    "6": {
        "id": "PureSpin_Nominal_CCW",
        "name": "Pure Spin Nominal CCW (wz=+0.35 rad/s)",
        "desc": "Nominal in-place spin counterclockwise",
        "segments": [(0.0, +0.35, 1.5)],
        "max_dist_m": 0.05,
        "max_yaw_deg": 45.0
    },
    "7": {
        "id": "PureSpin_Nominal_CW",
        "name": "Pure Spin Nominal CW (wz=-0.35 rad/s)",
        "desc": "Nominal in-place spin clockwise",
        "segments": [(0.0, -0.35, 1.5)],
        "max_dist_m": 0.05,
        "max_yaw_deg": 45.0
    },
    "8": {
        "id": "Transition_PureSpin_To_Arc",
        "name": "Pure Spin to Opposing Arc Transition",
        "desc": "Tests track sign reversal: Pure spin (wz=-0.35) -> Opposing arc (vx=0.02, wz=-0.15)",
        "segments": [(0.000, -0.350, 0.80), (0.020, -0.150, 1.20)],
        "max_dist_m": 0.15,
        "max_yaw_deg": 35.0
    },
    "9": {
        "id": "FailedArc_AssistedBreakout",
        "name": "Failed Arc Assisted Breakout Test",
        "desc": "120ms assist pulse (vx=0.04, wz=-0.25) followed by failed setpoint (vx=0.014, wz=-0.132)",
        "segments": [(0.040, -0.250, 0.12), (0.014, -0.132, 1.50)],
        "max_dist_m": 0.12,
        "max_yaw_deg": 25.0
    },
    "10": {
        "id": "Straight_Crawl_0.030mps",
        "name": "Straight Crawl (vx=0.030 m/s)",
        "desc": "Straight rolling speed comparison (vx=0.030 m/s, duration=1.8s, wheel target ~0.92 rad/s)",
        "segments": [(0.030, 0.0, 1.8)],
        "max_dist_m": 0.12,
        "max_yaw_deg": 10.0
    },
    "11": {
        "id": "Straight_Crawl_0.050mps",
        "name": "Straight Crawl (vx=0.050 m/s)",
        "desc": "Straight rolling speed test above 0.030 m/s (vx=0.050 m/s, duration=1.8s, wheel target ~1.54 rad/s)",
        "segments": [(0.050, 0.0, 1.8)],
        "max_dist_m": 0.15,
        "max_yaw_deg": 10.0
    },
    "12": {
        "id": "Reverse_Crawl_0.050mps",
        "name": "Reverse Crawl (vx=-0.050 m/s)",
        "desc": "Symmetric reverse straight rolling speed test (vx=-0.050 m/s, duration=1.8s, wheel target ~-1.54 rad/s)",
        "segments": [(-0.050, 0.0, 1.8)],
        "max_dist_m": 0.15,
        "max_yaw_deg": 10.0
    },
    "13": {
        "id": "Reverse_Crawl_0.015mps",
        "name": "Reverse Crawl (vx=-0.015 m/s)",
        "desc": "Symmetric reverse straight crawl speed test (vx=-0.015 m/s, duration=1.8s, wheel target ~-0.46 rad/s, nominal=-27mm)",
        "segments": [(-0.015, 0.0, 1.8)],
        "max_dist_m": 0.08,
        "max_yaw_deg": 10.0
    },
    "14": {
        "id": "Diagnostic_Reverse_Long_0.015mps",
        "name": "Reverse Long Crawl Diagnostic (vx=-0.015 m/s, 6.0s)",
        "desc": "Diagnostic step observing longer-duration reverse tracking, speed slew, and integral equilibrium",
        "segments": [(-0.015, 0.0, 6.0)],
        "max_dist_m": 0.20,
        "max_yaw_deg": 10.0,
        "max_speed_mps": 0.060
    }
}


def main():
    parser = argparse.ArgumentParser(description="Rover 1 Floor Speed Characterization")
    parser.add_argument("--case", choices=list(CASES.keys()) + [c["id"] for c in CASES.values()],
                        help="Select test case (1-14 or Case ID). Default: 1 (FailedArc_Baseline)")
    args = parser.parse_args()

    selected_key = "1"
    if args.case:
        for k, v in CASES.items():
            if args.case == k or args.case == v["id"]:
                selected_key = k
                break
    else:
        print("\n==================================================================")
        print("   Rover 1 Floor Speed Characterization - Single Case Selector")
        print("==================================================================")
        for k, v in sorted(CASES.items(), key=lambda x: int(x[0])):
            prefix = "[DEFAULT] " if k == "1" else "          "
            print(f" {prefix}{k}: {v['name']}")
            print(f"             {v['desc']}")
        print("------------------------------------------------------------------")
        print(f"Press Enter for default [1] or enter Case (1-{len(CASES)}): ")
        try:
            val = input("> ").strip()
            if val in CASES:
                selected_key = val
        except (KeyboardInterrupt, EOFError):
            print("\nAborted by operator.")
            sys.exit(1)

    case = CASES[selected_key]
    print(f"\nSelected Case: [{selected_key}] {case['name']}")

    try:
        app = FloorSpeedCharacterizer()
    except Exception as e:
        print(f"[INIT FAILED] {e}")
        sys.exit(1)

    # Preflight verification
    print("\nPreflight System Status Check:")
    try:
        t0 = app.fetch_all_telemetry()
        print(f"  Cockpit URL:    {app.cockpit_url}")
        print(f"  Bridge URL:     {app.bridge_url}")
        print(f"  Drive Mode:     {t0['drive'].get('mode')} (Armed: {t0['drive'].get('armed')})")
        print(f"  Wheel Diameter: {app.wheel_diameter_m} m (Radius: {app.wheel_radius_m} m)")
        print(f"  Track Width:    {app.effective_track_width_m} m")
        print(f"  Uptime:         {t0['drive'].get('uptimeMs')} ms")
        print(f"  IMU Heading:    {t0['continuous_yaw_deg']:.2f} deg")
    except Exception as e:
        print(f"[PREFLIGHT FAILED] {e}")
        sys.exit(1)

    nom_dist_mm = sum(vx * dur for vx, _, dur in case["segments"]) * 1000.0
    nom_yaw_deg = math.degrees(sum(wz * dur for _, wz, dur in case["segments"]))
    if abs(nom_dist_mm) > 0.1 and abs(nom_yaw_deg) < 0.1:
        exp_str = f"{nom_dist_mm:+.1f} mm linear travel (Pass window: {abs(nom_dist_mm)*0.889:.1f} to {abs(nom_dist_mm)*1.111:.1f} mm)"
        meas_str = "Measure net straight-line physical travel in millimeters (mm) with ruler."
    elif abs(nom_yaw_deg) > 0.1 and abs(nom_dist_mm) < 0.1:
        exp_str = f"{nom_yaw_deg:+.1f}° in-place rotation (Bounds: max_yaw={case['max_yaw_deg']}°)"
        meas_str = "Measure physical angle turned in degrees (°) using protractor / floor reference mark."
    else:
        exp_str = f"Net linear ~{nom_dist_mm:+.1f} mm, net yaw ~{nom_yaw_deg:+.1f}°"
        meas_str = "Observe smooth arc/transition handoff and measure net linear (mm) and angular (°) change."

    print("\n" + "=" * 80)
    print(f"ROVER ONE REUSABLE FLOOR CHARACTERIZATION: CASE [{selected_key}] {case['name']}")
    print(f"Segments: {case['segments']} | Bounds: max_dist={case['max_dist_m']:.3f}m, max_yaw={case['max_yaw_deg']:.1f}°")
    print("-" * 80)
    print("1. TEST GOAL:")
    print(f"   {case['desc']}")
    print("2. WHAT TO MONITOR & MEASURE (OPERATOR):")
    print("   • Before confirming ('CONFIRM'): Ensure >= 1.0 m clear floor space and align")
    print("     Rover 1 starting reference pointer.")
    print("   • During motion: Watch whether wheel breakaway and crawl/spin look smooth,")
    print("     hesitant, or stop-and-restart.")
    print(f"   • After stop & disarm: {meas_str}")
    print("3. EXPECTED RESULT (PASS CRITERIA):")
    print(f"   • Expected Motion   : {exp_str}")
    print("   • Terminal Status   : Overall Step Result = SUCCESS (TRACKING_NOMINAL)")
    print("   • Final Safe State  : zero=True, disarm=True (Mode 0 LOCKED)")
    print("=" * 80)

    print("\nOPERATOR CONFIRMATION REQUIRED:")
    print(f"Confirm Rover 1 has >= 1.0 m clear floor space.")
    print("Type 'CONFIRM' to arm and execute this single case: ")
    try:
        ans = input("> ").strip()
        if ans != "CONFIRM":
            print("Aborted by operator.")
            sys.exit(1)
    except (KeyboardInterrupt, EOFError):
        print("\nAborted.")
        sys.exit(1)

    # Execute single selected case
    exit_code = 0
    res = None
    try:
        res = app.run_profile_step(
            name=case["id"],
            segments=case["segments"],
            max_dist_m=case["max_dist_m"],
            max_yaw_deg=case["max_yaw_deg"],
            max_speed_mps=case.get("max_speed_mps", 0.0)
        )
        if res.get("status") not in ("SUCCESS", "SETTLED_EQUILIBRIUM", "INCONCLUSIVE_EQUILIBRIUM") or app.fault_latched:
            exit_code = 1
    except Exception as e:
        print(f"\n[EXECUTION EXCEPTION] {e}")
        exit_code = 1
        res = app.active_report
        if res is not None:
            res["status"] = f"ABORTED_EXCEPTION: {e}"
            res["exception"] = str(e)
    finally:
        if res is not None:
            out_name = f"floor_char_{case['id']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), out_name)
            try:
                with open(out_path, "w", encoding="utf-8") as f:
                    json.dump(res, f, indent=2)
                print(f"\n[REPORT PRESERVED] Saved to: {out_path}")
            except Exception as se:
                print(f"[ERROR] Failed to write report file: {se}")
        app.fail_closed_cleanup()

    if exit_code != 0:
        print(f"\n[FAILURE EXIT] Execution completed with failure status (exit code {exit_code})")
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
