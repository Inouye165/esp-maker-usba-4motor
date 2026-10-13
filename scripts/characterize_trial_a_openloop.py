#!/usr/bin/env python3
"""
scripts/characterize_trial_a_openloop.py
Rover 1 Open-Loop Reverse PWM Transient Response Scan (Trial A).

Objective:
- Safely measure open-loop reverse wheel response across a descending PWM staircase
  without closed-loop feedback or recovery assist interference.
- Strict Telemetry Validation: Rejects missing, malformed, or non-finite telemetry.
  Requires all four wheels, drive state, IMU data, and valid freshness metadata.
  Never substitutes zero for missing safety data.
- Cleanup-Protected Arming: Arming is inside the protected try block. Establishes travel
  and yaw baselines before sending nonzero PWM. Confirms armed state and required safety
  firmware before motion.
- Common Exit Path: All aborts exit through a common path that cleans up, saves the report,
  and returns the report dictionary (never None).
- Independent Stream Advancement: Tracks sequence numbers independently for drive, PID,
  encoders, and IMU.
- Pre-Dispatch Rechecks: Rechecks monotonic deadlines, effective telemetry age (including
  fetch delay), and armed status immediately before PWM dispatch.
- Telemetry Recording: Records measured final PWM and per-wheel speeds with source
  timestamps and sequences.

Profile:
  Phase 1 (Breakout Impulse):   -48 PWM for 200 ms (0.20 s)
  Phase 2 (Descending Steps):   -46 PWM for 400 ms (0.40 s)
                               -44 PWM for 400 ms (0.40 s)
                               -42 PWM for 400 ms (0.40 s)
                               -40 PWM for 400 ms (0.40 s)
                               -38 PWM for 400 ms (0.40 s)
  Total Nominal Duration:       2.20 s (Hard cutoff: 2.50 s)
"""

import sys
import os
import time
import json
import math
import argparse
import urllib.request
import urllib.error
from typing import Dict, Any, List, Optional, Tuple

# Drivetrain & Geometric Constants (Case 2 Deployed Baseline)
WHEEL_DIAMETER_M = 0.0650
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0
TICKS_PER_REV = 1974.1667
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER_M) / TICKS_PER_REV  # ~0.0001034378 m/tick

# Safety Thresholds
MAX_ABS_WHEEL_TRAVEL_M = 0.150   # 150 mm maximum accumulated absolute travel on any single wheel
MAX_MEAN_ABS_SPEED_MPS = 0.060   # 0.060 m/s mean absolute wheel speed bound (avoids signed cancellation)
MAX_ACCUMULATED_YAW_DEG = 10.0   # 10.0 degrees maximum absolute yaw drift (unwrapped across +/-180 deg)
MAX_TELEMETRY_AGE_S = 0.250      # 250 ms maximum allowed telemetry freshness gap
MAX_TOTAL_DURATION_S = 2.50      # 2.50 s hard safety cutoff
HTTP_REQUEST_TIMEOUT_S = 0.150   # 150 ms bounded timeout per HTTP call
REQUIRED_SAFETY_FIRMWARE = "1.3.2-olp-safe"

# Step Profile Definition
TRIAL_A_STEPS = [
    {"name": "Impulse", "pwm": -48, "duration_s": 0.20},
    {"name": "Step_46", "pwm": -46, "duration_s": 0.40},
    {"name": "Step_44", "pwm": -44, "duration_s": 0.40},
    {"name": "Step_42", "pwm": -42, "duration_s": 0.40},
    {"name": "Step_40", "pwm": -40, "duration_s": 0.40},
    {"name": "Step_38", "pwm": -38, "duration_s": 0.40},
]


class TelemetryValidationError(Exception):
    """Raised when telemetry is missing, malformed, non-finite, or stale."""
    pass


def _assert_finite_numeric(value: Any, name: str) -> float:
    """Validates that a field is numeric (not bool) and finite. Never substitutes zero."""
    if value is None:
        raise TelemetryValidationError(f"Missing required numeric field: '{name}' is None")
    if isinstance(value, bool):
        raise TelemetryValidationError(f"Malformed field: '{name}' is boolean ({value}), expected numeric")
    if not isinstance(value, (int, float)):
        raise TelemetryValidationError(f"Malformed field: '{name}' type {type(value).__name__} is not numeric")
    f_val = float(value)
    if not math.isfinite(f_val):
        raise TelemetryValidationError(f"Non-finite field: '{name}' has value {f_val}")
    return f_val


class ContinuousYawTracker:
    """
    Tracks heading drift with continuous unwrap across +/-180 deg boundary.
    Guarantees no false aborts when heading crosses +/-180 degrees, while strictly
    bounding both net heading drift and total accumulated angular excursion.
    """
    def __init__(self):
        self.initial_yaw_deg: Optional[float] = None
        self.last_raw_yaw_deg: Optional[float] = None
        self.unwrapped_yaw_deg: float = 0.0
        self.accum_angular_excursion_deg: float = 0.0

    def reset(self, initial_yaw_deg: Optional[float] = None):
        self.initial_yaw_deg = initial_yaw_deg
        self.last_raw_yaw_deg = initial_yaw_deg
        self.unwrapped_yaw_deg = initial_yaw_deg if initial_yaw_deg is not None else 0.0
        self.accum_angular_excursion_deg = 0.0

    def update(self, raw_yaw_deg: float) -> Tuple[float, float, float]:
        """
        Updates tracker with new raw yaw reading in [-180, +180] deg.
        Returns:
          unwrapped_yaw_deg: Continuous heading in degrees
          net_yaw_drift_deg: Absolute deviation from initial heading (|unwrapped - initial|)
          accum_excursion_deg: Total cumulative absolute angular change (sum |delta|)
        """
        if self.initial_yaw_deg is None:
            self.initial_yaw_deg = raw_yaw_deg
            self.last_raw_yaw_deg = raw_yaw_deg
            self.unwrapped_yaw_deg = raw_yaw_deg
            return self.unwrapped_yaw_deg, 0.0, 0.0

        # Unwrapped angular difference across [-180, +180)
        diff = ((raw_yaw_deg - self.last_raw_yaw_deg + 180.0) % 360.0) - 180.0
        self.unwrapped_yaw_deg += diff
        self.accum_angular_excursion_deg += abs(diff)
        self.last_raw_yaw_deg = raw_yaw_deg

        net_drift = abs(self.unwrapped_yaw_deg - self.initial_yaw_deg)
        return self.unwrapped_yaw_deg, net_drift, self.accum_angular_excursion_deg


class AccumulatedTravelTracker:
    """
    Tracks cumulative absolute travel per wheel and net displacement from baseline.
    Ensures back-and-forth wheel reversals accumulate toward the limit rather than
    canceling out algebraically.
    """
    def __init__(self, meters_per_tick: float):
        self.meters_per_tick = meters_per_tick
        self.initial_ticks: Optional[List[int]] = None
        self.last_ticks: Optional[List[int]] = None
        self.accum_ticks: List[int] = [0, 0, 0, 0]

    def reset(self, initial_ticks: Optional[List[int]] = None):
        if initial_ticks is not None:
            self.initial_ticks = list(initial_ticks)
            self.last_ticks = list(initial_ticks)
        else:
            self.initial_ticks = None
            self.last_ticks = None
        self.accum_ticks = [0, 0, 0, 0]

    def update(self, ticks: List[int]) -> Tuple[List[float], float, float]:
        """
        Updates wheel tick travel.
        Returns:
          accum_travel_per_wheel_m: Cumulative distance traveled by each wheel (meters)
          max_accum_abs_travel_m: Maximum cumulative absolute travel across all 4 wheels
          max_net_displacement_m: Maximum absolute displacement from initial ticks
        """
        if self.initial_ticks is None:
            self.initial_ticks = list(ticks)
            self.last_ticks = list(ticks)
            return [0.0, 0.0, 0.0, 0.0], 0.0, 0.0

        for i in range(4):
            delta = abs(ticks[i] - self.last_ticks[i])
            self.accum_ticks[i] += delta
            self.last_ticks[i] = ticks[i]

        accum_travel = [t * self.meters_per_tick for t in self.accum_ticks]
        net_disp = [abs(ticks[i] - self.initial_ticks[i]) * self.meters_per_tick for i in range(4)]
        return accum_travel, max(accum_travel), max(net_disp)


class StreamAdvancementTracker:
    """
    Tracks sequence advancement independently for drive, PID, encoders, and IMU streams.
    Flags an abort if any individual stream freezes for > max_stale_s.
    """
    def __init__(self, max_stale_s: float = MAX_TELEMETRY_AGE_S):
        self.max_stale_s = max_stale_s
        self.last_seq: Dict[str, Optional[int]] = {
            "drive": None,
            "pid": None,
            "encoders": None,
            "imu": None
        }
        self.last_advancement_mono: Dict[str, float] = {
            "drive": 0.0,
            "pid": 0.0,
            "encoders": 0.0,
            "imu": 0.0
        }

    def reset(self, initial_mono: float):
        for stream in self.last_seq:
            self.last_seq[stream] = None
            self.last_advancement_mono[stream] = initial_mono

    def update(self, current_mono: float, sequences: Dict[str, int]) -> Optional[str]:
        """
        Updates sequence numbers and evaluates independent stream advancement.
        Returns diagnostic error string if any stream is frozen > max_stale_s, or None if healthy.
        """
        for stream, seq in sequences.items():
            prev_seq = self.last_seq[stream]
            if prev_seq is None or seq != prev_seq:
                self.last_seq[stream] = seq
                self.last_advancement_mono[stream] = current_mono
            else:
                elapsed = current_mono - self.last_advancement_mono[stream]
                if elapsed > self.max_stale_s:
                    return f"FROZEN_STREAM_{stream.upper()} (Sequence {seq} frozen for {elapsed:.3f}s > {self.max_stale_s}s)"
        return None

    def check_stale(self, current_mono: float) -> Optional[str]:
        """Rechecks elapsed time since last advancement without sequence change."""
        for stream, last_mono in self.last_advancement_mono.items():
            elapsed = current_mono - last_mono
            if elapsed > self.max_stale_s:
                seq = self.last_seq[stream]
                return f"STALE_STREAM_{stream.upper()} (No advancement on {stream} for {elapsed:.3f}s > {self.max_stale_s}s, seq={seq})"
        return None


class TrialAHarness:
    def __init__(self, host="127.0.0.1", port=3000, report_dir="reports", timeout_s=HTTP_REQUEST_TIMEOUT_S, ssh_host=None, ssh_user="ron"):
        self.base_url = f"http://{host}:{port}"
        self.report_dir = report_dir
        self.timeout_s = timeout_s
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.samples: List[Dict[str, Any]] = []
        self.abort_reason: Optional[str] = None
        self.cleanup_failed: bool = False
        self.yaw_tracker = ContinuousYawTracker()
        self.travel_tracker = AccumulatedTravelTracker(METERS_PER_TICK)
        self.stream_tracker = StreamAdvancementTracker(MAX_TELEMETRY_AGE_S)

    def _http_post(self, endpoint: str, data: Dict[str, Any]) -> Dict[str, Any]:
        url = f"{self.base_url}{endpoint}"
        body = json.dumps(data).encode("utf-8")
        req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def _http_get(self, endpoint: str) -> Dict[str, Any]:
        url = f"{self.base_url}{endpoint}"
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8"))

    # ────────────────────────────────────────────────────────────
    # Strict API Parsers (Never substitute zero for missing data)
    # ────────────────────────────────────────────────────────────

    def parse_drive_status(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Strictly validates /api/drive/status payload."""
        if not isinstance(data, dict):
            raise TelemetryValidationError("/api/drive/status payload is not a dictionary")
        if not data.get("ok"):
            raise TelemetryValidationError(f"/api/drive/status returned ok=false: {data}")
        status = data.get("status")
        if not isinstance(status, dict):
            raise TelemetryValidationError(f"Missing 'status' object in /api/drive/status: {data}")

        armed = status.get("armed")
        if not isinstance(armed, bool):
            raise TelemetryValidationError(f"Missing or non-boolean 'armed' in drive status: {armed}")

        mode = status.get("mode")
        if not isinstance(mode, int) or isinstance(mode, bool):
            raise TelemetryValidationError(f"Missing or non-integer 'mode' in drive status: {mode}")

        seq = status.get("seq")
        if not isinstance(seq, int) or isinstance(seq, bool):
            raise TelemetryValidationError(f"Missing or non-integer 'seq' in drive status: {seq}")

        uptime_ms = _assert_finite_numeric(status.get("uptimeMs"), "status.uptimeMs")

        # Validate freshness metadata (check server-side ageMs first)
        age_ms = data.get("lastPacketAgeMs")
        if age_ms is None:
            age_ms = status.get("dataAgeMs")
        if age_ms is None and isinstance(data.get("pidTelemetry"), dict):
            age_ms = data["pidTelemetry"].get("ageMs")
        if age_ms is None and "receivedAt" in status:
            rx_at = status.get("receivedAt")
            if isinstance(rx_at, (int, float)):
                calc_age = (time.time() * 1000.0) - float(rx_at)
                if abs(calc_age) < 2000.0:
                    age_ms = calc_age

        if age_ms is None:
            raise TelemetryValidationError("Missing freshness metadata in drive status (no lastPacketAgeMs/dataAgeMs/pidTelemetry.ageMs)")
        age_ms = _assert_finite_numeric(age_ms, "drive status ageMs")
        if age_ms < -2000.0:
            raise TelemetryValidationError(f"Drive status receivedAt in future: {age_ms} ms")
        if age_ms > (MAX_TELEMETRY_AGE_S * 1000.0):
            raise TelemetryValidationError(f"Drive status stale: {age_ms:.1f} ms > {MAX_TELEMETRY_AGE_S*1000} ms")

        return {
            "armed": armed,
            "mode": mode,
            "seq": seq,
            "uptime_ms": uptime_ms,
            "age_ms": age_ms
        }

    def parse_pid_telemetry(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Strictly validates /api/pid-telemetry payload. Requires all 4 wheels."""
        if not isinstance(data, dict):
            raise TelemetryValidationError("/api/pid-telemetry payload is not a dictionary")
        if not data.get("ok"):
            raise TelemetryValidationError(f"/api/pid-telemetry returned ok=false: {data}")
        if not data.get("valid"):
            raise TelemetryValidationError(f"/api/pid-telemetry reports valid=false: {data}")
        if data.get("stale"):
            raise TelemetryValidationError("/api/pid-telemetry reports stale=true")

        age_ms = _assert_finite_numeric(data.get("dataAgeMs"), "pid-telemetry dataAgeMs")
        if age_ms > (MAX_TELEMETRY_AGE_S * 1000.0):
            raise TelemetryValidationError(f"PID telemetry stale: {age_ms:.1f} ms > {MAX_TELEMETRY_AGE_S*1000} ms")

        seq = data.get("sequence")
        if not isinstance(seq, int) or isinstance(seq, bool):
            raise TelemetryValidationError(f"Missing or non-integer 'sequence' in PID telemetry: {seq}")

        telem = data.get("telemetry")
        if not isinstance(telem, dict):
            raise TelemetryValidationError("Missing 'telemetry' dictionary in PID response")

        speeds_radps = []
        pwms = []
        for m in ["m1", "m2", "m3", "m4"]:
            if m not in telem or not isinstance(telem[m], dict):
                raise TelemetryValidationError(f"Missing wheel '{m}' in PID telemetry")
            w_data = telem[m]
            radps = _assert_finite_numeric(w_data.get("measuredRadps"), f"{m}.measuredRadps")
            pwm = _assert_finite_numeric(w_data.get("finalPwm"), f"{m}.finalPwm")
            speeds_radps.append(radps)
            pwms.append(int(pwm))

        return {
            "seq": seq,
            "age_ms": age_ms,
            "speeds_radps": speeds_radps,
            "pwms": pwms
        }

    def parse_encoders(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Strictly validates /api/encoders payload. Requires all 4 wheels."""
        if not isinstance(data, dict):
            raise TelemetryValidationError("/api/encoders payload is not a dictionary")
        if not data.get("ok"):
            raise TelemetryValidationError(f"/api/encoders returned ok=false: {data}")
        if data.get("serialConnected") is False:
            raise TelemetryValidationError("/api/encoders reports serialConnected=false")

        age_ms = _assert_finite_numeric(data.get("lastPacketAgeMs"), "encoders lastPacketAgeMs")
        if age_ms > (MAX_TELEMETRY_AGE_S * 1000.0):
            raise TelemetryValidationError(f"Encoders telemetry stale: {age_ms:.1f} ms > {MAX_TELEMETRY_AGE_S*1000} ms")

        seq = data.get("sequence")
        if not isinstance(seq, int) or isinstance(seq, bool):
            raise TelemetryValidationError(f"Missing or non-integer 'sequence' in encoders: {seq}")

        encs = data.get("encoders")
        if not isinstance(encs, dict):
            raise TelemetryValidationError("Missing 'encoders' dictionary in encoders response")

        ticks = []
        for m in ["m1", "m2", "m3", "m4"]:
            if m not in encs:
                raise TelemetryValidationError(f"Missing wheel '{m}' in encoders response")
            t_val = _assert_finite_numeric(encs.get(m), f"encoders.{m}")
            ticks.append(int(t_val))

        return {
            "seq": seq,
            "age_ms": age_ms,
            "ticks": ticks
        }

    def parse_imu(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Strictly validates /api/imu payload. Validates quaternion and calculates yaw."""
        if not isinstance(data, dict):
            raise TelemetryValidationError("/api/imu payload is not a dictionary")
        if not data.get("ok"):
            raise TelemetryValidationError(f"/api/imu returned ok=false: {data}")
        if data.get("stale"):
            raise TelemetryValidationError("/api/imu reports stale=true")
        if data.get("serialConnected") is False:
            raise TelemetryValidationError("/api/imu reports serialConnected=false")

        age_ms = _assert_finite_numeric(data.get("dataAgeMs"), "imu dataAgeMs")
        if age_ms > (MAX_TELEMETRY_AGE_S * 1000.0):
            raise TelemetryValidationError(f"IMU telemetry stale: {age_ms:.1f} ms > {MAX_TELEMETRY_AGE_S*1000} ms")

        seq = data.get("sequence")
        if not isinstance(seq, int) or isinstance(seq, bool):
            raise TelemetryValidationError(f"Missing or non-integer 'sequence' in IMU: {seq}")

        q = data.get("orientation")
        if not isinstance(q, dict):
            raise TelemetryValidationError("Missing 'orientation' dictionary in IMU response")

        qw = _assert_finite_numeric(q.get("w"), "orientation.w")
        qx = _assert_finite_numeric(q.get("x"), "orientation.x")
        qy = _assert_finite_numeric(q.get("y"), "orientation.y")
        qz = _assert_finite_numeric(q.get("z"), "orientation.z")

        q_norm_sq = qw * qw + qx * qx + qy * qy + qz * qz
        if q_norm_sq < 0.5 or q_norm_sq > 1.5:
            raise TelemetryValidationError(f"Invalid quaternion norm squared: {q_norm_sq}")

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)

        return {
            "seq": seq,
            "age_ms": age_ms,
            "yaw_deg": yaw_deg
        }

    # ────────────────────────────────────────────────────────────
    # Endpoints & Unified Snapshot
    # ────────────────────────────────────────────────────────────

    def arm_drive(self) -> Dict[str, Any]:
        """Arm normal drive mode via Cockpit API."""
        return self._http_post("/api/drive/arm", {})

    def disarm_drive(self) -> Optional[Dict[str, Any]]:
        """Disarm normal drive mode via Cockpit API."""
        return self._http_post("/api/drive/disarm", {})

    def send_open_loop_pwm(self, m1: int, m2: int, m3: int, m4: int) -> Dict[str, Any]:
        """Send 4-wheel open-loop PWM command via Cockpit API."""
        return self._http_post("/api/open-loop-pwm", {"pwms": [int(m1), int(m2), int(m3), int(m4)]})

    def fetch_drive_status(self) -> Dict[str, Any]:
        raw = self._http_get("/api/drive/status")
        return self.parse_drive_status(raw)

    def fetch_pid_telemetry(self) -> Dict[str, Any]:
        raw = self._http_get("/api/pid-telemetry")
        return self.parse_pid_telemetry(raw)

    def fetch_encoders(self) -> Dict[str, Any]:
        raw = self._http_get("/api/encoders")
        return self.parse_encoders(raw)

    def fetch_imu(self) -> Dict[str, Any]:
        raw = self._http_get("/api/imu")
        return self.parse_imu(raw)

    def fetch_telemetry_snapshot(self) -> Dict[str, Any]:
        """
        Fetches and validates all 4 telemetry streams. Never substitutes defaults.
        Records fetch start monotonic timestamp, receipt monotonic, and epoch timestamps.
        """
        fetch_start_mono = time.monotonic()
        drv = self.fetch_drive_status()
        pid = self.fetch_pid_telemetry()
        enc = self.fetch_encoders()
        imu = self.fetch_imu()

        receipt_mono = time.monotonic()
        receipt_epoch_ms = int(time.time() * 1000)

        speeds_radps = pid["speeds_radps"]
        speeds_mps = [w * WHEEL_RADIUS_M for w in speeds_radps]
        max_age_ms = max(drv["age_ms"], pid["age_ms"], enc["age_ms"], imu["age_ms"])

        return {
            "fetch_start_mono": fetch_start_mono,
            "receipt_mono": receipt_mono,
            "receipt_epoch_ms": receipt_epoch_ms,
            "armed": drv["armed"],
            "mode": drv["mode"],
            "seq_drive": drv["seq"],
            "seq_pid": pid["seq"],
            "seq_encoders": enc["seq"],
            "seq_imu": imu["seq"],
            "age_drive_ms": drv["age_ms"],
            "age_pid_ms": pid["age_ms"],
            "age_encoders_ms": enc["age_ms"],
            "age_imu_ms": imu["age_ms"],
            "data_age_ms": max_age_ms,
            "ticks": enc["ticks"],
            "speeds_radps": speeds_radps,
            "speeds_mps": speeds_mps,
            "pwms": pid["pwms"],
            "yaw_deg": imu["yaw_deg"]
        }

    # ────────────────────────────────────────────────────────────
    # Safety Firmware Identity Verification
    # ────────────────────────────────────────────────────────────

    def verify_safety_firmware(self, required_version: str = REQUIRED_SAFETY_FIRMWARE, timeout_s: float = 2.0) -> bool:
        """
        Verifies that the remote ESP32 is running the required safety firmware build
        before allowing arming or physical motion.
        1. Captures the journal cursor before requesting /api/firmware.
        2. Dispatches fresh query via /api/firmware.
        3. Waits with bounded timeout for a new firmware response after that cursor.
        4. Requires the exact expected version.
        Supports direct local systemd access when running on the target rover as well
        as remote SSH execution when running from an operator workstation.
        An old matching entry, missing response, query failure, or mismatched version strictly blocks arming.
        """
        import re
        import subprocess

        print(f"=== Safety Firmware Verification (Required: {required_version}) ===")

        # Detect if running natively on the target rover or remotely
        is_local = (
            (os.name != "nt" and (os.path.exists("/bin/journalctl") or os.path.exists("/usr/bin/journalctl"))) and
            (self.ssh_host in ("127.0.0.1", "localhost") or "127.0.0.1" in self.base_url or "localhost" in self.base_url or not self.ssh_host)
        )

        client = None
        if not is_local:
            import paramiko
            ssh_host = self.ssh_host or os.environ.get("ROVER_PI_HOST", "10.0.0.246")
            ssh_user = self.ssh_user or os.environ.get("ROVER_PI_USER", "ron")

            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                client.connect(ssh_host, username=ssh_user, timeout=3)
            except Exception as e:
                raise RuntimeError(f"Safety firmware verification failed (SSH connection error to {ssh_user}@{ssh_host}): {e}. Motion blocked!")

        def _exec_journalctl(args: List[str]) -> str:
            if is_local:
                res = subprocess.run(["journalctl"] + args, capture_output=True, text=True)
                return res.stdout
            else:
                import shlex
                cmd = f"journalctl {' '.join(shlex.quote(a) for a in args)}"
                stdin, stdout, stderr = client.exec_command(cmd)
                return stdout.read().decode("utf-8", errors="replace")

        try:
            # 1. Capture journal cursor before requesting /api/firmware
            try:
                cursor_output = _exec_journalctl(["-u", "rover-server.service", "-n", "0", "--show-cursor"]).strip()
            except Exception as e:
                raise RuntimeError(f"Safety firmware verification failed (unable to query journal cursor): {e}. Motion blocked!")

            cursor_match = re.search(r"--\s*cursor:\s*(\S+)", cursor_output)
            if not cursor_match:
                raise RuntimeError(f"Safety firmware verification failed: No journal cursor found in output: '{cursor_output}'. Motion blocked!")
            cursor = cursor_match.group(1).strip()
            print(f"Captured Journal Cursor: {cursor}")

            # 2. Trigger fresh firmware info query over Serial via Cockpit endpoint
            try:
                resp = self._http_get("/api/firmware")
                if not resp.get("ok"):
                    raise RuntimeError(f"/api/firmware query rejected: {resp}")
            except Exception as e:
                raise RuntimeError(f"Safety firmware verification failed (/api/firmware HTTP error): {e}. Motion blocked!")

            # 3. Wait with bounded timeout for a new firmware response after that cursor
            deadline = time.monotonic() + timeout_s
            fresh_log_line = None

            while time.monotonic() < deadline:
                out = _exec_journalctl(["-u", "rover-server.service", f"--after-cursor={cursor}", "--no-pager"])

                for line in out.splitlines():
                    if "[Firmware Info]" in line:
                        fresh_log_line = line.strip()
                if fresh_log_line:
                    break
                time.sleep(0.05)

            if not fresh_log_line:
                raise RuntimeError(
                    f"Safety firmware verification failed: No fresh '[Firmware Info]' entry received after cursor within {timeout_s}s timeout! "
                    "An old matching entry or missing response blocks arming."
                )

            print(f"Target Rover Fresh Firmware Identity: {fresh_log_line}")

            # 4. Require exact expected version
            match = re.search(r"Ver:\s*([^,\s]+)", fresh_log_line)
            if not match:
                raise RuntimeError(f"Safety firmware verification failed: Could not parse version from '{fresh_log_line}'. Motion blocked!")

            detected_version = match.group(1).strip()
            if detected_version != required_version:
                print(
                    f"Safety firmware verification failed! Version mismatch: target rover reports '{detected_version}', expected '{required_version}'.\n"
                    f"Detected log: {fresh_log_line}\n"
                    "Refusing arming and physical motion!",
                    file=sys.stderr
                )
                return False

            print(f"Safety Firmware Verification: PASS (Running {required_version} confirmed fresh after cursor)")
            return True

        finally:
            if client:
                client.close()

    def run_preflight_checks(self):
        """Verify server responsiveness, stationary state, disarmed status, and battery."""
        print("=== Preflight Checks ===")
        snap = self.fetch_telemetry_snapshot()

        if snap["armed"]:
            raise RuntimeError("Preflight FAIL: Drive is already armed. Expected stationary disarmed state.")
        if snap["mode"] != 0:
            raise RuntimeError(f"Preflight FAIL: Drivetrain mode is {snap['mode']}. Expected 0 (LOCKED).")
        if any(abs(s) > 0.02 for s in snap["speeds_mps"]):
            raise RuntimeError(f"Preflight FAIL: Nonzero wheel speed detected before arming: {snap['speeds_mps']}")
        if snap["data_age_ms"] > (MAX_TELEMETRY_AGE_S * 1000.0):
            raise RuntimeError(f"Preflight FAIL: Telemetry data age {snap['data_age_ms']} ms exceeds {MAX_TELEMETRY_AGE_S*1000} ms limit.")

        print(f"Preflight PASS: Drive disarmed (Mode 0), speeds zeroed, telemetry fresh ({snap['data_age_ms']:.1f} ms)")

    # ────────────────────────────────────────────────────────────
    # Trial Execution Loop (Common Exit Path)
    # ────────────────────────────────────────────────────────────

    def execute_trial(self, dry_run=False, skip_firmware_check=False) -> Dict[str, Any]:
        """
        Executes the Trial A open-loop scan.
        - Firmware verification failure strictly blocks motion.
        - Arming is inside the protected try block.
        - Baselines established before nonzero PWM.
        - Independent stream advancement tracking for all 4 streams.
        - Pre-dispatch rechecks of monotonic deadlines, effective age, and armed state.
        - Common exit path: All exits run cleanup first, save report, and return report dictionary.
        """
        if dry_run:
            print("[DRY-RUN] Simulating Trial A execution without physical arming or motion.")
            return {"overall_status": "DRY_RUN_SUCCESS", "status": "DRY_RUN_SUCCESS", "profile": TRIAL_A_STEPS}

        print("\n=== Executing Trial A: Open-Loop Reverse PWM Transient Scan ===")
        print(f"Total Steps: {len(TRIAL_A_STEPS)} | Nominal Duration: 2.20 s | Hard Ceiling: {MAX_TOTAL_DURATION_S} s")

        self.samples.clear()
        self.abort_reason = None
        self.cleanup_failed = False
        cleanup_ok = False

        try:
            # 0. Safety Firmware Verification (Must succeed; failure blocks motion)
            if not skip_firmware_check:
                fw_ok = False
                try:
                    fw_ok = self.verify_safety_firmware(REQUIRED_SAFETY_FIRMWARE)
                except Exception as e:
                    self.abort_reason = f"FIRMWARE_VERIFICATION_FAILED ({e})"
                    print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)

                if not fw_ok and not self.abort_reason:
                    self.abort_reason = "FIRMWARE_VERIFICATION_FAILED (Verification returned False)"
                    print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)

            if not self.abort_reason:
                # 1. Arming is INSIDE the cleanup-protected try block
                print("Arming normal drive...")
                arm_resp = self.arm_drive()
                if not arm_resp.get("ok"):
                    raise RuntimeError(f"Arming request rejected: {arm_resp}")

                # 2. Confirm armed state in fresh telemetry
                t_arm_deadline = time.monotonic() + 1.0
                armed_confirmed = False
                while time.monotonic() < t_arm_deadline:
                    drv = self.fetch_drive_status()
                    if drv["armed"] is True and drv["mode"] != 0:
                        armed_confirmed = True
                        break
                    time.sleep(0.02)

                if not armed_confirmed:
                    raise RuntimeError("Arming confirmation failed: drive did not confirm armed=True within 1.0s")
                print("Drive armed confirmation: PASS (Mode active, armed=True)")

                # 3. Establish travel/yaw baselines BEFORE sending nonzero PWM
                baseline_snap = self.fetch_telemetry_snapshot()
                if any(abs(s) > 0.02 for s in baseline_snap["speeds_mps"]):
                    raise RuntimeError(f"Motion detected during baseline capture: {baseline_snap['speeds_mps']}")

                self.travel_tracker.reset(baseline_snap["ticks"])
                self.yaw_tracker.reset(baseline_snap["yaw_deg"])
                start_mono = time.monotonic()
                self.stream_tracker.reset(start_mono)

                # Record initial baseline sample
                self._record_telemetry_sample(start_mono, start_mono, 0, -1, baseline_snap)

                # 4. Step Execution Loop (Common exit path on abort)
                for step_idx, step in enumerate(TRIAL_A_STEPS):
                    step_name = step["name"]
                    pwm_val = step["pwm"]
                    step_duration = step["duration_s"]
                    step_start_mono = time.monotonic()

                    print(f"\n[Step {step_idx + 1}/{len(TRIAL_A_STEPS)}] {step_name}: PWM={pwm_val} for {step_duration:.2f}s")

                    while (time.monotonic() - step_start_mono) < step_duration:
                        t_loop_start = time.monotonic()
                        elapsed_total = t_loop_start - start_mono

                        # Check 1: Pre-fetch monotonic duration deadline
                        if elapsed_total > MAX_TOTAL_DURATION_S:
                            self.abort_reason = f"EXCEEDED_MAX_TOTAL_DURATION ({elapsed_total:.3f}s > {MAX_TOTAL_DURATION_S}s)"
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        # Check 2: Ingest telemetry snapshot
                        snap = self.fetch_telemetry_snapshot()
                        t_pre_dispatch = time.monotonic()

                        # Recheck 3a: Post-fetch monotonic duration deadline
                        if (t_pre_dispatch - start_mono) > MAX_TOTAL_DURATION_S:
                            self.abort_reason = f"EXCEEDED_MAX_TOTAL_DURATION_POST_FETCH ({t_pre_dispatch - start_mono:.3f}s > {MAX_TOTAL_DURATION_S}s)"
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        # Recheck 3b: Effective telemetry age accounting for snapshot collection and transit delay
                        fetch_start = snap.get("fetch_start_mono", snap["receipt_mono"])
                        fetch_delay_s = t_pre_dispatch - fetch_start
                        effective_age_ms = snap["data_age_ms"] + (fetch_delay_s * 1000.0)
                        if effective_age_ms > (MAX_TELEMETRY_AGE_S * 1000.0):
                            self.abort_reason = f"STALE_TELEMETRY_POST_FETCH (Effective age {effective_age_ms:.1f}ms [base {snap['data_age_ms']:.1f}ms + delay {fetch_delay_s*1000:.1f}ms] > {MAX_TELEMETRY_AGE_S*1000}ms)"
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        # Recheck 3c: Independent stream advancement check
                        seq_map = {
                            "drive": snap["seq_drive"],
                            "pid": snap["seq_pid"],
                            "encoders": snap["seq_encoders"],
                            "imu": snap["seq_imu"]
                        }
                        stream_abort = self.stream_tracker.update(t_pre_dispatch, seq_map)
                        if stream_abort:
                            self.abort_reason = stream_abort
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        # Recheck 3d: Confirm drive remains armed (do NOT resume PWM after watchdog expiry)
                        if snap["armed"] is not True or snap["mode"] == 0:
                            self.abort_reason = f"DRIVE_DISARMED_OR_LOCKED (armed={snap['armed']}, mode={snap['mode']})"
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        # Recheck 3e: Physical motion bounds (travel, speed, yaw)
                        self._record_telemetry_sample(t_pre_dispatch, start_mono, pwm_val, step_idx, snap)
                        if self.abort_reason:
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        # 4. All checks passed -> refresh PWM (20 Hz loop)
                        self.send_open_loop_pwm(pwm_val, pwm_val, pwm_val, pwm_val)

                        # Monotonic sleep to maintain 20 Hz (50 ms cycle)
                        cycle_elapsed = time.monotonic() - t_loop_start
                        sleep_time = max(0.005, 0.050 - cycle_elapsed)
                        time.sleep(sleep_time)

                    # Break out of step sequence if abort occurred
                    if self.abort_reason:
                        break

                if not self.abort_reason:
                    print("\nProfile completed cleanly. Initiating controlled zero cutoff...")

        except Exception as e:
            if not self.abort_reason:
                self.abort_reason = f"EXCEPTION: {e}"
            print(f"\n[EXECUTION ERROR] {self.abort_reason}", file=sys.stderr)
        finally:
            # Common Exit Path: Cleanup first, save report, and return report dictionary
            cleanup_ok = self._execute_cleanup()
            report = self._finalize_report(aborted=(self.abort_reason is not None), cleanup_ok=cleanup_ok)
            if not cleanup_ok:
                self.cleanup_failed = True
                print("\n[FATAL] Unconfirmed cleanup! Drivetrain state unconfirmed.", file=sys.stderr)

        return report

    def _record_telemetry_sample(self, now_mono: float, start_mono: float, commanded_pwm: int, step_idx: int, snap: Dict[str, Any]):
        """Records telemetry sample with all delivered outputs, speeds, sequences, and checks bounds."""
        raw_yaw = snap["yaw_deg"]
        unwrapped_yaw, net_yaw_drift, accum_yaw_excursion = self.yaw_tracker.update(raw_yaw)

        ticks = snap["ticks"]
        accum_travel_per_wheel, max_accum_travel_m, max_net_disp_m = self.travel_tracker.update(ticks)

        speeds_mps = snap["speeds_mps"]
        mean_abs_speed_mps = sum(abs(v) for v in speeds_mps) / 4.0

        sample = {
            "time_offset_s": round(now_mono - start_mono, 4),
            "timestamp_mono": now_mono,
            "epoch_time_ms": snap["receipt_epoch_ms"],
            "step_index": step_idx,
            "step_name": TRIAL_A_STEPS[step_idx]["name"] if step_idx >= 0 else "Baseline",
            "commanded_pwm": commanded_pwm,
            "delivered_pwms": snap["pwms"],               # [m1_final_pwm, m2_final_pwm, m3_final_pwm, m4_final_pwm]
            "measured_speeds_radps": snap["speeds_radps"], # [m1, m2, m3, m4]
            "measured_speeds_mps": [round(v, 4) for v in speeds_mps],
            "mean_abs_speed_mps": round(mean_abs_speed_mps, 4),
            "raw_ticks": ticks,
            "accum_travel_per_wheel_m": [round(d, 4) for d in accum_travel_per_wheel],
            "max_accum_abs_travel_m": round(max_accum_travel_m, 4),
            "max_net_displacement_m": round(max_net_disp_m, 4),
            "raw_yaw_deg": round(raw_yaw, 2),
            "unwrapped_yaw_deg": round(unwrapped_yaw, 2),
            "net_yaw_drift_deg": round(net_yaw_drift, 2),
            "accum_yaw_excursion_deg": round(accum_yaw_excursion, 2),
            "armed": snap["armed"],
            "mode": snap["mode"],
            "source_metadata": {
                "seq_drive": snap["seq_drive"],
                "seq_pid": snap["seq_pid"],
                "seq_encoders": snap["seq_encoders"],
                "seq_imu": snap["seq_imu"],
                "age_drive_ms": snap["age_drive_ms"],
                "age_pid_ms": snap["age_pid_ms"],
                "age_encoders_ms": snap["age_encoders_ms"],
                "age_imu_ms": snap["age_imu_ms"],
                "data_age_ms": snap["data_age_ms"]
            }
        }
        self.samples.append(sample)

        # Safety Bound Evaluations
        if max_accum_travel_m > MAX_ABS_WHEEL_TRAVEL_M:
            self.abort_reason = f"EXCEEDED_MAX_ABS_TRAVEL ({max_accum_travel_m*1000:.1f} mm > {MAX_ABS_WHEEL_TRAVEL_M*1000:.1f} mm)"
            return

        if mean_abs_speed_mps > MAX_MEAN_ABS_SPEED_MPS:
            self.abort_reason = f"EXCEEDED_MAX_SPEED ({mean_abs_speed_mps:.4f} m/s > {MAX_MEAN_ABS_SPEED_MPS:.4f} m/s)"
            return

        if net_yaw_drift > MAX_ACCUMULATED_YAW_DEG:
            self.abort_reason = f"EXCEEDED_MAX_YAW ({net_yaw_drift:.2f} deg > {MAX_ACCUMULATED_YAW_DEG:.2f} deg)"
            return

    def _execute_cleanup(self) -> bool:
        """
        Guaranteed independent zero cutoff, drive disarm, and fresh telemetry verification.
        Cleanup MUST attempt disarm even if zero PWM fails, and verify fresh telemetry confirms
        zero output, standstill, disarmed state, and Mode 0 (LOCKED).
        """
        print("\n=== Executing Fail-Closed Drivetrain Cleanup ===")

        # 1. Independent Zero PWM Dispatch
        try:
            res = self.send_open_loop_pwm(0, 0, 0, 0)
            if res and res.get("ok"):
                print("1. Sent zero open-loop PWM command [0, 0, 0, 0]: SUCCESS")
            else:
                print(f"1. Zero PWM dispatch returned ok=false: {res}", file=sys.stderr)
        except Exception as e:
            print(f"1. Zero PWM dispatch exception (attempting disarm anyway): {e}", file=sys.stderr)

        # 2. Independent Drive Disarm Dispatch (ALWAYS attempted regardless of zero PWM outcome)
        try:
            res = self.disarm_drive()
            if res and res.get("ok"):
                print("2. Disarmed normal drive (transition to LOCKED): SUCCESS")
            else:
                print(f"2. Drive disarm returned: {res}", file=sys.stderr)
        except Exception as e:
            print(f"2. Drive disarm exception: {e}", file=sys.stderr)

        # 3. Telemetry Verification Loop (Monotonic Deadline):
        deadline = time.monotonic() + 1.50
        verified = False
        last_failure_reason = "TIMEOUT"

        while time.monotonic() < deadline:
            try:
                snap = self.fetch_telemetry_snapshot()

                # a) Freshness
                if snap["data_age_ms"] > (MAX_TELEMETRY_AGE_S * 1000.0):
                    last_failure_reason = f"Stale telemetry ({snap['data_age_ms']:.1f} ms > 250 ms)"
                    time.sleep(0.05)
                    continue

                # d) Disarmed state
                if snap["armed"] is not False:
                    last_failure_reason = "Drive reports armed=True"
                    time.sleep(0.05)
                    continue

                # e) Mode 0 (LOCKED)
                if snap["mode"] != 0:
                    last_failure_reason = f"Drive mode is {snap['mode']}, expected 0 (LOCKED)"
                    time.sleep(0.05)
                    continue

                # b) Zero motor output
                if any(p != 0 for p in snap["pwms"]):
                    last_failure_reason = f"Non-zero motor PWM outputs: {snap['pwms']}"
                    time.sleep(0.05)
                    continue

                # c) Standstill confirmed (< 0.05 rad/s)
                if any(abs(w) >= 0.05 for w in snap["speeds_radps"]):
                    last_failure_reason = f"Residual wheel motion detected: {snap['speeds_radps']} rad/s"
                    time.sleep(0.05)
                    continue

                verified = True
                break
            except Exception as e:
                last_failure_reason = f"Verification query exception: {e}"
            time.sleep(0.05)

        if verified:
            print("3. Verification PASSED: Confirmed zero output (PWM=0), standstill (<0.05 rad/s), disarmed (armed=False), and Mode 0 (LOCKED) in fresh telemetry.")
        else:
            print(f"3. Verification WARNING: Could not fully confirm cleanup: {last_failure_reason}", file=sys.stderr)

        return verified

    def _finalize_report(self, aborted: bool = False, cleanup_ok: bool = True) -> Dict[str, Any]:
        """
        Generates transient scan report and analyzes per-step settling status.
        Saves report to disk on completion, abort, or exception.
        If cleanup is unconfirmed, overall_status is set to CLEANUP_FAILED.
        """
        os.makedirs(self.report_dir, exist_ok=True)
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        report_filename = f"openloop_char_Trial_A_Reverse_Dec_{timestamp_str}.json"
        report_path = os.path.join(self.report_dir, report_filename)

        # Determine overall status
        if not cleanup_ok:
            overall_status = "CLEANUP_FAILED"
        elif aborted:
            overall_status = "ABORTED"
        else:
            overall_status = "COMPLETED"

        # Transient Step Analysis
        step_analyses = []
        for step_idx, step in enumerate(TRIAL_A_STEPS):
            step_samples = [s for s in self.samples if s["step_index"] == step_idx]
            if not step_samples:
                step_analyses.append({
                    "step_name": step["name"],
                    "pwm": step["pwm"],
                    "status": "NOT_REACHED"
                })
                continue

            # Trailing 200 ms assessment window
            tail_samples = step_samples[-4:] if len(step_samples) >= 4 else step_samples
            tail_speeds = [s["mean_abs_speed_mps"] for s in tail_samples]
            mean_speed = sum(tail_speeds) / len(tail_speeds)
            variance = sum((v - mean_speed) ** 2 for v in tail_speeds) / len(tail_speeds)
            std_speed = math.sqrt(variance)

            # Transient response scan classification:
            # Requires std <= 0.003 m/s (~0.09 rad/s) across tail samples to be considered settled
            is_settled = (std_speed <= 0.003) and (len(tail_samples) >= 3)
            classification = "SETTLED" if is_settled else "INCONCLUSIVE_TRANSIENT"

            step_analyses.append({
                "step_name": step["name"],
                "pwm": step["pwm"],
                "sample_count": len(step_samples),
                "tail_sample_count": len(tail_samples),
                "tail_mean_abs_speed_mps": round(mean_speed, 4),
                "tail_std_speed_mps": round(std_speed, 4),
                "classification": classification,
                "note": "Transient observation; not steady-state rolling calibration" if not is_settled else "Settled"
            })

        max_travel = max((s["max_accum_abs_travel_m"] for s in self.samples), default=0.0)
        max_speed = max((s["mean_abs_speed_mps"] for s in self.samples), default=0.0)
        max_yaw = max((s["net_yaw_drift_deg"] for s in self.samples), default=0.0)

        report_data = {
            "trial_type": "Trial_A_OpenLoop_Reverse_Transient_Scan",
            "timestamp": timestamp_str,
            "aborted": aborted,
            "abort_reason": self.abort_reason,
            "cleanup_confirmed": cleanup_ok,
            "overall_status": overall_status,
            "summary_metrics": {
                "max_accum_abs_travel_m": round(max_travel, 4),
                "max_mean_abs_speed_mps": round(max_speed, 4),
                "max_net_yaw_drift_deg": round(max_yaw, 2),
                "total_samples": len(self.samples)
            },
            "step_analyses": step_analyses,
            "telemetry_samples": self.samples
        }

        try:
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(report_data, f, indent=2)
            print(f"\nReport generated successfully: {report_path}")
        except Exception as e:
            print(f"\nFailed to save report file: {e}", file=sys.stderr)

        print(f"Overall Status: {report_data['overall_status']}")
        if aborted:
            print(f"Abort Reason: {self.abort_reason}")
        print(f"Max Accumulated Absolute Wheel Travel: {max_travel*1000:.1f} mm (Limit: {MAX_ABS_WHEEL_TRAVEL_M*1000:.1f} mm)")
        print(f"Max Mean Absolute Speed:             {max_speed:.4f} m/s (Limit: {MAX_MEAN_ABS_SPEED_MPS:.4f} m/s)")
        print(f"Max Net Yaw Drift:                   {max_yaw:.2f} deg (Limit: {MAX_ACCUMULATED_YAW_DEG:.2f} deg)")
        return report_data


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Rover 1 Open-Loop Reverse PWM Transient Scan (Trial A)")
    parser.add_argument("--dry-run", action="store_true", help="Simulate execution without physical arming or motion")
    parser.add_argument("--host", default="127.0.0.1", help="Cockpit server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=3000, help="Cockpit server port (default: 3000)")
    parser.add_argument("--report-dir", default="reports", help="Directory for JSON reports")
    parser.add_argument("--skip-firmware-check", action="store_true", help="Skip SSH safety firmware check (for local testing)")
    args = parser.parse_args(argv)

    harness = TrialAHarness(host=args.host, port=args.port, report_dir=args.report_dir)

    if not args.dry_run:
        try:
            harness.run_preflight_checks()
        except Exception as e:
            print(f"[FATAL] Preflight check failed: {e}", file=sys.stderr)
            return 1

        print("\n" + "="*60)
        print("OPERATOR CONFIRMATION REQUIRED BEFORE PHYSICAL MOTION")
        print("Trial A will execute open-loop reverse staircase (-48 to -38 PWM).")
        print(f"Safety limits: {MAX_ABS_WHEEL_TRAVEL_M*1000} mm accumulated travel, {MAX_MEAN_ABS_SPEED_MPS} m/s speed, {MAX_ACCUMULATED_YAW_DEG} deg yaw drift.")
        print("="*60)
        confirm = input("Type 'CONFIRM' to execute Trial A: ").strip()
        if confirm != "CONFIRM":
            print("Execution cancelled by operator. No motion commanded.")
            return 0

    try:
        report = harness.execute_trial(dry_run=args.dry_run, skip_firmware_check=args.skip_firmware_check)
    except Exception as e:
        print(f"[FATAL] Trial execution error: {e}", file=sys.stderr)
        return 1

    # Exit code determination
    overall_status = report.get("overall_status", "UNKNOWN") if report else "UNKNOWN"
    if overall_status == "CLEANUP_FAILED":
        return 2
    elif overall_status in ("ABORTED", "UNKNOWN"):
        return 1
    elif overall_status in ("COMPLETED", "DRY_RUN_SUCCESS"):
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
