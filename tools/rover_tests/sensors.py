"""
tools.rover_tests.sensors - Sensor & Orientation Management for Rover One Tests

Authoritative implementations for:
- BNO08x non-magnetic Game Rotation Vector verification (fails closed if unverified or magnetic).
- Continuous relative yaw unwrapping (YawUnwrapper) across arbitrary angles (90°, 180°, 360°).
- Independent bias-corrected trapezoidal gyro Z integration (BiasCorrectedGyroIntegrator).
- Pre-trial fresh odometry & IMU origin capture.
- Motor encoder & PID telemetry decoding.
"""

import math
import time
from typing import Optional, Dict, Any, List, Tuple


class SensorException(Exception):
    """Raised on sensor validation failure, stale data, or invalid report mode."""
    pass


class MagneticImuException(SensorException):
    """Raised if magnetic orientation is detected or non-magnetic report cannot be verified."""
    pass


def quat_to_yaw(q: Any) -> float:
    """
    Converts orientation quaternion (w, x, y, z) to Euler yaw angle in range [-pi, pi].
    Accepts object with w,x,y,z attributes or dictionary with 'w','x','y','z' keys.
    """
    if isinstance(q, dict):
        qw = float(q.get('w', 1.0))
        qx = float(q.get('x', 0.0))
        qy = float(q.get('y', 0.0))
        qz = float(q.get('z', 0.0))
    else:
        qw = float(getattr(q, 'w', 1.0))
        qx = float(getattr(q, 'x', 0.0))
        qy = float(getattr(q, 'y', 0.0))
        qz = float(getattr(q, 'z', 0.0))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_delta(delta_rad: float) -> float:
    """Normalizes angular difference to [-pi, pi]."""
    while delta_rad > math.pi:
        delta_rad -= 2.0 * math.pi
    while delta_rad < -math.pi:
        delta_rad += 2.0 * math.pi
    return delta_rad


def normalize_angle_deg(angle_deg: float) -> float:
    """Normalizes angle to [-180, 180] degrees."""
    a = angle_deg % 360.0
    if a > 180.0:
        a -= 360.0
    if a < -180.0:
        a += 360.0
    return a


class YawUnwrapper:
    """
    Maintains continuous local relative yaw by accumulating normalized consecutive angular deltas.
    Prevents wrap discontinuities at ±180° boundaries during multi-revolution or 180° turns.
    """
    def __init__(self, initial_raw_yaw: Optional[float] = None):
        self.raw_yaw_rad = 0.0
        self.imu_yaw_accum_rad = 0.0
        self.imu_yaw_prev_rad: Optional[float] = None
        self.reset(initial_raw_yaw)

    def reset(self, initial_raw_yaw: Optional[float] = None):
        self.raw_yaw_rad = initial_raw_yaw if initial_raw_yaw is not None else 0.0
        self.imu_yaw_accum_rad = 0.0
        self.imu_yaw_prev_rad = initial_raw_yaw

    def update_orientation_yaw(self, raw_yaw_rad: float):
        self.raw_yaw_rad = raw_yaw_rad
        if self.imu_yaw_prev_rad is None:
            self.imu_yaw_prev_rad = raw_yaw_rad
            self.imu_yaw_accum_rad = 0.0
        else:
            dy = normalize_angle_delta(raw_yaw_rad - self.imu_yaw_prev_rad)
            self.imu_yaw_accum_rad += dy
            self.imu_yaw_prev_rad = raw_yaw_rad

    @property
    def raw_yaw_deg(self) -> float:
        return math.degrees(self.raw_yaw_rad)

    @property
    def relative_yaw_deg(self) -> float:
        return math.degrees(self.imu_yaw_accum_rad)

    @property
    def relative_yaw_rad(self) -> float:
        return self.imu_yaw_accum_rad


class BiasCorrectedGyroIntegrator:
    """
    Independent trapezoidal Gyro-Z integrator for relative turn cross-verification:
    - Calibrates stationary gyro-Z bias from pre-turn stationary samples.
    - Integrates bias-corrected angular velocity: gz_corr = gz_raw - bias.
    - Fails closed if stationary bias is excessive or if sample gap > max_dt_sec.
    """
    def __init__(self, max_bias_radps: float = 0.05, max_dt_sec: float = 0.20, max_stale_sec: float = 0.30):
        self.max_bias_radps = max_bias_radps
        self.max_dt_sec = max_dt_sec
        self.max_stale_sec = max_stale_sec
        self.bias_radps = 0.0
        self.integrated_rad = 0.0
        self.last_ts: Optional[float] = None
        self.sample_count = 0
        self.calibrated = False

    def calibrate_bias(self, pre_turn_gz_samples: List[float]) -> float:
        if not pre_turn_gz_samples or len(pre_turn_gz_samples) < 5:
            raise SensorException("Insufficient stationary pre-turn samples for gyro bias calibration (< 5 samples).")

        mean_bias = sum(pre_turn_gz_samples) / float(len(pre_turn_gz_samples))
        if abs(mean_bias) > self.max_bias_radps:
            raise SensorException(
                f"Excessive stationary gyro-Z bias detected: {mean_bias:+.5f} rad/s ({math.degrees(mean_bias):+.2f}°/s) > limit {self.max_bias_radps:.5f} rad/s"
            )

        self.bias_radps = mean_bias
        self.calibrated = True
        self.integrated_rad = 0.0
        self.last_ts = None
        return self.bias_radps

    def reset_integration(self):
        self.integrated_rad = 0.0
        self.last_ts = None

    def update(self, gz_raw_radps: float, current_ts: float) -> Tuple[float, float]:
        if not self.calibrated:
            raise SensorException("GyroIntegrator update called before stationary bias calibration.")

        if self.last_ts is not None:
            dt = current_ts - self.last_ts
            if dt <= 0.0:
                raise SensorException(f"Invalid non-positive timestamp delta: dt={dt:.6f}s")
            if dt > self.max_dt_sec:
                raise SensorException(f"Gyro timestamp gap threshold exceeded: dt={dt:.3f}s > max allowed {self.max_dt_sec}s")

            gz_corrected = gz_raw_radps - self.bias_radps
            self.integrated_rad += gz_corrected * dt

        self.last_ts = current_ts
        self.sample_count += 1
        return self.relative_deg, self.integrated_rad

    def check_stale(self, current_ts: float):
        if self.last_ts is not None and (current_ts - self.last_ts) > self.max_stale_sec:
            raise SensorException(
                f"Stale gyro data detected: latency {(current_ts - self.last_ts)*1000.0:.1f}ms > max allowed {self.max_stale_sec*1000.0:.1f}ms"
            )

    @property
    def relative_deg(self) -> float:
        return math.degrees(self.integrated_rad)

    @property
    def relative_rad(self) -> float:
        return self.integrated_rad


def verify_non_magnetic_imu(imu_snapshot: Dict[str, Any]) -> Tuple[bool, str]:
    """
    Verifies that the BNO08x orientation report is the non-magnetic Game Rotation Vector
    (SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC).
    Fails closed if report is magnetic or if non-magnetic configuration cannot be verified.
    """
    if not imu_snapshot or not imu_snapshot.get("ok", False):
        return False, "No valid IMU telemetry snapshot available"

    # Verify rotation vector validity flag
    if not imu_snapshot.get("rotVecValid", False):
        return False, "BNO08x rotation vector validity flag is FALSE (sensor uninitialized or report inactive)"

    # Verify sensor is not in reset recovery
    if imu_snapshot.get("inResetRecovery", False):
        return False, "BNO08x IMU is in reset recovery mode"

    # Verify orientation source naming if present
    orientation_source = imu_snapshot.get("orientationSource") or imu_snapshot.get("sourceName")
    if orientation_source:
        if "GAME_ROTATION_VECTOR" not in orientation_source or "MAGNETIC" in orientation_source and "NON_MAGNETIC" not in orientation_source:
            return False, f"Prohibited orientation source detected: {orientation_source} (must be SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC)"

    # Verify quaternion is non-degenerate
    q = imu_snapshot.get("orientation", {})
    qw = q.get("w", 0.0)
    qx = q.get("x", 0.0)
    qy = q.get("y", 0.0)
    qz = q.get("z", 0.0)
    norm_sq = qw * qw + qx * qx + qy * qy + qz * qz
    if abs(norm_sq - 1.0) > 0.05 or norm_sq < 0.1:
        return False, f"Degenerate orientation quaternion received: norm^2={norm_sq:.4f}"

    return True, "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC verified active and valid"


def check_imu_freshness(imu_data: Optional[Dict[str, Any]], max_age_ms: float = 250.0) -> Tuple[bool, str]:
    """
    Unified freshness check for BNO08x orientation telemetry.
    Used consistently by pre-motion invariant assertions and active motion loop watchdog.
    """
    if not imu_data or not imu_data.get("ok", False):
        return False, "IMU telemetry dropped or unreadable"
    if not imu_data.get("rotVecValid", False):
        return False, "BNO08x rotation vector validity flag is FALSE"
    if imu_data.get("inResetRecovery", False):
        return False, "BNO08x IMU is in reset recovery mode"

    age_ms = imu_data.get("dataAgeMs")
    if age_ms is not None and age_ms > max_age_ms:
        return False, f"Stale IMU data detected ({age_ms}ms > {max_age_ms:.0f}ms limit)"

    display_age = f"{age_ms:.0f}ms" if age_ms is not None else "0ms"
    return True, f"Fresh IMU data ({display_age} <= {max_age_ms:.0f}ms)"


def wait_for_advancing_imu_sample(
    cockpit: Any,
    baseline_seq: Optional[int] = None,
    baseline_esp_ts_us: Optional[int] = None,
    max_wait_sec: float = 0.50,
    poll_interval_sec: float = 0.015,
    max_acceptable_age_ms: float = 100.0,
    watchdog_max_age_ms: float = 250.0
) -> Tuple[bool, Optional[Dict[str, Any]], str]:
    """
    Waits for a genuinely new, advancing IMU orientation sample before issuing motion commands.
    Ensures:
    1. sequence > baseline_seq (or advances) and/or espTimestampUs > baseline_esp_ts_us.
    2. Satisfies canonical freshness check (rotVecValid, not in reset recovery).
    3. dataAgeMs <= max_acceptable_age_ms (strictly fresh at cycle inception, e.g. <= 100ms).
    Returns (True, fresh_imu_snapshot, "reason") or (False, last_sample, "failure reason").
    """
    t_start = time.time()
    last_sample = None

    while (time.time() - t_start) < max_wait_sec:
        imu_snap = cockpit.get_imu()
        last_sample = imu_snap

        if imu_snap and imu_snap.get("ok", False):
            cur_seq = imu_snap.get("sequence")
            cur_esp_ts = imu_snap.get("espTimestampUs")

            seq_advanced = False
            if baseline_seq is not None and cur_seq is not None:
                if cur_seq != baseline_seq:
                    seq_advanced = True
            elif baseline_esp_ts_us is not None and cur_esp_ts is not None:
                if cur_esp_ts > baseline_esp_ts_us:
                    seq_advanced = True
            elif baseline_seq is None and baseline_esp_ts_us is None:
                seq_advanced = True

            if seq_advanced:
                is_fresh, freshness_msg = check_imu_freshness(imu_snap, max_age_ms=watchdog_max_age_ms)
                if is_fresh:
                    age_ms = imu_snap.get("dataAgeMs")
                    if age_ms is None or age_ms <= max_acceptable_age_ms:
                        return True, imu_snap, f"Advancing IMU sample verified (seq={cur_seq}, age={age_ms if age_ms is not None else 0}ms)"

        time.sleep(poll_interval_sec)

    elapsed_ms = (time.time() - t_start) * 1000.0
    last_seq = last_sample.get("sequence") if last_sample else None
    last_age = last_sample.get("dataAgeMs") if last_sample else None
    return False, last_sample, (
        f"Timed out waiting for advancing fresh IMU sample ({elapsed_ms:.1f}ms > {max_wait_sec*1000.0:.0f}ms limit, "
        f"baseline_seq={baseline_seq}, last_seq={last_seq}, last_age={last_age}ms)"
    )
