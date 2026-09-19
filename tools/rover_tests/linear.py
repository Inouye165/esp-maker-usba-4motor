"""
tools.rover_tests.linear - Linear Motion Specification and Wheel Kinematics Logic

Authoritative linear kinematics and symmetry enforcement per docs/EXACT_MOTION_CONTRACT.md:
- Forward (+Distance / +vx): All four wheels (M1 LF, M2 RF, M3 LR, M4 RR) positive.
- Reverse (-Distance / -vx): All four wheels (M1 LF, M2 RF, M3 LR, M4 RR) negative.
- Equal magnitude across all 4 wheels for pure straight-line travel.
- Uses effective wheel diameter 0.06695 m (radius 0.033475 m) and 1974.1667 ticks/rev.
"""

import math
from dataclasses import dataclass
from typing import Dict, Any, Tuple, Optional

WHEEL_DIAMETER_M = 0.06695
EFFECTIVE_WHEEL_DIAMETER_M = WHEEL_DIAMETER_M
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0  # 0.033475 m
TICKS_PER_REV = 1974.1666666667
TICKS_PER_REVOLUTION = TICKS_PER_REV
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER_M) / TICKS_PER_REV  # ~0.00010654 m/tick


class LinearConfigurationException(Exception):
    """Raised on invalid linear test parameter combinations or kinematics faults."""
    pass


@dataclass
class LinearParameters:
    distance: Optional[float] = None
    distance_m: Optional[float] = None   # Alias for distance
    direction: str = "forward"  # "forward" or "reverse"
    trials: int = 1
    repetitions: Optional[int] = None
    speed_mps: float = 0.20              # Constant requested linear speed (m/s)
    max_linear_speed: float = 0.20       # Backward-compatible alias for speed_mps
    creep_linear_speed: float = 0.05     # Legacy parameter (not used in constant-velocity characterization)
    creep_threshold_m: float = 0.150     # Legacy parameter (not used in constant-velocity characterization)
    settle_seconds: float = 2.0          # s
    distance_tolerance_m: float = 0.05   # m (50 mm tolerance)
    inter_trial_approval: bool = False
    dry_run: bool = False
    enable_balancing: bool = False
    enable_braking: bool = False
    clear_faults: bool = False
    report_directory: str = "reports"
    host: str = "127.0.0.1"
    port: int = 3000

    def __post_init__(self):
        if self.distance is None and self.distance_m is not None:
            self.distance = self.distance_m
        elif self.distance is not None and self.distance_m is None:
            self.distance_m = self.distance

        if self.speed_mps != 0.20 and self.max_linear_speed == 0.20:
            self.max_linear_speed = self.speed_mps
        elif self.max_linear_speed != 0.20 and self.speed_mps == 0.20:
            self.speed_mps = self.max_linear_speed

    def validate(self):
        if self.distance is None or self.distance <= 0.0:
            raise LinearConfigurationException(f"distance must be positive, got {self.distance}")

        if self.repetitions is not None:
            if not (1 <= self.repetitions <= 8):
                raise LinearConfigurationException(f"Repetitions must be between 1 and 8, got {self.repetitions}")
            self.trials = self.repetitions

        norm_dir = self.direction.lower()
        if norm_dir not in ("forward", "reverse", "fwd", "rev"):
            raise LinearConfigurationException(f"Direction must be 'forward' or 'reverse', got '{self.direction}'")
        if not (1 <= self.trials <= 8):
            raise LinearConfigurationException(f"Trials/repetitions must be between 1 and 8, got {self.trials}")
        if self.max_linear_speed <= 0.0:
            raise LinearConfigurationException(f"max_linear_speed must be positive, got {self.max_linear_speed}")
        if self.settle_seconds < 0.5:
            raise LinearConfigurationException(f"settle_seconds must be >= 0.5, got {self.settle_seconds}")
        if self.distance_tolerance_m <= 0.0:
            raise LinearConfigurationException(f"distance_tolerance_m must be positive, got {self.distance_tolerance_m}")

    @property
    def is_forward(self) -> bool:
        return self.direction.lower() in ("forward", "fwd")

    @property
    def signed_target_m(self) -> float:
        """
        Signed target distance in meters:
        Positive (+distance) for forward, negative (-distance) for reverse.
        """
        return abs(self.distance) if self.is_forward else -abs(self.distance)

    @property
    def signed_target_distance_m(self) -> float:
        """Alias for signed_target_m."""
        return self.signed_target_m


def compute_linear_wheel_speed_targets(
    vx_cmd: float,
    wheel_radius_m: float = WHEEL_RADIUS_M
) -> Dict[str, float]:
    """
    Computes commanded individual wheel speed targets (rad/s) for pure linear travel:
    - Pure linear: wz = 0.0
    - All four wheels receive the same sign and magnitude: w = vx / R_wheel
    """
    if abs(vx_cmd) < 1e-6:
        return {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

    wheel_radps = vx_cmd / wheel_radius_m
    return {
        "m1": wheel_radps,  # Left Front
        "m2": wheel_radps,  # Right Front
        "m3": wheel_radps,  # Left Rear
        "m4": wheel_radps   # Right Rear
    }


def verify_linear_wheel_command_symmetry(wheel_targets: Dict[str, float]) -> Tuple[bool, str]:
    """
    Verifies that all 4 wheels receive equal intended speed magnitudes and same direction sign.
    """
    m1 = wheel_targets.get("m1", 0.0)
    m2 = wheel_targets.get("m2", 0.0)
    m3 = wheel_targets.get("m3", 0.0)
    m4 = wheel_targets.get("m4", 0.0)

    # Check all zero
    if all(abs(v) < 1e-4 for v in (m1, m2, m3, m4)):
        return True, "Symmetric zero commanded"

    # Check all non-zero share same sign
    vals = [m1, m2, m3, m4]
    signs_positive = [v > 1e-4 for v in vals]
    signs_negative = [v < -1e-4 for v in vals]

    if not (all(signs_positive) or all(signs_negative)):
        return False, f"Wheels disagree in direction: M1={m1:.3f}, M2={m2:.3f}, M3={m3:.3f}, M4={m4:.3f}"

    # Check magnitude agreement across all four wheels
    mags = [abs(v) for v in vals]
    mean_mag = sum(mags) / 4.0
    max_dev = max(abs(m - mean_mag) for m in mags)
    if max_dev > 1e-3:
        return False, f"Commanded linear wheel speed magnitude asymmetry: max dev {max_dev:.4f} rad/s"

    return True, "Equal magnitude uniform polarity confirmed"


def ticks_to_meters(ticks: float) -> float:
    """Converts encoder tick count to linear distance in meters."""
    return ticks * METERS_PER_TICK


def meters_to_ticks(meters: float) -> float:
    """Converts linear distance in meters to encoder tick count."""
    return meters / METERS_PER_TICK


def compute_linear_distance_from_ticks(
    start_ticks: Dict[str, int],
    current_ticks: Dict[str, int],
    direction: str = "forward"
) -> float:
    """
    Computes average forward travel distance in meters from start to current encoder ticks across all wheels.
    """
    diffs = []
    for k in ("m1", "m2", "m3", "m4"):
        if k in current_ticks and k in start_ticks:
            diffs.append(current_ticks[k] - start_ticks[k])
    if not diffs:
        return 0.0
    avg_ticks = sum(diffs) / len(diffs)
    dist = ticks_to_meters(avg_ticks)
    if direction.lower() in ("reverse", "rev") and dist < 0:
        return abs(dist)
    return dist
