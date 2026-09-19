"""
tools.rover_tests.turn - In-Place Turn Specification and Wheel Polarity Logic

Authoritative turn kinematics and polarity enforcement per docs/EXACT_MOTION_CONTRACT.md:
- Clockwise (CW / +Yaw): Left side (M1 LF, M3 LR) positive; Right side (M2 RF, M4 RR) negative.
- Counter-Clockwise (CCW / -Yaw): Left side (M1 LF, M3 LR) negative; Right side (M2 RF, M4 RR) positive.
- Strict opposite-side polarity and equal-magnitude symmetry verification.
"""

from dataclasses import dataclass, field
from typing import Dict, Any, Tuple, Optional


class TurnConfigurationException(Exception):
    """Raised on invalid turn parameter combinations or polarity faults."""
    pass


@dataclass
class TurnParameters:
    degrees: Optional[float] = None
    direction: str = "cw"  # "cw" or "ccw"
    trials: int = 1
    repetitions: Optional[int] = None
    max_angular_speed: float = 0.80      # rad/s
    creep_angular_speed: float = 0.20    # rad/s
    creep_threshold_deg: float = 30.0    # deg
    settle_seconds: float = 2.0          # s
    angle_tolerance_deg: float = 2.0     # deg
    inter_trial_approval: bool = False
    dry_run: bool = False
    enable_balancing: Optional[bool] = None
    enable_braking: Optional[bool] = None
    clear_faults: bool = False
    stopping_advance_deg: Optional[float] = None
    report_directory: str = "reports"
    host: str = "127.0.0.1"
    port: int = 3000

    def validate(self):
        if self.degrees is None or self.degrees <= 0.0:
            raise TurnConfigurationException(f"degrees must be positive, got {self.degrees}")

        if self.repetitions is not None:
            if not (1 <= self.repetitions <= 8):
                raise TurnConfigurationException(f"Repetitions must be between 1 and 8, got {self.repetitions}")
            self.trials = self.repetitions

        if self.stopping_advance_deg is None:
            # Production default enables dynamic braking; default advance is 0.7° when enabled or unspecified
            braking_active = True if self.enable_braking is None else bool(self.enable_braking)
            self.stopping_advance_deg = 0.7 if braking_active else 0.0

        norm_dir = self.direction.lower()
        if norm_dir not in ("cw", "ccw"):
            raise TurnConfigurationException(f"Direction must be 'cw' or 'ccw', got '{self.direction}'")
        if not (1 <= self.trials <= 8):
            raise TurnConfigurationException(f"Trials/repetitions must be between 1 and 8, got {self.trials}")
        if self.max_angular_speed <= 0.0:
            raise TurnConfigurationException(f"max_angular_speed must be positive, got {self.max_angular_speed}")
        if self.creep_angular_speed <= 0.0:
            raise TurnConfigurationException(f"creep_angular_speed must be positive, got {self.creep_angular_speed}")
        if self.creep_angular_speed > self.max_angular_speed:
            raise TurnConfigurationException(
                f"creep_angular_speed ({self.creep_angular_speed}) cannot exceed max_angular_speed ({self.max_angular_speed})"
            )
        if self.creep_threshold_deg <= 0.0:
            raise TurnConfigurationException(f"creep_threshold_deg must be positive, got {self.creep_threshold_deg}")
        if self.settle_seconds < 0.5:
            raise TurnConfigurationException(f"settle_seconds must be >= 0.5, got {self.settle_seconds}")
        if self.angle_tolerance_deg <= 0.0:
            raise TurnConfigurationException(f"angle_tolerance_deg must be positive, got {self.angle_tolerance_deg}")
        if self.stopping_advance_deg < 0.0 or self.stopping_advance_deg >= self.degrees:
            raise TurnConfigurationException(
                f"stopping_advance_deg ({self.stopping_advance_deg}) must be >= 0.0 and < target degrees ({self.degrees})"
            )

    @property
    def signed_target_deg(self) -> float:
        """
        In Rover One kinematic frame:
        CW rotation corresponds to positive yaw change (+deg).
        CCW rotation corresponds to negative yaw change (-deg).
        """
        return abs(self.degrees) if self.direction.lower() == "cw" else -abs(self.degrees)


def compute_wheel_speed_targets(wz_cmd: float, track_width_m: float = 0.197, wheel_radius_m: float = 0.033475) -> Dict[str, float]:
    """
    Computes commanded individual wheel speed targets (rad/s) for pure in-place rotation:
    - Pure rotation: v_linear = 0.0.
    - Left side speed (m/s) = - (wz * track_width / 2.0)
      Wait: In Rover One convention:
      CW (+wz): M1/LF and M3/LR positive; M2/RF and M4/RR negative.
      CCW (-wz): M1/LF and M3/LR negative; M2/RF and M4/RR positive.
    """
    wheel_speed_radps = abs(wz_cmd * (track_width_m / 2.0) / wheel_radius_m)

    if wz_cmd > 0.0:
        # Clockwise
        return {
            "m1": wheel_speed_radps,   # Left Front:  Positive
            "m2": -wheel_speed_radps,  # Right Front: Negative
            "m3": wheel_speed_radps,   # Left Rear:   Positive
            "m4": -wheel_speed_radps   # Right Rear:  Negative
        }
    elif wz_cmd < 0.0:
        # Counter-Clockwise
        return {
            "m1": -wheel_speed_radps,  # Left Front:  Negative
            "m2": wheel_speed_radps,   # Right Front: Positive
            "m3": -wheel_speed_radps,  # Left Rear:   Negative
            "m4": wheel_speed_radps    # Right Rear:  Positive
        }
    else:
        return {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}


def verify_wheel_command_symmetry(wheel_targets: Dict[str, float]) -> Tuple[bool, str]:
    """
    Verifies that opposite sides receive opposite directions with equal intended speed magnitudes.
    """
    m1 = wheel_targets.get("m1", 0.0)
    m2 = wheel_targets.get("m2", 0.0)
    m3 = wheel_targets.get("m3", 0.0)
    m4 = wheel_targets.get("m4", 0.0)

    # Check zero
    if abs(m1) < 1e-4 and abs(m2) < 1e-4 and abs(m3) < 1e-4 and abs(m4) < 1e-4:
        return True, "Symmetric zero commanded"

    # Check same-side agreement
    if (m1 > 0 and m3 <= 0) or (m1 < 0 and m3 >= 0):
        return False, f"Left-side wheels disagree in direction: LF={m1}, LR={m3}"
    if (m2 > 0 and m4 <= 0) or (m2 < 0 and m4 >= 0):
        return False, f"Right-side wheels disagree in direction: RF={m2}, RR={m4}"

    # Check opposite-side opposition
    if (m1 > 0 and m2 >= 0) or (m1 < 0 and m2 <= 0):
        return False, f"Opposite sides not commanded in opposite directions: Left={m1}, Right={m2}"

    # Check equal magnitude
    left_mag = abs(m1)
    right_mag = abs(m2)
    if abs(left_mag - right_mag) > 1e-3:
        return False, f"Commanded speed magnitude asymmetry: Left={left_mag:.3f}, Right={right_mag:.3f}"

    return True, "Equal magnitude opposite polarity confirmed"
