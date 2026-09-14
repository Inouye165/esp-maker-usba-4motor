"""
tools.rover_tests.controllers - Motion Approach Controllers for Rover Test Harnesses

Implements the Canonical Exact Motion Contract (docs/EXACT_MOTION_CONTRACT.md):
- AngularApproachController: Deceleration from cruise yaw rate (0.80 rad/s)
  to creep yaw rate (0.20 rad/s) within 30.0° approach zone before zero command.
- Extensible BaseApproachController interface preparing for linear and outback subcommands.
"""

import time
from enum import Enum
from typing import Dict, Any, List, Optional


class ApproachPhase(str, Enum):
    IDLE = "IDLE"
    CRUISE = "CRUISE"
    SLOWDOWN = "SLOWDOWN"
    CREEP = "CREEP"
    ZERO = "ZERO"
    SETTLED = "SETTLED"


class BaseApproachController:
    """Common abstraction for approach controllers (angular, linear, etc.)."""
    def reset(self, target: float, start_time: Optional[float] = None):
        raise NotImplementedError

    def update(self, current_progress: float, current_time: Optional[float] = None) -> float:
        raise NotImplementedError

    def mark_settled(self, settled_val: float, current_time: Optional[float] = None):
        raise NotImplementedError

    def get_telemetry_summary(self) -> Dict[str, Any]:
        raise NotImplementedError


class AngularApproachController(BaseApproachController):
    """
    Canonical Angular Approach Controller governing in-place pivot maneuvers:
    - Cruise phase: Rotate at cruise speed (default 0.80 rad/s).
    - Creep phase: Step down to creep speed (default 0.20 rad/s) within approach zone (default 30.0°).
    - Completion: Command zero angular velocity (0.0 rad/s) at target angle.
    - Never abruptly cuts from full cruise speed to zero.
    """
    def __init__(
        self,
        cruise_wz_radps: float = 0.80,
        creep_wz_radps: float = 0.20,
        approach_zone_deg: float = 30.0
    ):
        self.cruise_wz_radps = abs(cruise_wz_radps)
        self.creep_wz_radps = abs(creep_wz_radps)
        self.approach_zone_deg = abs(approach_zone_deg)

        self.target_angle_deg = 0.0
        self.direction_sign = 1.0
        self.phase = ApproachPhase.IDLE

        # Milestones & Timing
        self.start_time: Optional[float] = None
        self.creep_entry_time: Optional[float] = None
        self.creep_entry_yaw_deg: Optional[float] = None
        self.threshold_crossing_time: Optional[float] = None
        self.threshold_crossing_yaw_deg: Optional[float] = None
        self.zero_command_time: Optional[float] = None
        self.settled_yaw_deg: Optional[float] = None
        self.settled_error_deg: Optional[float] = None
        self.phase_history: List[Dict[str, Any]] = []

    def reset(self, target: float, start_time: Optional[float] = None):
        self.target_angle_deg = target
        self.direction_sign = 1.0 if target >= 0.0 else -1.0
        self.phase = ApproachPhase.IDLE

        self.start_time = start_time if start_time is not None else time.time()
        self.creep_entry_time = None
        self.creep_entry_yaw_deg = None
        self.threshold_crossing_time = None
        self.threshold_crossing_yaw_deg = None
        self.zero_command_time = None
        self.settled_yaw_deg = None
        self.settled_error_deg = None
        self.phase_history = []

        target_mag = abs(target)
        initial_phase = ApproachPhase.CRUISE if target_mag > self.approach_zone_deg else ApproachPhase.CREEP
        self._set_phase(initial_phase, self.start_time, 0.0)

    def _set_phase(self, new_phase: ApproachPhase, current_time: float, current_yaw_deg: float):
        if self.phase != new_phase:
            self.phase = new_phase
            rel_t = current_time - self.start_time if self.start_time is not None else 0.0
            self.phase_history.append({
                "phase": str(new_phase),
                "t_rel": rel_t,
                "yaw_deg": current_yaw_deg
            })

    def update(self, current_progress: float, current_time: Optional[float] = None) -> float:
        if current_time is None:
            current_time = time.time()

        target_mag = abs(self.target_angle_deg)
        signed_progress = current_progress * self.direction_sign
        remaining_deg = target_mag - signed_progress

        # 1. Target Reached or Exceeded -> ZERO phase
        if remaining_deg <= 1e-4:
            if self.phase not in (ApproachPhase.ZERO, ApproachPhase.SETTLED):
                self._set_phase(ApproachPhase.ZERO, current_time, current_progress)
                rel_t = current_time - self.start_time if self.start_time is not None else 0.0
                if self.threshold_crossing_time is None:
                    self.threshold_crossing_time = rel_t
                    self.threshold_crossing_yaw_deg = current_progress
                if self.zero_command_time is None:
                    self.zero_command_time = rel_t
            return 0.0

        # 2. Inside Creep / Approach Zone -> CREEP phase
        elif remaining_deg <= (self.approach_zone_deg + 1e-4):
            if self.phase != ApproachPhase.CREEP:
                self._set_phase(ApproachPhase.CREEP, current_time, current_progress)
                rel_t = current_time - self.start_time if self.start_time is not None else 0.0
                if self.creep_entry_time is None:
                    self.creep_entry_time = rel_t
                    self.creep_entry_yaw_deg = current_progress
            return self.direction_sign * self.creep_wz_radps

        # 3. Outside Approach Zone -> CRUISE phase
        else:
            if self.phase != ApproachPhase.CRUISE:
                self._set_phase(ApproachPhase.CRUISE, current_time, current_progress)
            return self.direction_sign * self.cruise_wz_radps

    def mark_settled(self, settled_val: float, current_time: Optional[float] = None):
        if current_time is None:
            current_time = time.time()
        self._set_phase(ApproachPhase.SETTLED, current_time, settled_val)
        self.settled_yaw_deg = settled_val
        self.settled_error_deg = settled_val - self.target_angle_deg

    def get_telemetry_summary(self) -> Dict[str, Any]:
        return {
            "target_angle_deg": self.target_angle_deg,
            "current_phase": str(self.phase),
            "creep_entry_time_s": self.creep_entry_time,
            "creep_entry_yaw_deg": self.creep_entry_yaw_deg,
            "threshold_crossing_time_s": self.threshold_crossing_time,
            "threshold_crossing_yaw_deg": self.threshold_crossing_yaw_deg,
            "zero_command_time_s": self.zero_command_time,
            "settled_yaw_deg": self.settled_yaw_deg,
            "settled_error_deg": self.settled_error_deg,
            "phase_history": self.phase_history
        }
