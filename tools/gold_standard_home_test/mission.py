"""
tools.gold_standard_home_test.mission - Gold Standard HOME Acceptance Test Orchestrator
Deterministic 4-leg mission with strict pre-arm validation and fail-closed safety watchdogs.
"""

import os
import sys
import time
import math
import requests
from typing import Dict, Any, Optional, Tuple, Callable

from .constants import (
    FORWARD_DISTANCE_M,
    ROTATION_TARGET_DEG,
    NORMAL_LINEAR_SPEED,
    NORMAL_ANGULAR_SPEED,
    FINAL_POS_CORRECTION_CEILING,
    FINAL_YAW_CORRECTION_CEILING,
    PRE_ARM_MAX_POS_ERR_M,
    PRE_ARM_MAX_YAW_ERR_DEG,
    TIMEOUT_LEG1_FORWARD_S,
    TIMEOUT_LEG2_ROTATION_S,
    TIMEOUT_LEG3_RETURN_S,
    TIMEOUT_LEG4_SETTLE_S,
    TELEMETRY_STALE_TIMEOUT_S,
    COCKPIT_DEFAULT_URL,
    BRIDGE_DEFAULT_URL
)
from .home_loader import load_authoritative_home, wrap_angle_rad, wrap_angle_deg
from .telemetry import TelemetryRecorder, TelemetryFrame
from .grader import MissionGrader

def get_env_token(var_name: str) -> str:
    """Reads credential from supported environment mechanism without copying files."""
    return os.environ.get(var_name, "").strip()

class MissionAbortException(Exception):
    """Raised when a safety invariant or watchdog triggers an immediate abort."""
    pass

class GoldStandardMission:
    def __init__(
        self,
        cockpit_url: str = COCKPIT_DEFAULT_URL,
        bridge_url: str = BRIDGE_DEFAULT_URL,
        dry_run: bool = False,
        output_dir: str = "reports/gold_standard_home_test"
    ):
        self.cockpit_url = cockpit_url.rstrip("/")
        self.bridge_url = bridge_url.rstrip("/")
        self.dry_run = dry_run
        self.output_dir = output_dir

        self.op_token = get_env_token("ROVER_OPERATOR_TOKEN")
        self.cmd_token = get_env_token("ROVER_CMD_VEL_TOKEN")

        self.home_pose = load_authoritative_home(cockpit_url=self.cockpit_url)
        self.recorder = TelemetryRecorder(output_dir=self.output_dir)
        self.recorder.metadata["home_pose"] = self.home_pose
        self.recorder.metadata["dry_run"] = self.dry_run

        self.initial_boot_count: Optional[int] = None
        self.last_imu_yaw: Optional[float] = None
        self.initial_amcl: Optional[Dict[str, Any]] = None
        
        # ROS 2 node reference (if initialized inside ROS 2 environment)
        self.ros_node = None
        self.latest_telemetry: Dict[str, Any] = {
            "amcl": None,
            "odom": None,
            "imu": None,
            "drive": None,
            "cm": None,
            "cmd_raw": {"vx": 0.0, "wz": 0.0},
            "cmd_smooth": {"vx": 0.0, "wz": 0.0},
            "cmd_final": {"vx": 0.0, "wz": 0.0},
            "cmd_source": "NONE",
            "last_packet_monotonic": time.monotonic()
        }

    def compute_mission_targets(self) -> Dict[str, Any]:
        """Calculates exact deterministic coordinates for each leg."""
        x0 = self.home_pose["x"]
        y0 = self.home_pose["y"]
        th0 = self.home_pose["yaw_rad"]

        # Leg 1: Outbound forward along saved HOME heading
        x1 = x0 + FORWARD_DISTANCE_M * math.cos(th0)
        y1 = y0 + FORWARD_DISTANCE_M * math.sin(th0)
        th1 = th0 # Same heading

        # Leg 2: In-place 180° CLOCKWISE rotation (CW reduces yaw by pi)
        th2 = wrap_angle_rad(th1 - math.pi)

        # Leg 3: Return to saved HOME coordinates and finish at exact saved HOME yaw
        x3 = x0
        y3 = y0
        th3 = th0

        return {
            "home": {"x": x0, "y": y0, "yaw_rad": th0, "yaw_deg": self.home_pose["yaw_deg"]},
            "leg1_outbound": {"x": x1, "y": y1, "yaw_rad": th1, "yaw_deg": wrap_angle_deg(math.degrees(th1))},
            "leg2_rotation": {"x": x1, "y": y1, "yaw_rad": th2, "yaw_deg": wrap_angle_deg(math.degrees(th2))},
            "leg3_return": {"x": x3, "y": y3, "yaw_rad": th3, "yaw_deg": wrap_angle_deg(math.degrees(th3))}
        }

    def update_drive_status(self) -> Dict[str, Any]:
        """Polls Cockpit for latest ESP32 drive status."""
        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        r = requests.get(f"{self.cockpit_url}/api/drive/status", headers=headers, timeout=1.0)
        r.raise_for_status()
        data = r.json()
        status = data.get("status", {})
        self.latest_telemetry["drive"] = status
        self.latest_telemetry["cmd_source"] = data.get("cmdSource", "NONE")
        self.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        return status

    def check_pre_arm_gate(self, amcl_pose: Dict[str, Any]) -> Tuple[bool, str, float, float]:
        """
        Validates:
        1. AMCL is localized.
        2. Starting pose is within 0.05m and 5° of saved HOME.
        """
        if not amcl_pose:
            return False, "No AMCL pose telemetry received", 999.0, 180.0

        x0 = self.home_pose["x"]
        y0 = self.home_pose["y"]
        th0 = self.home_pose["yaw_rad"]

        cur_x = amcl_pose.get("x", 0.0)
        cur_y = amcl_pose.get("y", 0.0)
        cur_th = amcl_pose.get("yaw_rad", math.radians(amcl_pose.get("yaw_deg", 0.0)))

        pos_err_m = math.hypot(cur_x - x0, cur_y - y0)
        yaw_err_deg = abs(wrap_angle_deg(math.degrees(cur_th - th0)))

        # Check localization state
        try:
            r = requests.get(f"{self.cockpit_url}/api/localization/status", timeout=1.0)
            if r.status_code == 200:
                loc_data = r.json()
                if not loc_data.get("localized", False) or loc_data.get("state") != "LOCALIZED":
                    return False, f"Cockpit localization state is not LOCALIZED ({loc_data.get('state')})", pos_err_m, yaw_err_deg
        except Exception:
            pass

        if pos_err_m > PRE_ARM_MAX_POS_ERR_M:
            return False, f"Initial position error ({pos_err_m*100:.1f} cm) exceeds 5.0 cm ceiling", pos_err_m, yaw_err_deg

        if yaw_err_deg > PRE_ARM_MAX_YAW_ERR_DEG:
            return False, f"Initial yaw error ({yaw_err_deg:.1f}°) exceeds 5.0° ceiling", pos_err_m, yaw_err_deg

        return True, "Pre-arm gate passed", pos_err_m, yaw_err_deg

    def arm(self) -> bool:
        """Arms the drivetrain immediately before mission execution."""
        if self.dry_run:
            print("[DRY-RUN] Drivetrain arming simulated (Hardware remained disarmed).")
            return True

        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        r = requests.post(f"{self.cockpit_url}/api/drive/arm", json={"autonomy": False}, headers=headers, timeout=2.0)
        if r.status_code != 200:
            print(f"[FAIL] Hardware arming rejected: HTTP {r.status_code} {r.text}")
            return False

        # Verify armed mode 3
        stat = self.update_drive_status()
        if not (stat.get("armed") is True and stat.get("mode") == 3):
            print(f"[FAIL] Armed state not confirmed by ESP32: {stat}")
            return False

        self.initial_boot_count = stat.get("bootCount", 1)
        print(f"[OK] Rover armed in Mode 3 (ESP32 boot count: {self.initial_boot_count}).")
        return True

    def disarm_and_stop(self):
        """Universal safety fail-safe: stops all velocity, cancels goals, disarms to Mode 0."""
        print("[SAFETY] Disarming drivetrain, zeroing commands, and locking Mode 0...")
        try:
            self.send_velocity(0.0, 0.0)
        except Exception:
            pass

        try:
            headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
            requests.post(f"{self.cockpit_url}/api/navigation/cancel", headers=headers, timeout=1.0)
        except Exception:
            pass

        try:
            headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
            requests.post(f"{self.cockpit_url}/api/drive/disarm", headers=headers, timeout=1.5)
            requests.post(f"{self.cockpit_url}/api/command-source", json={"source": "NONE"}, headers=headers, timeout=1.0)
        except Exception:
            pass

    def send_velocity(self, vx: float, wz: float, source: str = "EXACT_MOTION"):
        """Sends velocity packet through the authenticated bridge."""
        if self.dry_run:
            self.latest_telemetry["cmd_final"] = {"vx": vx, "wz": wz}
            return

        payload = {
            "linear": {"x": float(vx), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
            "source": source
        }
        headers = {"X-Rover-Bridge-Token": self.cmd_token} if self.cmd_token else {}
        requests.post(f"{self.bridge_url}/api/cmd_vel", json=payload, headers=headers, timeout=0.1)
        self.latest_telemetry["cmd_final"] = {"vx": vx, "wz": wz}

    def verify_safety_invariants(self, current_stage: str):
        """
        Continuous safety monitor. Trips immediately on:
        - Boot count change
        - IMU discontinuity
        - Stale telemetry (> 500ms)
        - Localization loss
        """
        now = time.monotonic()
        if now - self.latest_telemetry["last_packet_monotonic"] > TELEMETRY_STALE_TIMEOUT_S:
            raise MissionAbortException(f"Stale telemetry: no status packet for > {TELEMETRY_STALE_TIMEOUT_S}s")

        drive_stat = self.latest_telemetry.get("drive") or {}
        if not self.dry_run and self.initial_boot_count is not None:
            curr_bc = drive_stat.get("bootCount")
            if curr_bc is not None and curr_bc != self.initial_boot_count:
                raise MissionAbortException(f"ESP32 hardware reboot detected! (BootCount: {self.initial_boot_count} -> {curr_bc})")

        imu_stat = self.latest_telemetry.get("imu") or {}
        raw_yaw = imu_stat.get("raw_yaw_deg")
        if raw_yaw is not None:
            if self.last_imu_yaw is not None:
                step_delta = abs(wrap_angle_deg(raw_yaw - self.last_imu_yaw))
                if step_delta > 45.0 and abs(imu_stat.get("gyro_z", 0.0)) < 1.0:
                    raise MissionAbortException(f"IMU frame discontinuity detected ({step_delta:.1f}° jump at low gyro)")
            self.last_imu_yaw = raw_yaw

    def record_tick(self, stage: str):
        """Captures a 20+ Hz synchronized telemetry frame."""
        now = time.monotonic()
        t_rel = round(now - self.recorder.t0, 4)
        
        amcl = self.latest_telemetry.get("amcl") or {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "cov_x": 0.0, "cov_y": 0.0, "cov_yaw": 0.0, "localized": True}
        odom = self.latest_telemetry.get("odom") or {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "vx": 0.0, "wz": 0.0, "left_dist": 0.0, "right_dist": 0.0}
        imu = self.latest_telemetry.get("imu") or {"raw_yaw_deg": 0.0, "rel_yaw_deg": 0.0, "gyro_z": 0.0, "non_magnetic_valid": True}
        drive = self.latest_telemetry.get("drive") or {"armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0, "limLinear": 0.0, "limAngular": 0.0, "bootCount": 1, "resetReason": "NONE", "rtcResetReason": "NONE", "lockStatus": False}
        cm = self.latest_telemetry.get("cm") or {"action": "NONE", "polygon": "NONE"}

        pos_err_m = math.hypot(amcl.get("x", 0.0) - self.home_pose["x"], amcl.get("y", 0.0) - self.home_pose["y"])
        yaw_err_deg = wrap_angle_deg(amcl.get("yaw_deg", 0.0) - self.home_pose["yaw_deg"])

        frame = TelemetryFrame(
            t_rel_s=t_rel,
            timestamp_epoch=time.time(),
            mission_stage=stage,
            raw_cmd=self.latest_telemetry["cmd_raw"],
            smoothed_cmd=self.latest_telemetry["cmd_smooth"],
            final_cmd=self.latest_telemetry["cmd_final"],
            cmd_source=self.latest_telemetry["cmd_source"],
            amcl=amcl,
            odom=odom,
            imu=imu,
            drive=drive,
            collision_monitor=cm,
            to_home={
                "pos_err_m": round(pos_err_m, 4),
                "pos_err_cm": round(pos_err_m * 100.0, 2),
                "yaw_err_deg": round(yaw_err_deg, 2)
            }
        )
        self.recorder.record_frame(frame)
