"""
test.test_gold_standard_home_test - Automated Unit & Safety Abort Tests
Tests HOME loading, pre-arm gates, target geometry, grading criteria, and fail-safe aborts without physical movement.
"""

import os
import math
import json
import pytest
from unittest.mock import MagicMock, patch

from tools.gold_standard_home_test.constants import (
    FORWARD_DISTANCE_M,
    ROTATION_TARGET_DEG,
    NORMAL_LINEAR_SPEED,
    NORMAL_ANGULAR_SPEED,
    PASS_FINAL_POS_ERR_M,
    PASS_FINAL_YAW_ERR_DEG,
    PRE_ARM_MAX_POS_ERR_M,
    PRE_ARM_MAX_YAW_ERR_DEG
)
from tools.gold_standard_home_test.home_loader import (
    load_home_from_file,
    load_authoritative_home,
    wrap_angle_rad,
    wrap_angle_deg
)
from tools.gold_standard_home_test.grader import MissionGrader
from tools.gold_standard_home_test.mission import GoldStandardMission, MissionAbortException


class TestHomeLoader:
    def test_load_home_from_file(self, tmp_path):
        dummy_home = {
            "pose": {
                "position": {"x": 1.193853, "y": -0.045221, "z": 0.0},
                "orientation": {"z": -0.044029, "w": 0.99903},
                "yaw_deg": -5.047,
                "yaw_rad": -0.088087
            },
            "description": "Test HOME pose"
        }
        json_file = tmp_path / "home_pose_slam.json"
        with open(json_file, "w", encoding="utf-8") as f:
            json.dump(dummy_home, f)

        res = load_home_from_file(str(json_file))
        assert res is not None
        assert abs(res["x"] - 1.193853) < 1e-5
        assert abs(res["y"] - (-0.045221)) < 1e-5
        assert abs(res["yaw_deg"] - (-5.047)) < 1e-3
        assert abs(res["yaw_rad"] - (-0.088087)) < 1e-5

    def test_load_home_angle_wrapping(self):
        assert abs(wrap_angle_deg(370.0) - 10.0) < 1e-5
        assert abs(wrap_angle_deg(-190.0) - 170.0) < 1e-5
        assert abs(wrap_angle_rad(3.5 * math.pi) - (-0.5 * math.pi)) < 1e-5


class TestMissionGeometry:
    def test_mission_target_calculation(self):
        mission = GoldStandardMission(dry_run=True)
        # Mock HOME pose
        mission.home_pose = {
            "x": 1.0,
            "y": 2.0,
            "yaw_rad": 0.0,
            "yaw_deg": 0.0
        }
        targets = mission.compute_mission_targets()
        
        # Leg 1: 0.6096 m forward along 0 heading
        l1 = targets["leg1_outbound"]
        assert abs(l1["x"] - (1.0 + FORWARD_DISTANCE_M)) < 1e-5
        assert abs(l1["y"] - 2.0) < 1e-5
        assert abs(l1["yaw_deg"] - 0.0) < 1e-5

        # Leg 2: 180° CW in-place rotation from 0 deg -> -180 deg
        l2 = targets["leg2_rotation"]
        assert abs(l2["x"] - (1.0 + FORWARD_DISTANCE_M)) < 1e-5
        assert abs(l2["y"] - 2.0) < 1e-5
        assert abs(abs(l2["yaw_deg"]) - 180.0) < 1e-5

        # Leg 3: Return to exact HOME coordinates & heading
        l3 = targets["leg3_return"]
        assert abs(l3["x"] - 1.0) < 1e-5
        assert abs(l3["y"] - 2.0) < 1e-5
        assert abs(l3["yaw_deg"] - 0.0) < 1e-5


class TestPreArmGate:
    def test_pre_arm_gate_accepted(self):
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        
        # Current pose within 2 cm and 1.5 deg
        amcl_ok = {"x": 1.02, "y": 2.0, "yaw_deg": 1.5, "yaw_rad": math.radians(1.5)}
        with patch("requests.get") as mock_get:
            mock_get.return_value.status_code = 200
            mock_get.return_value.json.return_value = {"localized": True, "state": "LOCALIZED"}
            ok, msg, p_err, y_err = mission.check_pre_arm_gate(amcl_ok)
            assert ok is True
            assert p_err <= PRE_ARM_MAX_POS_ERR_M
            assert y_err <= PRE_ARM_MAX_YAW_ERR_DEG

    def test_pre_arm_gate_rejected_position(self):
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        
        # Position error 8 cm (> 5 cm limit)
        amcl_bad = {"x": 1.08, "y": 2.0, "yaw_deg": 0.0, "yaw_rad": 0.0}
        with patch("requests.get") as mock_get:
            mock_get.return_value.status_code = 200
            mock_get.return_value.json.return_value = {"localized": True, "state": "LOCALIZED"}
            ok, msg, p_err, y_err = mission.check_pre_arm_gate(amcl_bad)
            assert ok is False
            assert "exceeds 5.0 cm ceiling" in msg

    def test_pre_arm_gate_rejected_yaw(self):
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        
        # Yaw error 8.0 deg (> 5.0 deg limit)
        amcl_bad = {"x": 1.0, "y": 2.0, "yaw_deg": 8.0, "yaw_rad": math.radians(8.0)}
        with patch("requests.get") as mock_get:
            mock_get.return_value.status_code = 200
            mock_get.return_value.json.return_value = {"localized": True, "state": "LOCALIZED"}
            ok, msg, p_err, y_err = mission.check_pre_arm_gate(amcl_bad)
            assert ok is False
            assert "exceeds 5.0° ceiling" in msg


class TestSafetyAborts:
    def test_stale_telemetry_abort(self):
        mission = GoldStandardMission(dry_run=True)
        mission.latest_telemetry["last_packet_monotonic"] = 0.0 # Stale
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "Stale telemetry" in str(excinfo.value)

    def test_boot_count_change_abort(self):
        mission = GoldStandardMission(dry_run=False) # Enable hardware checks
        mission.initial_boot_count = 1
        import time
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.latest_telemetry["drive"] = {"bootCount": 2}
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "ESP32 hardware reboot detected" in str(excinfo.value)


class TestGrader:
    def test_grade_perfect_run(self):
        run_data = {
            "metadata": {"duration_s": 15.0, "watchdog_trips": 0, "rejections": 0},
            "samples": [
                {
                    "t_rel_s": 0.0,
                    "mission_stage": "LEG1_FORWARD",
                    "amcl": {"x": 1.0, "y": 2.0},
                    "to_home": {"pos_err_m": 0.0, "yaw_err_deg": 0.0},
                    "drive": {"bootCount": 1, "armed": True, "mode": 3, "reqLinear": 0.2, "reqAngular": 0.0},
                    "final_cmd": {"vx": 0.20, "wz": 0.0}
                },
                {
                    "t_rel_s": 5.0,
                    "mission_stage": "LEG1_FORWARD",
                    "amcl": {"x": 1.0 + FORWARD_DISTANCE_M, "y": 2.0},
                    "to_home": {"pos_err_m": FORWARD_DISTANCE_M, "yaw_err_deg": 0.0},
                    "drive": {"bootCount": 1, "armed": True, "mode": 3, "reqLinear": 0.2, "reqAngular": 0.0},
                    "final_cmd": {"vx": 0.20, "wz": 0.0}
                },
                {
                    "t_rel_s": 8.0,
                    "mission_stage": "LEG2_ROTATION",
                    "imu": {"raw_yaw_deg": 180.0},
                    "to_home": {"pos_err_m": FORWARD_DISTANCE_M, "yaw_err_deg": 180.0},
                    "drive": {"bootCount": 1, "armed": True, "mode": 3, "reqLinear": 0.0, "reqAngular": -0.4},
                    "final_cmd": {"vx": 0.0, "wz": -0.40}
                },
                {
                    "t_rel_s": 10.0,
                    "mission_stage": "LEG2_ROTATION",
                    "imu": {"raw_yaw_deg": 0.0},
                    "to_home": {"pos_err_m": FORWARD_DISTANCE_M, "yaw_err_deg": 180.0},
                    "drive": {"bootCount": 1, "armed": True, "mode": 3, "reqLinear": 0.0, "reqAngular": -0.4},
                    "final_cmd": {"vx": 0.0, "wz": -0.40}
                },
                {
                    "t_rel_s": 14.0,
                    "mission_stage": "LEG3_RETURN",
                    "amcl": {"x": 1.02, "y": 2.01},
                    "to_home": {"pos_err_m": 0.022, "yaw_err_deg": 1.5},
                    "drive": {"bootCount": 1, "armed": True, "mode": 3, "reqLinear": 0.1, "reqAngular": 0.0},
                    "final_cmd": {"vx": 0.10, "wz": 0.0}
                },
                {
                    "t_rel_s": 15.0,
                    "mission_stage": "LEG4_SETTLE",
                    "amcl": {"x": 1.01, "y": 2.005},
                    "to_home": {"pos_err_m": 0.011, "yaw_err_deg": 1.2},
                    "drive": {"bootCount": 1, "armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0},
                    "final_cmd": {"vx": 0.0, "wz": 0.0}
                }
            ]
        }
        res = MissionGrader.grade_run(run_data)
        assert res["overall_status"] == "PASS"
        assert res["criteria"]["final_home_position_error"]["passed"] is True
        assert res["criteria"]["final_home_yaw_error"]["passed"] is True
        assert res["criteria"]["final_safe_state"]["passed"] is True

    def test_grade_fail_position_error(self):
        run_data = {
            "metadata": {"duration_s": 15.0, "watchdog_trips": 0, "rejections": 0},
            "samples": [
                {
                    "t_rel_s": 15.0,
                    "mission_stage": "LEG4_SETTLE",
                    "to_home": {"pos_err_m": 0.08, "yaw_err_deg": 1.0}, # 8 cm > 4 cm limit
                    "drive": {"bootCount": 1, "armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0},
                    "final_cmd": {"vx": 0.0, "wz": 0.0}
                }
            ]
        }
        res = MissionGrader.grade_run(run_data)
        assert res["overall_status"] == "FAIL"
        assert res["criteria"]["final_home_position_error"]["passed"] is False
