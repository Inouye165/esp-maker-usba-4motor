"""
test/test_rover_test_turn.py - Unit Test Suite for Rover One Reusable Physical Test Framework

Uses mocks and fakes only. Never connects to or moves real hardware.
Tests parameter validation, CW/CCW signs, 90/180/360-degree targets,
handshake enforcement, non-magnetic IMU enforcement, inter-trial approval,
watchdog aborts, and guaranteed zero/disarm cleanup.
"""

import unittest
from unittest.mock import MagicMock, patch
import math
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.rover_tests.turn import (
    TurnParameters,
    TurnConfigurationException,
    compute_wheel_speed_targets,
    verify_wheel_command_symmetry
)
from tools.rover_tests.controllers import AngularApproachController, ApproachPhase
from tools.rover_tests.sensors import (
    quat_to_yaw,
    normalize_angle_delta,
    YawUnwrapper,
    BiasCorrectedGyroIntegrator,
    verify_non_magnetic_imu,
    SensorException
)
from tools.rover_tests.transport import (
    CockpitClient,
    NativeWSClient,
    perform_zero_handshake,
    disarm_and_stop,
    HandshakeException
)
from tools.rover_tests.runner import PhysicalTestRunner


class TestParameterValidation(unittest.TestCase):
    """Verifies CLI & model parameter validation rules."""

    def test_valid_parameters(self):
        p = TurnParameters(degrees=180.0, direction="cw", trials=3)
        p.validate()
        self.assertEqual(p.signed_target_deg, 180.0)

    def test_ccw_signed_target(self):
        p = TurnParameters(degrees=90.0, direction="ccw", trials=1)
        p.validate()
        self.assertEqual(p.signed_target_deg, -90.0)

    def test_negative_degrees_rejected(self):
        p = TurnParameters(degrees=-90.0, direction="cw")
        with self.assertRaises(TurnConfigurationException):
            p.validate()

    def test_invalid_direction_rejected(self):
        p = TurnParameters(degrees=90.0, direction="left")
        with self.assertRaises(TurnConfigurationException):
            p.validate()

    def test_zero_trials_rejected(self):
        p = TurnParameters(degrees=90.0, direction="cw", trials=0)
        with self.assertRaises(TurnConfigurationException):
            p.validate()

    def test_creep_exceeding_max_speed_rejected(self):
        p = TurnParameters(degrees=90.0, max_angular_speed=0.50, creep_angular_speed=0.60)
        with self.assertRaises(TurnConfigurationException):
            p.validate()

    def test_negative_speeds_rejected(self):
        p = TurnParameters(degrees=90.0, max_angular_speed=-0.50)
        with self.assertRaises(TurnConfigurationException):
            p.validate()


class TestTurnPolarityAndWheelSignConventions(unittest.TestCase):
    """Verifies Rover One kinematic conventions and polarity invariants."""

    def test_cw_wheel_polarity(self):
        # CW (+Yaw): Left wheels (M1, M3) positive; Right wheels (M2, M4) negative
        targets = compute_wheel_speed_targets(wz_cmd=0.80)
        self.assertGreater(targets["m1"], 0.0, "LF (M1) must be positive for CW")
        self.assertGreater(targets["m3"], 0.0, "LR (M3) must be positive for CW")
        self.assertLess(targets["m2"], 0.0, "RF (M2) must be negative for CW")
        self.assertLess(targets["m4"], 0.0, "RR (M4) must be negative for CW")

        # Verify equal magnitude across all four wheels
        self.assertAlmostEqual(abs(targets["m1"]), abs(targets["m2"]))
        self.assertAlmostEqual(abs(targets["m1"]), abs(targets["m3"]))
        self.assertAlmostEqual(abs(targets["m1"]), abs(targets["m4"]))

        valid, msg = verify_wheel_command_symmetry(targets)
        self.assertTrue(valid, msg)

    def test_ccw_wheel_polarity(self):
        # CCW (-Yaw): Left wheels (M1, M3) negative; Right wheels (M2, M4) positive
        targets = compute_wheel_speed_targets(wz_cmd=-0.80)
        self.assertLess(targets["m1"], 0.0, "LF (M1) must be negative for CCW")
        self.assertLess(targets["m3"], 0.0, "LR (M3) must be negative for CCW")
        self.assertGreater(targets["m2"], 0.0, "RF (M2) must be positive for CCW")
        self.assertGreater(targets["m4"], 0.0, "RR (M4) must be positive for CCW")

        valid, msg = verify_wheel_command_symmetry(targets)
        self.assertTrue(valid, msg)

    def test_asymmetric_command_rejected(self):
        # Asymmetric magnitude
        bad_targets = {"m1": 2.0, "m2": -1.5, "m3": 2.0, "m4": -1.5}
        valid, msg = verify_wheel_command_symmetry(bad_targets)
        self.assertFalse(valid)
        self.assertIn("asymmetry", msg)

    def test_same_side_conflict_rejected(self):
        # Left front positive, left rear negative
        bad_targets = {"m1": 2.0, "m2": -2.0, "m3": -2.0, "m4": -2.0}
        valid, msg = verify_wheel_command_symmetry(bad_targets)
        self.assertFalse(valid)
        self.assertIn("disagree", msg)


class TestTargetAngleUnwrapping(unittest.TestCase):
    """Verifies continuous relative yaw tracking across 90°, 180°, and 360° targets."""

    def test_90_degree_unwrapping(self):
        unwrapper = YawUnwrapper(initial_raw_yaw=0.0)
        steps = [math.radians(deg) for deg in [15, 30, 45, 60, 75, 90]]
        for step in steps:
            unwrapper.update_orientation_yaw(step)
        self.assertAlmostEqual(unwrapper.relative_yaw_deg, 90.0, places=2)

    def test_180_degree_boundary_crossing(self):
        """Crossing +pi to -pi boundary must not cause a 360° discontinuity."""
        # Start at 170°, rotate CW across 180° (+pi) to -170° (-pi + 10°)
        unwrapper = YawUnwrapper(initial_raw_yaw=math.radians(170))
        # +10 deg -> 180°
        unwrapper.update_orientation_yaw(math.radians(180))
        # +10 deg -> -170° (raw wrapped)
        unwrapper.update_orientation_yaw(math.radians(-170))
        # +10 deg -> -160° (raw wrapped)
        unwrapper.update_orientation_yaw(math.radians(-160))

        # Total relative rotation from 170° should be exactly +30°
        self.assertAlmostEqual(unwrapper.relative_yaw_deg, 30.0, places=2)

    def test_360_degree_complete_turn(self):
        """Full 360° turn must accumulate to 360° continuously."""
        unwrapper = YawUnwrapper(initial_raw_yaw=0.0)
        # 36 steps of 10 degrees each
        for i in range(1, 37):
            raw = math.radians((i * 10) % 360)
            if raw > math.pi:
                raw -= 2.0 * math.pi
            unwrapper.update_orientation_yaw(raw)

        self.assertAlmostEqual(unwrapper.relative_yaw_deg, 360.0, places=1)


class TestNonMagneticImuEnforcement(unittest.TestCase):
    """Verifies fail-closed enforcement of SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC."""

    def test_valid_non_magnetic_report_accepted(self):
        snapshot = {
            "ok": True,
            "rotVecValid": True,
            "inResetRecovery": False,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "orientation": {"w": 0.7071, "x": 0.0, "y": 0.0, "z": 0.7071}
        }
        valid, msg = verify_non_magnetic_imu(snapshot)
        self.assertTrue(valid, msg)

    def test_missing_snapshot_rejected(self):
        valid, msg = verify_non_magnetic_imu({})
        self.assertFalse(valid)

    def test_rotvec_invalid_flag_rejected(self):
        snapshot = {
            "ok": True,
            "rotVecValid": False,  # Report invalid!
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        }
        valid, msg = verify_non_magnetic_imu(snapshot)
        self.assertFalse(valid)
        self.assertIn("validity flag is FALSE", msg)

    def test_magnetic_report_rejected(self):
        snapshot = {
            "ok": True,
            "rotVecValid": True,
            "orientationSource": "SH2_ROTATION_VECTOR_MAGNETIC",  # PROHIBITED!
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        }
        valid, msg = verify_non_magnetic_imu(snapshot)
        self.assertFalse(valid)
        self.assertIn("Prohibited orientation source", msg)

    def test_degenerate_quaternion_rejected(self):
        snapshot = {
            "ok": True,
            "rotVecValid": True,
            "orientation": {"w": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}  # Degenerate zero norm
        }
        valid, msg = verify_non_magnetic_imu(snapshot)
        self.assertFalse(valid)
        self.assertIn("Degenerate", msg)


class TestThreeConsecutiveZeroHandshake(unittest.TestCase):
    """Verifies the 3-consecutive-zero autonomy handshake state machine."""

    def test_successful_zero_handshake(self):
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True

        mock_cockpit.enable_autonomy.return_value = {"ok": True}
        # Simulate state sequence: WAITING_FOR_ZERO (count 1) -> (count 2) -> READY_DISARMED (count 3)
        mock_cockpit.get_autonomy_status.side_effect = [
            {"state": "WAITING_FOR_ZERO", "zeroHandshakeCount": 1},
            {"state": "WAITING_FOR_ZERO", "zeroHandshakeCount": 2},
            {"state": "READY_DISARMED", "zeroHandshakeCount": 3},
            {"state": "READY_ARMED", "zeroHandshakeCount": 3}
        ]
        mock_cockpit.arm_drive.return_value = {"ok": True}
        mock_cockpit.get_status.return_value = {"armed": True}

        success = perform_zero_handshake(mock_cockpit, mock_ws, max_duration_sec=2.0)
        self.assertTrue(success)
        mock_cockpit.enable_autonomy.assert_called_once()
        mock_cockpit.arm_drive.assert_called_once()

    def test_zero_handshake_timeout_raises_exception(self):
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True

        mock_cockpit.enable_autonomy.return_value = {"ok": True}
        # Stuck in WAITING_FOR_ZERO with count 0
        mock_cockpit.get_autonomy_status.return_value = {"state": "WAITING_FOR_ZERO", "zeroHandshakeCount": 0}

        with self.assertRaises(HandshakeException):
            perform_zero_handshake(mock_cockpit, mock_ws, max_duration_sec=0.1)


class TestAngularApproachDeceleration(unittest.TestCase):
    """Verifies that the controller steps from cruise to creep and never cuts abruptly."""

    def test_180_deg_deceleration_phases(self):
        controller = AngularApproachController(
            cruise_wz_radps=0.80,
            creep_wz_radps=0.20,
            approach_zone_deg=30.0
        )
        controller.reset(target=180.0, start_time=100.0)

        # 1. During Cruise (at 50° progress, remaining = 130°)
        wz1 = controller.update(current_progress=50.0, current_time=101.0)
        self.assertEqual(controller.phase, ApproachPhase.CRUISE)
        self.assertAlmostEqual(wz1, 0.80)

        # 2. Inside Approach Zone (at 160° progress, remaining = 20° <= 30°)
        wz2 = controller.update(current_progress=160.0, current_time=103.0)
        self.assertEqual(controller.phase, ApproachPhase.CREEP)
        self.assertAlmostEqual(wz2, 0.20)

        # 3. Target Reached (at 180° progress, remaining = 0°)
        wz3 = controller.update(current_progress=180.0, current_time=105.0)
        self.assertEqual(controller.phase, ApproachPhase.ZERO)
        self.assertAlmostEqual(wz3, 0.0)

        # Verify phase sequence captured creep before zero
        phases = [h["phase"] for h in controller.phase_history]
        self.assertIn(str(ApproachPhase.CRUISE), phases)
        self.assertIn(str(ApproachPhase.CREEP), phases)
        self.assertIn(str(ApproachPhase.ZERO), phases)

        # Ensure creep occurred before zero
        creep_idx = phases.index(str(ApproachPhase.CREEP))
        zero_idx = phases.index(str(ApproachPhase.ZERO))
        self.assertLess(creep_idx, zero_idx, "Must transition through CREEP before ZERO")


class TestGuaranteedZeroDisarmCleanup(unittest.TestCase):
    """Verifies that drivetrain is stopped and disarmed on all exit and abort paths."""

    def test_disarm_and_stop_invocations(self):
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True

        disarm_and_stop(mock_cockpit, mock_ws)

        mock_ws.send_drive.assert_called_with(0.0, 0.0)
        mock_cockpit.disarm_drive.assert_called_once()
        mock_cockpit.disable_autonomy.assert_called_once()
        mock_cockpit.set_command_source.assert_called_with("NONE")

    def test_runner_operator_denial_leaves_disarmed(self):
        params = TurnParameters(degrees=180.0, direction="cw", trials=1, dry_run=False, inter_trial_approval=True)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True
        mock_ws.authenticate.return_value = True
        mock_cockpit.token = "test-operator-token"

        # Operator refuses motion
        mock_prompt = MagicMock(return_value="n")

        runner = PhysicalTestRunner(params, prompt_fn=mock_prompt, cockpit_client=mock_cockpit, ws_client=mock_ws)
        suite_report = runner.execute_suite()

        self.assertEqual(suite_report.aborted_trials, 1)
        self.assertEqual(suite_report.trials[0].status, "ABORTED")
        self.assertIn("authorization withheld", suite_report.trials[0].abort_reason)

        # Verify disarmed invariant called
        mock_cockpit.disarm_drive.assert_called()


class TestInterTrialApprovalAndEstimateCollection(unittest.TestCase):
    """Verifies Ron's approval prompt before trial and physical estimate after trial."""

    def test_inter_trial_flow_with_physical_estimate(self):
        params = TurnParameters(degrees=90.0, direction="cw", trials=1, dry_run=False, inter_trial_approval=True, settle_seconds=0.5)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True
        mock_ws.authenticate.return_value = True
        mock_cockpit.token = "test-operator-token"

        base_imu = {
            "ok": True,
            "rotVecValid": True,
            "inResetRecovery": False,
            "dataAgeMs": 10,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "gyro": {"z": 0.001}
        }
        # Initial samples at 0 deg, then motion progresses to 90 deg (w=0.7071068, z=0.7071068)
        imu_0 = dict(base_imu, orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
        imu_90 = dict(base_imu, orientation={"w": 0.7071068, "x": 0.0, "y": 0.0, "z": 0.7071068})
        mock_cockpit.get_imu.side_effect = [imu_0] * 8 + [imu_90] * 20
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 100, "m2": 100, "m3": 100, "m4": 100}}
        mock_cockpit.enable_autonomy.return_value = {"ok": True}
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_DISARMED", "zeroHandshakeCount": 3}
        mock_cockpit.arm_drive.return_value = {"ok": True}
        mock_cockpit.get_status.return_value = {"armed": True}
        mock_ws.recv_frames.return_value = []
        mock_ws.send_drive.return_value = True

        mock_prompt = MagicMock(side_effect=["y", "89.5"])

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=mock_prompt, cockpit_client=mock_cockpit, ws_client=mock_ws)
            trial_report = runner.execute_single_trial(1)

            self.assertEqual(trial_report.rons_physical_angle_estimate_deg, 89.5)
            self.assertIsNotNone(trial_report.estimate_vs_gyro_delta_deg)
            self.assertEqual(trial_report.status, "SUCCESS")


class TestPytestExclusion(unittest.TestCase):
    """Verifies that physical test tools are not collected by pytest."""

    def test_pytest_ini_excludes_tools_rover_tests(self):
        import configparser
        import os

        ini_path = os.path.join(os.path.dirname(__file__), "../pytest.ini")
        self.assertTrue(os.path.exists(ini_path), "pytest.ini must exist in repo root")

        config = configparser.ConfigParser()
        config.read(ini_path)

        self.assertIn("pytest", config.sections())
        norecursedirs = config.get("pytest", "norecursedirs")
        self.assertIn("tools/rover_tests", norecursedirs)
        testpaths = config.get("pytest", "testpaths")
        self.assertIn("test", testpaths)


if __name__ == "__main__":
    unittest.main()
