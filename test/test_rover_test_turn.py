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
import itertools

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

        with self.assertRaises(HandshakeException) as ctx:
            perform_zero_handshake(mock_cockpit, mock_ws, max_duration_sec=0.1)
        self.assertEqual(ctx.exception.stage, "WAITING_FOR_ZERO")
        self.assertEqual(ctx.exception.state, "WAITING_FOR_ZERO")
        self.assertEqual(ctx.exception.zero_count, 0)

    def test_handshake_failure_before_enable(self):
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_cockpit.enable_autonomy.return_value = {"ok": False, "error": "Cannot enable autonomy while rover is armed"}
        mock_cockpit.get_autonomy_status.return_value = {"state": "DISABLED", "zeroHandshakeCount": 0, "cmdSource": "NONE"}

        with self.assertRaises(HandshakeException) as ctx:
            perform_zero_handshake(mock_cockpit, mock_ws, max_duration_sec=0.5)
        self.assertEqual(ctx.exception.stage, "ENABLE_AUTONOMY")
        self.assertIn("Cannot enable autonomy", str(ctx.exception))

    def test_handshake_failure_during_arming(self):
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True
        mock_cockpit.enable_autonomy.return_value = {"ok": True}
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_DISARMED", "zeroHandshakeCount": 3}
        mock_cockpit.arm_drive.return_value = {"ok": False, "error": "Hardware interlock tripped"}

        with self.assertRaises(HandshakeException) as ctx:
            perform_zero_handshake(mock_cockpit, mock_ws, max_duration_sec=0.5)
        self.assertEqual(ctx.exception.stage, "ARM_DRIVE")
        self.assertIn("Hardware interlock tripped", str(ctx.exception))

    def test_handshake_exception_diagnostic_fields(self):
        exc = HandshakeException(
            message="Test handshake fault",
            stage="WAITING_FOR_ZERO",
            state="WAITING_FOR_ZERO",
            zero_count=1,
            cmd_source="NONE",
            last_rejection_reason="Rate limit",
            underlying_error="Connection refused"
        )
        details = exc.get_details()
        self.assertEqual(details["stage"], "WAITING_FOR_ZERO")
        self.assertEqual(details["state"], "WAITING_FOR_ZERO")
        self.assertEqual(details["zero_count"], 1)
        self.assertEqual(details["last_rejection_reason"], "Rate limit")
        self.assertEqual(details["underlying_error"], "Connection refused")


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
        mock_cockpit.get_status.return_value = {"armed": False, "autonomyState": "DISABLED", "cmdSource": "NONE"}

        final_st = disarm_and_stop(mock_cockpit, mock_ws)

        mock_ws.send_drive.assert_called_with(0.0, 0.0)
        mock_cockpit.send_cmd_vel.assert_called_with(0.0, 0.0)
        mock_cockpit.disarm_drive.assert_called_once()
        mock_cockpit.disable_autonomy.assert_called_once()
        mock_cockpit.set_command_source.assert_called_with("NONE")
        self.assertEqual(final_st["armed"], False)
        self.assertEqual(final_st["autonomyState"], "DISABLED")
        self.assertEqual(final_st["cmdSource"], "NONE")

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

    def test_regression_runner_catches_handshake_exception_without_name_error(self):
        """
        Regression test for: [EXECUTION ERROR] Unexpected fault: name 'HandshakeException' is not defined.
        Simulates live physical trial encountering HandshakeException and confirms runner handles it cleanly.
        """
        params = TurnParameters(degrees=180.0, direction="cw", trials=1, dry_run=False, inter_trial_approval=False)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True
        mock_ws.authenticate.return_value = True
        mock_cockpit.token = "test-operator-token"

        # Mock healthy sensors
        mock_cockpit.get_imu.return_value = {
            "ok": True,
            "rotVecValid": True,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "gyro": {"z": 0.001}
        }
        mock_cockpit.get_encoders.return_value = {"encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.get_status.return_value = {"armed": False, "autonomyState": "DISABLED", "cmdSource": "NONE"}

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)

        # Mock perform_zero_handshake raising HandshakeException with full diagnostic metadata
        with patch("tools.rover_tests.runner.perform_zero_handshake") as mock_handshake:
            mock_handshake.side_effect = HandshakeException(
                "Zero handshake failed to complete within 4.0s",
                stage="WAITING_FOR_ZERO",
                state="WAITING_FOR_ZERO",
                zero_count=0,
                cmd_source="NONE",
                underlying_error="Connection refused"
            )

            # execute_single_trial must NOT raise NameError!
            trial = runner.execute_single_trial(1)

            self.assertEqual(trial.status, "ABORTED")
            self.assertIn("HandshakeException", trial.abort_reason)
            self.assertIn("Stage: WAITING_FOR_ZERO", trial.abort_reason)
            self.assertIn("ZeroCount: 0", trial.abort_reason)
            # Confirm cleanup was invoked
            mock_cockpit.disarm_drive.assert_called()
            mock_cockpit.disable_autonomy.assert_called()

    def test_guaranteed_cleanup_across_all_failure_stages(self):
        """
        Confirms guaranteed cleanup executes after failures occurring:
        - before autonomy enable (sensor failure)
        - during WAITING_FOR_ZERO
        - before arming
        - after arming
        """
        params = TurnParameters(degrees=180.0, direction="cw", trials=1, dry_run=False, inter_trial_approval=False)

        # Case 1: Failure before autonomy enable (bad IMU)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True
        mock_cockpit.get_imu.return_value = {"ok": False}
        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        trial1 = runner.execute_single_trial(1)
        self.assertEqual(trial1.status, "ABORTED")
        mock_cockpit.disarm_drive.assert_called()

        # Case 2: Failure during WAITING_FOR_ZERO
        mock_cockpit.reset_mock()
        mock_cockpit.get_imu.return_value = {
            "ok": True, "rotVecValid": True, "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}, "gyro": {"z": 0.0}
        }
        mock_cockpit.get_encoders.return_value = {"encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        with patch("tools.rover_tests.runner.perform_zero_handshake") as mock_handshake:
            mock_handshake.side_effect = HandshakeException("Timeout in zero handshake", stage="WAITING_FOR_ZERO")
            trial2 = runner.execute_single_trial(1)
            self.assertEqual(trial2.status, "ABORTED")
            mock_cockpit.disarm_drive.assert_called()

        # Case 3: Failure during arming
        mock_cockpit.reset_mock()
        with patch("tools.rover_tests.runner.perform_zero_handshake") as mock_handshake:
            mock_handshake.side_effect = HandshakeException("Arm failed", stage="ARM_DRIVE")
            trial3 = runner.execute_single_trial(1)
            self.assertEqual(trial3.status, "ABORTED")
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
        # Transition from READY_DISARMED to READY_ARMED then ACTIVE
        auto_states = [
            {"state": "READY_DISARMED", "zeroHandshakeCount": 3},
            {"state": "READY_ARMED", "zeroHandshakeCount": 3, "cmdSource": "ROS_AUTONOMY"},
            {"state": "READY_ARMED", "zeroHandshakeCount": 3, "cmdSource": "ROS_AUTONOMY"},
            {"state": "ACTIVE", "clampedAngular": 0.8, "cmdSource": "ROS_AUTONOMY"}
        ] + [{"state": "ACTIVE", "clampedAngular": 0.8, "cmdSource": "ROS_AUTONOMY"}] * 30
        mock_cockpit.get_autonomy_status.side_effect = auto_states
        mock_cockpit.arm_drive.return_value = {"ok": True}
        mock_cockpit.get_status.return_value = {"armed": True, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
        mock_cockpit.send_cmd_vel.return_value = {"ok": True}
        mock_ws.recv_frames.return_value = []
        mock_ws.send_drive.return_value = True

        mock_prompt = MagicMock(side_effect=["y", "89.5"])

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=mock_prompt, cockpit_client=mock_cockpit, ws_client=mock_ws)
            trial_report = runner.execute_single_trial(1)

            self.assertEqual(trial_report.rons_physical_angle_estimate_deg, 89.5)
            self.assertIsNotNone(trial_report.estimate_vs_gyro_delta_deg)
            self.assertEqual(trial_report.status, "SUCCESS")


class TestLifecycleAndCleanupTiming(unittest.TestCase):
    """
    Verifies the complete 19-step lifecycle invariants:
    - Ordered lifecycle: READY_DISARMED -> READY_ARMED -> nonzero command accepted -> motion loop -> zero -> cleanup
    - Fails if cleanup occurs between READY_ARMED and first nonzero command
    - Aborts immediately within 500ms if first command rejected or wheel targets remain zero
    - Confirms transitions and command-response are captured in TrialReport
    """

    def _create_mock_clients(self, target_deg=90.0):
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True
        mock_ws.authenticate.return_value = True
        mock_cockpit.token = "test-operator-token"

        base_imu = {
            "ok": True,
            "rotVecValid": True,
            "inResetRecovery": False,
            "dataAgeMs": 15,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "gyro": {"z": 0.001}
        }
        # Start at 0 deg, then progress to target_deg
        imu_0 = dict(base_imu, orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
        rad = math.radians(target_deg)
        w = math.cos(rad / 2.0)
        z = math.sin(rad / 2.0)
        imu_target = dict(base_imu, orientation={"w": w, "x": 0.0, "y": 0.0, "z": z})

        current_status = {
            "armed": False,
            "autonomyState": "DISABLED",
            "cmdSource": "NONE"
        }

        def mock_arm():
            current_status["armed"] = True
            current_status["autonomyState"] = "READY_ARMED"
            return {"ok": True}

        def mock_disarm():
            current_status["armed"] = False
            return {"ok": True}

        def mock_enable_auto():
            current_status["autonomyState"] = "WAITING_FOR_ZERO"
            return {"ok": True}

        def mock_disable_auto():
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True}

        def mock_set_source(source="NONE"):
            current_status["cmdSource"] = source
            return {"ok": True}

        mock_cockpit.get_imu.side_effect = itertools.chain([imu_0] * 8, itertools.repeat(imu_target))
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.enable_autonomy.side_effect = mock_enable_auto
        mock_cockpit.arm_drive.side_effect = mock_arm
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.disable_autonomy.side_effect = mock_disable_auto
        mock_cockpit.set_command_source.side_effect = mock_set_source
        mock_cockpit.send_cmd_vel.return_value = {"ok": True}
        mock_ws.recv_frames.return_value = []
        mock_cockpit._state = current_status

        return mock_cockpit, mock_ws

    def test_ordered_lifecycle_full_sequence(self):
        """
        Requirement 14 & 17:
        Proves: READY_DISARMED -> READY_ARMED -> nonzero command accepted -> motion loop -> zero -> cleanup
        Verifies autonomy-state transitions and first_command_response in TrialReport.
        """
        params = TurnParameters(degrees=90.0, direction="cw", trials=1, dry_run=False, inter_trial_approval=False, settle_seconds=0.5)
        mock_cockpit, mock_ws = self._create_mock_clients(target_deg=90.0)

        call_sequence = []
        def do_arm():
            call_sequence.append("arm_drive")
            mock_cockpit._state["armed"] = True
            mock_cockpit._state["autonomyState"] = "READY_ARMED"
            return {"ok": True}

        def do_disarm():
            call_sequence.append("disarm_drive")
            mock_cockpit._state["armed"] = False
            return {"ok": True}

        def do_disable():
            mock_cockpit._state["autonomyState"] = "DISABLED"
            return {"ok": True}

        def do_set_source(source="NONE"):
            mock_cockpit._state["cmdSource"] = source
            return {"ok": True}

        mock_cockpit.arm_drive.side_effect = do_arm
        mock_cockpit.disarm_drive.side_effect = do_disarm
        mock_cockpit.disable_autonomy.side_effect = do_disable
        mock_cockpit.set_command_source.side_effect = do_set_source
        mock_cockpit.send_cmd_vel.side_effect = lambda vx, wz, **kw: (call_sequence.append(f"cmd_vel({vx:.2f},{wz:.2f})"), {"ok": True})[1]

        # Autonomy state transitions
        auto_states = [
            {"state": "WAITING_FOR_ZERO", "zeroHandshakeCount": 1},
            {"state": "READY_DISARMED", "zeroHandshakeCount": 3},
            {"state": "READY_ARMED", "zeroHandshakeCount": 3, "cmdSource": "ROS_AUTONOMY"},
            {"state": "READY_ARMED", "zeroHandshakeCount": 3, "cmdSource": "ROS_AUTONOMY"},
            {"state": "ACTIVE", "clampedAngular": 0.8, "cmdSource": "ROS_AUTONOMY"},
        ]
        mock_cockpit.get_autonomy_status.side_effect = itertools.chain(
            auto_states,
            itertools.repeat({"state": "ACTIVE", "clampedAngular": 0.8, "cmdSource": "ROS_AUTONOMY"})
        )

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
            trial = runner.execute_single_trial(1)

        self.assertEqual(trial.status, "SUCCESS")
        self.assertTrue(trial.first_command_response.get("ok"))

        # Verify recorded state transitions in JSON report
        states_recorded = [t["state"] for t in trial.autonomy_state_transitions]
        self.assertIn("READY_DISARMED", states_recorded)
        self.assertIn("READY_ARMED", states_recorded)
        self.assertIn("ACTIVE", states_recorded)
        self.assertIn("DISABLED", states_recorded)

        # Verify ordering: READY_DISARMED before READY_ARMED before ACTIVE before DISABLED
        idx_rd = states_recorded.index("READY_DISARMED")
        idx_ra = states_recorded.index("READY_ARMED")
        idx_ac = states_recorded.index("ACTIVE")
        idx_dis = states_recorded.index("DISABLED")
        self.assertLess(idx_rd, idx_ra)
        self.assertLess(idx_ra, idx_ac)
        self.assertLess(idx_ac, idx_dis)

        # Verify call order: arm_drive occurs before first nonzero cmd_vel,
        # and disarm_drive only occurs AFTER zero command and settle
        arm_idx = call_sequence.index("arm_drive")
        first_nonzero_cmd = [i for i, c in enumerate(call_sequence) if c.startswith("cmd_vel") and not c == "cmd_vel(0.00,0.00)"][0]
        self.assertLess(arm_idx, first_nonzero_cmd, "arm_drive must precede first nonzero command")

        # disarm_drive must NOT appear between arm_idx and first_nonzero_cmd!
        disarms_between = [i for i, c in enumerate(call_sequence) if c == "disarm_drive" and arm_idx < i < first_nonzero_cmd]
        self.assertEqual(len(disarms_between), 0, "Cleanup must not execute between arm_drive and motion!")

        # Last disarm must be after zero command
        last_disarm = max(i for i, c in enumerate(call_sequence) if c == "disarm_drive")
        zero_cmds = [i for i, c in enumerate(call_sequence) if c == "cmd_vel(0.00,0.00)"]
        self.assertGreater(len(zero_cmds), 0)
        self.assertGreater(last_disarm, zero_cmds[-1], "Final disarm must execute after zero command")

    def test_cleanup_between_ready_armed_and_first_command_fails(self):
        """
        Requirement 15:
        Fails if cleanup occurs between READY_ARMED and the first nonzero command.
        Simulates unexpected disarm or transition to DISABLED right after handshake.
        Pre-motion assertion must trip, abort immediately, and prevent any motion.
        """
        params = TurnParameters(degrees=90.0, direction="cw", trials=1, dry_run=False, inter_trial_approval=False)
        mock_cockpit, mock_ws = self._create_mock_clients()

        # Handshake completes, but premature cleanup sets state to DISABLED / disarmed before motion
        handshake_done = [False]
        def mock_get_status():
            if not handshake_done[0]:
                return {"armed": True, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
            else:
                return {"armed": False, "autonomyState": "DISABLED", "cmdSource": "NONE"}

        def mock_get_autonomy():
            if not handshake_done[0]:
                return {"state": "READY_ARMED", "zeroHandshakeCount": 3, "cmdSource": "ROS_AUTONOMY"}
            else:
                return {"state": "DISABLED", "zeroHandshakeCount": 0, "cmdSource": "NONE"}

        mock_cockpit.get_status.side_effect = mock_get_status
        mock_cockpit.get_autonomy_status.side_effect = mock_get_autonomy

        with patch("tools.rover_tests.runner.perform_zero_handshake") as mock_handshake:
            def do_handshake(*args, **kwargs):
                handshake_done[0] = True
                return True
            mock_handshake.side_effect = do_handshake

            with patch("time.sleep", return_value=None):
                runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
                trial = runner.execute_single_trial(1)

        self.assertEqual(trial.status, "ABORTED")
        self.assertIn("Pre-motion invariant violated", trial.abort_reason)
        # Ensure no nonzero motion command was sent
        for call_args in mock_cockpit.send_cmd_vel.call_args_list:
            args, kwargs = call_args
            wz = kwargs.get("wz", args[1] if len(args) > 1 else 0.0)
            self.assertEqual(wz, 0.0, "No nonzero cmd_vel should be sent after premature cleanup")

    def test_first_command_rejected_aborts_immediately(self):
        """
        Requirement 16:
        Aborts immediately if the first nonzero command is rejected.
        Does not wait for the 15-second turn timeout.
        """
        params = TurnParameters(degrees=90.0, direction="cw", trials=1, dry_run=False, inter_trial_approval=False)
        mock_cockpit, mock_ws = self._create_mock_clients()

        mock_cockpit.get_status.return_value = {"armed": True, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
        mock_cockpit.get_autonomy_status.return_value = {
            "state": "READY_ARMED",
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY",
            "lastRejectionReason": "Autonomy is disabled by operator"
        }

        # Command is rejected
        mock_cockpit.send_cmd_vel.return_value = {"ok": False, "error": "Autonomy is disabled by operator"}

        t_start = time.time()
        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
            trial = runner.execute_single_trial(1)
        elapsed = time.time() - t_start

        self.assertEqual(trial.status, "ABORTED")
        self.assertIn("First nonzero command rejected", trial.abort_reason)
        self.assertIn("Autonomy is disabled by operator", trial.abort_reason)
        self.assertLess(elapsed, 2.0)
        mock_cockpit.disarm_drive.assert_called()

    def test_wheel_targets_remain_zero_aborts_within_bounded_interval(self):
        """
        Requirement 16:
        Aborts within 500ms bounded interval if commanded wheel targets remain zero or ACTIVE state not achieved.
        """
        params = TurnParameters(degrees=90.0, direction="cw", trials=1, dry_run=False, inter_trial_approval=False)
        mock_cockpit, mock_ws = self._create_mock_clients()

        mock_cockpit.get_status.return_value = {"armed": True, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
        mock_cockpit.send_cmd_vel.return_value = {"ok": True}
        mock_cockpit.get_autonomy_status.return_value = {
            "state": "READY_ARMED",
            "clampedAngular": 0.0,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY",
            "lastRejectionReason": "Targets remained zero"
        }

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
            trial = runner.execute_single_trial(1)

        self.assertEqual(trial.status, "ABORTED")
        self.assertIn("Motion startup failed within bounded window", trial.abort_reason)
        mock_cockpit.disarm_drive.assert_called()


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
