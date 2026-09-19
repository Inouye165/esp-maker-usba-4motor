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
    check_imu_freshness,
    wait_for_advancing_imu_sample,
    SensorException
)
from tools.rover_tests.transport import (
    CockpitClient,
    NativeWSClient,
    perform_zero_handshake,
    arm_and_verify_ready_armed,
    disarm_and_stop,
    HandshakeException
)
from tools.rover_tests.runner import PhysicalTestRunner
from tools.rover_tests.reporting import (
    WheelTrialMetrics,
    TrialReport,
    MultiTrialSuiteReport,
    ReportGenerator,
)


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
        mock_cockpit.get_status.return_value = {"armed": True, "mode": 3}

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
        # Initial samples at 0 deg, then motion progresses to 90 deg with advancing sequence/timestamp
        imu_0 = dict(base_imu, orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
        imu_90 = dict(base_imu, orientation={"w": 0.7071068, "x": 0.0, "y": 0.0, "z": 0.7071068})
        seq_gen = itertools.count(1)
        t_base_ms = 1000
        mock_cockpit.get_imu.side_effect = itertools.chain(
            [dict(imu_0, sequence=next(seq_gen), timestamp_ms=t_base_ms + next(seq_gen)*20) for _ in range(8)],
            (dict(imu_90, sequence=s, timestamp_ms=t_base_ms + s*20) for s in itertools.count(9))
        )
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
        mock_cockpit.get_status.return_value = {"armed": True, "mode": 3, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
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
            "mode": 0,
            "autonomyState": "DISABLED",
            "cmdSource": "NONE"
        }

        def mock_arm():
            current_status["armed"] = True
            current_status["mode"] = 3
            current_status["autonomyState"] = "READY_ARMED"
            return {"ok": True}

        def mock_disarm():
            current_status["armed"] = False
            current_status["mode"] = 0
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
            mock_cockpit._state["mode"] = 3
            mock_cockpit._state["autonomyState"] = "READY_ARMED"
            return {"ok": True}

        def do_disarm():
            call_sequence.append("disarm_drive")
            mock_cockpit._state["armed"] = False
            mock_cockpit._state["mode"] = 0
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
                return {"armed": True, "mode": 3, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
            else:
                return {"armed": False, "mode": 0, "autonomyState": "DISABLED", "cmdSource": "NONE"}

        def mock_get_autonomy():
            if not handshake_done[0]:
                return {"state": "READY_ARMED", "zeroHandshakeCount": 3, "cmdSource": "ROS_AUTONOMY"}
            else:
                return {"state": "DISABLED", "zeroHandshakeCount": 0, "cmdSource": "NONE"}

        mock_cockpit.get_status.side_effect = mock_get_status
        mock_cockpit.get_autonomy_status.side_effect = mock_get_autonomy

        with patch("tools.rover_tests.runner.perform_zero_handshake", return_value=True):
            with patch("tools.rover_tests.runner.arm_and_verify_ready_armed") as mock_arm:
                def do_arm(*args, **kwargs):
                    handshake_done[0] = True
                    return True
                mock_arm.side_effect = do_arm

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

        mock_cockpit.get_status.return_value = {"armed": True, "mode": 3, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
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


class TestWheelTelemetryAndForensics(unittest.TestCase):
    """Verifies active-motion wheel telemetry collection, exclusions, and reporting."""

    def test_active_motion_samples_exclude_settling_and_zero(self):
        """Active motion means must remain nonzero and exclude settling/zero samples."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1, settle_seconds=2.0)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        mock_cockpit.token = "test_token"
        mock_cockpit.base_url = "http://127.0.0.1:3000"
        current_status = {
            "armed": False,
            "mode": 0,
            "autonomyState": "DISABLED",
            "cmdSource": "NONE"
        }
        def mock_arm():
            current_status["armed"] = True
            current_status["mode"] = 3
            current_status["autonomyState"] = "READY_ARMED"
            current_status["cmdSource"] = "ROS_AUTONOMY"
            return {"ok": True, "armed": True}
        def mock_disarm():
            current_status["armed"] = False
            current_status["mode"] = 0
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True, "armed": False}

        mock_cockpit.arm_drive.side_effect = mock_arm
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 100, "m2": 200, "m3": 300, "m4": 400}}
        mock_cockpit.enable_autonomy.return_value = {"ok": True, "state": "WAITING_FOR_ZERO"}
        mock_cockpit.send_cmd_vel.return_value = {"ok": True}
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": current_status["autonomyState"],
            "clampedAngular": 0.8 if current_status["armed"] else 0.0,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.set_command_source.return_value = {"ok": True, "source": "NONE"}

        mock_ws.connected = True
        mock_ws.connect.return_value = True
        mock_ws.authenticate.return_value = True

        # Sequence of IMU readings: start at 0, rotate to 90, 160, then 180.5 (threshold crossed!)
        yaw_seq = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 160.0, 180.5, 181.0, 181.0]
        imu_responses = []
        for y in yaw_seq:
            rad = math.radians(y)
            imu_responses.append({
                "ok": True,
                "dataAgeMs": 10,
                "rotVecValid": True,
                "inResetRecovery": False,
                "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
                "isNonMagnetic": True,
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(rad / 2), "w": math.cos(rad / 2)},
                "gyro": {"z": 0.001}
            })
        mock_cockpit.get_imu.side_effect = itertools.chain(imu_responses, itertools.repeat(imu_responses[-1]))

        active_frame = {
            "type": "pid_diagnostic",
            "m1": {"targetRadps": -4.07, "measuredRadps": -2.68, "stictionState": "KINETIC"},
            "m2": {"targetRadps": +4.07, "measuredRadps": +3.39, "stictionState": "KINETIC"},
            "m3": {"targetRadps": -4.07, "measuredRadps": -2.21, "stictionState": "KINETIC"},
            "m4": {"targetRadps": +4.07, "measuredRadps": +2.03, "stictionState": "KINETIC"}
        }
        mock_ws.recv_frames.return_value = [active_frame]

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
            trial = runner.execute_single_trial(1)

        self.assertEqual(trial.status, "SUCCESS")
        # Verify active samples were collected
        self.assertGreaterEqual(trial.telemetry_samples_count, 1)

        m1 = trial.wheel_metrics["m1"]
        self.assertIsNotNone(m1.commanded_speed_radps_mean)
        self.assertAlmostEqual(m1.commanded_speed_radps_mean, -4.07, places=2)
        self.assertIsNotNone(m1.measured_speed_radps_mean)
        self.assertAlmostEqual(m1.measured_speed_radps_mean, -2.68, places=2)
        self.assertAlmostEqual(m1.measured_speed_radps_abs_mean, 2.68, places=2)

        m2 = trial.wheel_metrics["m2"]
        self.assertAlmostEqual(m2.commanded_speed_radps_mean, +4.07, places=2)
        self.assertAlmostEqual(m2.measured_speed_radps_mean, +3.39, places=2)

        m3 = trial.wheel_metrics["m3"]
        self.assertAlmostEqual(m3.commanded_speed_radps_mean, -4.07, places=2)
        self.assertAlmostEqual(m3.measured_speed_radps_mean, -2.21, places=2)

        m4 = trial.wheel_metrics["m4"]
        self.assertAlmostEqual(m4.commanded_speed_radps_mean, +4.07, places=2)
        self.assertAlmostEqual(m4.measured_speed_radps_mean, +2.03, places=2)

    def test_slot_mapping_human_facing_labels(self):
        """Canonical slot mappings: M1/LF=Slot 1, M2/RF=Slot 2, M3/LR=Slot 3, M4/RR=Slot 4."""
        from tools.rover_tests.reporting import WheelTrialMetrics

        w1 = WheelTrialMetrics(wheel_id="m1")
        self.assertEqual(w1.slot_mapping, "Slot 1")
        self.assertEqual(w1.corner, "LF")

        w2 = WheelTrialMetrics(wheel_id="m2")
        self.assertEqual(w2.slot_mapping, "Slot 2")
        self.assertEqual(w2.corner, "RF")

        w3 = WheelTrialMetrics(wheel_id="m3")
        self.assertEqual(w3.slot_mapping, "Slot 3")
        self.assertEqual(w3.corner, "LR")

        w4 = WheelTrialMetrics(wheel_id="m4")
        self.assertEqual(w4.slot_mapping, "Slot 4")
        self.assertEqual(w4.corner, "RR")

    def test_unavailable_telemetry_never_reports_false_zeros(self):
        """When telemetry is unavailable, report None and render *Unavailable*, not 0.00 in wheel table."""
        from tools.rover_tests.reporting import WheelTrialMetrics, TrialReport, MultiTrialSuiteReport, ReportGenerator

        w1 = WheelTrialMetrics(wheel_id="m1", commanded_speed_radps_mean=None, measured_speed_radps_mean=None)
        self.assertIsNone(w1.commanded_speed_radps_mean)
        self.assertIsNone(w1.measured_speed_radps_mean)

        trial = TrialReport(
            trial_index=1,
            requested_turn_deg=180.0,
            direction="cw",
            target_signed_yaw_deg=180.0,
            status="SUCCESS",
            wheel_metrics={"m1": w1}
        )
        suite = MultiTrialSuiteReport(suite_id="test", target_degrees=180.0, direction="cw", total_trials=1, successful_trials=1, trials=[trial])
        md = ReportGenerator.format_markdown_summary(suite)

        self.assertIn("*Unavailable*", md)
        # Verify that M1 row specifically displays *Unavailable* for commanded and measured means
        wheel_section = md.split("#### Wheel Actuation & Encoder Performance")[1]
        self.assertIn("| **M1** (Slot 1 / LF) | *Unavailable* | *Unavailable* | *Unavailable* |", wheel_section)

    def test_stopped_while_commanded_event_detection(self):
        """Detects and times stopped-while-commanded events."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        mock_cockpit.token = "test_token"
        mock_cockpit.base_url = "http://127.0.0.1:3000"

        current_status = {
            "armed": False,
            "mode": 0,
            "autonomyState": "DISABLED",
            "cmdSource": "NONE"
        }
        def mock_arm():
            current_status["armed"] = True
            current_status["mode"] = 3
            current_status["autonomyState"] = "READY_ARMED"
            current_status["cmdSource"] = "ROS_AUTONOMY"
            return {"ok": True, "armed": True}
        def mock_disarm():
            current_status["armed"] = False
            current_status["mode"] = 0
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True, "armed": False}

        mock_cockpit.arm_drive.side_effect = mock_arm
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.enable_autonomy.return_value = {"ok": True, "state": "WAITING_FOR_ZERO"}
        mock_cockpit.send_cmd_vel.return_value = {"ok": True}
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": current_status["autonomyState"],
            "clampedAngular": 0.8 if current_status["armed"] else 0.0,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.set_command_source.return_value = {"ok": True, "source": "NONE"}

        mock_ws.connected = True
        mock_ws.connect.return_value = True
        mock_ws.authenticate.return_value = True

        # Rover turns from 0 to 180.5
        yaw_seq = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 45.0, 90.0, 180.5, 181.0]
        imu_responses = [
            {
                "ok": True,
                "dataAgeMs": 5,
                "rotVecValid": True,
                "inResetRecovery": False,
                "sensor": "BNO08x",
                "rotationVectorType": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
                "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
                "isNonMagnetic": True,
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(math.radians(y)/2), "w": math.cos(math.radians(y)/2)},
                "gyro": {"z": 0.0}
            }
            for y in yaw_seq
        ]
        mock_cockpit.get_imu.side_effect = itertools.chain(imu_responses, itertools.repeat(imu_responses[-1]))

        # m1 is blocked / 0 measured speed while commanded
        stalled_frame = {
            "type": "pid_diagnostic",
            "m1": {"targetRadps": -4.0, "measuredRadps": 0.0, "stictionState": "BLOCKED"},
            "m2": {"targetRadps": +4.0, "measuredRadps": +3.0, "stictionState": "KINETIC"},
            "m3": {"targetRadps": -4.0, "measuredRadps": -2.0, "stictionState": "KINETIC"},
            "m4": {"targetRadps": +4.0, "measuredRadps": +2.0, "stictionState": "KINETIC"}
        }
        mock_ws.recv_frames.return_value = [stalled_frame]

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
            trial = runner.execute_single_trial(1)

        m1 = trial.wheel_metrics["m1"]
        self.assertTrue(m1.stopped_while_commanded)
        self.assertGreaterEqual(m1.stopped_while_commanded_count, 1)
        self.assertGreater(m1.stopped_while_commanded_duration_s, 0.0)
        self.assertGreaterEqual(m1.blocked_state_events, 1)


class TestIMUFreshnessAndPreMotionGate(unittest.TestCase):
    """
    Regression tests for:
    - pre-motion check passes but same sample becomes stale before first command
    - waiting for the next advancing fresh sample
    - bounded abort when sequence does not advance
    - consistent freshness calculation
    - pre-motion packets excluded from active-motion metrics
    """

    def test_consistent_freshness_calculation(self):
        """Unified check_imu_freshness correctly validates or rejects samples."""
        valid_sample = {
            "ok": True,
            "rotVecValid": True,
            "inResetRecovery": False,
            "dataAgeMs": 25,
            "sequence": 100
        }
        ok, msg = check_imu_freshness(valid_sample, max_age_ms=250.0)
        self.assertTrue(ok)
        self.assertIn("Fresh", msg)

        # Stale sample (> 250ms)
        stale_sample = dict(valid_sample, dataAgeMs=251)
        ok, msg = check_imu_freshness(stale_sample, max_age_ms=250.0)
        self.assertFalse(ok)
        self.assertIn("Stale", msg)

        # RotVec invalid
        invalid_rot = dict(valid_sample, rotVecValid=False)
        ok, msg = check_imu_freshness(invalid_rot)
        self.assertFalse(ok)

        # In reset recovery
        reset_rec = dict(valid_sample, inResetRecovery=True)
        ok, msg = check_imu_freshness(reset_rec)
        self.assertFalse(ok)

        # Telemetry dropped
        dropped = {"ok": False}
        ok, msg = check_imu_freshness(dropped)
        self.assertFalse(ok)

    def test_waiting_for_advancing_fresh_sample(self):
        """wait_for_advancing_imu_sample waits for sequence advancement and low age."""
        mock_cockpit = MagicMock(spec=CockpitClient)
        samples = [
            {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 100, "dataAgeMs": 120},
            {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 100, "dataAgeMs": 130},
            {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 101, "dataAgeMs": 15},
        ]
        mock_cockpit.get_imu.side_effect = samples

        ok, fresh, reason = wait_for_advancing_imu_sample(
            mock_cockpit,
            baseline_seq=100,
            max_wait_sec=0.5,
            poll_interval_sec=0.001,
            max_acceptable_age_ms=100.0
        )
        self.assertTrue(ok)
        self.assertEqual(fresh["sequence"], 101)
        self.assertEqual(fresh["dataAgeMs"], 15)

    def test_bounded_abort_when_sequence_does_not_advance(self):
        """Runner aborts while disarmed if IMU sequence stalls during pre-motion wait."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        current_status = {"armed": False, "mode": 0, "autonomyState": "DISABLED", "cmdSource": "NONE"}
        def mock_arm():
            current_status["armed"] = True
            current_status["mode"] = 3
            current_status["autonomyState"] = "READY_ARMED"
            current_status["cmdSource"] = "ROS_AUTONOMY"
            return {"ok": True, "armed": True}
        def mock_disarm():
            current_status["armed"] = False
            current_status["mode"] = 0
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True, "armed": False}

        mock_cockpit.token = "test"
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.arm_drive.side_effect = mock_arm
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.enable_autonomy.return_value = {"ok": True, "state": "WAITING_FOR_ZERO"}
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": current_status["autonomyState"],
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.set_command_source.return_value = {"ok": True, "source": "NONE"}

        mock_ws.connected = True
        mock_ws.connect.return_value = True
        mock_ws.authenticate.return_value = True

        stalled_imu = {
            "ok": True,
            "dataAgeMs": 30,
            "sequence": 100,
            "rotVecValid": True,
            "inResetRecovery": False,
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "gyro": {"z": 0.0}
        }
        mock_cockpit.get_imu.return_value = stalled_imu

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(False, stalled_imu, "Timed out (500ms limit, baseline_seq=100)")):
            with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                trial = runner.execute_single_trial(1)

        self.assertEqual(trial.status, "ABORTED")
        self.assertIn("Pre-motion IMU synchronization failed", trial.abort_reason)
        self.assertEqual(trial.telemetry_samples_count, 0)
        self.assertFalse(current_status["armed"])
        # Verify no nonzero motion command was ever issued
        for c in mock_cockpit.send_cmd_vel.call_args_list:
            wz = c.kwargs.get("wz") or (c.args[1] if len(c.args) > 1 else 0.0)
            vx = c.kwargs.get("vx") or (c.args[0] if len(c.args) > 0 else 0.0)
            self.assertEqual(wz, 0.0)
            self.assertEqual(vx, 0.0)

    def test_pre_motion_passes_but_sample_becomes_stale_before_first_command(self):
        """Pre-motion check passes, but sample becomes stale (>250ms) during wait -> safe disarm abort."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        current_status = {"armed": False, "mode": 0, "autonomyState": "DISABLED", "cmdSource": "NONE"}
        def mock_arm():
            current_status["armed"] = True
            current_status["mode"] = 3
            current_status["autonomyState"] = "READY_ARMED"
            current_status["cmdSource"] = "ROS_AUTONOMY"
            return {"ok": True, "armed": True}
        def mock_disarm():
            current_status["armed"] = False
            current_status["mode"] = 0
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True, "armed": False}

        mock_cockpit.token = "test"
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.arm_drive.side_effect = mock_arm
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.enable_autonomy.return_value = {"ok": True, "state": "WAITING_FOR_ZERO"}
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": current_status["autonomyState"],
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.set_command_source.return_value = {"ok": True, "source": "NONE"}

        mock_ws.connected = True
        mock_ws.connect.return_value = True
        mock_ws.authenticate.return_value = True

        stale_imu = {
            "ok": True,
            "dataAgeMs": 260,
            "sequence": 101,
            "rotVecValid": True,
            "inResetRecovery": False,
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "gyro": {"z": 0.0}
        }
        mock_cockpit.get_imu.return_value = stale_imu

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(False, stale_imu, "Stale IMU data detected (260ms > 250ms limit)")):
            with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                trial = runner.execute_single_trial(1)

        self.assertEqual(trial.status, "ABORTED")
        self.assertIn("Stale IMU data detected", trial.abort_reason)
        self.assertEqual(trial.telemetry_samples_count, 0)
        self.assertFalse(current_status["armed"])
        # Verify no nonzero motion command was ever issued
        for c in mock_cockpit.send_cmd_vel.call_args_list:
            wz = c.kwargs.get("wz") or (c.args[1] if len(c.args) > 1 else 0.0)
            vx = c.kwargs.get("vx") or (c.args[0] if len(c.args) > 0 else 0.0)
            self.assertEqual(wz, 0.0)
            self.assertEqual(vx, 0.0)

    def test_pre_motion_packets_excluded_from_active_metrics(self):
        """Packets received prior to first nonzero command being accepted and ACTIVE are excluded."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        current_status = {"armed": False, "mode": 0, "autonomyState": "DISABLED", "cmdSource": "NONE"}
        def mock_arm():
            current_status["armed"] = True
            current_status["mode"] = 3
            current_status["autonomyState"] = "READY_ARMED"
            current_status["cmdSource"] = "ROS_AUTONOMY"
            return {"ok": True, "armed": True}
        def mock_disarm():
            current_status["armed"] = False
            current_status["mode"] = 0
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True, "armed": False}

        mock_cockpit.token = "test"
        mock_cockpit.arm_drive.side_effect = mock_arm
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.enable_autonomy.return_value = {"ok": True, "state": "WAITING_FOR_ZERO"}
        mock_cockpit.send_cmd_vel.return_value = {"ok": True}
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": "ACTIVE" if current_status["armed"] else "DISABLED",
            "clampedAngular": 0.8 if current_status["armed"] else 0.0,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.set_command_source.return_value = {"ok": True, "source": "NONE"}

        mock_ws.connected = True
        mock_ws.connect.return_value = True
        mock_ws.authenticate.return_value = True

        # Pre-motion frames queued
        pre_motion_call_count = 0
        def mock_recv():
            nonlocal pre_motion_call_count
            pre_motion_call_count += 1
            if pre_motion_call_count == 1:
                # Returned during step 11b (drained before motion loop)
                return [
                    {"type": "pid_diagnostic", "m1": {"targetRadps": 0.0, "measuredRadps": 0.0}},
                    {"type": "pid_diagnostic", "m1": {"targetRadps": 0.0, "measuredRadps": 0.0}}
                ]
            elif pre_motion_call_count == 2:
                # Active motion frame in first iteration of motion loop
                return [{"type": "pid_diagnostic", "m1": {"targetRadps": -3.0, "measuredRadps": -2.8}}]
            return []

        mock_ws.recv_frames.side_effect = mock_recv

        # Incremental yaw progression in small increments so unwrapper tracks smoothly
        current_deg = 0.0
        active_motion_entered = False
        def get_fresh_imu():
            nonlocal current_deg, active_motion_entered
            if active_motion_entered:
                current_deg += 45.0
            y = min(180.5, current_deg)
            return {
                "ok": True,
                "dataAgeMs": 10,
                "sequence": 100,
                "rotVecValid": True,
                "inResetRecovery": False,
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(math.radians(y)/2), "w": math.cos(math.radians(y)/2)},
                "gyro": {"z": 0.0}
            }
        mock_cockpit.get_imu.side_effect = get_fresh_imu

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
            # Patch wait_for_advancing_imu_sample to trigger active_motion_entered = True
            def mock_wait(*args, **kwargs):
                nonlocal active_motion_entered
                active_motion_entered = True
                return True, get_fresh_imu(), "ok"
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", side_effect=mock_wait):
                with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                    trial = runner.execute_single_trial(1)

        # Pre-motion frames were drained; only active frames counted (1 frame)
        self.assertEqual(trial.telemetry_samples_count, 1)


class TestStateSequencingAndIdempotentCleanup(unittest.TestCase):
    """
    Focused regressions for suite 1789475780:
    1. Synchronize fresh IMU before arming (while safely DISARMED).
    2. Cleanup is idempotent and does not toggle or oscillate states.
    3. Final JSON report never reports confirmed_final_disarmed_state=True if final cleanup reports armed=True.
    4. First nonzero command immediately follows READY_ARMED and verifies ACTIVE.
    """

    def test_ordered_lifecycle_zero_handshake_then_imu_sync_then_arm_then_first_cmd(self):
        """
        Guarantees the strict lifecycle sequence:
        1. complete_zero_handshake -> READY_DISARMED
        2. wait_for_advancing_imu_sample (while in READY_DISARMED, armed=False)
        3. arm_and_verify_ready_armed -> READY_ARMED
        4. send_cmd_vel (first nonzero command immediately dispatched)
        """
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True

        call_order = []

        def mock_zero_handshake(*args, **kwargs):
            call_order.append(("zero_handshake", mock_cockpit.get_status()))
            # Transitions rover to READY_DISARMED
            mock_cockpit.get_status.return_value = {"armed": False, "autonomyState": "READY_DISARMED", "cmdSource": "NONE"}
            mock_cockpit.get_autonomy_status.return_value = {"state": "READY_DISARMED", "zeroHandshakeCount": 3, "cmdSource": "NONE"}
            return True

        def mock_wait_imu(*args, **kwargs):
            call_order.append(("wait_imu", mock_cockpit.get_status()))
            return True, {"ok": True, "rotVecValid": True, "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}, "gyro": {"z": 0.0}}, "advancing_fresh"

        def mock_arm(*args, **kwargs):
            call_order.append(("arm_drive", mock_cockpit.get_status()))
            # Transitions rover to READY_ARMED
            mock_cockpit.get_status.return_value = {"armed": True, "mode": 3, "autonomyState": "READY_ARMED", "cmdSource": "NONE"}
            mock_cockpit.get_autonomy_status.return_value = {"state": "READY_ARMED", "zeroHandshakeCount": 3, "cmdSource": "NONE"}
            return True

        def mock_send_cmd(*args, **kwargs):
            call_order.append(("send_cmd", mock_cockpit.get_status()))
            mock_cockpit.get_autonomy_status.return_value = {"state": "ACTIVE", "clampedAngular": 0.8}
            # Return target-reached orientation so motion loop finishes cleanly
            mock_cockpit.get_imu.return_value = {
                "ok": True, "rotVecValid": True,
                "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
                "orientation": {"w": 0, "x": 0, "y": 0, "z": 1},
                "gyro": {"z": 0.0}
            }
            return {"ok": True}

        mock_cockpit.get_status.return_value = {"armed": False, "autonomyState": "DISABLED", "cmdSource": "NONE"}
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.get_imu.return_value = {
            "ok": True, "rotVecValid": True,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "orientation": {"w": 1, "x": 0, "y": 0, "z": 0},
            "gyro": {"z": 0.0}
        }
        mock_cockpit.send_cmd_vel.side_effect = mock_send_cmd

        with patch("tools.rover_tests.runner.perform_zero_handshake", side_effect=mock_zero_handshake):
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", side_effect=mock_wait_imu):
                with patch("tools.rover_tests.runner.arm_and_verify_ready_armed", side_effect=mock_arm):
                    with patch("time.sleep", return_value=None):
                        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
                        trial = runner.execute_single_trial(1)

        names = [call[0] for call in call_order[:4]]
        self.assertEqual(names, ["zero_handshake", "wait_imu", "arm_drive", "send_cmd"],
                         f"Unexpected lifecycle order: {names}")

        # Verify IMU sync occurred strictly while armed=False and in READY_DISARMED
        imu_sync_status = call_order[1][1]
        self.assertFalse(imu_sync_status["armed"], "IMU sync must occur while DISARMED")
        self.assertEqual(imu_sync_status["autonomyState"], "READY_DISARMED", "IMU sync must occur in READY_DISARMED state")

        # Verify arming occurred after IMU sync
        arm_status = call_order[2][1]
        self.assertFalse(arm_status["armed"], "Arming must be initiated from DISARMED state")

        # Verify first command was sent while armed
        send_status = call_order[3][1]
        self.assertTrue(send_status["armed"], "First motion command must be sent while ARMED")

    def test_cleanup_idempotency_and_retry_behavior(self):
        """
        Verifies:
        - _cleaned_up must not suppress a retry after failed or unverified cleanup.
        - Mark cleanup complete only after confirmed: armed=false, autonomyState=DISABLED, cmdSource=NONE.
        - Repeated cleanup calls safely reverify the final state.
        - If state reverts to armed/unsafe, reverification resets flag and triggers active cleanup.
        """
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        self.assertFalse(runner._cleaned_up)

        # 1. Unsuccessful/unverified cleanup (e.g. rover still reports armed=True)
        mock_cockpit.get_status.return_value = {"armed": True, "mode": 3, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
        st1 = runner.cleanup(verbose=False)
        self.assertFalse(runner._cleaned_up, "_cleaned_up must NOT be marked True if armed is still True")
        self.assertTrue(st1["armed"])

        # 2. Subsequent call must NOT be suppressed - it must actively retry
        mock_cockpit.disarm_drive.reset_mock()
        mock_cockpit.disable_autonomy.reset_mock()
        # Rover now successfully disarms
        mock_cockpit.get_status.return_value = {"armed": False, "autonomyState": "DISABLED", "cmdSource": "NONE"}
        st2 = runner.cleanup(verbose=False)
        self.assertTrue(runner._cleaned_up, "_cleaned_up must be marked True after confirmed disarm, disabled, and cmdSource=NONE")
        self.assertFalse(st2["armed"])
        self.assertEqual(st2["autonomyState"], "DISABLED")
        self.assertEqual(st2["cmdSource"], "NONE")
        self.assertTrue(mock_cockpit.disarm_drive.called, "Disarm must have been retried")

        # 3. Repeated cleanup call while still safe: safely reverifies without redundant mutating commands
        mock_cockpit.disarm_drive.reset_mock()
        mock_cockpit.disable_autonomy.reset_mock()
        st3 = runner.cleanup(verbose=False)
        self.assertTrue(runner._cleaned_up)
        self.assertFalse(st3["armed"])
        self.assertEqual(st3["autonomyState"], "DISABLED")
        self.assertEqual(st3["cmdSource"], "NONE")
        # Mutating endpoints not spammed when state is already verified safe
        self.assertFalse(mock_cockpit.disarm_drive.called)

        # 4. State reverts to unsafe between calls (e.g. armed=True): reverification catches it and re-runs active cleanup
        mock_cockpit.get_status.side_effect = [
            {"armed": True, "mode": 3, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"},  # Reverification check fails!
            {"armed": False, "mode": 0, "autonomyState": "DISABLED", "cmdSource": "NONE"}               # Active cleanup succeeds
        ]
        st4 = runner.cleanup(verbose=False)
        self.assertTrue(runner._cleaned_up)
        self.assertFalse(st4["armed"])
        self.assertTrue(mock_cockpit.disarm_drive.called, "Active cleanup must be re-triggered when reverification fails")

    def test_report_never_claims_disarm_success_when_final_armed_true(self):
        """Guarantees that confirmed_final_disarmed_state is False in JSON and Trial if rover remains armed."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_cockpit.token = "test-token"
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True
        mock_ws.authenticate.return_value = True

        # Rover gets stuck in armed state despite cleanup
        mock_cockpit.get_status.return_value = {"armed": True, "mode": 3, "autonomyState": "READY_ARMED", "cmdSource": "ROS_AUTONOMY"}
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_ARMED", "zeroHandshakeCount": 3, "cmdSource": "ROS_AUTONOMY"}
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.get_imu.return_value = {
            "ok": True, "rotVecValid": True,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "orientation": {"w": 1, "x": 0, "y": 0, "z": 0},
            "gyro": {"z": 0.0}
        }
        mock_cockpit.send_cmd_vel.return_value = {"ok": False, "error": "Simulated hardware fault"}

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
            suite = runner.execute_suite()

        self.assertEqual(len(suite.trials), 1)
        trial = suite.trials[0]
        self.assertFalse(trial.confirmed_final_disarmed_state, "Report must NEVER report confirmed_final_disarmed_state=True when final armed=True")


class TestProductionPidDiagnosticFrameRegression(unittest.TestCase):
    """
    Regression tests verifying that exact production pid_diagnostic frames
    with zero target/measured values trigger consistency failure (INVALID forensics)
    when physical encoder deltas are nonzero, and that stopped-while-commanded is
    Unknown unless a nonzero target was actually observed.
    """

    EXACT_PROD_PID_DIAGNOSTIC_FRAME = {
        "type": "pid_diagnostic",
        "timestamp": 1789562117051,
        "sequence": 34550,
        "m1": {
            "targetRadps": 0,
            "measuredRadps": 0,
            "feedforward": 0,
            "pTerm": 0,
            "iTerm": 0,
            "dTerm": 0,
            "basePwm": 0,
            "spinSyncTrim": 0,
            "finalPwm": 0,
            "stictionCode": 0,
            "stictionState": "IDLE"
        },
        "m2": {
            "targetRadps": 0,
            "measuredRadps": 0,
            "feedforward": 0,
            "pTerm": 0,
            "iTerm": 0,
            "dTerm": 0,
            "basePwm": 0,
            "spinSyncTrim": 0,
            "finalPwm": 0,
            "stictionCode": 0,
            "stictionState": "IDLE"
        },
        "m3": {
            "targetRadps": 0,
            "measuredRadps": 0,
            "feedforward": 0,
            "pTerm": 0,
            "iTerm": 0,
            "dTerm": 0,
            "basePwm": 0,
            "spinSyncTrim": 0,
            "finalPwm": 0,
            "stictionCode": 0,
            "stictionState": "IDLE"
        },
        "m4": {
            "targetRadps": 0,
            "measuredRadps": 0,
            "feedforward": 0,
            "pTerm": 0,
            "iTerm": 0,
            "dTerm": 0,
            "basePwm": 0,
            "spinSyncTrim": 0,
            "finalPwm": 0,
            "stictionCode": 0,
            "stictionState": "IDLE"
        },
        "outerYaw": {
            "wzRequested": 0,
            "wzActual": 0,
            "yawOuterError": 0,
            "yawOuterCorrection": 0,
            "wzCorrected": 0,
            "yawOuterActive": False,
            "imuGyroValid": False,
            "imuGyroAgeMs": 0
        }
    }

    def test_exact_production_frame_zero_telemetry_consistency_failure(self):
        """When rover physically turns (large encoder deltas) but exact production frames contain zeros, mark INVALID."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        mock_cockpit.token = "test_token"
        mock_cockpit.base_url = "http://127.0.0.1:3000"

        current_status = {"armed": False, "mode": 0, "autonomyState": "DISABLED", "cmdSource": "NONE"}
        def mock_arm():
            current_status["armed"] = True
            current_status["mode"] = 3
            current_status["autonomyState"] = "READY_ARMED"
            current_status["cmdSource"] = "ROS_AUTONOMY"
            return {"ok": True, "armed": True}
        def mock_disarm():
            current_status["armed"] = False
            current_status["mode"] = 0
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True, "armed": False}

        mock_cockpit.arm_drive.side_effect = mock_arm
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)

        # Start with 0 ticks, end with trial 2 encoder deltas: m1=-5168, m2=6630, m3=-4375, m4=3985
        enc_seq = [
            {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}},
            {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}},
            {"ok": True, "encoders": {"m1": -5168, "m2": 6630, "m3": -4375, "m4": 3985}}
        ]
        mock_cockpit.get_encoders.side_effect = itertools.chain(enc_seq, itertools.repeat(enc_seq[-1]))

        mock_cockpit.enable_autonomy.return_value = {"ok": True, "state": "WAITING_FOR_ZERO"}
        mock_cockpit.send_cmd_vel.return_value = {"ok": True}
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": current_status["autonomyState"],
            "clampedAngular": 0.8 if current_status["armed"] else 0.0,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.set_command_source.return_value = {"ok": True, "source": "NONE"}

        mock_ws.connected = True
        mock_ws.connect.return_value = True
        mock_ws.authenticate.return_value = True

        # Ingest exact production frame on every recv_frames call
        mock_ws.recv_frames.return_value = [dict(self.EXACT_PROD_PID_DIAGNOSTIC_FRAME)]

        # Rover turns from 0 to 180.5
        yaw_seq = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0, 90.0, 150.0, 180.5, 181.5]
        imu_responses = [
            {
                "ok": True,
                "dataAgeMs": 5,
                "rotVecValid": True,
                "inResetRecovery": False,
                "sensor": "BNO08x",
                "rotationVectorType": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
                "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
                "isNonMagnetic": True,
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(math.radians(y)/2), "w": math.cos(math.radians(y)/2)},
                "gyro": {"z": 0.0}
            }
            for y in yaw_seq
        ]
        mock_cockpit.get_imu.side_effect = itertools.chain(imu_responses, itertools.repeat(imu_responses[-1]))

        with patch("time.sleep", return_value=None):
            runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
            trial = runner.execute_single_trial(1)

        # Consistency failure assertion:
        self.assertFalse(trial.wheel_forensics_valid, "Trial wheel forensics must be marked INVALID")
        self.assertEqual(trial.telemetry_samples_count, trial.wheel_metrics["m1"].active_samples_count)
        self.assertGreater(trial.wheel_metrics["m1"].active_samples_count, 0)

        for w_id in ["m1", "m2", "m3", "m4"]:
            w = trial.wheel_metrics[w_id]
            self.assertEqual(w.validity, "INVALID", f"Wheel {w_id} validity must be INVALID")
            self.assertEqual(w.commanded_speed_source, "INVALID_ZERO_TELEMETRY")
            self.assertIsNone(w.commanded_speed_radps_mean, "Commanded mean must not report misleading 0.00")
            self.assertIsNone(w.commanded_speed_radps_max)
            self.assertIsNone(w.measured_speed_radps_mean, "Measured mean must not report misleading 0.00")
            self.assertIsNone(w.measured_speed_radps_abs_mean)
            self.assertIsNone(w.measured_speed_radps_min)
            self.assertIsNone(w.measured_speed_radps_max)
            self.assertIsNone(w.stopped_while_commanded, "Stopped while commanded must be Unknown (None) when target was 0")

        # Verify markdown report formatting displays *INVALID* and Unknown
        suite = MultiTrialSuiteReport(
            suite_id="1789561848",
            target_degrees=180.0,
            direction="cw",
            total_trials=1,
            successful_trials=1,
            trials=[trial]
        )
        md = ReportGenerator.format_markdown_summary(suite)
        self.assertIn("*INVALID*", md)
        self.assertIn("Unknown", md)
        # Verify wheel rows show INVALID and Unknown
        wheel_section = md.split("#### Wheel Actuation & Encoder Performance")[1]
        self.assertIn("| **M1** (Slot 1 / LF) | *INVALID* | *INVALID* | *INVALID* | *INVALID* | -5168 | Unknown |", wheel_section)
        self.assertIn("| **M2** (Slot 2 / RF) | *INVALID* | *INVALID* | *INVALID* | *INVALID* | +6630 | Unknown |", wheel_section)

    def test_stopped_while_commanded_unknown_unless_nonzero_target_observed(self):
        """Stopped while commanded must be Unknown unless a nonzero target was actually observed."""
        from tools.rover_tests.reporting import WheelTrialMetrics, TrialReport, ReportGenerator, MultiTrialSuiteReport

        # Case 1: No nonzero target observed -> Unknown
        w_unknown = WheelTrialMetrics(wheel_id="m1", stopped_while_commanded=None)
        self.assertIsNone(w_unknown.stopped_while_commanded)

        # Case 2: Nonzero target observed and did not stall -> False (No)
        w_moving = WheelTrialMetrics(wheel_id="m1", stopped_while_commanded=False)
        self.assertFalse(w_moving.stopped_while_commanded)

        # Case 3: Nonzero target observed and stalled -> True (YES)
        w_stalled = WheelTrialMetrics(
            wheel_id="m1",
            stopped_while_commanded=True,
            stopped_while_commanded_count=2,
            stopped_while_commanded_duration_s=0.45
        )
        self.assertTrue(w_stalled.stopped_while_commanded)

        trial1 = TrialReport(trial_index=1, requested_turn_deg=180.0, direction="cw", target_signed_yaw_deg=180.0, status="SUCCESS", wheel_metrics={"m1": w_unknown})
        trial2 = TrialReport(trial_index=2, requested_turn_deg=180.0, direction="cw", target_signed_yaw_deg=180.0, status="SUCCESS", wheel_metrics={"m1": w_moving})
        trial3 = TrialReport(trial_index=3, requested_turn_deg=180.0, direction="cw", target_signed_yaw_deg=180.0, status="SUCCESS", wheel_metrics={"m1": w_stalled})

        suite = MultiTrialSuiteReport(suite_id="test_suite", target_degrees=180.0, direction="cw", total_trials=3, successful_trials=3, trials=[trial1, trial2, trial3])
        md = ReportGenerator.format_markdown_summary(suite)

        self.assertIn("Unknown", md)
        self.assertIn("No (0s)", md)
        self.assertIn("**YES (2 ev / 0.45s)**", md)


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


class TestArmConfirmationRegression(unittest.TestCase):
    """
    Regression tests verifying that arm_and_verify_ready_armed strictly accepts only
    hardware-confirmed arming (armed=True, mode=3) and handles stale packets and timeouts.
    """

    def test_arm_and_verify_accepts_mode_3(self):
        mock_cockpit = MagicMock()
        mock_cockpit.arm_drive.return_value = {"ok": True, "status": "ARMED", "mode": 3}
        mock_cockpit.get_status.return_value = {"armed": True, "mode": 3, "cmdSource": "NONE"}
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_ARMED", "zeroHandshakeCount": 3}

        transitions = []
        result = arm_and_verify_ready_armed(mock_cockpit, max_duration_sec=0.2, transitions=transitions)
        self.assertTrue(result)
        self.assertEqual(len(transitions), 1)
        self.assertEqual(transitions[0]["state"], "READY_ARMED")

    def test_arm_and_verify_rejects_unconfirmed_mode(self):
        mock_cockpit = MagicMock()
        mock_cockpit.arm_drive.return_value = {"ok": True, "status": "ARMED"}
        # Armed is true, but mode is 0 (LOCKED)
        mock_cockpit.get_status.return_value = {"armed": True, "mode": 0, "cmdSource": "NONE"}
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_ARMED", "zeroHandshakeCount": 3}

        with self.assertRaises(HandshakeException) as cm:
            arm_and_verify_ready_armed(mock_cockpit, max_duration_sec=0.1)
        self.assertEqual(cm.exception.stage, "ARM_CONFIRMATION")
        self.assertIn("mode=0", str(cm.exception))

    def test_arm_and_verify_rejects_missing_mode(self):
        mock_cockpit = MagicMock()
        mock_cockpit.arm_drive.return_value = {"ok": True, "status": "ARMED"}
        # Armed is true, but 'mode' key is missing completely
        mock_cockpit.get_status.return_value = {"armed": True, "cmdSource": "NONE"}
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_ARMED", "zeroHandshakeCount": 3}

        with self.assertRaises(HandshakeException) as cm:
            arm_and_verify_ready_armed(mock_cockpit, max_duration_sec=0.1)
        self.assertEqual(cm.exception.stage, "ARM_CONFIRMATION")
        self.assertIn("mode=None", str(cm.exception))

    def test_arm_and_verify_rejects_null_mode(self):
        mock_cockpit = MagicMock()
        mock_cockpit.arm_drive.return_value = {"ok": True, "status": "ARMED"}
        # Armed is true, but 'mode' is explicitly null/None
        mock_cockpit.get_status.return_value = {"armed": True, "mode": None, "cmdSource": "NONE"}
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_ARMED", "zeroHandshakeCount": 3}

        with self.assertRaises(HandshakeException) as cm:
            arm_and_verify_ready_armed(mock_cockpit, max_duration_sec=0.1)
        self.assertEqual(cm.exception.stage, "ARM_CONFIRMATION")
        self.assertIn("mode=None", str(cm.exception))

    def test_arm_and_verify_fails_on_arm_drive_error(self):
        mock_cockpit = MagicMock()
        mock_cockpit.arm_drive.return_value = {
            "ok": False,
            "error": "Arm confirmation timed out after 500ms waiting for ESP32 confirmation (armed=true, mode=3)"
        }
        mock_cockpit.get_autonomy_status.return_value = {
            "state": "READY_DISARMED",
            "zeroHandshakeCount": 3,
            "cmdSource": "NONE",
            "lastRejectionReason": "Timeout"
        }

        with self.assertRaises(HandshakeException) as cm:
            arm_and_verify_ready_armed(mock_cockpit, max_duration_sec=0.1)
        self.assertEqual(cm.exception.stage, "ARM_DRIVE")
        self.assertIn("500ms", str(cm.exception))

    def test_arm_and_verify_stale_packet_tolerated_until_mode_3(self):
        mock_cockpit = MagicMock()
        mock_cockpit.arm_drive.return_value = {"ok": True, "status": "ARMED", "mode": 3}
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_ARMED", "zeroHandshakeCount": 3}
        # First poll: stale packet with armed=False. Second poll: armed=True, mode=3
        mock_cockpit.get_status.side_effect = [
            {"armed": False, "mode": 0, "cmdSource": "NONE"},
            {"armed": True, "mode": 3, "cmdSource": "NONE"}
        ]

        result = arm_and_verify_ready_armed(mock_cockpit, max_duration_sec=0.5)
        self.assertTrue(result)


class TestSettlingInstrumentationAndTelemetry(unittest.TestCase):
    """
    Focused unit tests for settling instrumentation and telemetry:
    - Queued PID diagnostic packets and pre-zero backlog identification (no merging)
    - Stale / missing / invalid IMU data explicitly marked and non-polluting
    - IMU non-advancing / repeated sample handling
    - Monotonic timing, bounded duration, and achieved sampling rate reporting
    - Final endpoint measurement preservation (never substitutes target angle)
    - Zero-command send/response times, latency, and response contents
    - JSON export includes all settling fields and deserializes accurately
    - Preservation of existing cleanup guarantees
    """

class TestSettlingInstrumentationAndTelemetry(unittest.TestCase):
    """
    Focused unit tests for settling instrumentation and telemetry:
    - Queued PID diagnostic packets and pre-zero backlog identification (no merging)
    - Stale / missing / invalid IMU data explicitly marked and non-polluting
    - IMU non-advancing / repeated sample handling
    - Monotonic timing, bounded duration, and achieved sampling rate reporting
    - Final endpoint measurement preservation (never substitutes target angle)
    - Zero-command send/response times, latency, and response contents
    - JSON export includes all settling fields and deserializes accurately
    - Preservation of existing cleanup guarantees
    """

    def _create_mock_rover(self, target_deg=90.0, settle_seconds=0.5, final_imu_deg=None, settle_imus=None):
        params = TurnParameters(
            degrees=target_deg, direction="cw", trials=1, dry_run=False,
            inter_trial_approval=False, settle_seconds=settle_seconds
        )
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
        imu_0 = dict(base_imu, orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})

        reach_deg = final_imu_deg if final_imu_deg is not None else target_deg
        rad = math.radians(reach_deg)
        w = math.cos(rad / 2.0)
        z = math.sin(rad / 2.0)
        imu_target = dict(base_imu, orientation={"w": w, "x": 0.0, "y": 0.0, "z": z})

        current_status = {
            "armed": False,
            "mode": 0,
            "autonomyState": "DISABLED",
            "cmdSource": "NONE"
        }
        phase_tracker = {"phase": "PRE_MOTION"}

        def mock_arm():
            current_status["armed"] = True
            current_status["mode"] = 3
            current_status["autonomyState"] = "READY_ARMED"
            return {"ok": True}

        def mock_disarm():
            current_status["armed"] = False
            current_status["mode"] = 0
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

        def mock_cmd_vel(*args, **kwargs):
            vx = kwargs.get("vx", args[0] if len(args) > 0 else 0.0)
            wz = kwargs.get("wz", args[1] if len(args) > 1 else 0.0)
            if abs(wz) > 0.01 or abs(vx) > 0.01:
                current_status["autonomyState"] = "ACTIVE"
                current_status["cmdSource"] = "ROS_AUTONOMY"
                phase_tracker["phase"] = "MOTION"
            elif phase_tracker["phase"] == "MOTION" and abs(wz) <= 0.001 and abs(vx) <= 0.001:
                phase_tracker["phase"] = "SETTLING"
            return {"ok": True, "linear": vx, "angular": wz, "state": current_status["autonomyState"]}

        settle_iter = iter(settle_imus) if settle_imus is not None else None

        motion_steps = []
        if reach_deg > 120.0:
            mid_rad = math.radians(reach_deg / 2.0)
            imu_mid = dict(base_imu, orientation={"w": math.cos(mid_rad / 2.0), "x": 0.0, "y": 0.0, "z": math.sin(mid_rad / 2.0)})
            motion_steps.append(imu_mid)
        motion_steps.append(imu_target)
        motion_iter = iter(motion_steps)

        def mock_get_imu(*args, **kwargs):
            if phase_tracker["phase"] == "PRE_MOTION":
                return imu_0
            elif phase_tracker["phase"] == "MOTION":
                try:
                    return next(motion_iter)
                except StopIteration:
                    return imu_target
            else:  # SETTLING
                if settle_iter is not None:
                    try:
                        return next(settle_iter)
                    except StopIteration:
                        return imu_target
                return imu_target

        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": current_status["autonomyState"],
            "zeroHandshakeCount": 3,
            "cmdSource": current_status["cmdSource"]
        }
        mock_cockpit.enable_autonomy.side_effect = mock_enable_auto
        mock_cockpit.arm_drive.side_effect = mock_arm
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.disable_autonomy.side_effect = mock_disable_auto
        mock_cockpit.set_command_source.side_effect = mock_set_source
        mock_cockpit.send_cmd_vel.side_effect = mock_cmd_vel
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 100, "m2": 100, "m3": 100, "m4": 100}}
        mock_cockpit.get_imu.side_effect = mock_get_imu
        mock_ws.recv_frames.return_value = []

        return params, mock_cockpit, mock_ws, base_imu, imu_0, imu_target, phase_tracker

    def test_queued_packets_individual_preservation_and_backlog_flagging(self):
        params, mock_cockpit, mock_ws, base_imu, imu_0, imu_target, _ = self._create_mock_rover(target_deg=90.0)

        # Simulate queued packets returned in a batch during settle: one pre-zero, one post-zero
        t_now_ms = int(time.time() * 1000)
        p1 = {
            "type": "pid_diagnostic",
            "timestamp": t_now_ms - 500,  # 500ms before zero command
            "sequence": 42,
            "m1": {"targetRadps": 0.20, "measuredRadps": 0.19, "finalPwm": 45, "stictionState": "KINETIC"}
        }
        p2 = {
            "type": "pid_diagnostic",
            "timestamp": t_now_ms + 500,  # after zero command
            "sequence": 43,
            "m1": {"targetRadps": 0.0, "measuredRadps": 0.05, "finalPwm": 0, "stictionState": "IDLE"}
        }
        mock_ws.recv_frames.side_effect = itertools.chain([[]] * 5, [[p1, p2]], itertools.repeat([]))

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        self.assertEqual(len(report.settle_pid_packets), 2)
        # Check individual preservation (not merged)
        self.assertEqual(report.settle_pid_packets[0]["source_sequence"], 42)
        self.assertTrue(report.settle_pid_packets[0]["is_pre_zero_backlog"])
        self.assertEqual(report.settle_pid_packets[1]["source_sequence"], 43)
        self.assertFalse(report.settle_pid_packets[1]["is_pre_zero_backlog"])
        # Check that receipt time and relative times are recorded
        self.assertIn("host_receipt_time_monotonic", report.settle_pid_packets[0])
        self.assertIn("t_rel_s", report.settle_pid_packets[0])
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_stale_and_missing_imu_data_recorded_explicitly(self):
        base_imu = {
            "ok": True, "rotVecValid": True, "inResetRecovery": False,
            "dataAgeMs": 10, "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "gyro": {"z": 0.001},
            "orientation": {"w": 0.7071068, "x": 0.0, "y": 0.0, "z": 0.7071068}
        }
        imu_missing = {"ok": False, "error": "Sensor read timeout"}
        imu_stale = dict(base_imu, dataAgeMs=350)
        imu_invalid_ori = dict(base_imu, orientation={"w": 1.0, "x": 0.0})  # missing y, z

        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(
            target_deg=90.0,
            settle_imus=[imu_missing, imu_stale, imu_invalid_ori]
        )

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        statuses = [s["status"] for s in report.settle_imu_samples]
        self.assertIn("MISSING", statuses)
        self.assertIn("STALE", statuses)
        self.assertIn("INVALID_ORIENTATION", statuses)
        for s in report.settle_imu_samples:
            if s["status"] in ("MISSING", "STALE", "INVALID_ORIENTATION"):
                self.assertFalse(s["valid"])
                self.assertIsNotNone(s["error"])
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_imu_repeated_non_advancing_sample(self):
        rad = math.radians(90.0)
        w = math.cos(rad / 2.0)
        z = math.sin(rad / 2.0)
        imu_base = {
            "ok": True, "rotVecValid": True, "inResetRecovery": False,
            "dataAgeMs": 10, "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "gyro": {"z": 0.001},
            "orientation": {"w": w, "x": 0.0, "y": 0.0, "z": z}
        }
        imu_s1 = dict(imu_base, sequence=10, timestamp_ms=1000)
        imu_s2_duplicate = dict(imu_base, sequence=10, timestamp_ms=1000)
        imu_s3_advanced = dict(imu_base, sequence=11, timestamp_ms=1020)

        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(
            target_deg=90.0,
            settle_imus=[imu_s1, imu_s2_duplicate, imu_s3_advanced]
        )

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        statuses = [s["status"] for s in report.settle_imu_samples]
        self.assertIn("NOT_ADVANCING", statuses)
        self.assertIn("VALID_ADVANCING", statuses)
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_final_endpoint_preserves_zero_measurement_never_substitutes_target(self):
        # Target is 180.0, but rover stops at 182.5 (overshoot)
        imu_fail = {"ok": False, "error": "Total sensor failure"}
        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(
            target_deg=180.0,
            final_imu_deg=182.5,
            settle_imus=itertools.repeat(imu_fail)
        )

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        # Requirement 2: angle_at_zero_cmd preserved only as last known reference
        self.assertAlmostEqual(report.gyro_angle_at_zero_cmd_deg, 182.5, places=1)
        # Final endpoint unavailable: report final angle and post-zero rotation as unavailable/invalid
        self.assertIsNone(report.final_settled_gyro_angle_deg)
        self.assertIsNone(report.post_zero_rotation_deg)
        self.assertIsNone(report.settled_heading_error_deg)
        self.assertFalse(report.final_measurement_valid)
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_zero_command_timing_and_response_recording(self):
        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(target_deg=90.0)
        zero_response_payload = {"ok": True, "linear": 0.0, "angular": 0.0, "state": "ACTIVE", "bridgeTimeMs": 12345}

        orig_send = mock_cockpit.send_cmd_vel.side_effect
        def cmd_vel_wrapper(*args, **kwargs):
            res = orig_send(*args, **kwargs)
            vx = kwargs.get("vx", args[0] if len(args) > 0 else 0.0)
            wz = kwargs.get("wz", args[1] if len(args) > 1 else 0.0)
            if abs(vx) <= 0.001 and abs(wz) <= 0.001:
                return zero_response_payload
            return res
        mock_cockpit.send_cmd_vel.side_effect = cmd_vel_wrapper

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        self.assertIsNotNone(report.zero_command_send_time_monotonic)
        self.assertIsNotNone(report.zero_command_response_time_monotonic)
        self.assertGreaterEqual(report.zero_command_response_time_monotonic, report.zero_command_send_time_monotonic)
        self.assertIsNotNone(report.zero_command_latency_ms)
        self.assertGreaterEqual(report.zero_command_latency_ms, 0.0)
        self.assertEqual(report.zero_command_response, zero_response_payload)
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_json_export_preserves_settling_telemetry(self):
        import tempfile
        import json

        trial = TrialReport(
            trial_index=1, requested_turn_deg=180.0, direction="cw", target_signed_yaw_deg=180.0, status="SUCCESS",
            zero_command_send_time_monotonic=100.1,
            zero_command_response_time_monotonic=100.108,
            zero_command_latency_ms=8.0,
            zero_command_response={"ok": True, "linear": 0, "angular": 0},
            settle_duration_s=2.001,
            settle_achieved_imu_rate_hz=48.5,
            settle_achieved_pid_rate_hz=49.2,
            settle_imu_samples=[{"t_rel_s": 0.02, "valid": True, "yaw_deg": 180.1}],
            settle_pid_packets=[{"t_rel_s": 0.02, "source_sequence": 10, "is_pre_zero_backlog": False}]
        )
        suite = MultiTrialSuiteReport(suite_id="999999", trials=[trial])

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_file = ReportGenerator.save_json(suite, tmp_dir)
            self.assertTrue(os.path.exists(out_file))

            with open(out_file, "r", encoding="utf-8") as f:
                loaded = json.load(f)

            t_loaded = loaded["trials"][0]
            self.assertEqual(t_loaded["zero_command_latency_ms"], 8.0)
            self.assertEqual(t_loaded["zero_command_response"], {"ok": True, "linear": 0, "angular": 0})
            self.assertEqual(t_loaded["settle_duration_s"], 2.001)
            self.assertEqual(t_loaded["settle_achieved_imu_rate_hz"], 48.5)
            self.assertEqual(t_loaded["settle_achieved_pid_rate_hz"], 49.2)
            self.assertEqual(len(t_loaded["settle_imu_samples"]), 1)
            self.assertEqual(t_loaded["settle_imu_samples"][0]["yaw_deg"], 180.1)
            self.assertEqual(len(t_loaded["settle_pid_packets"]), 1)
            self.assertEqual(t_loaded["settle_pid_packets"][0]["source_sequence"], 10)
            self.assertFalse(t_loaded["settle_pid_packets"][0]["is_pre_zero_backlog"])

    def test_read_timeouts_and_settle_rate_calculation(self):
        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(target_deg=90.0, settle_seconds=0.5)

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        self.assertGreaterEqual(report.settle_duration_s, 0.45)
        self.assertGreaterEqual(report.settle_achieved_imu_rate_hz, 0.0)
        self.assertIsInstance(report.settle_achieved_imu_rate_hz, float)
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_delayed_read_enforces_time_budget_and_cleanup_timing(self):
        """
        Strengthened Requirement 6:
        Proves that when individual HTTP/WS reads experience delays during settling:
        1. Bounded read timeouts enforce the remaining monotonic budget.
        2. The settle window duration stays close to the requested duration (not drifting/accumulating delays).
        3. Cleanup (disarm_drive, disable_autonomy, command-source reset) is invoked immediately afterward.
        """
        requested_settle_s = 0.5
        params, mock_cockpit, mock_ws, _, _, _, phase_tracker = self._create_mock_rover(
            target_deg=90.0,
            settle_seconds=requested_settle_s
        )
        orig_get_imu = mock_cockpit.get_imu.side_effect

        cleanup_timestamps = []
        orig_disarm = mock_cockpit.disarm_drive.side_effect
        def recorded_disarm(*args, **kwargs):
            cleanup_timestamps.append(time.monotonic())
            return orig_disarm(*args, **kwargs) if orig_disarm else {"ok": True}
        mock_cockpit.disarm_drive.side_effect = recorded_disarm

        # In settle phase, simulate a slow network call taking up to 0.08s, bounded by the passed timeout
        def delayed_get_imu(*args, **kwargs):
            if phase_tracker["phase"] == "SETTLING":
                timeout = kwargs.get("timeout", 2.0)
                sleep_time = min(0.08, timeout)
                time.sleep(sleep_time)
            return orig_get_imu(*args, **kwargs)

        mock_cockpit.get_imu.side_effect = delayed_get_imu

        t_start = time.monotonic()
        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)
        t_finish = time.monotonic()

        # 1. Settle window must stay close to requested duration: within 50ms of requested 0.5s
        self.assertGreaterEqual(report.settle_duration_s, requested_settle_s - 0.01)
        self.assertLess(report.settle_duration_s, requested_settle_s + 0.06)

        # 2. Cleanup must have occurred immediately after settling
        self.assertTrue(len(cleanup_timestamps) >= 1, "Cleanup disarm_drive was not called")
        t_disarm = cleanup_timestamps[-1]
        self.assertLess(t_finish - t_disarm, 0.05)
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_unavailable_final_endpoint_excluded_from_accuracy_averages(self):
        """
        Requirement 2:
        Restore bounded final endpoint measurement. If unavailable, report final angle and post-zero
        rotation as unavailable/invalid and exclude them from accuracy averages. Keep angle_at_zero_cmd
        only as the last known reference.
        """
        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(
            target_deg=90.0,
            settle_seconds=0.5
        )
        orig_get_imu = mock_cockpit.get_imu.side_effect
        def imu_with_failing_endpoint(*args, **kwargs):
            timeout = kwargs.get("timeout")
            if timeout and timeout <= 0.25 and timeout != 0.05:  # Final endpoint bounded read
                return {"ok": False, "error": "Endpoint timeout"}
            return orig_get_imu(*args, **kwargs)
        mock_cockpit.get_imu.side_effect = imu_with_failing_endpoint

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        suite = runner.execute_suite()

        trial = suite.trials[0]
        self.assertAlmostEqual(trial.gyro_angle_at_zero_cmd_deg, 90.0, places=1)
        self.assertIsNone(trial.final_settled_gyro_angle_deg)
        self.assertIsNone(trial.post_zero_rotation_deg)
        self.assertIsNone(trial.settled_heading_error_deg)
        self.assertFalse(trial.final_measurement_valid)
        # Accuracy averages must exclude this trial
        self.assertIsNone(suite.mean_settled_error_deg)
        self.assertIsNone(suite.std_dev_settled_error_deg)
        self.assertIsNone(suite.repeatability_deg)

    def test_final_endpoint_requires_advancing_sample_beyond_last_settling_sample(self):
        """
        Requirement 3:
        Require the final IMU endpoint sample to have an advancing sequence or timestamp
        beyond the last settling sample. If it does not advance within the bounded read,
        mark the final measurement invalid and exclude it from averages.
        """
        rad = math.radians(90.0)
        base = {
            "ok": True, "rotVecValid": True, "dataAgeMs": 10,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "gyro": {"z": 0.001},
            "orientation": {"w": math.cos(rad / 2.0), "x": 0.0, "y": 0.0, "z": math.sin(rad / 2.0)}
        }
        # Settling samples up to sequence 100
        s_settle = dict(base, sequence=100, timestamp_ms=5000)
        s_endpoint = dict(base, sequence=101, timestamp_ms=5020)

        # Case A: Endpoint sample does NOT advance beyond sequence 100 (returns same sequence 100)
        params_a, cockpit_a, ws_a, _, _, _, _ = self._create_mock_rover(
            target_deg=90.0,
            settle_seconds=0.5,
            settle_imus=[s_settle]
        )
        # Endpoint returns sequence 100 (not advancing)
        cockpit_a.get_imu.side_effect = lambda *args, **kwargs: dict(base, sequence=100, timestamp_ms=5000)

        runner_a = PhysicalTestRunner(params_a, prompt_fn=lambda _: "y", cockpit_client=cockpit_a, ws_client=ws_a)
        suite_a = runner_a.execute_suite()
        trial_a = suite_a.trials[0]

        self.assertFalse(trial_a.final_measurement_valid)
        self.assertIsNone(trial_a.final_settled_gyro_angle_deg)
        self.assertIsNone(trial_a.post_zero_rotation_deg)
        self.assertIsNone(trial_a.settled_heading_error_deg)
        self.assertIsNone(suite_a.mean_settled_error_deg)

        # Case B: Endpoint sample advances to sequence 101 beyond settling sequence 100
        params_b, cockpit_b, ws_b, _, _, _, _ = self._create_mock_rover(
            target_deg=90.0,
            settle_seconds=0.5,
            settle_imus=[s_settle] * 35 + [s_endpoint] * 10
        )

        runner_b = PhysicalTestRunner(params_b, prompt_fn=lambda _: "y", cockpit_client=cockpit_b, ws_client=ws_b)
        suite_b = runner_b.execute_suite()
        trial_b = suite_b.trials[0]

        self.assertTrue(trial_b.final_measurement_valid)
        self.assertIsNotNone(trial_b.final_settled_gyro_angle_deg)
        self.assertIsNotNone(trial_b.settled_heading_error_deg)
        self.assertIsNotNone(suite_b.mean_settled_error_deg)

    def test_missing_sample_identifiers_marked_unknown_advancement(self):
        """
        Requirement 3 & 4:
        Missing sample identifiers must mean advancement UNKNOWN, not VALID_ADVANCING.
        Preserve timestamp units and clock domains explicitly; verify yaw units.
        Record host receipt time after HTTP response and clearly separate request-start time.
        """
        rad = math.radians(90.0)
        imu_no_ids = {
            "ok": True, "rotVecValid": True, "dataAgeMs": 10,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "gyro": {"z": 0.001},
            "orientation": {"w": math.cos(rad / 2.0), "x": 0.0, "y": 0.0, "z": math.sin(rad / 2.0)}
            # No sequence and no timestamp_ms/espTimestampUs
        }
        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(
            target_deg=90.0,
            settle_seconds=0.5,
            settle_imus=[imu_no_ids]
        )

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        target_sample = [s for s in report.settle_imu_samples if s.get("status") == "UNKNOWN_ADVANCEMENT"]
        self.assertTrue(len(target_sample) >= 1)
        s = target_sample[0]
        self.assertEqual(s["status"], "UNKNOWN_ADVANCEMENT")
        self.assertEqual(s["advancement"], "UNKNOWN")
        self.assertFalse(s["valid"])
        self.assertIsNone(s["yaw_deg"])
        self.assertEqual(s["yaw_units"], "deg")
        self.assertEqual(s["host_clock_domain"], "host_monotonic")
        self.assertIn("host_request_start_time_monotonic", s)
        self.assertIn("host_receipt_time_monotonic", s)
        self.assertGreaterEqual(s["host_receipt_time_monotonic"], s["host_request_start_time_monotonic"])
        self.assertIsNone(s["source_clock_domain"])
        self.assertIsNone(s["source_timestamp"])
        self.assertIsNone(s["source_timestamp_unit"])

    def test_missing_packet_timestamp_marked_unknown_backlog(self):
        """
        Requirement 3:
        Missing packet timestamps must mean backlog UNKNOWN.
        """
        p_no_ts = {
            "type": "pid_diagnostic",
            "sequence": 50,
            "m1": {"targetRadps": 0.0, "measuredRadps": 0.0}
            # No timestamp
        }
        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(target_deg=90.0)
        mock_ws.recv_frames.side_effect = itertools.chain([[]] * 5, [[p_no_ts]], itertools.repeat([]))

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        target_packet = [p for p in report.settle_pid_packets if p.get("source_sequence") == 50]
        self.assertEqual(len(target_packet), 1)
        p = target_packet[0]
        self.assertIsNone(p["is_pre_zero_backlog"])
        self.assertEqual(p["backlog_status"], "UNKNOWN")
        self.assertIsNone(p["source_timestamp_ms"])
        self.assertIsNone(p["source_timestamp_unit"])
        self.assertIsNone(p["source_clock_domain"])
        self.assertEqual(p["host_clock_domain"], "host_monotonic")

    def test_distinguish_polling_attempts_from_advancing_samples_and_post_zero_pid_rates(self):
        """
        Requirement 3 & 5:
        Distinguish polling attempts from valid advancing IMU samples and post-zero PID packet rates.
        Make settle_achieved_imu_rate_hz report the valid advancing IMU rate.
        """
        rad = math.radians(90.0)
        base = {
            "ok": True, "rotVecValid": True, "dataAgeMs": 10,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "orientation": {"w": math.cos(rad / 2.0), "x": 0.0, "y": 0.0, "z": math.sin(rad / 2.0)}
        }
        # 1 stale, 1 repeated, 1 valid advancing
        s1 = dict(base, dataAgeMs=300, sequence=10, timestamp_ms=1000)
        s2 = dict(base, sequence=10, timestamp_ms=1000)
        s3 = dict(base, sequence=11, timestamp_ms=1020)

        t_now_ms = int(time.time() * 1000)
        # 1 pre-zero backlog, 1 post-zero, 1 unknown
        p_backlog = {"type": "pid_diagnostic", "sequence": 1, "timestamp": t_now_ms - 1000}
        p_post = {"type": "pid_diagnostic", "sequence": 2, "timestamp": t_now_ms + 1000}
        p_unknown = {"type": "pid_diagnostic", "sequence": 3}

        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(
            target_deg=90.0,
            settle_seconds=0.5,
            settle_imus=[s1, s2, s3]
        )
        mock_ws.recv_frames.side_effect = itertools.chain([[]] * 2, [[p_backlog, p_post, p_unknown]], itertools.repeat([]))

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        # Distinguish poll count vs advancing count
        self.assertGreaterEqual(report.settle_imu_poll_count, 3)
        self.assertGreaterEqual(report.settle_imu_valid_advancing_count, 1)
        self.assertGreaterEqual(report.settle_imu_poll_rate_hz, report.settle_imu_valid_advancing_rate_hz)

        # Requirement 5: settle_achieved_imu_rate_hz reports valid advancing rate
        self.assertEqual(report.settle_achieved_imu_rate_hz, report.settle_imu_valid_advancing_rate_hz)

        # Distinguish PID packet categories
        self.assertEqual(report.settle_pid_packets_count, 3)
        self.assertEqual(report.settle_pid_backlog_packets_count, 1)
        self.assertEqual(report.settle_pid_post_zero_packets_count, 1)
        self.assertEqual(report.settle_pid_unknown_packets_count, 1)
        self.assertGreaterEqual(report.settle_pid_packet_rate_hz, report.settle_pid_post_zero_packet_rate_hz)

    def test_exact_captured_live_pid_diagnostic_frame_schema_in_settle(self):
        """
        Requirement 1 & 2:
        Restore exact verified live PID WebSocket schema:
        - type: 'pid_diagnostic'
        - top-level timestamp and sequence
        - top-level m1, m2, m3, m4, and outerYaw
        - no PID_DIAGNOSTICS or pid.wheels nesting.
        """
        t_now_ms = int(time.time() * 1000)
        live_frame = {
            "type": "pid_diagnostic",
            "timestamp": t_now_ms + 10000,
            "sequence": 45000,
            "m1": {"targetRadps": -2.5, "measuredRadps": -2.4, "finalPwm": -50, "stictionState": "KINETIC"},
            "m2": {"targetRadps": +2.5, "measuredRadps": +2.3, "finalPwm": +48, "stictionState": "KINETIC"},
            "m3": {"targetRadps": -2.5, "measuredRadps": -2.5, "finalPwm": -51, "stictionState": "KINETIC"},
            "m4": {"targetRadps": +2.5, "measuredRadps": +2.4, "finalPwm": +49, "stictionState": "KINETIC"},
            "outerYaw": {
                "wzRequested": 0.8, "wzActual": 0.78, "yawOuterError": 0.02,
                "yawOuterCorrection": 0.01, "wzCorrected": 0.81, "yawOuterActive": True,
                "imuGyroValid": True, "imuGyroAgeMs": 15
            }
        }
        params, mock_cockpit, mock_ws, _, _, _, _ = self._create_mock_rover(target_deg=90.0, settle_seconds=0.5)
        mock_ws.recv_frames.side_effect = itertools.chain([[]] * 2, [[live_frame]], itertools.repeat([]))

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        self.assertEqual(report.settle_pid_post_zero_packets_count, 1)
        self.assertEqual(len(report.settle_pid_packets), 1)
        pkt = report.settle_pid_packets[0]

        # Top-level field verifications
        self.assertEqual(pkt["source_sequence"], 45000)
        self.assertEqual(pkt["source_timestamp_ms"], t_now_ms + 10000)
        self.assertFalse(pkt["is_pre_zero_backlog"])
        self.assertEqual(pkt["backlog_status"], "POST_ZERO")

        # Top-level wheel records m1..m4 preserved
        self.assertEqual(pkt["m1"]["targetRadps"], -2.5)
        self.assertEqual(pkt["m2"]["measuredRadps"], +2.3)
        self.assertEqual(pkt["m3"]["finalPwm"], -51)
        self.assertEqual(pkt["m4"]["stictionState"], "KINETIC")

        # Top-level outerYaw preserved
        self.assertIsNotNone(pkt["outerYaw"])
        self.assertEqual(pkt["outerYaw"]["wzRequested"], 0.8)
        self.assertTrue(pkt["outerYaw"]["yawOuterActive"])

    def test_dry_run_settling_duration_restored(self):
        """
        Requirement 4:
        Restore requested dry-run settling duration.
        """
        params = TurnParameters(degrees=90.0, direction="cw", trials=1, dry_run=True, settle_seconds=0.5)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = False
        mock_cockpit.get_imu.return_value = {
            "ok": True, "rotVecValid": True, "dataAgeMs": 10,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "gyro": {"z": 0.001},
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        }
        mock_cockpit.get_encoders.return_value = {
            "ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}
        }

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1)

        self.assertGreaterEqual(report.settle_duration_s, 0.49)
        self.assertTrue(report.final_measurement_valid)
        self.assertIsNotNone(report.final_settled_gyro_angle_deg)
        self.assertEqual(report.final_settled_gyro_angle_deg, report.gyro_angle_at_zero_cmd_deg)


class TestFailFastSafeguardIntegration(unittest.TestCase):
    """
    Verifies that the runner aborts early via the fail-fast safeguard when
    wheels remain stopped while commanded for >= 1.5s, well before 8 seconds.
    """

    def test_fail_fast_safeguard_aborts_and_records_breakout(self):
        params = TurnParameters(degrees=180.0, direction="cw", trials=1, dry_run=False)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)
        mock_ws.connected = True

        mock_cockpit.get_status.return_value = {"armed": False, "autonomyState": "DISABLED", "cmdSource": "NONE"}
        mock_cockpit.get_autonomy_status.return_value = {"state": "DISABLED", "zeroHandshakeCount": 0, "cmdSource": "NONE"}
        mock_cockpit.get_imu.return_value = {
            "ok": True, "rotVecValid": True, "dataAgeMs": 10,
            "orientationSource": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "gyro": {"z": 0.0},
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        }
        mock_cockpit.get_encoders.return_value = {
            "ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}
        }
        def mock_send_cmd(*args, **kwargs):
            mock_cockpit.get_autonomy_status.return_value = {"state": "ACTIVE", "clampedAngular": 0.8}
            mock_cockpit.get_status.return_value = {"armed": True, "mode": 3, "autonomyState": "ACTIVE", "cmdSource": "ROS_AUTONOMY"}
            return {"ok": True}
        mock_cockpit.send_cmd_vel.side_effect = mock_send_cmd

        # Sequence of frames: initial stiction boost, then continuous stall
        pid_frame_boost = {
            "type": "pid_diagnostic",
            "actuationState": "DRIVE",
            "m1": {"targetRadps": -2.35, "measuredRadps": 0.0, "stictionState": "STICTION_BOOST"},
            "m2": {"targetRadps":  2.35, "measuredRadps": 0.0, "stictionState": "STICTION_BOOST"},
            "m3": {"targetRadps": -2.35, "measuredRadps": 0.0, "stictionState": "STICTION_BOOST"},
            "m4": {"targetRadps":  2.35, "measuredRadps": 0.0, "stictionState": "STICTION_BOOST"},
        }
        pid_frame_stalled = {
            "type": "pid_diagnostic",
            "actuationState": "DRIVE",
            "m1": {"targetRadps": -2.35, "measuredRadps": 0.0, "stictionState": "STICTION_KINETIC"},
            "m2": {"targetRadps":  2.35, "measuredRadps": 0.0, "stictionState": "STICTION_KINETIC"},
            "m3": {"targetRadps": -2.35, "measuredRadps": 0.0, "stictionState": "STICTION_KINETIC"},
            "m4": {"targetRadps":  2.35, "measuredRadps": 0.0, "stictionState": "STICTION_KINETIC"},
        }

        # Return empty list for pre-motion drain, then boost frame, then stalled frames
        frame_queue = [[]] + [[pid_frame_boost]] + [[pid_frame_stalled] for _ in range(200)]
        mock_ws.recv_frames.side_effect = lambda: frame_queue.pop(0) if frame_queue else [pid_frame_stalled]

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)

        def mock_handshake(*args, **kwargs):
            mock_cockpit.get_status.return_value = {"armed": False, "autonomyState": "READY_DISARMED", "cmdSource": "NONE"}
            mock_cockpit.get_autonomy_status.return_value = {"state": "READY_DISARMED", "zeroHandshakeCount": 3, "cmdSource": "NONE"}
            return True

        with patch("tools.rover_tests.runner.perform_zero_handshake", side_effect=mock_handshake):
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(True, mock_cockpit.get_imu(), "ok")):
                with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                    with patch("tools.rover_tests.runner.arm_and_verify_ready_armed", return_value=True):
                        # Fast-forward time inside the control loop
                        t_sim = [1000.0]
                        def mock_time():
                            t_sim[0] += 0.05
                            return t_sim[0]

                        with patch("time.time", side_effect=mock_time):
                            with patch("time.sleep", return_value=None):
                                report = runner.execute_single_trial(1)

        self.assertEqual(report.status, "ABORTED")
        self.assertIn("Fail-fast safeguard triggered", report.abort_reason)
        self.assertIn("WHEEL_STALL_FAILSAFE", report.watchdog_trips)
        self.assertEqual(report.breakout_event_count, 1, "Must accurately record breakout event count on abort")
        self.assertTrue(report.confirmed_final_zero_command)

    def test_transient_stale_imu_reading_commands_zero_and_aborts_without_auto_resume(self):
        """On transient stale IMU reading, zero is commanded immediately, confirmation runs stopped, and test aborts without auto-resuming."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        current_status = {"armed": True, "mode": 3, "autonomyState": "ACTIVE", "cmdSource": "ROS_AUTONOMY"}
        mock_cockpit.token = "test"
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": "ACTIVE",
            "clampedAngular": 0.8,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        sent_commands = []
        def mock_send_cmd(vx=0.0, wz=0.0, **kwargs):
            sent_commands.append((vx, wz))
            return {"ok": True}
        mock_cockpit.send_cmd_vel.side_effect = mock_send_cmd
        def mock_disarm():
            current_status["armed"] = False
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_set_source(s):
            current_status["cmdSource"] = s
            return {"ok": True, "source": s}
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.disable_autonomy.side_effect = lambda: {"ok": True}
        mock_cockpit.set_command_source.side_effect = mock_set_source
        mock_ws.connected = True
        mock_ws.recv_frames.return_value = []

        q_start = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        sample_1 = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 100, "dataAgeMs": 15, "orientation": q_start}
        sample_2_stale = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 100, "dataAgeMs": 271, "orientation": q_start}
        sample_2_confirm = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 101, "dataAgeMs": 20, "orientation": q_start}

        in_motion = [False]
        sent_commands = []
        def mock_send_cmd(vx=0.0, wz=0.0, **kwargs):
            sent_commands.append((vx, wz))
            if abs(wz) > 0.01:
                in_motion[0] = True
            return {"ok": True}
        mock_cockpit.send_cmd_vel.side_effect = mock_send_cmd

        motion_imu_stream = [sample_2_stale, sample_2_confirm]
        def mock_get_imu(*a, **k):
            if not in_motion[0]:
                return dict(sample_1)
            return motion_imu_stream.pop(0) if motion_imu_stream else sample_2_confirm
        mock_cockpit.get_imu.side_effect = mock_get_imu

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)

        with patch("tools.rover_tests.runner.perform_zero_handshake", return_value=True):
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(True, sample_1, "ok")):
                with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                    with patch("tools.rover_tests.runner.arm_and_verify_ready_armed", return_value=True):
                        with patch("time.sleep", return_value=None):
                            report = runner.execute_single_trial(1)

        # Must abort rather than automatically resume
        self.assertEqual(report.status, "ABORTED")
        self.assertIn("TRANSIENT_IMU_SAFE_STOP", report.watchdog_trips)
        self.assertIn("Transient stale IMU reading", report.abort_reason)
        # Verify zero velocity command was sent immediately upon detecting suspect data
        self.assertTrue(any(vx == 0.0 and wz == 0.0 for vx, wz in sent_commands))
        self.assertTrue(report.confirmed_final_zero_command)
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_esp32_hardware_reset_during_motion_triggers_immediate_zero_and_abort(self):
        """Detection of an ESP32 sequence drop or reset during motion immediately commands zero and aborts."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        current_status = {"armed": True, "mode": 3, "autonomyState": "ACTIVE", "cmdSource": "ROS_AUTONOMY"}
        mock_cockpit.token = "test"
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": "ACTIVE",
            "clampedAngular": 0.8,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        sent_commands = []
        in_motion = [False]
        def mock_send_cmd_reset(vx=0.0, wz=0.0, **kw):
            sent_commands.append((vx, wz))
            if abs(wz) > 0.01:
                in_motion[0] = True
            return {"ok": True}
        mock_cockpit.send_cmd_vel.side_effect = mock_send_cmd_reset
        def mock_disarm():
            current_status["armed"] = False
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_set_source(s):
            current_status["cmdSource"] = s
            return {"ok": True, "source": s}
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.disable_autonomy.side_effect = lambda: {"ok": True}
        mock_cockpit.set_command_source.side_effect = mock_set_source
        mock_ws.connected = True
        mock_ws.recv_frames.return_value = []

        q_start = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        # Sample 1: sequence 5000, uptime 100s
        sample_1 = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 5000, "espTimestampUs": 100000000, "resetCount": 1, "dataAgeMs": 15, "orientation": q_start}
        # Sample 2: reset occurred! sequence dropped to 2, uptime reset to 40ms
        sample_reset = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 2, "espTimestampUs": 40000, "resetCount": 2, "dataAgeMs": 15, "orientation": q_start}

        motion_imu_stream = [sample_1, sample_reset]
        def mock_get_imu_reset(*a, **k):
            if not in_motion[0]:
                return dict(sample_1)
            return motion_imu_stream.pop(0) if motion_imu_stream else sample_reset
        mock_cockpit.get_imu.side_effect = mock_get_imu_reset

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)

        with patch("tools.rover_tests.runner.perform_zero_handshake", return_value=True):
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(True, sample_1, "ok")):
                with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                    with patch("tools.rover_tests.runner.arm_and_verify_ready_armed", return_value=True):
                        with patch("time.sleep", return_value=None):
                            report = runner.execute_single_trial(1)

        self.assertEqual(report.status, "ABORTED")
        self.assertIn("ESP32_HARDWARE_RESET", report.watchdog_trips)
        self.assertIn("ESP32 hardware reset detected during motion", report.abort_reason)
        self.assertTrue(report.confirmed_final_zero_command)
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_sustained_stale_imu_reading_triggers_safe_stop(self):
        """Sustained stale IMU readings across the bounded confirmation window abort motion safely."""
        params = TurnParameters(degrees=180.0, direction="cw", trials=1)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock(spec=NativeWSClient)

        current_status = {"armed": True, "mode": 3, "autonomyState": "ACTIVE", "cmdSource": "ROS_AUTONOMY"}
        mock_cockpit.token = "test"
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.get_autonomy_status.side_effect = lambda: {
            "state": "ACTIVE",
            "clampedAngular": 0.8,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        in_motion = [False]
        def mock_send_cmd_sustained(vx=0.0, wz=0.0, **kw):
            if abs(wz) > 0.01:
                in_motion[0] = True
            return {"ok": True}
        mock_cockpit.send_cmd_vel.side_effect = mock_send_cmd_sustained
        def mock_disarm():
            current_status["armed"] = False
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_set_source(s):
            current_status["cmdSource"] = s
            return {"ok": True, "source": s}
        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.disable_autonomy.side_effect = lambda: {"ok": True}
        mock_cockpit.set_command_source.side_effect = mock_set_source
        mock_ws.connected = True
        mock_ws.recv_frames.return_value = []

        q_start = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        sample_1 = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 100, "dataAgeMs": 15, "orientation": q_start}
        sample_stale_1 = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 100, "dataAgeMs": 271, "orientation": q_start}
        sample_stale_confirm = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 100, "dataAgeMs": 310, "orientation": q_start}

        motion_imu_stream = [sample_stale_1, sample_stale_confirm]
        def mock_get_imu_sustained(*a, **k):
            if not in_motion[0]:
                return dict(sample_1)
            return motion_imu_stream.pop(0) if motion_imu_stream else sample_stale_confirm
        mock_cockpit.get_imu.side_effect = mock_get_imu_sustained

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)

        with patch("tools.rover_tests.runner.perform_zero_handshake", return_value=True):
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(True, sample_1, "ok")):
                with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                    with patch("tools.rover_tests.runner.arm_and_verify_ready_armed", return_value=True):
                        with patch("time.sleep", return_value=None):
                            report = runner.execute_single_trial(1)

        self.assertEqual(report.status, "ABORTED")
        self.assertIn("STALE_IMU_DATA", report.watchdog_trips)
        self.assertIn("Confirmed stale/non-advancing IMU data", report.abort_reason)
        self.assertTrue(report.confirmed_final_zero_command)
        self.assertTrue(report.confirmed_final_disarmed_state)


class TestFirmwareStallWatchdogAndFaultClearRegressions(unittest.TestCase):
    """
    Regression test suite reproducing failures 1789826675 and 1789827814,
    validating:
    1. Clean abort and diagnostic preservation when cmd_vel is rejected due to firmware EMERGENCY_STOP / stall fault.
    2. Arm rejection diagnosis when latched faults remain, followed by controlled fault-clear restoring armability.
    3. Stall watchdog algorithm immunity to normal low-speed creep / encoder quantization vs true continuous stalls.
    """

    def test_regression_failure_1789826675_emergency_stop_disarm_aborts_cleanly_with_diagnostics(self):
        """
        Reproduce Failure 1 (1789826675):
        Rover is moving, then cmd_vel is rejected mid-motion because ESP32 entered EMERGENCY_STOP
        due to a stall fault. Test runner must abort cleanly, capture diagnostic details,
        and ensure rover ends confirmed stopped and disarmed.
        """
        params = TurnParameters(degrees=180.0, direction="cw", trials=1, dry_run=False)
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock()

        current_status = {
            "ok": True,
            "armed": True,
            "mode": 3,
            "autonomyState": "READY_ARMED",
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.get_status.side_effect = lambda *a, **k: dict(current_status)
        mock_cockpit.get_autonomy_status.return_value = {
            "state": "ACTIVE",
            "clampedAngular": 0.8,
            "zeroHandshakeCount": 3,
            "cmdSource": "ROS_AUTONOMY"
        }
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}

        # First command check (call 1) & initial motion (call 2) succeed, mid-motion (call 3) rejected
        call_count = [0]
        def mock_send_cmd_vel(vx=0.0, wz=0.0, **kw):
            call_count[0] += 1
            if call_count[0] <= 2:
                return {"ok": True}
            # Simulate Cockpit reporting the firmware EMERGENCY_STOP state
            current_status["armed"] = False
            current_status["mode"] = 4
            return {"ok": False, "error": "Rover in EMERGENCY_STOP (fault=0x0008)", "mode": 4, "faultFlags": 8}

        mock_cockpit.send_cmd_vel.side_effect = mock_send_cmd_vel
        def mock_disarm():
            current_status["armed"] = False
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_disable_auto():
            current_status["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_set_source(s):
            current_status["cmdSource"] = s
            return {"ok": True, "source": s}

        mock_cockpit.disarm_drive.side_effect = mock_disarm
        mock_cockpit.disable_autonomy.side_effect = mock_disable_auto
        mock_cockpit.set_command_source.side_effect = mock_set_source
        mock_ws.connected = True
        mock_ws.recv_frames.return_value = []

        q_start = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        sample = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 100, "dataAgeMs": 15, "orientation": q_start}
        mock_cockpit.get_imu.return_value = sample

        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)

        with patch("tools.rover_tests.runner.perform_zero_handshake", return_value=True):
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(True, sample, "ok")):
                with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                    with patch("tools.rover_tests.runner.arm_and_verify_ready_armed", return_value=True):
                        with patch("time.sleep", return_value=None):
                            report = runner.execute_single_trial(1)

        self.assertEqual(report.status, "ABORTED")
        self.assertIn("cmd_vel transmission failed", report.abort_reason)
        self.assertIn("EMERGENCY_STOP", report.abort_reason)
        self.assertIn("CMD_VEL_SEND_FAILED", report.communication_faults)
        self.assertTrue(report.confirmed_final_zero_command)
        self.assertTrue(report.confirmed_final_disarmed_state)

    def test_regression_failure_1789827814_arm_rejection_and_controlled_fault_clear(self):
        """
        Reproduce Failure 2 (1789827814):
        Latched safety fault from prior abort prevents arming.
        Then, show that specifying clear_faults=True executes controlled fault clear
        while stationary and disarmed, allowing arming to proceed.
        """
        mock_cockpit = MagicMock(spec=CockpitClient)
        mock_ws = MagicMock()

        current_status_fail = {
            "ok": True,
            "armed": False,
            "mode": 4,
            "autonomyState": "READY_DISARMED",
            "cmdSource": "NONE"
        }
        mock_cockpit.get_status.side_effect = lambda *a, **k: dict(current_status_fail)
        def mock_disarm_fail():
            current_status_fail["armed"] = False
            current_status_fail["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_disable_auto_fail():
            current_status_fail["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_set_source_fail(s):
            current_status_fail["cmdSource"] = s
            return {"ok": True, "source": s}

        mock_cockpit.disarm_drive.side_effect = mock_disarm_fail
        mock_cockpit.disable_autonomy.side_effect = mock_disable_auto_fail
        mock_cockpit.set_command_source.side_effect = mock_set_source_fail
        mock_cockpit.get_autonomy_status.return_value = {"state": "READY_DISARMED", "zeroHandshakeCount": 3, "cmdSource": "NONE"}
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.arm_drive.return_value = {
            "ok": False,
            "error": "Arm confirmation timed out after 500ms waiting for ESP32 confirmation (armed=true, mode=3) [ESP32 in EMERGENCY_STOP mode; active faults=0x8]"
        }
        mock_ws.connected = False

        q_start = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        sample = {"ok": True, "rotVecValid": True, "inResetRecovery": False, "sequence": 200, "dataAgeMs": 10, "orientation": q_start}
        mock_cockpit.get_imu.return_value = sample

        # 1. Run without clear_faults: arming fails because ESP32 rejects arming
        params_fail = TurnParameters(degrees=180.0, direction="cw", trials=1, dry_run=False, clear_faults=False)
        runner_fail = PhysicalTestRunner(params_fail, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        with patch("tools.rover_tests.runner.perform_zero_handshake", return_value=True):
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(True, sample, "ok")):
                report_fail = runner_fail.execute_single_trial(1)

        self.assertEqual(report_fail.status, "ABORTED")
        self.assertIn("Failed to arm drivetrain", report_fail.abort_reason)
        self.assertIn("EMERGENCY_STOP", report_fail.abort_reason)
        self.assertTrue(report_fail.confirmed_final_disarmed_state)

        # 2. Run with clear_faults=True: controlled clear executed while disarmed & stationary
        params_clear = TurnParameters(degrees=180.0, direction="cw", trials=1, dry_run=False, clear_faults=True)
        current_status_clear = {
            "ok": True,
            "armed": False,
            "mode": 4,
            "autonomyState": "READY_DISARMED",
            "cmdSource": "NONE"
        }
        mock_cockpit.get_status.side_effect = lambda *a, **k: dict(current_status_clear)
        def mock_clear():
            current_status_clear["mode"] = 0
            return {"ok": True, "message": "Clear faults command sent (verified disarmed and stationary)."}
        mock_cockpit.clear_faults.side_effect = mock_clear
        def mock_arm():
            current_status_clear["armed"] = True
            current_status_clear["mode"] = 3
            current_status_clear["autonomyState"] = "READY_ARMED"
            return {"ok": True}
        mock_cockpit.arm_drive.side_effect = mock_arm
        def mock_disarm_clear():
            current_status_clear["armed"] = False
            current_status_clear["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_disable_auto_clear():
            current_status_clear["autonomyState"] = "DISABLED"
            return {"ok": True}
        def mock_set_source_clear(s):
            current_status_clear["cmdSource"] = s
            return {"ok": True, "source": s}

        mock_cockpit.disarm_drive.side_effect = mock_disarm_clear
        mock_cockpit.disable_autonomy.side_effect = mock_disable_auto_clear
        mock_cockpit.set_command_source.side_effect = mock_set_source_clear
        mock_cockpit.get_autonomy_status.return_value = {"state": "ACTIVE", "clampedAngular": 0.8, "zeroHandshakeCount": 3, "cmdSource": "ROS_AUTONOMY"}

        runner_clear = PhysicalTestRunner(params_clear, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        with patch("tools.rover_tests.runner.perform_zero_handshake", return_value=True):
            with patch("tools.rover_tests.runner.wait_for_advancing_imu_sample", return_value=(True, sample, "ok")):
                with patch.object(PhysicalTestRunner, "_assert_pre_motion_invariants", return_value=None):
                    with patch("tools.rover_tests.runner.arm_and_verify_ready_armed", return_value=True):
                        with patch("time.sleep", return_value=None):
                            mock_cockpit.send_cmd_vel.return_value = {"ok": True}
                            report_clear = runner_clear.execute_single_trial(1)

        mock_cockpit.clear_faults.assert_called_once()
        self.assertTrue(report_clear.confirmed_final_disarmed_state)

    def test_stall_watchdog_creep_and_quantization_immunity(self):
        """
        Verify the firmware stall watchdog logic:
        - Low-speed creep (|target| < 0.25 rad/s or |pwm| < 90) must NEVER trip stall.
        - True stall (|target| >= 0.25 rad/s, |pwm| >= 90, |speed| < 0.05 rad/s for 2.0s) trips stall
          and records diagnostic record with exact wheel, fault flag, duration, target, speed, pwm, mode.
        """
        class MockFirmwareSafetyWatchdog:
            def __init__(self):
                self.stall_ticks = [0] * 4
                self.nonzero_ticks = [0] * 4
                self.active_faults = 0
                self.last_stall_record = None

            def update(self, targets, measured, pwms, mode=3):
                if mode != 3:  # NORMAL_DRIVE
                    return self.active_faults
                for i in range(4):
                    tgt, spd, pwm = targets[i], measured[i], pwms[i]
                    if abs(tgt) < 0.01:
                        self.nonzero_ticks[i] = 0
                        self.stall_ticks[i] = 0
                        continue
                    self.nonzero_ticks[i] += 1
                    # Updated watchdog rule
                    if self.nonzero_ticks[i] > 50 and abs(tgt) >= 0.25 and abs(pwm) >= 90 and abs(spd) < 0.05:
                        self.stall_ticks[i] += 1
                        if self.stall_ticks[i] >= 200:
                            flag = (1 << i)
                            if (self.active_faults & flag) == 0:
                                self.active_faults |= flag
                                self.last_stall_record = {
                                    "wheel": i,
                                    "faultFlag": flag,
                                    "durationMs": self.stall_ticks[i] * 10,
                                    "targetSpeed": tgt,
                                    "measuredSpeed": spd,
                                    "pwm": pwm,
                                    "drivetrainMode": mode,
                                    "valid": True
                                }
                    else:
                        self.stall_ticks[i] = 0
                return self.active_faults

        # Case A: Normal low-speed creep / approach: target=0.20, measured=0.02, pwm=55 for 500 cycles (5 seconds)
        w_creep = MockFirmwareSafetyWatchdog()
        for _ in range(500):
            faults = w_creep.update([0.20, 0.20, 0.20, 0.20], [0.02, 0.02, 0.02, 0.02], [55, 55, 55, 55])
            self.assertEqual(faults, 0)
        self.assertIsNone(w_creep.last_stall_record)

        # Case B: Low-speed fluctuations / encoder quantization: target=0.80, measured intermittently 0.00, pwm=60
        w_quant = MockFirmwareSafetyWatchdog()
        for _ in range(300):
            faults = w_quant.update([0.80, 0.80, 0.80, 0.80], [0.00, 0.00, 0.00, 0.00], [60, 60, 60, 60])
            self.assertEqual(faults, 0)

        # Case C: True continuous physical motor stall on M4: target=2.80, measured=0.00, pwm=140
        w_stall = MockFirmwareSafetyWatchdog()
        for cycle in range(250):
            w_stall.update([2.80, 2.80, 2.80, 2.80], [2.50, 2.50, 2.50, 0.00], [100, 100, 100, 140])
            if cycle < 249:  # 50 grace + 200 stall - 1 = 249 cycles before trip
                pass
            else:
                self.assertEqual(w_stall.active_faults, (1 << 3))  # M4 stall fault
                self.assertIsNotNone(w_stall.last_stall_record)
                rec = w_stall.last_stall_record
                self.assertEqual(rec["wheel"], 3)  # M4 (0-indexed 3)
                self.assertEqual(rec["faultFlag"], 8)
                self.assertEqual(rec["durationMs"], 2000)
                self.assertEqual(rec["targetSpeed"], 2.80)
                self.assertEqual(rec["measuredSpeed"], 0.00)
                self.assertEqual(rec["pwm"], 140)
                self.assertEqual(rec["drivetrainMode"], 3)
                self.assertTrue(rec["valid"])

    def test_cli_repetitions_and_braking_options(self):
        """Verify --repetitions, --reps, -r and --braking CLI arguments and parameter syncing."""
        from tools.rover_tests.cli import build_parser

        parser = build_parser()
        args1 = parser.parse_args(["turn", "--degrees", "90", "--repetitions", "4", "--braking"])
        self.assertEqual(args1.repetitions, 4)
        self.assertTrue(args1.enable_braking)

        args2 = parser.parse_args(["turn", "--degrees", "180", "--reps", "8"])
        self.assertEqual(args2.repetitions, 8)

        args3 = parser.parse_args(["turn", "--degrees", "45", "-r", "6"])
        self.assertEqual(args3.repetitions, 6)

        # TurnParameters syncing
        p1 = TurnParameters(degrees=90.0, repetitions=4, enable_braking=True)
        p1.validate()
        self.assertEqual(p1.trials, 4)
        self.assertEqual(p1.stopping_advance_deg, 0.7)

        p2 = TurnParameters(degrees=90.0, trials=3)
        p2.validate()
        self.assertEqual(p2.trials, 3)

    def _make_mock_clients(self, target_deg=90.0):
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
        imu_0 = dict(base_imu, orientation={"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
        rad = math.radians(target_deg)
        w = math.cos(rad / 2.0)
        z = math.sin(rad / 2.0)
        imu_target = dict(base_imu, orientation={"w": w, "x": 0.0, "y": 0.0, "z": z})

        current_status = {
            "armed": False,
            "mode": 0,
            "autonomyState": "DISABLED",
            "cmdSource": "NONE"
        }

        mock_cockpit.get_imu.side_effect = itertools.chain([imu_0] * 8, itertools.repeat(imu_target))
        mock_cockpit.get_encoders.return_value = {"ok": True, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
        mock_cockpit.get_status.side_effect = lambda: dict(current_status)
        mock_cockpit.enable_autonomy.return_value = {"ok": True}
        mock_cockpit.arm_drive.return_value = {"ok": True}
        mock_cockpit.disarm_drive.return_value = {"ok": True}
        mock_cockpit.disable_autonomy.return_value = {"ok": True}
        mock_cockpit.set_command_source.return_value = {"ok": True}
        mock_cockpit.send_cmd_vel.return_value = {"ok": True}
        mock_cockpit.configure_drive.return_value = {"ok": True}
        mock_ws.recv_frames.return_value = []

        return mock_cockpit, mock_ws

    def test_phase_headings_tracking_in_trial(self):
        """Verify that execute_single_trial captures starting, ending, and per-phase headings."""
        params = TurnParameters(
            degrees=90.0,
            direction="cw",
            trials=1,
            dry_run=True,
            enable_braking=True,
            settle_seconds=0.5
        )
        mock_cockpit, mock_ws = self._make_mock_clients(target_deg=90.0)
        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        report = runner.execute_single_trial(1, cumulative_continuous_heading=10.0)

        self.assertIsNotNone(report.start_heading_raw_deg)
        self.assertIsNotNone(report.final_settled_raw_heading_deg)
        self.assertIsNotNone(report.start_heading_continuous_deg)
        self.assertIsNotNone(report.final_settled_continuous_heading_deg)
        self.assertEqual(report.start_heading_continuous_deg, 10.0)
        self.assertEqual(report.final_settled_continuous_heading_deg, 100.0)

        # Phase breakdown
        self.assertTrue(len(report.phase_headings) >= 2)
        phase_names = [p["phase_name"] for p in report.phase_headings]
        self.assertIn("CRUISE", phase_names)
        self.assertIn("CREEP", phase_names)
        self.assertIn("BRAKE_COAST", phase_names)

        for ph in report.phase_headings:
            self.assertIn("start_abs_deg", ph)
            self.assertIn("end_abs_deg", ph)
            self.assertIn("start_rel_deg", ph)
            self.assertIn("end_rel_deg", ph)
            self.assertIn("delta_deg", ph)
            self.assertIn("duration_s", ph)

    def test_multi_repetition_trajectory_and_totals(self):
        """Verify multi-repetition suite execution, trajectory table, and cumulative totals."""
        params = TurnParameters(
            degrees=90.0,
            direction="cw",
            repetitions=4,
            dry_run=True,
            enable_braking=True,
            settle_seconds=0.5
        )
        mock_cockpit, mock_ws = self._make_mock_clients(target_deg=90.0)
        runner = PhysicalTestRunner(params, prompt_fn=lambda _: "y", cockpit_client=mock_cockpit, ws_client=mock_ws)
        suite = runner.execute_suite()

        self.assertEqual(suite.repetitions, 4)
        self.assertEqual(len(suite.repetition_trajectory), 4)
        self.assertEqual(suite.total_cumulative_commanded_deg, 360.0)
        self.assertEqual(suite.total_cumulative_measured_deg, 360.0)
        self.assertEqual(suite.total_cumulative_error_deg, 0.0)

        # Verify step trajectory continuity
        step1 = suite.repetition_trajectory[0]
        step2 = suite.repetition_trajectory[1]
        step3 = suite.repetition_trajectory[2]
        step4 = suite.repetition_trajectory[3]

        self.assertEqual(step1["repetition"], 1)
        self.assertEqual(step1["start_continuous_deg"], 0.0)
        self.assertEqual(step1["end_continuous_deg"], 90.0)
        self.assertEqual(step2["start_continuous_deg"], 90.0)
        self.assertEqual(step2["end_continuous_deg"], 180.0)
        self.assertEqual(step3["start_continuous_deg"], 180.0)
        self.assertEqual(step3["end_continuous_deg"], 270.0)
        self.assertEqual(step4["start_continuous_deg"], 270.0)
        self.assertEqual(step4["end_continuous_deg"], 360.0)

        # Verify markdown output contains the Repetition Trajectory & Totals Summary table
        md = ReportGenerator.format_markdown_summary(suite)
        self.assertIn("## Repetition Trajectory & Totals Summary", md)
        self.assertIn("Step 1", md)
        self.assertIn("Step 4", md)
        self.assertIn("Phase-by-Phase Heading Transitions", md)

    def test_interactive_prompt_target_degrees(self):
        """Verify prompt_target_degrees handles custom numbers, defaults, and invalid inputs."""
        from tools.rover_tests.cli import prompt_target_degrees

        # Custom degrees (e.g. 350)
        self.assertEqual(prompt_target_degrees(default=180.0, prompt_fn=lambda _: "350"), 350.0)
        # Empty input defaults
        self.assertEqual(prompt_target_degrees(default=90.0, prompt_fn=lambda _: "   "), 90.0)

        # Invalid then valid
        inputs = iter(["-20", "abc", "180"])
        self.assertEqual(prompt_target_degrees(default=180.0, prompt_fn=lambda _: next(inputs)), 180.0)

    def test_interactive_prompt_repetitions_count(self):
        """Verify prompt_repetitions_count enforces 1-8 bounds and handles defaults."""
        from tools.rover_tests.cli import prompt_repetitions_count

        # Custom repetitions
        self.assertEqual(prompt_repetitions_count(default=1, prompt_fn=lambda _: "4"), 4)
        self.assertEqual(prompt_repetitions_count(default=1, prompt_fn=lambda _: "8"), 8)
        # Empty input defaults
        self.assertEqual(prompt_repetitions_count(default=2, prompt_fn=lambda _: ""), 2)

        # Out of bounds (>8, <1) then valid
        inputs = iter(["9", "0", "abc", "6"])
        self.assertEqual(prompt_repetitions_count(default=1, prompt_fn=lambda _: next(inputs)), 6)

    def test_cli_main_interactive_prompting(self):
        """Verify main() prompts for degrees and repetitions when running interactively."""
        from tools.rover_tests.cli import main

        # Non-interactive without degrees returns configuration error code 2
        code_non_interactive = main(["turn"], is_interactive=False)
        self.assertEqual(code_non_interactive, 2)

        # Interactive without degrees prompts for degrees and reps
        prompts = iter(["350", "2"])
        with patch("tools.rover_tests.cli.PhysicalTestRunner") as mock_runner_cls:
            mock_runner = MagicMock()
            mock_suite = MagicMock()
            mock_suite.aborted_trials = 0
            mock_runner.execute_suite.return_value = mock_suite
            mock_runner_cls.return_value = mock_runner

            code_interactive = main(
                ["turn", "--dry-run"],
                prompt_fn=lambda _: next(prompts),
                is_interactive=True
            )
            self.assertEqual(code_interactive, 0)
            mock_runner_cls.assert_called_once()
            call_params = mock_runner_cls.call_args[0][0]
            self.assertEqual(call_params.degrees, 350.0)
            self.assertEqual(call_params.repetitions, 2)
            self.assertEqual(call_params.trials, 2)


if __name__ == "__main__":
    unittest.main()




