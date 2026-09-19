"""
test/test_rover_test_linear.py - Unit Test Suite for Rover One Linear Test Framework

Uses mocks and fakes only. Never connects to or moves real hardware.
Tests parameter validation, forward/reverse signs, 1-meter targets,
handshake enforcement, linear approach controller, symmetry verification,
front-to-rear wheel differences, and dry-run runner execution.
"""

import unittest
from unittest.mock import MagicMock, patch
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.rover_tests.linear import (
    LinearParameters,
    LinearConfigurationException,
    compute_linear_wheel_speed_targets,
    verify_linear_wheel_command_symmetry,
    ticks_to_meters,
    meters_to_ticks,
    compute_linear_distance_from_ticks,
    EFFECTIVE_WHEEL_DIAMETER_M,
    TICKS_PER_REVOLUTION
)
from tools.rover_tests.controllers import LinearApproachController, ApproachPhase
from tools.rover_tests.runner import PhysicalTestRunner
from tools.rover_tests.reporting import (
    WheelTrialMetrics,
    TrialReport,
    MultiTrialSuiteReport,
    ReportGenerator,
)
from tools.rover_tests.cli import build_parser, main as cli_main


class TestLinearParameterValidation(unittest.TestCase):
    """Verifies Linear CLI & model parameter validation rules."""

    def test_valid_forward_parameters(self):
        p = LinearParameters(distance_m=1.0, direction="forward", trials=3)
        p.validate()
        self.assertEqual(p.signed_target_distance_m, 1.0)
        self.assertEqual(p.direction, "forward")

    def test_valid_reverse_parameters(self):
        p = LinearParameters(distance_m=1.0, direction="reverse", trials=2)
        p.validate()
        self.assertEqual(p.signed_target_distance_m, -1.0)
        self.assertEqual(p.direction, "reverse")

    def test_negative_distance_rejected(self):
        p = LinearParameters(distance_m=-1.0, direction="forward")
        with self.assertRaises(LinearConfigurationException):
            p.validate()

    def test_zero_distance_rejected(self):
        p = LinearParameters(distance_m=0.0, direction="forward")
        with self.assertRaises(LinearConfigurationException):
            p.validate()

    def test_invalid_direction_rejected(self):
        p = LinearParameters(distance_m=1.0, direction="sideways")
        with self.assertRaises(LinearConfigurationException):
            p.validate()

    def test_zero_trials_rejected(self):
        p = LinearParameters(distance_m=1.0, direction="forward", trials=0)
        with self.assertRaises(LinearConfigurationException):
            p.validate()

    def test_zero_speed_rejected(self):
        p = LinearParameters(distance_m=1.0, max_linear_speed=0.0)
        with self.assertRaises(LinearConfigurationException):
            p.validate()

    def test_negative_speeds_rejected(self):
        p = LinearParameters(distance_m=1.0, max_linear_speed=-0.20)
        with self.assertRaises(LinearConfigurationException):
            p.validate()


class TestLinearKinematicsAndPolarity(unittest.TestCase):
    """Verifies Rover One kinematic conventions and polarity for straight-line travel."""

    def test_forward_wheel_polarity(self):
        # Forward (+vx): all 4 wheels positive (M1, M2, M3, M4)
        targets = compute_linear_wheel_speed_targets(vx_cmd=0.20)
        self.assertGreater(targets["m1"], 0.0, "LF (M1) must be positive forward")
        self.assertGreater(targets["m2"], 0.0, "RF (M2) must be positive forward")
        self.assertGreater(targets["m3"], 0.0, "LR (M3) must be positive forward")
        self.assertGreater(targets["m4"], 0.0, "RR (M4) must be positive forward")
        
        expected_radps = 0.20 / (EFFECTIVE_WHEEL_DIAMETER_M / 2.0)
        self.assertAlmostEqual(targets["m1"], expected_radps, places=3)
        self.assertAlmostEqual(targets["m2"], expected_radps, places=3)
        self.assertAlmostEqual(targets["m3"], expected_radps, places=3)
        self.assertAlmostEqual(targets["m4"], expected_radps, places=3)

    def test_reverse_wheel_polarity(self):
        # Reverse (-vx): all 4 wheels negative
        targets = compute_linear_wheel_speed_targets(vx_cmd=-0.20)
        self.assertLess(targets["m1"], 0.0, "LF (M1) must be negative reverse")
        self.assertLess(targets["m2"], 0.0, "RF (M2) must be negative reverse")
        self.assertLess(targets["m3"], 0.0, "LR (M3) must be negative reverse")
        self.assertLess(targets["m4"], 0.0, "RR (M4) must be negative reverse")

    def test_linear_symmetry_verification(self):
        targets = compute_linear_wheel_speed_targets(vx_cmd=0.20)
        is_sym, _ = verify_linear_wheel_command_symmetry(targets)
        self.assertTrue(is_sym)

        # Asymmetric target should fail verification
        bad_targets = dict(targets)
        bad_targets["m1"] = 1.0
        is_sym_bad, _ = verify_linear_wheel_command_symmetry(bad_targets)
        self.assertFalse(is_sym_bad)

    def test_ticks_to_meters_conversion(self):
        # 1 full revolution in ticks
        one_rev_m = math.pi * EFFECTIVE_WHEEL_DIAMETER_M
        computed_m = ticks_to_meters(TICKS_PER_REVOLUTION)
        self.assertAlmostEqual(computed_m, one_rev_m, places=4)

        # Invert
        ticks = meters_to_ticks(computed_m)
        self.assertAlmostEqual(ticks, TICKS_PER_REVOLUTION, places=2)

    def test_distance_from_all_wheel_ticks(self):
        start = {"m1": 0, "m2": 0, "m3": 0, "m4": 0}
        curr = {
            "m1": TICKS_PER_REVOLUTION,
            "m2": TICKS_PER_REVOLUTION,
            "m3": TICKS_PER_REVOLUTION,
            "m4": TICKS_PER_REVOLUTION
        }
        dist = compute_linear_distance_from_ticks(start, curr, "forward")
        expected_dist = math.pi * EFFECTIVE_WHEEL_DIAMETER_M
        self.assertAlmostEqual(dist, expected_dist, places=4)


class TestLinearApproachController(unittest.TestCase):
    """Verifies constant-velocity linear characterization controller (no creep, no advance)."""

    def test_forward_approach_profile(self):
        ctrl = LinearApproachController(
            cruise_speed_mps=0.20
        )
        ctrl.reset(target=1.0, start_time=0.0)

        # Initial state: cruise
        cmd = ctrl.update(current_progress=0.0, current_time=0.1)
        self.assertAlmostEqual(cmd, 0.20, places=3)
        self.assertEqual(ctrl.phase, ApproachPhase.CRUISE)

        # Midpoint: still constant cruise
        cmd = ctrl.update(current_progress=0.5, current_time=2.5)
        self.assertAlmostEqual(cmd, 0.20, places=3)
        self.assertEqual(ctrl.phase, ApproachPhase.CRUISE)

        # Near end (0.86m, where creep used to be): still constant cruise
        cmd = ctrl.update(current_progress=0.86, current_time=4.3)
        self.assertAlmostEqual(cmd, 0.20, places=3)
        self.assertEqual(ctrl.phase, ApproachPhase.CRUISE)

        # Just before target: still constant cruise
        cmd = ctrl.update(current_progress=0.99, current_time=4.9)
        self.assertAlmostEqual(cmd, 0.20, places=3)
        self.assertEqual(ctrl.phase, ApproachPhase.CRUISE)

        # At target: stop (0.0 m/s immediately)
        cmd = ctrl.update(current_progress=1.00, current_time=5.0)
        self.assertAlmostEqual(cmd, 0.0, places=3)
        self.assertEqual(ctrl.phase, ApproachPhase.ZERO)

    def test_reverse_approach_profile(self):
        ctrl = LinearApproachController(
            cruise_speed_mps=0.20
        )
        ctrl.reset(target=-1.0, start_time=0.0)

        # Cruise reverse: constant -0.20 m/s
        cmd = ctrl.update(current_progress=0.0, current_time=0.1)
        self.assertAlmostEqual(cmd, -0.20, places=3)
        self.assertEqual(ctrl.phase, ApproachPhase.CRUISE)

        # Mid-point reverse
        cmd = ctrl.update(current_progress=-0.90, current_time=4.5)
        self.assertAlmostEqual(cmd, -0.20, places=3)
        self.assertEqual(ctrl.phase, ApproachPhase.CRUISE)

        # Target reached reverse: 0.0 m/s immediately
        cmd = ctrl.update(current_progress=-1.01, current_time=5.0)
        self.assertAlmostEqual(cmd, 0.0, places=3)
        self.assertEqual(ctrl.phase, ApproachPhase.ZERO)


class TestLinearCliAndDryRun(unittest.TestCase):
    """Verifies CLI parsing, phase breakdown, and stationary dry-run execution."""

    def test_cli_parser_linear(self):
        parser = build_parser()
        args = parser.parse_args(["linear", "--distance", "1.0", "--direction", "forward", "--dry-run"])
        self.assertEqual(args.subcommand, "linear")
        self.assertEqual(args.distance, 1.0)
        self.assertEqual(args.direction, "forward")
        self.assertTrue(args.dry_run)

    def test_cli_parser_linear_reverse(self):
        parser = build_parser()
        args = parser.parse_args(["linear", "--distance", "1.5", "--direction", "reverse", "--dry-run"])
        self.assertEqual(args.distance, 1.5)
        self.assertEqual(args.direction, "reverse")

    @patch("tools.rover_tests.runner.time.sleep", return_value=None)
    def test_dry_run_forward_execution(self, mock_sleep):
        params = LinearParameters(
            distance_m=1.0,
            direction="forward",
            trials=1,
            dry_run=True,
            enable_balancing=True,
            enable_braking=True,
            report_directory="test_reports_tmp"
        )
        runner = PhysicalTestRunner(params)
        suite = runner.execute_suite()

        self.assertEqual(suite.successful_trials, 1)
        self.assertEqual(suite.aborted_trials, 0)
        self.assertTrue(suite.wheel_balancing_enabled)
        self.assertTrue(suite.dynamic_braking_enabled)

        report = suite.trials[0]
        self.assertEqual(report.requested_distance_m, 1.0)
        self.assertIsNotNone(report.measured_distance_m)
        self.assertIsNotNone(report.acceleration_phase)
        self.assertIsNotNone(report.steady_speed_phase)
        self.assertIsNotNone(report.stopping_phase)
        self.assertIsNotNone(report.steady_speed_wheel_metrics)
        self.assertIsNotNone(report.steady_speed_front_to_rear_left_radps)
        self.assertIsNotNone(report.steady_speed_front_to_rear_right_radps)

        # Clean up temporary test report dir
        if os.path.exists("test_reports_tmp"):
            import shutil
            shutil.rmtree("test_reports_tmp")

    @patch("tools.rover_tests.runner.time.sleep", return_value=None)
    def test_dry_run_reverse_execution(self, mock_sleep):
        params = LinearParameters(
            distance_m=1.0,
            direction="reverse",
            trials=1,
            dry_run=True,
            enable_balancing=False,
            enable_braking=False,
            report_directory="test_reports_tmp"
        )
        runner = PhysicalTestRunner(params)
        suite = runner.execute_suite()

        self.assertEqual(suite.successful_trials, 1)
        self.assertEqual(suite.aborted_trials, 0)
        self.assertFalse(suite.wheel_balancing_enabled)
        self.assertFalse(suite.dynamic_braking_enabled)

        report = suite.trials[0]
        self.assertEqual(report.requested_distance_m, 1.0)
        self.assertIsNotNone(report.acceleration_phase)
        self.assertIsNotNone(report.steady_speed_phase)
        self.assertIsNotNone(report.stopping_phase)

        # Clean up temporary test report dir
        if os.path.exists("test_reports_tmp"):
            import shutil
            shutil.rmtree("test_reports_tmp")

    @patch("tools.rover_tests.runner.time.sleep", return_value=None)
    def test_live_configuration_readback_and_confirmation(self, mock_sleep):
        """Verifies runner reads back and confirms live production settings before motion without assumed values."""
        params = LinearParameters(
            distance_m=1.0,
            direction="forward",
            trials=1,
            dry_run=False,
            inter_trial_approval=False,
            report_directory="test_reports_tmp"
        )
        runner = PhysicalTestRunner(params)
        mock_cockpit = MagicMock()
        mock_cockpit.get_drive_config.return_value = {
            "ok": True,
            "config": {
                "wheelBalancing": False,
                "dynamicBraking": True,
                "brakeDurationMs": 120,
                "maxTriggerSpeed": 0.25
            }
        }
        mock_cockpit.get_pid_telemetry.return_value = {
            "ok": True,
            "telemetry": {
                "m1": {"stictionState": "IDLE"}
            }
        }
        mock_cockpit.get_imu.return_value = {
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "sensor_type": "SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC",
            "calibration_status": 3
        }
        mock_cockpit.get_encoders.return_value = {
            "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}
        }
        mock_cockpit.arm.return_value = {"ok": True}
        mock_cockpit.set_command_source.return_value = {"ok": True}
        mock_cockpit.disarm.return_value = {"ok": True}
        mock_cockpit.get_status.return_value = {"armed": False, "autonomyState": "DISABLED", "cmdSource": "NONE"}

        runner.cockpit = mock_cockpit
        runner.prompt_fn = lambda _: "y"

        from tools.rover_tests.runner import HandshakeException
        with patch("tools.rover_tests.runner.complete_zero_handshake", side_effect=HandshakeException("Simulated exit after config readback")):
            trial = runner.execute_single_linear_trial(1)

        # Confirm readback occurred from live cockpit endpoints
        mock_cockpit.get_drive_config.assert_called()
        mock_cockpit.get_pid_telemetry.assert_called()
        # Confirm no mutation occurred since enable_balancing/braking were None
        mock_cockpit.configure_drive.assert_not_called()

        # Confirm verified live values populated trial report
        self.assertFalse(trial.wheel_balancing_enabled)
        self.assertTrue(trial.dynamic_braking_enabled)
        self.assertEqual(trial.dynamic_brake_duration_ms, 120)
        self.assertEqual(trial.dynamic_brake_max_speed, 0.25)
        self.assertTrue(trial.anti_stall_confirmed)

        if os.path.exists("test_reports_tmp"):
            import shutil
            shutil.rmtree("test_reports_tmp")


if __name__ == "__main__":
    unittest.main()
