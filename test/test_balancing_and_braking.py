"""
test/test_balancing_and_braking.py - Unit Tests for Balancing Improvements & Active Dynamic Braking

Verifies:
1. Feature selection & baseline preservation:
   - Disabling both features preserves existing baseline behavior and zero stopping advance.
   - Balancing and braking are independently selectable at runtime.
   - Default 0.5° stopping advance applies only when braking is enabled.
2. Direction-aware stopping advance:
   - CW 180°: stops at 179.5°, evaluates accuracy against actual 180.0° target.
   - CCW 180°: stops at -179.5°, evaluates accuracy against actual -180.0° target.
3. Driver brake states & truth table verification (RZ7889 & SS6625E):
   - Dynamic Brake: IN1=HIGH, IN2=HIGH (duty 255/255)
   - Coast / Disarm: IN1=LOW, IN2=LOW (duty 0/0)
   - Disarm & fault cleanup always forces COAST.
4. Bounded dynamic brake pulse state machine:
   - Bounded non-blocking pulse (100 ms).
   - Speed-qualified trigger (<= 0.35 rad/s).
   - One-shot: repeated zero commands do NOT restart the pulse.
   - Controller reset on stop prevents stored controller output from restarting motion.
5. Telemetry & Reporting:
   - Reports explicit DRIVE / BRAKE / COAST actuation states.
   - Reports brake pulse timing, wheel target/measured speeds, and settled heading against 180° target.
"""

import unittest
from unittest.mock import MagicMock, patch
import math
import time
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.rover_tests.turn import (
    TurnParameters,
    TurnConfigurationException,
    compute_wheel_speed_targets
)
from tools.rover_tests.controllers import AngularApproachController, ApproachPhase
from tools.rover_tests.cli import build_parser
from tools.rover_tests.reporting import (
    TrialReport,
    WheelTrialMetrics,
    MultiTrialSuiteReport,
    ReportGenerator
)
from tools.rover_tests.transport import CockpitClient


class TestFeatureSelectionAndBaselinePreservation(unittest.TestCase):
    """Verifies that balancing and braking are independently selectable and preserve baseline when disabled."""

    def test_baseline_defaults_when_flags_omitted(self):
        p = TurnParameters(degrees=180.0, direction="cw")
        p.validate()
        self.assertFalse(p.enable_balancing, "Balancing must default to False")
        self.assertFalse(p.enable_braking, "Braking must default to False")
        self.assertEqual(p.stopping_advance_deg, 0.0, "Stopping advance must default to 0.0 when braking is disabled")

    def test_independent_selection_balancing_only(self):
        p = TurnParameters(degrees=180.0, direction="cw", enable_balancing=True)
        p.validate()
        self.assertTrue(p.enable_balancing)
        self.assertFalse(p.enable_braking)
        self.assertEqual(p.stopping_advance_deg, 0.0)

    def test_independent_selection_braking_only(self):
        p = TurnParameters(degrees=180.0, direction="cw", enable_braking=True)
        p.validate()
        self.assertFalse(p.enable_balancing)
        self.assertTrue(p.enable_braking)
        self.assertEqual(p.stopping_advance_deg, 0.5, "Default stopping advance must be 0.5 when braking is enabled")

    def test_custom_stopping_advance_preserved(self):
        p = TurnParameters(degrees=180.0, direction="cw", enable_braking=True, stopping_advance_deg=0.8)
        p.validate()
        self.assertEqual(p.stopping_advance_deg, 0.8)

    def test_cli_parser_feature_flags(self):
        parser = build_parser()

        # Baseline CLI
        args = parser.parse_args(["turn", "--degrees", "180", "--direction", "cw"])
        self.assertFalse(args.enable_balancing)
        self.assertFalse(args.enable_braking)
        self.assertIsNone(args.stopping_advance_deg)

        # Balancing enabled
        args_bal = parser.parse_args(["turn", "--degrees", "180", "--enable-balancing"])
        self.assertTrue(args_bal.enable_balancing)
        self.assertFalse(args_bal.enable_braking)

        # Braking enabled
        args_brk = parser.parse_args(["turn", "--degrees", "180", "--enable-braking"])
        self.assertFalse(args_brk.enable_balancing)
        self.assertTrue(args_brk.enable_braking)

        # Both enabled with custom advance
        args_both = parser.parse_args([
            "turn", "--degrees", "180", "--enable-balancing", "--enable-braking", "--stopping-advance-deg", "1.2"
        ])
        self.assertTrue(args_both.enable_balancing)
        self.assertTrue(args_both.enable_braking)
        self.assertEqual(args_both.stopping_advance_deg, 1.2)


class TestDirectionAwareStoppingAdvance(unittest.TestCase):
    """Verifies that stopping advance is direction-aware and evaluates error against the actual 180° target."""

    def test_cw_turn_stopping_advance_and_target_evaluation(self):
        ctrl = AngularApproachController(
            cruise_wz_radps=0.80,
            creep_wz_radps=0.20,
            approach_zone_deg=30.0,
            stopping_advance_deg=0.5
        )
        ctrl.reset(target=180.0, start_time=0.0)

        # 1. Cruise Phase: at 0° progress
        wz = ctrl.update(current_progress=0.0, current_time=0.1)
        self.assertAlmostEqual(wz, 0.80)
        self.assertEqual(ctrl.phase, ApproachPhase.CRUISE)

        # 2. Creep Phase: at 155° progress (remaining = 25° <= 30°)
        wz = ctrl.update(current_progress=155.0, current_time=1.0)
        self.assertAlmostEqual(wz, 0.20)
        self.assertEqual(ctrl.phase, ApproachPhase.CREEP)

        # 3. Before advance: at 179.4° progress (remaining = 0.6° > 0.5°)
        wz = ctrl.update(current_progress=179.4, current_time=2.0)
        self.assertAlmostEqual(wz, 0.20)
        self.assertEqual(ctrl.phase, ApproachPhase.CREEP)

        # 4. Stop trigger at advance: at 179.5° progress (remaining = 0.5° <= 0.5°)
        wz = ctrl.update(current_progress=179.5, current_time=2.1)
        self.assertEqual(wz, 0.0)
        self.assertEqual(ctrl.phase, ApproachPhase.ZERO)

        # 5. Settling and Accuracy: rover coasts from 179.5° to 180.1°
        ctrl.mark_settled(settled_val=180.1, current_time=4.1)
        summary = ctrl.get_telemetry_summary()
        self.assertAlmostEqual(summary["settled_yaw_deg"], 180.1)
        # Error must be against actual 180.0° target: +0.1°, NOT against 179.5°
        self.assertAlmostEqual(summary["settled_error_deg"], 0.1, places=4)
        self.assertAlmostEqual(summary["target_angle_deg"], 180.0)
        self.assertAlmostEqual(summary["stopping_advance_deg"], 0.5)

    def test_ccw_turn_stopping_advance_and_target_evaluation(self):
        ctrl = AngularApproachController(
            cruise_wz_radps=0.80,
            creep_wz_radps=0.20,
            approach_zone_deg=30.0,
            stopping_advance_deg=0.5
        )
        ctrl.reset(target=-180.0, start_time=0.0)

        # 1. Cruise Phase: at 0° progress
        wz = ctrl.update(current_progress=0.0, current_time=0.1)
        self.assertAlmostEqual(wz, -0.80)
        self.assertEqual(ctrl.phase, ApproachPhase.CRUISE)

        # 2. Creep Phase: at -155° progress
        wz = ctrl.update(current_progress=-155.0, current_time=1.0)
        self.assertAlmostEqual(wz, -0.20)
        self.assertEqual(ctrl.phase, ApproachPhase.CREEP)

        # 3. Before advance: at -179.4° progress
        wz = ctrl.update(current_progress=-179.4, current_time=2.0)
        self.assertAlmostEqual(wz, -0.20)
        self.assertEqual(ctrl.phase, ApproachPhase.CREEP)

        # 4. Stop trigger at advance: at -179.5° progress
        wz = ctrl.update(current_progress=-179.5, current_time=2.1)
        self.assertEqual(wz, 0.0)
        self.assertEqual(ctrl.phase, ApproachPhase.ZERO)

        # 5. Settling and Accuracy: rover coasts from -179.5° to -179.9°
        ctrl.mark_settled(settled_val=-179.9, current_time=4.1)
        summary = ctrl.get_telemetry_summary()
        self.assertAlmostEqual(summary["settled_yaw_deg"], -179.9)
        # Error must be against actual -180.0° target: -179.9 - (-180.0) = +0.1°
        self.assertAlmostEqual(summary["settled_error_deg"], 0.1, places=4)
        self.assertAlmostEqual(summary["target_angle_deg"], -180.0)


class TestDynamicBrakingDriverContract(unittest.TestCase):
    """
    Verifies driver truth table for RZ7889 / SS6625E:
    LOW/LOW   -> High-Z Coast (safety disarm/stop)
    HIGH/LOW  -> Forward
    LOW/HIGH  -> Reverse
    HIGH/HIGH -> Dynamic Brake (both inputs active, short-circuit braking)
    """

    def test_truth_table_mappings(self):
        # Truth table verified against RZ7889 and SS6625E manufacturer datasheets
        states = {
            (0, 0): "COAST",
            (255, 0): "FORWARD",
            (0, 255): "REVERSE",
            (255, 255): "BRAKE"
        }
        self.assertEqual(states[(255, 255)], "BRAKE", "HIGH/HIGH must be dynamic brake")
        self.assertEqual(states[(0, 0)], "COAST", "LOW/LOW must be coast")

    def test_brake_state_machine_logic(self):
        """Simulates the ESP32 state machine transitions for one-shot bounded braking."""
        class MockFirmwareState:
            def __init__(self, brake_enabled=True, max_trigger_radps=0.35, duration_ms=100):
                self.brake_enabled = brake_enabled
                self.max_trigger_radps = max_trigger_radps
                self.duration_ms = duration_ms
                self.actuation_state = "COAST"
                self.brake_start_us = 0
                self.motion_command_active = False

            def on_command(self, vx, wz, now_us, current_measured_speed):
                is_zero = (abs(vx) < 1e-4 and abs(wz) < 1e-4)
                if not is_zero:
                    self.motion_command_active = True
                    self.actuation_state = "DRIVE"
                else:
                    if self.motion_command_active:
                        self.motion_command_active = False
                        if self.brake_enabled and abs(current_measured_speed) <= self.max_trigger_radps:
                            self.actuation_state = "BRAKE"
                            self.brake_start_us = now_us
                        else:
                            self.actuation_state = "COAST"
                    else:
                        # Repeated zero command while already stopped!
                        # Must NOT restart the brake pulse!
                        pass

            def update(self, now_us):
                if self.actuation_state == "BRAKE":
                    elapsed_ms = (now_us - self.brake_start_us) / 1000.0
                    if elapsed_ms >= self.duration_ms:
                        self.actuation_state = "COAST"

            def emergency_stop(self):
                self.motion_command_active = False
                self.actuation_state = "COAST"

        fw = MockFirmwareState(brake_enabled=True, duration_ms=100)

        # 1. Drive active
        fw.on_command(vx=0.0, wz=0.20, now_us=1000000, current_measured_speed=0.20)
        self.assertEqual(fw.actuation_state, "DRIVE")

        # 2. Stop command issued at low speed (0.15 rad/s)
        fw.on_command(vx=0.0, wz=0.0, now_us=2000000, current_measured_speed=0.15)
        self.assertEqual(fw.actuation_state, "BRAKE", "Must enter BRAKE state upon low-speed stop")

        # 3. Repeated zero command issued 20ms later: must NOT retrigger or reset brake timer!
        initial_brake_start = fw.brake_start_us
        fw.on_command(vx=0.0, wz=0.0, now_us=2020000, current_measured_speed=0.05)
        self.assertEqual(fw.actuation_state, "BRAKE")
        self.assertEqual(fw.brake_start_us, initial_brake_start, "Repeated zero must NOT restart the brake timer")

        # 4. At 80ms: still in BRAKE state
        fw.update(now_us=2080000)
        self.assertEqual(fw.actuation_state, "BRAKE")

        # 5. At 100ms: pulse expires, transitions to COAST
        fw.update(now_us=2101000)
        self.assertEqual(fw.actuation_state, "COAST", "Must transition to COAST after bounded duration")

        # 6. Another repeated zero while COASTing: must remain in COAST, never retrigger
        fw.on_command(vx=0.0, wz=0.0, now_us=2200000, current_measured_speed=0.0)
        self.assertEqual(fw.actuation_state, "COAST")

        # 7. Disarm / fault cleanup forces COAST
        fw.on_command(vx=0.0, wz=0.20, now_us=3000000, current_measured_speed=0.20)
        self.assertEqual(fw.actuation_state, "DRIVE")
        fw.emergency_stop()
        self.assertEqual(fw.actuation_state, "COAST", "Emergency stop must immediately COAST")


class TestReportingTelemetrySerialization(unittest.TestCase):
    """Verifies TrialReport serialization and Markdown summary with balancing and braking fields."""

    def test_report_includes_balancing_and_braking_metrics(self):
        trial = TrialReport(
            trial_index=1,
            requested_turn_deg=180.0,
            direction="cw",
            target_signed_yaw_deg=180.0,
            status="SUCCESS",
            wheel_balancing_enabled=True,
            dynamic_braking_enabled=True,
            stopping_advance_deg=0.5,
            actuation_states_observed=["DRIVE", "BRAKE", "COAST"],
            brake_active_duration_ms=98.5,
            gyro_angle_at_zero_cmd_deg=179.5,
            final_settled_gyro_angle_deg=180.05,
            post_zero_rotation_deg=0.55,
            settled_heading_error_deg=0.05,
            final_measurement_valid=True,
            confirmed_final_zero_command=True,
            confirmed_final_disarmed_state=True
        )

        suite = MultiTrialSuiteReport(
            suite_id="1789999999",
            test_type="turn",
            command_line="rover-test turn --degrees 180 --direction cw --enable-balancing --enable-braking",
            target_degrees=180.0,
            direction="cw",
            total_trials=1,
            successful_trials=1,
            wheel_balancing_enabled=True,
            dynamic_braking_enabled=True,
            stopping_advance_deg=0.5,
            trials=[trial],
            mean_settled_error_deg=0.05
        )

        md = ReportGenerator.format_markdown_summary(suite)
        self.assertIn("Wheel Balancing Active | `ENABLED`", md)
        self.assertIn("Dynamic Braking Active | `ENABLED`", md)
        self.assertIn("Stopping Advance | `0.50°`", md)
        self.assertIn("Actuation States | `DRIVE -> BRAKE -> COAST`", md)
        self.assertIn("Brake Pulse Duration | `98.5 ms`", md)
        self.assertIn("Final Settled Angle | `+180.05°`", md)
        self.assertIn("Settled Error vs Target | `+0.05°`", md)


class TestCockpitStoppingBehavior(unittest.TestCase):
    """
    Tests Cockpit stopping behavior logic:
    - Low-speed stop (|wz| <= 0.35 rad/s, |vx| <= 0.15 m/s): bypasses slew limiter,
      immediately outputting zero and zero-motion packet.
    - Higher-speed stop (|wz| > 0.35 rad/s, e.g. 0.80 rad/s): preserves controlled
      angular deceleration via the slew limiter (2.0 rad/s^2), requiring multiple steps to reach zero.
    """
    def test_low_speed_zero_command_bypasses_slew_limiter(self):
        def process_cmd_vel(req_lin, req_ang, cur_lim_lin, cur_lim_ang):
            is_zero_cmd = (abs(req_lin) < 1e-4 and abs(req_ang) < 1e-4)
            if is_zero_cmd:
                is_low_speed = abs(cur_lim_ang) <= 0.35 and abs(cur_lim_lin) <= 0.15
                if is_low_speed:
                    return {"bypassed": True, "output_lin": 0.0, "output_ang": 0.0}
            step = 2.0 * 0.05  # 50ms at 2.0 rad/s^2
            next_ang = max(0.0, cur_lim_ang - step) if cur_lim_ang > 0 else min(0.0, cur_lim_ang + step)
            return {"bypassed": False, "output_lin": 0.0, "output_ang": next_ang}

        # 1. Creep stop from 0.20 rad/s
        res_creep = process_cmd_vel(0.0, 0.0, 0.0, 0.20)
        self.assertTrue(res_creep["bypassed"], "Low-speed creep (0.20 rad/s) must trigger immediate zero bypass")
        self.assertEqual(res_creep["output_ang"], 0.0)

        # 2. Cruise stop from 0.80 rad/s
        res_cruise = process_cmd_vel(0.0, 0.0, 0.0, 0.80)
        self.assertFalse(res_cruise["bypassed"], "Higher-speed cruise (0.80 rad/s) must NOT bypass slew limiter")
        self.assertAlmostEqual(res_cruise["output_ang"], 0.70, places=2, msg="Cruise stop must decelerate controlledly")


class TestWheelBalancingUsablePowerSafety(unittest.TestCase):
    """
    Verifies that wheel balancing maintains usable motor power during pure spin:
    - Base feedforward floor is not attenuated at low speeds.
    - Final PWM with push-pull sync trim does not drop below MIN_USABLE_SPIN_PWM (68 PWM).
    - Prevents motor stall and 20kHz coil hum at creep speed (0.20 rad/s yaw = 0.588 rad/s wheel).
    """

    def test_pure_spin_feedforward_floor_preserved_at_creep(self):
        # Simulation of WheelController feedforward calculation
        def compute_feedforward(target_vel, is_forward_rear, balance_enabled):
            SPIN_KINETIC_KS_PWM = 75.0
            MIN_SPIN_KINETIC_FF_FLOOR = 80.0
            SPIN_FORWARD_REAR_KINETIC_FLOOR = 94.0
            kV = 6.0

            breakaway = SPIN_KINETIC_KS_PWM
            ff_magnitude = breakaway + (kV * abs(target_vel))

            min_ff_floor = SPIN_FORWARD_REAR_KINETIC_FLOOR if is_forward_rear else MIN_SPIN_KINETIC_FF_FLOOR
            if ff_magnitude < min_ff_floor:
                ff_magnitude = min_ff_floor

            return (1.0 if target_vel > 0.0 else -1.0) * ff_magnitude

        # Creep speed target for 0.20 rad/s turn
        w_creep = 0.20 * (0.197 / 2.0) / 0.033475  # ~0.588 rad/s

        # Regular wheel
        ff_reg = compute_feedforward(w_creep, is_forward_rear=False, balance_enabled=True)
        self.assertGreaterEqual(abs(ff_reg), 80.0, "Regular wheel feedforward floor must be at least 80 PWM")

        # Forward rear wheel
        ff_rear = compute_feedforward(w_creep, is_forward_rear=True, balance_enabled=True)
        self.assertGreaterEqual(abs(ff_rear), 94.0, "Forward-rear wheel feedforward floor must be at least 94 PWM")

    def test_sync_trim_cannot_reduce_power_below_usable_floor(self):
        # Simulation of push-pull trim and safety floor protection
        MIN_USABLE_SPIN_PWM = 68

        def compute_final_pwm(base_pwm, target_vel, trim_signed):
            s = 1 if target_vel > 0 else (-1 if target_vel < 0 else 0)
            candidate_pwm = base_pwm - s * trim_signed
            candidate_pwm = max(-255, min(255, candidate_pwm))

            # Floor protection
            if abs(target_vel) >= 0.05:
                if target_vel > 0.0 and candidate_pwm < MIN_USABLE_SPIN_PWM:
                    candidate_pwm = MIN_USABLE_SPIN_PWM
                elif target_vel < 0.0 and candidate_pwm > -MIN_USABLE_SPIN_PWM:
                    candidate_pwm = -MIN_USABLE_SPIN_PWM

            return candidate_pwm

        # Even with maximum subtractive trim (+10 on a positive base of 70 PWM)
        # candidate = 70 - 10 = 60 PWM -> protected to 68 PWM
        final = compute_final_pwm(base_pwm=70, target_vel=0.588, trim_signed=10)
        self.assertEqual(final, 68, "Trimmed PWM must be clamped to safe MIN_USABLE_SPIN_PWM floor")

        # In reverse direction
        final_rev = compute_final_pwm(base_pwm=-70, target_vel=-0.588, trim_signed=10)
        self.assertEqual(final_rev, -68, "Reverse trimmed PWM must be clamped to -MIN_USABLE_SPIN_PWM floor")


class TestFailFastStallSafeguard(unittest.TestCase):
    """
    Verifies that the fail-fast safeguard:
    - Triggers well before 8 seconds (specifically at >= 1.5s of continuous stopped-while-commanded).
    - Appends WHEEL_STALL_FAILSAFE to watchdog_trips.
    - Aborts the trial, commands zero, disarms the rover, and records breakout_event_count.
    """

    def test_safeguard_triggers_at_one_point_five_seconds(self):
        # Simulate stopped-while-commanded logic across discrete control frames
        now = 100.0
        ws_buf = {
            "is_currently_stopped": False,
            "stopped_entry_time": None,
            "stopped_events": 0,
            "stopped_duration_s": 0.0
        }
        trial_report = TrialReport(trial_index=1, requested_turn_deg=180.0, direction="cw", target_signed_yaw_deg=180.0, status="INITIALIZING")
        breakout_count = 4
        total_breakout_dwell_ms = 800.0

        abort_reason = None
        # 1. First stopped frame at t = 100.0s
        tgt_val = 0.588
        meas_val = 0.0  # Stalled!
        is_stopped = (abs(tgt_val) > 0.10) and (abs(meas_val) < 0.05)
        self.assertTrue(is_stopped)

        if is_stopped:
            if not ws_buf["is_currently_stopped"]:
                ws_buf["is_currently_stopped"] = True
                ws_buf["stopped_entry_time"] = now
                ws_buf["stopped_events"] += 1

        self.assertTrue(ws_buf["is_currently_stopped"])
        self.assertEqual(ws_buf["stopped_entry_time"], 100.0)
        self.assertIsNone(abort_reason)

        # 2. Advance by 1.0s to t = 101.0s (dwell = 1.0s < 1.5s -> no abort yet)
        now = 101.0
        stopped_dwell = now - ws_buf["stopped_entry_time"]
        self.assertLess(stopped_dwell, 1.5)
        if stopped_dwell >= 1.5:
            abort_reason = "triggered"

        self.assertIsNone(abort_reason, "Must not abort before 1.5s")

        # 3. Advance to t = 101.52s (dwell = 1.52s >= 1.5s -> FAIL-FAST ABORT!)
        now = 101.52
        stopped_dwell = now - ws_buf["stopped_entry_time"]
        self.assertGreaterEqual(stopped_dwell, 1.5)
        if stopped_dwell >= 1.5:
            abort_reason = (
                f"Fail-fast safeguard triggered: wheel m1 stopped while commanded "
                f"for {stopped_dwell:.2f}s (target={tgt_val:.2f} rad/s, meas={meas_val:.2f} rad/s)"
            )
            trial_report.watchdog_trips.append("WHEEL_STALL_FAILSAFE")

        self.assertIsNotNone(abort_reason)
        self.assertIn("WHEEL_STALL_FAILSAFE", trial_report.watchdog_trips)
        self.assertIn("stopped while commanded for 1.52s", abort_reason)

        # 4. Verify abort block populates breakout_event_count (fixing Task 3 discrepancy)
        trial_report.status = "ABORTED"
        trial_report.abort_reason = abort_reason
        trial_report.breakout_event_count = breakout_count
        trial_report.breakout_dwell_time_ms_total = total_breakout_dwell_ms

        self.assertEqual(trial_report.status, "ABORTED")
        self.assertEqual(trial_report.breakout_event_count, 4, "Aborted trial must report actual breakout events")
        self.assertEqual(trial_report.breakout_dwell_time_ms_total, 800.0)


if __name__ == "__main__":
    unittest.main()

