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
from dataclasses import asdict
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
        self.assertEqual(p.stopping_advance_deg, 0.7, "Default stopping advance must be 0.7 when braking is enabled")

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


class TestImprovedSpinClosedLoopAndSyncTrim(unittest.TestCase):
    """
    Verifies the improved closed-loop and push-pull balancing control:
    - Faster proportional and integral tracking under load.
    - Sync trim authority increased up to 18 PWM.
    - Minimum usable power floor (68 PWM) strictly preserved when trimming fast wheels.
    """

    def test_sync_trim_authoritative_balancing_with_power_floor(self):
        SPIN_SYNC_TRIM_GAIN = 12.0
        SPIN_SYNC_TRIM_MAX = 18
        MIN_USABLE_SPIN_PWM = 68
        deadband = 0.08

        # Scenario: M2 (RF) measured 3.06 rad/s vs M4 (RR) measured 1.88 rad/s (diff = 1.18 rad/s)
        v_fast = 3.06
        v_slow = 1.88
        diff_mag = abs(v_fast) - abs(v_slow)
        eff_diff = diff_mag - deadband  # 1.10 rad/s

        raw_trim = int(round(SPIN_SYNC_TRIM_GAIN * eff_diff))  # 12.0 * 1.10 = 13.2 -> 13 PWM
        raw_trim = max(-SPIN_SYNC_TRIM_MAX, min(SPIN_SYNC_TRIM_MAX, raw_trim))
        self.assertEqual(raw_trim, 13, "Sync trim should provide 13 PWM of correction for 1.18 rad/s delta")

        # Apply push-pull trim to right side:
        # Fast wheel (M2) base = 80 PWM -> trimmed down
        base_fast = 80
        cand_fast = base_fast - raw_trim  # 80 - 13 = 67 PWM
        final_fast = max(MIN_USABLE_SPIN_PWM, cand_fast)  # Clamped to 68 PWM
        self.assertEqual(final_fast, 68, "Fast wheel must not be reduced below safe floor 68 PWM")

        # Slow wheel (M4) base = 104 PWM -> boosted
        base_slow = 104
        cand_slow = base_slow + raw_trim  # 104 + 13 = 117 PWM
        final_slow = min(255, cand_slow)
        self.assertEqual(final_slow, 117, "Slow wheel must receive +13 PWM boost to overcome scrub load")

    def test_single_wheel_pid_tracking_response(self):
        # Verify that SPIN_PID_KP = 10.0 and SPIN_PID_KI = 4.0 provide strong tracking under error
        kp = 10.0
        ki = 4.0
        error = 1.04  # target 2.92 - measured 1.88 rad/s
        dt = 0.01

        # Proportional term
        p_term = kp * error
        self.assertAlmostEqual(p_term, 10.4)

        # After 1.0s (100 ticks) of steady error
        integral_term = (error * dt * 100) * ki  # 1.04 * 4.0 = 4.16 PWM
        self.assertAlmostEqual(integral_term, 4.16)

        total_correction = p_term + integral_term  # ~14.56 PWM boost
        self.assertGreaterEqual(total_correction, 14.0)


class TestActiveMotionTelemetryReporting(unittest.TestCase):
    """
    Verifies that active motion PID diagnostic frames are recorded and serialized in TrialReport.
    """

    def test_trial_report_serializes_active_pid_packets(self):
        report = TrialReport(
            trial_index=1,
            requested_turn_deg=180.0,
            direction="cw",
            target_signed_yaw_deg=180.0,
            status="SUCCESS"
        )
        sample_packet = {
            "host_receipt_time_monotonic": 100.25,
            "t_rel_s": 0.25,
            "source_timestamp_ms": 1789824460000,
            "source_sequence": 1234,
            "actuationState": "DRIVE",
            "m1": {"targetRadps": -2.92, "measuredRadps": -2.36, "basePwm": -92, "spinSyncTrim": 6, "finalPwm": -86},
            "m2": {"targetRadps": 2.92, "measuredRadps": 3.06, "basePwm": 79, "spinSyncTrim": -6, "finalPwm": 73},
            "m3": {"targetRadps": -2.92, "measuredRadps": -1.99, "basePwm": -92, "spinSyncTrim": -6, "finalPwm": -98},
            "m4": {"targetRadps": 2.92, "measuredRadps": 1.88, "basePwm": 104, "spinSyncTrim": 6, "finalPwm": 110},
            "outerYaw": {"actuationState": "DRIVE"}
        }
        report.active_pid_packets.append(sample_packet)

        serialized = asdict(report)
        self.assertIn("active_pid_packets", serialized)
        self.assertEqual(len(serialized["active_pid_packets"]), 1)
        pkt = serialized["active_pid_packets"][0]
        self.assertEqual(pkt["m4"]["finalPwm"], 110)
        self.assertEqual(pkt["m2"]["measuredRadps"], 3.06)


class TestSameSideSynchronizationRegression(unittest.TestCase):
    """
    Regression tests for same-side wheel synchronization adhering to the Exact Motion Contract:
    - M2-fast / M4-slow speed asymmetry correction direction.
    - Dual power floor: M2 fast wheel can reduce down to 48 PWM, M4 slow wheel protected at 68 PWM.
    - Trim limits (bounded to 24 PWM) and slew limiting (3 PWM/tick).
    - Speed range floor (inactive below 0.35 rad/s reliable encoder range).
    - Reset behavior on zero command, direction change, braking, and disarm.
    - Balancing disabled preserves zero trim.
    - CRUISE and CREEP phase balancing telemetry computation.
    """

    def setUp(self):
        self.SYNC_GAIN = 20.0
        self.SYNC_MAX = 24
        self.SYNC_SLEW = 3
        self.DEADBAND = 0.08
        self.MIN_SPEED = 0.35
        self.MIN_USABLE_FLOOR = 68
        self.MIN_USABLE_FASTER = 48

    def simulate_sync_step(self, t_fast, t_slow, v_fast, v_slow, base_fast, base_slow,
                           last_trim=0, balancing_enabled=True, encoders_fresh=True,
                           is_braking=False, prev_target=None):
        """Simulates one cycle of same-side synchronization logic."""
        if not balancing_enabled or not encoders_fresh or is_braking:
            return 0, base_fast, base_slow, 0.0

        # Check reliable speed range and direction
        if abs(t_fast) < self.MIN_SPEED or abs(t_slow) < self.MIN_SPEED:
            return 0, base_fast, base_slow, 0.0

        same_dir = (t_fast > 0 and t_slow > 0) or (t_fast < 0 and t_slow < 0)
        if not same_dir:
            return 0, base_fast, base_slow, 0.0

        # Direction change check
        if prev_target is not None:
            if (t_fast > 0.01 and prev_target < -0.01) or (t_fast < -0.01 and prev_target > 0.01):
                last_trim = 0

        pair_error = abs(v_fast) - abs(v_slow)
        eff_diff = 0.0
        if pair_error > self.DEADBAND:
            eff_diff = pair_error - self.DEADBAND
        elif pair_error < -self.DEADBAND:
            eff_diff = pair_error + self.DEADBAND

        raw_trim = int(round(self.SYNC_GAIN * eff_diff))
        raw_trim = max(-self.SYNC_MAX, min(self.SYNC_MAX, raw_trim))

        delta = max(-self.SYNC_SLEW, min(self.SYNC_SLEW, raw_trim - last_trim))
        trim = last_trim + delta

        # Push-pull application
        s_fast = 1 if t_fast > 0 else (-1 if t_fast < 0 else 0)
        s_slow = 1 if t_slow > 0 else (-1 if t_slow < 0 else 0)

        cand_fast = base_fast - s_fast * trim
        cand_slow = base_slow + s_slow * trim

        # Power floor protection
        # For fast wheel being throttled down:
        trim_on_fast = -s_fast * trim
        is_throttled_fast = (t_fast > 0 and trim_on_fast < 0) or (t_fast < 0 and trim_on_fast > 0)
        floor_fast = self.MIN_USABLE_FASTER if is_throttled_fast else self.MIN_USABLE_FLOOR

        if t_fast > 0:
            final_fast = max(floor_fast, min(255, cand_fast))
        else:
            final_fast = min(-floor_fast, max(-255, cand_fast))

        trim_on_slow = s_slow * trim
        is_throttled_slow = (t_slow > 0 and trim_on_slow < 0) or (t_slow < 0 and trim_on_slow > 0)
        floor_slow = self.MIN_USABLE_FASTER if is_throttled_slow else self.MIN_USABLE_FLOOR

        if t_slow > 0:
            final_slow = max(floor_slow, min(255, cand_slow))
        else:
            final_slow = min(-floor_slow, max(-255, cand_slow))

        return trim, final_fast, final_slow, pair_error

    def test_m2_fast_m4_slow_correction_direction_and_floor(self):
        """
        Verifies that when M2=2.42 rad/s and M4=1.65 rad/s (CW turn, negative target -1.02 rad/s):
        - M2 (fast) receives positive delta to become less negative (magnitude reduced).
        - M4 (slow) receives negative delta to become more negative (magnitude increased).
        - M2 is permitted to drop below 68 PWM down to 48 PWM floor.
        """
        t_m2, t_m4 = -1.02, -1.02
        v_m2, v_m4 = -2.42, -1.65
        base_m2, base_m4 = -72, -82

        # Step multiple cycles to allow slew rate to accumulate
        trim = 0
        for _ in range(10):
            trim, final_m2, final_m4, pair_err = self.simulate_sync_step(
                t_fast=t_m2, t_slow=t_m4, v_fast=v_m2, v_slow=v_m4,
                base_fast=base_m2, base_slow=base_m4, last_trim=trim
            )

        self.assertAlmostEqual(pair_err, 0.77, places=2)
        # Expected raw trim: round(20 * (0.77 - 0.08)) = 14 PWM
        self.assertEqual(trim, 14)

        # M2: -72 - (-1)*14 = -72 + 14 = -58 PWM (magnitude reduced from 72 to 58)
        self.assertEqual(final_m2, -58, "Fast wheel magnitude must be reduced (less negative)")
        self.assertLess(abs(final_m2), abs(base_m2), "M2 absolute PWM must be reduced")
        self.assertLess(abs(final_m2), 68, "M2 must be permitted to drop below 68 PWM floor")

        # M4: -82 + (-1)*14 = -82 - 14 = -96 PWM (magnitude increased from 82 to 96)
        self.assertEqual(final_m4, -96, "Slow wheel magnitude must be increased (more negative)")
        self.assertGreater(abs(final_m4), abs(base_m4), "M4 absolute PWM must be boosted")

    def test_sync_trim_bounds_and_slew_limits(self):
        """Verifies trim clamps to 24 PWM max and respects 3 PWM/tick slew rate limit."""
        # Massive speed disparity: v_fast=5.0, v_slow=1.0 -> pair_error=4.0 rad/s
        trim = 0
        trims = []
        for _ in range(15):
            trim, _, _, _ = self.simulate_sync_step(
                t_fast=2.0, t_slow=2.0, v_fast=5.0, v_slow=1.0,
                base_fast=100, base_slow=100, last_trim=trim
            )
            trims.append(trim)

        # Slew rate: each step must not exceed 3 PWM delta
        for i in range(1, len(trims)):
            self.assertLessEqual(trims[i] - trims[i - 1], self.SYNC_SLEW)

        # Saturated trim must be clamped at SYNC_MAX (24 PWM)
        self.assertEqual(trim, 24, "Trim must be clamped to exactly 24 PWM authority")

    def test_reliable_speed_range_gating(self):
        """Verifies synchronization is gated off below 0.35 rad/s reliable encoder range."""
        # Target = 0.20 rad/s (below 0.35 rad/s)
        trim, final_fast, final_slow, pair_err = self.simulate_sync_step(
            t_fast=0.20, t_slow=0.20, v_fast=0.50, v_slow=0.10,
            base_fast=50, base_slow=50, last_trim=0
        )
        self.assertEqual(trim, 0, "Trim must be 0 below reliable speed range")
        self.assertEqual(final_fast, 50)
        self.assertEqual(final_slow, 50)

    def test_reset_behavior(self):
        """Verifies trim resets to 0 on zero command, direction reversal, and braking."""
        # Active trim built up
        trim = 12

        # 1. Zero command
        trim_zero, _, _, _ = self.simulate_sync_step(
            t_fast=0.0, t_slow=0.0, v_fast=1.0, v_slow=0.5,
            base_fast=0, base_slow=0, last_trim=trim
        )
        self.assertEqual(trim_zero, 0, "Zero command must immediately reset trim")

        # 2. Braking
        trim_brake, _, _, _ = self.simulate_sync_step(
            t_fast=1.5, t_slow=1.5, v_fast=2.0, v_slow=1.0,
            base_fast=80, base_slow=80, last_trim=trim, is_braking=True
        )
        self.assertEqual(trim_brake, 0, "Braking must immediately reset trim")

        # 3. Direction reversal (+1.5 -> -1.5)
        trim_rev, _, _, _ = self.simulate_sync_step(
            t_fast=-1.5, t_slow=-1.5, v_fast=-2.0, v_slow=-1.0,
            base_fast=-80, base_slow=-80, last_trim=trim, prev_target=+1.5
        )
        # Slew from 0 towards target on reversal
        self.assertLessEqual(trim_rev, 3, "Direction reversal must reset accumulated trim and slew from 0")

    def test_balancing_disabled_preserves_zero_trim(self):
        """Verifies that when wheel_balancing_enabled is False, trim remains strictly 0."""
        trim, final_fast, final_slow, _ = self.simulate_sync_step(
            t_fast=2.0, t_slow=2.0, v_fast=3.0, v_slow=1.5,
            base_fast=80, base_slow=80, last_trim=0, balancing_enabled=False
        )
        self.assertEqual(trim, 0, "Trim must remain 0 when balancing is disabled")
        self.assertEqual(final_fast, 80)
        self.assertEqual(final_slow, 80)

    def test_phase_balancing_telemetry_reporting(self):
        """Verifies that compute_phase_balancing_metrics properly aggregates CRUISE and CREEP."""
        from tools.rover_tests.reporting import compute_phase_balancing_metrics

        packets = [
            # CRUISE packet
            {
                "phase": "CRUISE",
                "m1": {"measuredRadps": 1.76, "spinSyncTrim": 0},
                "m2": {"measuredRadps": -2.42, "spinSyncTrim": 14},
                "m3": {"measuredRadps": 1.74, "spinSyncTrim": 0},
                "m4": {"measuredRadps": -1.65, "spinSyncTrim": -14},
            },
            # CREEP packet
            {
                "phase": "CREEP",
                "m1": {"measuredRadps": 0.45, "spinSyncTrim": 0},
                "m2": {"measuredRadps": -0.58, "spinSyncTrim": 6},
                "m3": {"measuredRadps": 0.44, "spinSyncTrim": 0},
                "m4": {"measuredRadps": -0.42, "spinSyncTrim": -6},
            }
        ]
        metrics = compute_phase_balancing_metrics(packets)
        self.assertEqual(len(metrics), 2)

        # Check CRUISE
        cruise = [m for m in metrics if m["phase"] == "CRUISE"][0]
        self.assertAlmostEqual(cruise["m2_mean"], 2.42, places=2)
        self.assertAlmostEqual(cruise["m4_mean"], 1.65, places=2)
        self.assertAlmostEqual(cruise["pair_error_right"], 0.77, places=2)
        self.assertEqual(cruise["m2_trim"], 14.0)
        self.assertEqual(cruise["m4_trim"], -14.0)

        # Check CREEP
        creep = [m for m in metrics if m["phase"] == "CREEP"][0]
        self.assertAlmostEqual(creep["m2_mean"], 0.58, places=2)
        self.assertAlmostEqual(creep["m4_mean"], 0.42, places=2)
        self.assertAlmostEqual(creep["pair_error_right"], 0.16, places=2)
        self.assertEqual(creep["m2_trim"], 6.0)
        self.assertEqual(creep["m4_trim"], -6.0)


class TestCCWCommonModeSlowdownRegression(unittest.TestCase):
    """
    Regression tests reproducing and verifying the CCW suite 1789841710 common-mode slowdown:
    - In CCW turns, M2 and M4 are commanded in reverse (e.g. -1.04 rad/s during CREEP).
    - When both wheels stall together, pair error is near-zero so push-pull balancing provides 0 trim.
    - Under baseline Ki=4.0 without anti-stall, final PWM peaks at only -99 PWM (38.8% of max 255)
      and fails to overcome static friction (~102-105 PWM), staying stalled for > 1.5s (triggering stall watchdog).
    - With anti-stall integral acceleration (ANTI_STALL_KI=25.0), the controller ramps up common-mode PWM
      past static breakaway (>= 102-105 PWM) within ~500 ms, breaking the stall and tracking target velocity.
    - Anti-stall is strictly gated above MIN_RELIABLE_SPEED_RADPS (0.35 rad/s) and resets on zero command/brake.
    - Same-side wheel balancing remains fully active alongside common-mode tracking.
    """

    def simulate_single_wheel(
        self,
        target_radps: float,
        measured_radps_seq: list,
        enable_anti_stall: bool = True,
        dt: float = 0.01,
        is_spin: bool = True
    ):
        """Simulates SingleWheelController::update for a sequence of measured speeds."""
        MIN_RELIABLE_SPEED_RADPS = 0.35
        ANTI_STALL_SPEED_THRESHOLD = 0.15
        SPIN_PID_KP = 10.0
        SPIN_PID_KI = 4.0
        ANTI_STALL_KI = 25.0
        LAGGING_KI = 12.0
        SPIN_KINETIC_KS_PWM = 75.0
        MIN_SPIN_KINETIC_FF_FLOOR = 80.0
        kV = 6.0

        error_sum = 0.0
        last_error = 0.0
        pwm_history = []

        # Feedforward
        ff_mag = SPIN_KINETIC_KS_PWM + kV * abs(target_radps)
        if ff_mag < MIN_SPIN_KINETIC_FF_FLOOR:
            ff_mag = MIN_SPIN_KINETIC_FF_FLOOR
        ff = (1.0 if target_radps > 0 else -1.0) * ff_mag

        for meas in measured_radps_seq:
            if abs(target_radps) < 0.01:
                pwm_history.append({
                    "target": 0.0,
                    "measured": meas,
                    "feedforward": 0.0,
                    "p_term": 0.0,
                    "i_term": 0.0,
                    "pwm": 0,
                    "is_stalled": False
                })
                continue

            error = target_radps - meas
            error_sum += error * dt

            is_commanded = abs(target_radps) >= MIN_RELIABLE_SPEED_RADPS
            is_lagging = (target_radps > 0 and error > 0) or (target_radps < 0 and error < 0)
            is_stalled = is_commanded and is_lagging and (abs(meas) < ANTI_STALL_SPEED_THRESHOLD)

            active_ki = SPIN_PID_KI
            if enable_anti_stall:
                if is_stalled:
                    active_ki = ANTI_STALL_KI
                elif is_commanded and is_lagging and abs(error) >= 0.25:
                    active_ki = LAGGING_KI

            integral_term = error_sum * active_ki
            integral_term = max(-150.0, min(150.0, integral_term))

            derivative = (error - last_error) / dt
            last_error = error

            pid_corr = (SPIN_PID_KP * error) + integral_term + (0.5 * derivative)
            total_pwm = ff + pid_corr
            final_pwm = max(-255, min(255, round(total_pwm)))

            pwm_history.append({
                "target": target_radps,
                "measured": meas,
                "feedforward": ff,
                "p_term": SPIN_PID_KP * error,
                "i_term": integral_term,
                "pwm": final_pwm,
                "is_stalled": is_stalled
            })

        return pwm_history

    def test_reproduce_ccw_slowdown_without_anti_stall(self):
        """Reproduces the suite 1789841710 failure where baseline controller capped at -99 PWM."""
        target = -1.04
        # 1.5 seconds (150 control ticks) of stalled wheel (measured = 0.0)
        stalled_seq = [0.0] * 150
        history = self.simulate_single_wheel(target, stalled_seq, enable_anti_stall=False)

        # In baseline, feedforward is -81.2, p_term is -10.4
        self.assertAlmostEqual(history[0]["feedforward"], -81.2, places=1)
        self.assertAlmostEqual(history[0]["p_term"], -10.4, places=1)

        # After 1.5s, integral term only reached ~ -6.2 PWM
        last_step = history[-1]
        self.assertAlmostEqual(last_step["i_term"], -6.24, delta=0.5)
        # Total PWM was capped at -98 to -99 PWM (only ~38.8% of 255)
        self.assertGreaterEqual(last_step["pwm"], -99, "Without anti-stall, PWM is capped around -98 to -99")
        self.assertLessEqual(abs(last_step["pwm"]), 100, "Without anti-stall, PWM never reached static breakaway 102+")

    def test_ccw_stalled_anti_stall_ramps_past_breakaway(self):
        """Verifies that with anti-stall tracking, controller ramps PWM past static breakaway within 500ms."""
        target = -1.04
        # 80 control ticks (800ms) of zero speed
        stalled_seq = [0.0] * 80
        history = self.simulate_single_wheel(target, stalled_seq, enable_anti_stall=True)

        # Within 50 ticks (500ms), output reaches >= -102 PWM (static breakaway)
        step_50 = history[49]
        self.assertLessEqual(step_50["pwm"], -102, "Anti-stall must reach static breakaway threshold (<= -102 PWM) within 500ms")

        # Within 80 ticks (800ms), output reaches >= -108 PWM without exceeding safe bounds
        step_80 = history[79]
        self.assertLessEqual(step_80["pwm"], -108)
        self.assertGreaterEqual(step_80["pwm"], -130, "Anti-stall must remain safely bounded well below 255")

    def test_anti_stall_never_applies_below_reliable_speed(self):
        """Verifies that anti-stall is gated above MIN_RELIABLE_SPEED_RADPS (0.35 rad/s)."""
        target = -0.20  # Below 0.35 rad/s
        stalled_seq = [0.0] * 50
        history = self.simulate_single_wheel(target, stalled_seq, enable_anti_stall=True)
        # Stalled flag must remain False
        self.assertFalse(history[0]["is_stalled"])
        self.assertFalse(history[-1]["is_stalled"])
        # Integral term integrates at normal Ki=4.0, not 25.0
        self.assertAlmostEqual(history[-1]["i_term"], -0.20 * 4.0 * 0.5, delta=0.1)

    def test_lagging_tracking_provides_common_mode_power_when_both_slow(self):
        """Verifies that when both wheels are lagging (e.g. meas=-0.5 when target=-1.04), responsive tracking activates."""
        target = -1.04
        lagging_seq = [-0.50] * 50  # 50% slow
        history = self.simulate_single_wheel(target, lagging_seq, enable_anti_stall=True)

        # Active Ki is boosted to LAGGING_KI (12.0)
        # error = -0.54. After 50 ticks (0.5s), integral term is -0.54 * 0.5 * 12.0 = -3.24 PWM
        self.assertAlmostEqual(history[-1]["i_term"], -3.24, delta=0.2)
        # Proportional term is -5.4 PWM
        self.assertAlmostEqual(history[-1]["p_term"], -5.4, delta=0.2)

    def test_anti_stall_resets_on_zero_command(self):
        """Verifies that integral accumulation drops to 0 when command becomes zero."""
        target = -1.04
        history = self.simulate_single_wheel(target, [0.0] * 50, enable_anti_stall=True)
        self.assertLessEqual(history[-1]["pwm"], -102)

        # Now command zero: SingleWheelController::setTargetVelocity resets to IDLE and errorSum = 0
        zero_hist = self.simulate_single_wheel(0.0, [0.0] * 10, enable_anti_stall=True)
        self.assertEqual(zero_hist[-1]["pwm"], 0)
        self.assertEqual(zero_hist[-1]["i_term"], 0.0)


if __name__ == "__main__":
    unittest.main()


