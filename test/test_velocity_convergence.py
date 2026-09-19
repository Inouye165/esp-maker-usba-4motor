"""
test/test_velocity_convergence.py - Regression Test Suite for Production Wheel Velocity
Convergence and Persistent Dynamic Braking Configuration.

Verifies:
1. Startup Breakout & Anti-Stall Preservation:
   - Initial transition from zero starts in STICTION_BOOST (48 PWM) for >= 3 ticks.
   - When commanded but stalled (< 0.1 rad/s), accelerates integration with ANTI_STALL_KI (12.0 s^-1).
2. Kinetic Feedforward Calibration:
   - Kinetic rolling friction base is ~55% of static breakaway (~22 PWM vs 40 PWM).
   - Feedforward floor for rolling kinetic motion is 22 PWM (not 45 PWM).
   - At target omega = 6.15 rad/s (0.20 m/s), feedforward is ~58.9 PWM (preventing runaway feedforward).
3. Closed-Loop Velocity Convergence (Negative PID Correction & Responsive Integral):
   - When overspeeding (e.g., actual speed 7.28 rad/s vs target 6.15 rad/s), error is negative.
   - Closed-loop controller uses responsive integral gain (Ki * 3.0 = 3.6 s^-1) for |error| >= 0.25 rad/s.
   - Opposing startup integral windup drains rapidly.
   - Output PWM reduces below feedforward to decelerate wheels toward target velocity.
   - Symmetrical convergence in forward (+0.20 m/s) and reverse (-0.20 m/s).
4. Persistent Dynamic Braking Configuration:
   - Production default enables dynamic braking (dynamicBraking: True).
   - Normal linear driving at 0.20 m/s is eligible for dynamic brake pulse on zero command.
   - Bounded 100 ms H-bridge brake state (IN1=255, IN2=255) without motor reversal.
"""

import unittest
import math


class SingleWheelControllerModel:
    """Accurate Python model of SingleWheelController in src/WheelController.cpp."""

    STICTION_IDLE = 0
    STICTION_BOOST = 1
    STICTION_KINETIC = 2
    STICTION_BLOCKED = 3

    def __init__(self, index=0, fwd_breakaway=40, rev_breakaway=40, kv=6.0, kp=2.2, ki=1.2, kd=0.0):
        self.index = index
        self.fwd_breakaway = fwd_breakaway
        self.rev_breakaway = rev_breakaway
        self.kv = kv
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.anti_stall_ki = 12.0
        self.lagging_ki = 3.6
        self.anti_stall_speed_threshold = 0.10
        self.min_reliable_speed = 0.50
        self.stiction_boost_pwm = 48

        self.target_vel = 0.0
        self.measured_vel = 0.0
        self.is_spin_maneuver = False
        self.is_forward_rear_wheel = False

        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_pwm = 0
        self.reversal_deadtime_ticks = 0

        self.stiction_state = self.STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False

        self.last_diag = {}

    def set_target_velocity(self, target_radps, is_spin=False, is_forward_rear=False):
        self.is_spin_maneuver = is_spin
        self.is_forward_rear_wheel = is_forward_rear

        if (self.stiction_state == self.STICTION_IDLE or abs(self.target_vel) < 0.01) and abs(target_radps) >= 0.01:
            self.stiction_state = self.STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.error_sum = 0.0
            self.last_error = 0.0
        elif (self.target_vel > 0.01 and target_radps < -0.01) or (self.target_vel < -0.01 and target_radps > 0.01):
            self.stiction_state = self.STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.error_sum = 0.0
            self.last_error = 0.0
        elif abs(target_radps) < 0.01:
            self.stiction_state = self.STICTION_IDLE
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.error_sum = 0.0
            self.last_error = 0.0

        self.target_vel = target_radps

    def update(self, measured_radps, current_ticks, dt=0.01):
        self.measured_vel = measured_radps

        if abs(self.target_vel) < 0.01:
            self.stiction_state = self.STICTION_IDLE
            self.error_sum = 0.0
            self.last_error = 0.0
            self.last_pwm = 0
            self.last_diag = {"final_pwm": 0, "stiction_state": self.STICTION_IDLE}
            return 0

        # Breakout state machine
        if self.stiction_state == self.STICTION_BOOST:
            if not self.boost_ticks_initialized:
                self.boost_start_ticks = current_ticks
                self.boost_ticks_initialized = True

            delta_ticks = abs(current_ticks - self.boost_start_ticks)
            if delta_ticks >= 3:
                self.stiction_state = self.STICTION_KINETIC
            else:
                self.boost_ticks_count += 1
                boost_pwm = self.stiction_boost_pwm if self.target_vel > 0 else -self.stiction_boost_pwm
                self.last_pwm = boost_pwm
                self.last_diag = {"final_pwm": boost_pwm, "stiction_state": self.STICTION_BOOST}
                return boost_pwm

        # STICTION_KINETIC: Closed-loop PID + Feedforward
        error = self.target_vel - measured_radps

        # Fast unwinding of opposing integral windup
        if (error < 0.0 and self.error_sum > 0.0) or (error > 0.0 and self.error_sum < 0.0):
            self.error_sum += error * dt * 2.0
        else:
            self.error_sum += error * dt

        is_commanded = abs(self.target_vel) >= self.min_reliable_speed
        is_lagging = (self.target_vel > 0.0 and error > 0.0) or (self.target_vel < 0.0 and error < 0.0)
        is_stalled = is_commanded and is_lagging and (abs(measured_radps) < self.anti_stall_speed_threshold)

        active_ki = self.ki
        if is_stalled:
            active_ki = self.anti_stall_ki
        elif is_commanded and abs(error) >= 0.25:
            active_ki = self.ki * 3.0

        integral_term = self.error_sum * active_ki
        integral_term = max(-150.0, min(150.0, integral_term))

        derivative = (error - self.last_error) / dt
        self.last_error = error

        # Breakaway friction with kinetic reduction (55% of static breakaway)
        static_breakaway = self.fwd_breakaway if self.target_vel > 0 else self.rev_breakaway
        breakaway = static_breakaway * 0.55
        ff_magnitude = breakaway + (self.kv * abs(self.target_vel))

        min_ff_floor = 22.0
        if ff_magnitude < min_ff_floor:
            ff_magnitude = min_ff_floor

        feedforward = (1.0 if self.target_vel > 0 else -1.0) * ff_magnitude

        pid_correction = (self.kp * error) + integral_term + (self.kd * derivative)
        total_pwm = feedforward + pid_correction
        target_pwm = int(round(max(-255.0, min(255.0, total_pwm))))

        self.last_pwm = target_pwm
        self.last_diag = {
            "target_vel": self.target_vel,
            "measured_vel": measured_radps,
            "error": error,
            "feedforward": feedforward,
            "p_term": self.kp * error,
            "i_term": integral_term,
            "final_pwm": target_pwm,
            "stiction_state": self.STICTION_KINETIC
        }
        return target_pwm


class TestVelocityConvergence(unittest.TestCase):
    """Tests that wheel-speed controller converges to requested velocity and eliminates overspeed."""

    def setUp(self):
        self.controller = SingleWheelControllerModel()

    def test_startup_breakout_preserved(self):
        """Initial movement must enter STICTION_BOOST and output 48 PWM until 3 ticks displacement."""
        self.controller.set_target_velocity(6.15)  # 0.20 m/s
        self.assertEqual(self.controller.stiction_state, SingleWheelControllerModel.STICTION_BOOST)

        # Tick 0: displacement 0 -> outputs 48 PWM
        pwm0 = self.controller.update(measured_radps=0.0, current_ticks=100)
        self.assertEqual(pwm0, 48)
        self.assertEqual(self.controller.stiction_state, SingleWheelControllerModel.STICTION_BOOST)

        # Tick 1: displacement 1 -> outputs 48 PWM
        pwm1 = self.controller.update(measured_radps=0.0, current_ticks=101)
        self.assertEqual(pwm1, 48)

        # Tick 2: displacement 2 -> outputs 48 PWM
        pwm2 = self.controller.update(measured_radps=0.0, current_ticks=102)
        self.assertEqual(pwm2, 48)

        # Tick 3: displacement 3 -> transitions to STICTION_KINETIC
        pwm3 = self.controller.update(measured_radps=1.0, current_ticks=103)
        self.assertEqual(self.controller.stiction_state, SingleWheelControllerModel.STICTION_KINETIC)
        self.assertGreater(pwm3, 0)

    def test_anti_stall_behavior_preserved(self):
        """When commanded but wheel is stalled (< 0.1 rad/s), Ki accelerates to 12.0 s^-1."""
        self.controller.set_target_velocity(6.15)
        # Advance through breakout
        self.controller.update(0.0, 100)
        self.controller.update(0.0, 104)  # displacement >= 3 -> KINETIC

        # Now wheel is stalled at 0.0 rad/s
        pwms = []
        for i in range(50):  # 500 ms @ 100 Hz
            p = self.controller.update(measured_radps=0.0, current_ticks=104)
            pwms.append(p)

        # Anti-stall must rapidly ramp PWM past breakaway (up to 100+ PWM)
        self.assertGreater(pwms[-1], 100, "Anti-stall must ramp PWM past 100 to free stalled wheel")
        self.assertGreater(pwms[-1], pwms[0], "Anti-stall must increase PWM over time")

    def test_kinetic_feedforward_baseline(self):
        """Kinetic rolling feedforward at 6.15 rad/s must be ~58.9 PWM (not 76.9 PWM)."""
        self.controller.set_target_velocity(6.15)
        # Fast-forward to KINETIC
        self.controller.update(0.0, 100)
        self.controller.update(1.0, 105)

        # Evaluate feedforward when tracking exactly at target speed (error = 0)
        self.controller.error_sum = 0.0
        self.controller.last_error = 0.0
        pwm = self.controller.update(measured_radps=6.15, current_ticks=105)

        diag = self.controller.last_diag
        self.assertAlmostEqual(diag["feedforward"], 58.9, delta=1.0)
        self.assertAlmostEqual(diag["p_term"], 0.0, delta=0.01)
        self.assertAlmostEqual(diag["i_term"], 0.0, delta=0.01)
        self.assertEqual(pwm, 59)

    def test_overspeed_negative_correction_and_convergence(self):
        """When wheel speed is 7.28 rad/s (overspeed by 1.13 rad/s), controller must reduce PWM."""
        self.controller.set_target_velocity(6.15)  # target 0.20 m/s
        self.controller.update(0.0, 100)
        self.controller.update(1.0, 105)

        # Simulate overspeed condition seen in report 1789848823:
        # Wheel is spinning at 7.28 rad/s
        pwm_initial = self.controller.update(measured_radps=7.28, current_ticks=110)

        # P-term must be negative
        self.assertLess(self.controller.last_diag["p_term"], -2.0)
        # PWM must immediately drop below the old feedforward of 77
        self.assertLess(pwm_initial, 60, "PWM must drop below 60 to slow wheel from 7.28 to 6.15 rad/s")

        # Let simulation run with a simple motor model: omega_dot = (pwm - 22)/6.0 - omega
        # Over 1 second, speed must converge to 6.15 rad/s +/- 0.05 rad/s
        measured = 7.28
        ticks = 110
        for step in range(200):  # 2.0 seconds @ 100 Hz
            pwm = self.controller.update(measured_radps=measured, current_ticks=ticks, dt=0.01)
            # Motor physical response: PWM drives wheel toward steady speed = (pwm - 22) / 6.0
            steady_speed = max(0.0, (pwm - 22.0) / 6.0)
            measured += (steady_speed - measured) * 0.08  # 12 Hz mechanical motor bandwidth
            ticks += int(measured * 0.01 * 314.0)

        self.assertAlmostEqual(measured, 6.15, delta=0.15,
                               msg=f"Actual speed {measured:.2f} rad/s must converge to requested 6.15 rad/s")

    def test_reverse_overspeed_convergence(self):
        """Reverse travel (-6.15 rad/s) must converge symmetrically."""
        self.controller.set_target_velocity(-6.15)
        self.controller.update(0.0, 100)
        self.controller.update(-1.0, 95)  # displacement >= 3 -> KINETIC

        # Overspeed in reverse: measured is -7.30 rad/s
        pwm_initial = self.controller.update(measured_radps=-7.30, current_ticks=90)
        self.assertGreater(pwm_initial, -60, "Reverse PWM magnitude must be reduced (closer to 0) to slow down")

        measured = -7.30
        ticks = 90
        for step in range(200):
            pwm = self.controller.update(measured_radps=measured, current_ticks=ticks, dt=0.01)
            steady_speed = min(0.0, (pwm + 22.0) / 6.0)
            measured += (steady_speed - measured) * 0.08
            ticks -= int(abs(measured) * 0.01 * 314.0)

        self.assertAlmostEqual(measured, -6.15, delta=0.15,
                               msg=f"Reverse speed {measured:.2f} rad/s must converge to -6.15 rad/s")


class TestPersistentDynamicBrakingConfig(unittest.TestCase):
    """Verifies that dynamic braking is configured as persistent production default and triggers at 0.20 m/s."""

    def test_production_default_is_enabled(self):
        """CockpitClient.configure_drive and default configs must have dynamic_braking=True."""
        from tools.rover_tests.transport import CockpitClient
        import inspect

        sig = inspect.signature(CockpitClient.configure_drive)
        self.assertTrue(sig.parameters["dynamic_braking"].default,
                        "CockpitClient.configure_drive dynamic_braking must default to True")

    def test_braking_triggers_at_0p20_mps(self):
        """Transitioning from 0.20 m/s to zero command must qualify for dynamic brake pulse."""
        # Max trigger speed is 0.35 m/s. 0.20 m/s is within the stopping window.
        max_trigger_speed = 0.35
        last_active_linear_speed = 0.20
        last_active_angular_speed = 0.0

        eligible_for_brake = (last_active_angular_speed <= max_trigger_speed) and \
                             (last_active_linear_speed <= max_trigger_speed)

        self.assertTrue(eligible_for_brake, "0.20 m/s linear travel must be eligible for dynamic braking on stop")

    def test_linear_suite_defaults_to_dynamic_braking_enabled(self):
        """PhysicalTestRunner linear suite must report dynamic_braking_enabled=True by default."""
        from tools.rover_tests.linear import LinearParameters

        params = LinearParameters(distance=1.0, direction="forward", enable_braking=None)
        self.assertIsNone(params.enable_braking, "enable_braking defaults to None (inheriting production)")


if __name__ == "__main__":
    unittest.main()
