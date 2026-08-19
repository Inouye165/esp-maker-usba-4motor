import unittest
import math

# Python Unit Test Harness for ESP32 Spin-Specific Stiction/Kinetic Assistance Layer

STICTION_IDLE = 0
STICTION_BOOST = 1
STICTION_KINETIC = 2
STICTION_BLOCKED = 3

STICTION_BOOST_PWM = 48
SPIN_STICTION_BOOST_PWM = 102
SPIN_BREAKOUT_MAX_CYCLES = 20
SPIN_KS_PWM = 58.0
SPIN_KINETIC_KS_PWM = 75.0
MIN_SPIN_KINETIC_FF_FLOOR = 80.0

SPIN_FORWARD_REAR_KINETIC_FLOOR = 94.0
SPIN_REVERSE_REAR_KINETIC_FLOOR = 80.0
SPIN_BREAKOUT_ENCODER_THRESHOLD = 3

class SingleWheelControllerSim:
    def __init__(self, idx):
        self.index = idx
        self.target_vel = 0.0
        self.measured_vel = 0.0
        self.is_spin_maneuver = False
        self.is_forward_rear_wheel = False
        self.last_pwm = 0
        self.stiction_state = STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False
        self.kp = 2.2
        self.ki = 0.0
        self.kd = 0.0
        self.kv = 6.0
        self.forward_breakaway = 40.0
        self.error_sum = 0.0
        self.last_error = 0.0

    def reset(self):
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_pwm = 0
        self.stiction_state = STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False

    def force_kinetic(self):
        self.stiction_state = STICTION_KINETIC


    def set_target_velocity(self, target_radps, is_spin=False, is_forward_rear=False):
        self.is_spin_maneuver = is_spin
        self.is_forward_rear_wheel = is_forward_rear
        if (self.stiction_state == STICTION_IDLE or abs(self.target_vel) < 0.01) and abs(target_radps) >= 0.01:
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
        elif (self.target_vel > 0.01 and target_radps < -0.01) or (self.target_vel < -0.01 and target_radps > 0.01):
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
        elif abs(target_radps) < 0.01:
            self.stiction_state = STICTION_IDLE
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
        self.target_vel = target_radps

    def update(self, measured_radps, current_ticks, dt=0.01):
        self.measured_vel = measured_radps
        if abs(self.target_vel) < 0.01:
            self.stiction_state = STICTION_IDLE
            self.last_pwm = 0
            return 0

        if self.stiction_state == STICTION_BOOST:
            if not self.boost_ticks_initialized:
                self.boost_start_ticks = current_ticks
                self.boost_ticks_initialized = True
            delta_ticks = abs(current_ticks - self.boost_start_ticks)
            if not self.is_spin_maneuver and delta_ticks >= 3:
                self.stiction_state = STICTION_KINETIC
            elif not self.is_spin_maneuver:
                self.boost_ticks_count += 1
                if self.boost_ticks_count >= 50:
                    self.stiction_state = STICTION_BLOCKED
                    self.last_pwm = 0
                    return 0
                boost_mag = STICTION_BOOST_PWM
                self.last_pwm = boost_mag if self.target_vel > 0 else -boost_mag
                return self.last_pwm
            else:
                self.boost_ticks_count += 1
                if self.boost_ticks_count >= SPIN_BREAKOUT_MAX_CYCLES:
                    self.stiction_state = STICTION_KINETIC
                else:
                    if self.boost_ticks_count >= 50:
                        self.stiction_state = STICTION_BLOCKED
                        self.last_pwm = 0
                        return 0
                    boost_mag = SPIN_STICTION_BOOST_PWM # 102 PWM
                    self.last_pwm = boost_mag if self.target_vel > 0 else -boost_mag
                    return self.last_pwm

        if self.stiction_state == STICTION_BLOCKED:
            self.last_pwm = 0
            return 0

        error = self.target_vel - measured_radps
        self.error_sum += error * dt
        integral_term = max(-150.0, min(150.0, self.error_sum * self.ki))
        derivative = (error - self.last_error) / dt
        self.last_error = error

        breakaway = SPIN_KINETIC_KS_PWM if self.is_spin_maneuver else self.forward_breakaway
        ff_magnitude = breakaway + (self.kv * abs(self.target_vel))


        if self.stiction_state == STICTION_KINETIC and abs(self.target_vel) >= 0.01:
            if self.is_spin_maneuver:
                min_ff_floor = SPIN_FORWARD_REAR_KINETIC_FLOOR if self.is_forward_rear_wheel else MIN_SPIN_KINETIC_FF_FLOOR
            else:
                min_ff_floor = 45.0
            if ff_magnitude < min_ff_floor:
                ff_magnitude = min_ff_floor

        feedforward = (1.0 if self.target_vel > 0 else -1.0) * ff_magnitude
        active_kp = 6.0 if self.is_spin_maneuver else self.kp
        pid_correction = (active_kp * error) + integral_term + (self.kd * derivative)
        total_pwm = feedforward + pid_correction
        self.last_pwm = max(-255, min(255, int(round(total_pwm))))
        return self.last_pwm


class WheelControllerSim:
    def __init__(self):
        self.controllers = [SingleWheelControllerSim(i) for i in range(4)]
        self.spin_sustained_cycles = 0

    def reset(self):
        self.spin_sustained_cycles = 0
        for c in self.controllers: c.reset()

    def set_targets(self, left_target_radps, right_target_radps, is_spin=False):
        m1_fwd_rear = False
        m2_fwd_rear = False
        m3_fwd_rear = is_spin and (left_target_radps > 0.01)
        m4_fwd_rear = is_spin and (right_target_radps > 0.01)

        self.controllers[0].set_target_velocity(left_target_radps, is_spin, m1_fwd_rear)
        self.controllers[1].set_target_velocity(right_target_radps, is_spin, m2_fwd_rear)
        self.controllers[2].set_target_velocity(left_target_radps, is_spin, m3_fwd_rear)
        self.controllers[3].set_target_velocity(right_target_radps, is_spin, m4_fwd_rear)

    def update(self, measured_vels, ticks, dt=0.01):
        for i in range(4):
            self.controllers[i].update(measured_vels[i], ticks[i], dt)

        any_spin_boost = any(c.stiction_state == STICTION_BOOST for c in self.controllers)
        if any_spin_boost:
            lr_moving = abs(measured_vels[2]) >= 0.10
            rr_moving = abs(measured_vels[3]) >= 0.10
            if lr_moving and rr_moving:
                self.spin_sustained_cycles += 1
            else:
                self.spin_sustained_cycles = 0
            max_boost_cycles = max(c.boost_ticks_count for c in self.controllers)
            early_exit_ok = (max_boost_cycles >= 10) and (self.spin_sustained_cycles >= 3)
            hard_timeout_ok = (max_boost_cycles >= 20)
            if early_exit_ok or hard_timeout_ok:
                for c in self.controllers: c.force_kinetic()
        else:
            self.spin_sustained_cycles = 0


class TestSpinAssistLayer(unittest.TestCase):
    def setUp(self):
        self.wc = WheelControllerSim()

    def test_01_ccw_direction_selection(self):
        self.wc.set_targets(-2.0, +2.0, is_spin=True)
        self.assertFalse(self.wc.controllers[0].is_forward_rear_wheel) # M1/LF
        self.assertFalse(self.wc.controllers[1].is_forward_rear_wheel) # M2/RF
        self.assertFalse(self.wc.controllers[2].is_forward_rear_wheel) # M3/LR (reverse)
        self.assertTrue(self.wc.controllers[3].is_forward_rear_wheel)  # M4/RR (forward rear)

    def test_02_cw_direction_selection(self):
        self.wc.set_targets(+2.0, -2.0, is_spin=True)
        self.assertFalse(self.wc.controllers[0].is_forward_rear_wheel) # M1/LF
        self.assertFalse(self.wc.controllers[1].is_forward_rear_wheel) # M2/RF
        self.assertTrue(self.wc.controllers[2].is_forward_rear_wheel)  # M3/LR (forward rear)
        self.assertFalse(self.wc.controllers[3].is_forward_rear_wheel) # M4/RR (reverse)

    def test_03_breakout_startup_boost_magnitude(self):
        self.wc.set_targets(-2.0, +2.0, is_spin=True)
        self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[0].last_pwm, -102) # LF
        self.assertEqual(self.wc.controllers[1].last_pwm, +102) # RF
        self.assertEqual(self.wc.controllers[2].last_pwm, -102) # LR
        self.assertEqual(self.wc.controllers[3].last_pwm, +102) # RR

    def test_04_twitch_and_min_dwell_protection(self):
        # Verify backlash/tiny-twitch does NOT end breakout during early dwell (< 100 ms / 10 cycles)
        self.wc.set_targets(-2.0, +2.0, is_spin=True)
        # Cycle 1: 5 ticks displacement twitch
        self.wc.update([0.5, 0.5, 0.5, 0.5], [5]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].stiction_state, STICTION_BOOST)

        # Cycles 2 to 8 (80 ms total): motion present, but minimum dwell (100 ms) not reached
        for _ in range(7):
            self.wc.update([0.5, 0.5, 0.5, 0.5], [10]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].stiction_state, STICTION_BOOST)
        self.assertEqual(self.wc.controllers[3].last_pwm, +102)

    def test_05_sustained_rear_motion_early_exit_after_dwell(self):
        # Verify sustained rear motion for 3 cycles ends breakout after min dwell (10 cycles / 100 ms)
        self.wc.set_targets(-2.0, +2.0, is_spin=True)
        # Cycles 1 to 7: zero motion
        for _ in range(7):
            self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].stiction_state, STICTION_BOOST)

        # Cycles 8, 9, 10: sustained rear motion >= 0.10 rad/s
        for _ in range(3):
            self.wc.update([0.0, 0.0, -0.2, 0.2], [0]*4, 0.01)

        # After 10 cycles and 3 sustained velocity cycles -> transitions to STICTION_KINETIC
        self.assertEqual(self.wc.controllers[3].stiction_state, STICTION_KINETIC)

    def test_06_hard_200ms_timeout_fallback(self):
        # Verify hard 200 ms (20 cycles) timeout forces kinetic transition if sustained motion isn't met
        self.wc.set_targets(-2.0, +2.0, is_spin=True)
        for _ in range(19):
            self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].stiction_state, STICTION_BOOST)

        # Cycle 20 (200 ms) -> hard timeout transitions to STICTION_KINETIC
        self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].stiction_state, STICTION_KINETIC)

    def test_07_post_breakout_kinetic_floors(self):
        self.wc.set_targets(-2.0, +2.0, is_spin=True)
        for c in self.wc.controllers: c.stiction_state = STICTION_KINETIC
        
        # Perfect target velocity match (PID error = 0)
        self.wc.update([-2.0, 2.0, -2.0, 2.0], [0]*4, 0.01)
        # FF = 75 + (6 * 2.0) = 87 PWM
        self.assertEqual(self.wc.controllers[3].last_pwm, +94) # Forward rear M4/RR gets 94 PWM floor
        self.assertEqual(self.wc.controllers[2].last_pwm, -87) # Reverse rear M3/LR gets 87 PWM (75 + 12)
        self.assertEqual(self.wc.controllers[0].last_pwm, -87) # Front LF gets 87 PWM
        self.assertEqual(self.wc.controllers[1].last_pwm, +87) # Front RF gets 87 PWM


    def test_08_zero_command_and_disarm_immediate_cancel(self):
        self.wc.set_targets(-2.0, +2.0, is_spin=True)
        self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].stiction_state, STICTION_BOOST)

        # Zero command or disarm cancels breakout state immediately
        self.wc.set_targets(0.0, 0.0, is_spin=False)
        self.wc.update([0.0]*4, [0]*4, 0.01)
        for c in self.wc.controllers:
            self.assertEqual(c.stiction_state, STICTION_IDLE)
            self.assertEqual(c.last_pwm, 0)

    def test_09_straight_driving_non_interference(self):
        self.wc.set_targets(2.0, 2.0, is_spin=False) # Straight driving
        self.assertFalse(self.wc.controllers[3].is_forward_rear_wheel)
        self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].last_pwm, 48) # Standard 48 PWM, not 102 PWM

        for c in self.wc.controllers: c.stiction_state = STICTION_KINETIC
        self.wc.update([2.0]*4, [0]*4, 0.01)
        self.assertLess(self.wc.controllers[3].last_pwm, 80) # Standard rolling floor (< 80 PWM)

    def test_10_pure_spin_kinetic_ff_75_base(self):
        self.wc.set_targets(-4.195, +4.195, is_spin=True) # Pure spin
        for c in self.wc.controllers: c.stiction_state = STICTION_KINETIC
        
        # Measured velocity matches target (error = 0) -> pure feedforward
        self.wc.update([-4.195, 4.195, -4.195, 4.195], [0]*4, 0.01)
        
        # FF = 75.0 + (6.0 * 4.195) = 100.17 PWM -> ~100 PWM
        self.assertEqual(self.wc.controllers[3].last_pwm, +100)
        self.assertEqual(self.wc.controllers[1].last_pwm, +100)
        self.assertEqual(self.wc.controllers[0].last_pwm, -100)
        self.assertEqual(self.wc.controllers[2].last_pwm, -100)

        # Verify low-speed forward-rear floor (94 PWM) still works for target = 1.0 rad/s
        self.wc.set_targets(-1.0, +1.0, is_spin=True)
        for c in self.wc.controllers: c.stiction_state = STICTION_KINETIC
        self.wc.update([-1.0, 1.0, -1.0, 1.0], [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].last_pwm, +94)
        self.assertEqual(self.wc.controllers[0].last_pwm, -81)

        # Switch to straight driving -> must use normal feedforward (40 + 6*4.195 = 65 PWM)
        self.wc.set_targets(+4.195, +4.195, is_spin=False)
        for c in self.wc.controllers: c.stiction_state = STICTION_KINETIC
        self.wc.update([4.195]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[3].last_pwm, +65)

if __name__ == '__main__':
    unittest.main()



