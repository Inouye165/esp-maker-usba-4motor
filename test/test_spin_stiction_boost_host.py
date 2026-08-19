import unittest
import math

# Emulated C++ SingleWheelController & WheelController
STICTION_IDLE = 0
STICTION_BOOST = 1
STICTION_KINETIC = 2
STICTION_BLOCKED = 3

STICTION_BOOST_PWM = 48
SPIN_STICTION_BOOST_PWM = 56
MIN_KINETIC_FF_FLOOR = 45.0
KP_SPEED = 2.2
KI_SPEED = 1.2
KD_SPEED = 0.05
KV = 6.0

class SingleWheelControllerSim:
    def __init__(self, motor_idx):
        self.index = motor_idx
        self.target_vel = 0.0
        self.measured_vel = 0.0
        self.is_spin_maneuver = False
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_pwm = 0
        self.stiction_state = STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False

    def set_target_velocity(self, target_radps, is_spin=False):
        self.is_spin_maneuver = is_spin
        # Transition from near-zero to non-zero: enter STICTION_BOOST
        if abs(self.target_vel) < 0.01 and abs(target_radps) >= 0.01:
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.error_sum = 0.0
            self.last_error = 0.0
        # Direction change across zero while active: trigger boost in new direction
        elif (self.target_vel > 0.01 and target_radps < -0.01) or (self.target_vel < -0.01 and target_radps > 0.01):
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.error_sum = 0.0
            self.last_error = 0.0
        # Transition back to near-zero: reset to IDLE
        elif abs(target_radps) < 0.01:
            self.stiction_state = STICTION_IDLE
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.error_sum = 0.0
            self.last_error = 0.0
        self.target_vel = target_radps

    def update(self, measured_radps, current_ticks, dt=0.01):
        self.measured_vel = measured_radps
        if abs(self.target_vel) < 0.01:
            self.stiction_state = STICTION_IDLE
            self.error_sum = 0.0
            self.last_error = 0.0
            self.last_pwm = 0
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            return 0

        if self.stiction_state == STICTION_BOOST:
            if not self.boost_ticks_initialized:
                self.boost_start_ticks = current_ticks
                self.boost_ticks_initialized = True

            delta_ticks = abs(current_ticks - self.boost_start_ticks)
            if delta_ticks >= 3:
                self.stiction_state = STICTION_KINETIC
            else:
                self.boost_ticks_count += 1
                if self.boost_ticks_count >= 50:
                    self.stiction_state = STICTION_BLOCKED
                    self.last_pwm = 0
                    return 0

                boost_mag = SPIN_STICTION_BOOST_PWM if self.is_spin_maneuver else STICTION_BOOST_PWM
                boost_pwm = boost_mag if self.target_vel > 0 else -boost_mag
                self.last_pwm = boost_pwm
                return self.last_pwm

        if self.stiction_state == STICTION_BLOCKED:
            self.last_pwm = 0
            return 0

        # KINETIC PID
        error = self.target_vel - measured_radps
        self.error_sum += error * dt
        integral = max(-150.0, min(150.0, self.error_sum * KI_SPEED))
        derivative = (error - self.last_error) / dt
        self.last_error = error

        breakaway = 40.0
        ff_mag = breakaway + (KV * abs(self.target_vel))
        if ff_mag < MIN_KINETIC_FF_FLOOR:
            ff_mag = MIN_KINETIC_FF_FLOOR
        feedforward = (1.0 if self.target_vel > 0 else -1.0) * ff_mag
        pid = (KP_SPEED * error) + integral + (KD_SPEED * derivative)
        total = feedforward + pid
        self.last_pwm = int(max(-255, min(255, round(total))))
        return self.last_pwm


class TestSpinStictionBoost(unittest.TestCase):
    def test_straight_start_uses_48_pwm(self):
        w = SingleWheelControllerSim(0)
        w.set_target_velocity(2.0, is_spin=False)
        pwm = w.update(measured_radps=0.0, current_ticks=100)
        self.assertEqual(pwm, 48)
        self.assertEqual(w.stiction_state, STICTION_BOOST)

    def test_pure_cw_spin_start_uses_56_pwm(self):
        # CW: Left wheel forward (+56), Right wheel reverse (-56)
        w_left = SingleWheelControllerSim(0)
        w_right = SingleWheelControllerSim(1)
        w_left.set_target_velocity(4.72, is_spin=True)
        w_right.set_target_velocity(-4.72, is_spin=True)

        pwm_l = w_left.update(0.0, 100)
        pwm_r = w_right.update(0.0, 100)
        self.assertEqual(pwm_l, +56)
        self.assertEqual(pwm_r, -56)

    def test_pure_ccw_spin_start_uses_56_pwm(self):
        # CCW: Left wheel reverse (-56), Right wheel forward (+56)
        w_left = SingleWheelControllerSim(0)
        w_right = SingleWheelControllerSim(1)
        w_left.set_target_velocity(-4.72, is_spin=True)
        w_right.set_target_velocity(+4.72, is_spin=True)

        pwm_l = w_left.update(0.0, 100)
        pwm_r = w_right.update(0.0, 100)
        self.assertEqual(pwm_l, -56)
        self.assertEqual(pwm_r, +56)

    def test_breakout_transition_to_kinetic(self):
        w = SingleWheelControllerSim(0)
        w.set_target_velocity(4.72, is_spin=True)
        self.assertEqual(w.update(0.0, 100), 56) # tick 0, delta = 0
        self.assertEqual(w.update(0.0, 102), 56) # tick 1, delta = 2
        pwm_kinetic = w.update(1.0, 105)         # tick 2, delta = 5 >= 3 -> enters KINETIC
        self.assertEqual(w.stiction_state, STICTION_KINETIC)
        self.assertGreaterEqual(pwm_kinetic, 45) # Retains kinetic floor

    def test_blocked_timeout_at_500ms(self):
        w = SingleWheelControllerSim(0)
        w.set_target_velocity(4.72, is_spin=True)
        for tick in range(49):
            pwm = w.update(0.0, 100)
            self.assertEqual(pwm, 56)
            self.assertEqual(w.stiction_state, STICTION_BOOST)
        # Tick 50 (500 ms at 100 Hz)
        pwm_blocked = w.update(0.0, 100)
        self.assertEqual(pwm_blocked, 0)
        self.assertEqual(w.stiction_state, STICTION_BLOCKED)

    def test_zero_target_resets_to_idle(self):
        w = SingleWheelControllerSim(0)
        w.set_target_velocity(4.72, is_spin=True)
        w.update(0.0, 100)
        self.assertEqual(w.stiction_state, STICTION_BOOST)
        w.set_target_velocity(0.0)
        self.assertEqual(w.stiction_state, STICTION_IDLE)
        self.assertEqual(w.update(0.0, 100), 0)

    def test_nonzero_speed_adjustment_remains_kinetic(self):
        w = SingleWheelControllerSim(0)
        w.set_target_velocity(2.0, is_spin=False)
        w.update(0.0, 100)
        w.update(2.0, 105) # Breakout -> KINETIC
        self.assertEqual(w.stiction_state, STICTION_KINETIC)
        # Adjust speed from 2.0 to 3.0 rad/s
        w.set_target_velocity(3.0, is_spin=False)
        self.assertEqual(w.stiction_state, STICTION_KINETIC) # Does NOT retrigger boost!

if __name__ == '__main__':
    unittest.main()
