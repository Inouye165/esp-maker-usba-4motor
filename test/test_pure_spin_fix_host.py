import unittest
import math

STICTION_IDLE = 0
STICTION_BOOST = 1
STICTION_KINETIC = 2
STICTION_BLOCKED = 3

STICTION_BOOST_PWM = 48
SPIN_STICTION_BOOST_PWM = 85
SPIN_KS_PWM = 58.0
MIN_SPIN_KINETIC_FF_FLOOR = 80.0

class SingleWheelControllerSim:
    def __init__(self, idx):
        self.index = idx
        self.target_vel = 0.0
        self.measured_vel = 0.0
        self.is_spin_maneuver = False
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

    def force_kinetic(self):
        self.stiction_state = STICTION_KINETIC

    def set_target_velocity(self, target_radps, is_spin=False):
        self.is_spin_maneuver = is_spin
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

            delta = abs(current_ticks - self.boost_start_ticks)
            if not self.is_spin_maneuver and delta >= 3:
                self.stiction_state = STICTION_KINETIC
            elif not self.is_spin_maneuver:
                self.boost_ticks_count += 1
                if self.boost_ticks_count >= 50:
                    self.stiction_state = STICTION_BLOCKED
                    self.last_pwm = 0
                    return 0
                self.last_pwm = STICTION_BOOST_PWM if self.target_vel > 0 else -STICTION_BOOST_PWM
                return self.last_pwm
            else:
                self.boost_ticks_count += 1
                if self.boost_ticks_count >= 50:
                    self.stiction_state = STICTION_BLOCKED
                    self.last_pwm = 0
                    return 0
                self.last_pwm = SPIN_STICTION_BOOST_PWM if self.target_vel > 0 else -SPIN_STICTION_BOOST_PWM
                return self.last_pwm

        if self.stiction_state == STICTION_BLOCKED:
            self.last_pwm = 0
            return 0

        error = self.target_vel - measured_radps
        self.error_sum += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error

        breakaway = SPIN_KS_PWM if self.is_spin_maneuver else self.forward_breakaway
        ff_mag = breakaway + (self.kv * abs(self.target_vel))
        min_floor = MIN_SPIN_KINETIC_FF_FLOOR if self.is_spin_maneuver else 45.0
        if ff_mag < min_floor:
            ff_mag = min_floor

        feedforward = (1.0 if self.target_vel > 0 else -1.0) * ff_mag
        pid_correction = (self.kp * error) + (self.ki * self.error_sum) + (self.kd * derivative)
        total_pwm = feedforward + pid_correction
        self.last_pwm = max(-255, min(255, int(round(total_pwm))))
        return self.last_pwm

    def reset(self):
        self.stiction_state = STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False
        self.last_pwm = 0
        self.error_sum = 0.0
        self.last_error = 0.0

class TestRotationShimRampAndHandoff(unittest.TestCase):
    def setUp(self):
        self.controllers = [SingleWheelControllerSim(i) for i in range(4)]
        self.current_ticks = [0, 0, 0, 0]
        self.measured_vel = [0.0, 0.0, 0.0, 0.0]
        self.imu_gz = 0.0
        self.spin_recovery_state = 0
        self.spin_start_time_ms = 0
        self.spin_state_entered_ms = 0
        self.requested_angular = 0.0
        self.candidate_a_consumed = False
        self.chassis_rebreakout_consumed = False
        self.spin_breakout_sustained_ticks = 0
        self.zero_debounce_ticks = 0
        self.stalled_kinetic_ticks = 0
        self.stall_window_initialized = False
        self.stall_window_start_ticks = [0, 0, 0, 0]
        self.prev_maneuver = 0
        self.candidate_a_fired = 0
        self.chassis_rebreakout_fired = 0

    def set_targets(self, left, right, is_spin):
        self.controllers[0].set_target_velocity(left, is_spin)
        self.controllers[2].set_target_velocity(left, is_spin)
        self.controllers[1].set_target_velocity(right, is_spin)
        self.controllers[3].set_target_velocity(right, is_spin)

    def transition_all_to_kinetic(self):
        for c in self.controllers:
            c.force_kinetic()

    def update_spin_recovery(self, lin_cmd, ang_cmd, now_ms):
        is_spin = (abs(lin_cmd) <= 0.005) and (abs(ang_cmd) >= 0.005)
        if not is_spin:
            self.spin_recovery_state = 0
            self.spin_breakout_sustained_ticks = 0
            return

        cmd_sign = 1.0 if ang_cmd >= 0.0 else -1.0
        imu_rotating = (self.imu_gz * cmd_sign >= 0.08)

        moving = 0
        for i in range(4):
            v = self.measured_vel[i]
            exp_sign = -cmd_sign if (i == 0 or i == 2) else cmd_sign
            if v * exp_sign >= 0.10:
                moving += 1

        genuine = (moving >= 3) and imu_rotating
        if genuine:
            self.spin_breakout_sustained_ticks += 1
            if self.spin_breakout_sustained_ticks >= 3:
                self.transition_all_to_kinetic()
                self.spin_recovery_state = 0
        else:
            self.spin_breakout_sustained_ticks = 0

        if self.spin_recovery_state == 0:
            if not self.candidate_a_consumed:
                any_boost = any(c.stiction_state == STICTION_BOOST for c in self.controllers)
                if any_boost:
                    self.spin_recovery_state = 1
                    self.spin_start_time_ms = now_ms
                    self.requested_angular = ang_cmd
        elif self.spin_recovery_state == 1:
            if genuine:
                self.spin_recovery_state = 0
            elif now_ms - self.spin_start_time_ms >= 350:
                self.spin_recovery_state = 2
                self.spin_state_entered_ms = now_ms
                self.candidate_a_consumed = True
                self.candidate_a_fired += 1
                for c in self.controllers: c.reset()
        elif self.spin_recovery_state == 2:
            if now_ms - self.spin_state_entered_ms >= 30:
                self.spin_recovery_state = 3
                self.spin_state_entered_ms = now_ms
                for c in self.controllers: c.reset()
        elif self.spin_recovery_state == 3:
            if now_ms - self.spin_state_entered_ms >= 50:
                self.spin_recovery_state = 4
                self.spin_state_entered_ms = now_ms
                for c in self.controllers: c.reset()
        elif self.spin_recovery_state == 4:
            if now_ms - self.spin_state_entered_ms >= 30:
                self.spin_recovery_state = 5
                for c in self.controllers: c.reset()

    def update_chassis_supervisor(self, lin_cmd, ang_cmd, is_armed, now_ms):
        is_zero = (abs(lin_cmd) < 0.005) and (abs(ang_cmd) < 0.005)
        if not is_armed:
            self.candidate_a_consumed = False
            self.chassis_rebreakout_consumed = False
            self.zero_debounce_ticks = 0
            self.stalled_kinetic_ticks = 0
            self.stall_window_initialized = False
            self.prev_maneuver = 0
        elif is_zero:
            self.zero_debounce_ticks += 1
            if self.zero_debounce_ticks >= 5:
                self.candidate_a_consumed = False
                self.chassis_rebreakout_consumed = False
                self.stalled_kinetic_ticks = 0
                self.stall_window_initialized = False
                self.prev_maneuver = 0
        else:
            self.zero_debounce_ticks = 0

        curr_maneuver = 1 if (is_armed and not is_zero and abs(lin_cmd) <= 0.005 and abs(ang_cmd) >= 0.005) else (2 if is_armed and not is_zero else 0)

        if is_armed and not is_zero and not self.chassis_rebreakout_consumed:
            all_kinetic = all(c.stiction_state == STICTION_KINETIC for c in self.controllers)
            zero_vel_count = sum(1 for v in self.measured_vel if abs(v) < 0.05)
            if not self.stall_window_initialized:
                self.stall_window_start_ticks = list(self.current_ticks)
                self.stall_window_initialized = True
            static_count = sum(1 for i in range(4) if abs(self.current_ticks[i] - self.stall_window_start_ticks[i]) < 3)
            imu_stat = (abs(self.imu_gz) < 0.04)

            if all_kinetic and (zero_vel_count >= 3) and (static_count >= 3) and imu_stat:
                self.stalled_kinetic_ticks += 1
                if self.stalled_kinetic_ticks >= 20:
                    for c in self.controllers: c.reset()
                    self.chassis_rebreakout_consumed = True
                    self.chassis_rebreakout_fired += 1
                    self.stalled_kinetic_ticks = 0
                    self.stall_window_initialized = False
                    if curr_maneuver == 1:
                        self.spin_recovery_state = 0
                        self.candidate_a_consumed = False
            else:
                self.stalled_kinetic_ticks = 0
                self.stall_window_initialized = False
        else:
            self.stalled_kinetic_ticks = 0
            self.stall_window_initialized = False
        self.prev_maneuver = curr_maneuver

    def test_ramp_wz_0_20(self):
        w_tgt = (0.340858 / (2.0 * 0.0325)) * 0.20
        self.set_targets(-w_tgt, w_tgt, True)
        self.transition_all_to_kinetic()
        p0 = abs(self.controllers[1].update(0.0, 0, 0.01))
        p_half = abs(self.controllers[1].update(0.5, 10, 0.01))
        self.assertGreaterEqual(p0, 82)
        self.assertGreaterEqual(p_half, 81)

    def test_ramp_wz_0_40(self):
        w_tgt = (0.340858 / (2.0 * 0.0325)) * 0.40
        self.set_targets(-w_tgt, w_tgt, True)
        self.transition_all_to_kinetic()
        p0 = abs(self.controllers[1].update(0.0, 0, 0.01))
        self.assertGreaterEqual(p0, 84)

    def test_ramp_wz_0_60(self):
        w_tgt = (0.340858 / (2.0 * 0.0325)) * 0.60
        self.set_targets(-w_tgt, w_tgt, True)
        self.transition_all_to_kinetic()
        p0 = abs(self.controllers[1].update(0.0, 0, 0.01))
        self.assertGreaterEqual(p0, 86)

    def test_ramp_wz_0_80(self):
        w_tgt = (0.340858 / (2.0 * 0.0325)) * 0.80
        self.set_targets(-w_tgt, w_tgt, True)
        self.transition_all_to_kinetic()
        p0 = abs(self.controllers[1].update(0.0, 0, 0.01))
        pss = abs(self.controllers[1].update(w_tgt, 50, 0.01))
        self.assertGreaterEqual(p0, 92)
        self.assertEqual(pss, 83)

if __name__ == '__main__':
    unittest.main()
