import unittest
import math

# Dynamic Stiction States matching WheelController.h
STICTION_IDLE = 0
STICTION_BOOST = 1
STICTION_KINETIC = 2
STICTION_BLOCKED = 3

STICTION_BOOST_PWM = 48
SPIN_STICTION_BOOST_PWM = 102
SPIN_BREAKOUT_MAX_CYCLES = 20
KP_SPEED = 2.2
KI_SPEED = 1.2
KD_SPEED = 0.05
WHEEL_SEPARATION_M = 0.3408575
WHEEL_RADIUS_M = 0.033475

def velocity_to_wheels(linear, angular):
    left_vel_mps = linear - (angular * WHEEL_SEPARATION_M / 2.0)
    right_vel_mps = linear + (angular * WHEEL_SEPARATION_M / 2.0)
    return left_vel_mps / WHEEL_RADIUS_M, right_vel_mps / WHEEL_RADIUS_M

class SingleWheelControllerSim:
    def __init__(self, idx):
        self.index = idx
        self.target_vel = 0.0
        self.measured_vel = 0.0
        self.is_spin_maneuver = False
        self.is_forward_rear_wheel = False
        self.is_opposing_maneuver = False
        self.blocked_retries_in_episode = 0
        self.last_pwm = 0
        self.stiction_state = STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False
        self.kp = KP_SPEED
        self.ki = KI_SPEED
        self.kd = KD_SPEED
        self.error_sum = 0.0
        self.last_error = 0.0
        self.kv = 6.0
        self.forward_breakaway = 40.0

    def reset(self):
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_pwm = 0
        self.stiction_state = STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False
        self.is_opposing_maneuver = False
        self.blocked_retries_in_episode = 0

    def force_kinetic(self):
        self.stiction_state = STICTION_KINETIC

    def set_target_velocity(self, target_radps, is_spin=False, is_forward_rear=False, is_opposing=False):
        was_opposing_or_spin = self.is_spin_maneuver or self.is_opposing_maneuver
        self.is_spin_maneuver = is_spin
        self.is_forward_rear_wheel = is_forward_rear
        self.is_opposing_maneuver = is_opposing
        is_now_opposing_or_spin = is_spin or is_opposing

        # Direction change across zero while active:
        # Require meaningful target (|targetRadps| >= 0.10) to qualify as a deliberate reversal rather than noise
        is_reversal = (self.target_vel > 0.05 and target_radps < -0.05) or (self.target_vel < -0.05 and target_radps > 0.05)
        
        # Transition from near-zero OR from STICTION_IDLE to non-zero: enter STICTION_BOOST
        if (self.stiction_state == STICTION_IDLE or abs(self.target_vel) < 0.01) and abs(target_radps) >= 0.01:
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.error_sum = 0.0
            self.last_error = 0.0
        # Deliberate direction change across zero while active and NOT blocked: trigger boost in new direction
        elif is_reversal and self.stiction_state != STICTION_BLOCKED:
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.blocked_retries_in_episode = 0
            self.error_sum = 0.0
            self.last_error = 0.0
        # Deliberate direction change while blocked: allow one reversal attempt if commanded speed is substantial (>= 0.15 rad/s)
        elif is_reversal and self.stiction_state == STICTION_BLOCKED and abs(target_radps) >= 0.15 and self.blocked_retries_in_episode == 0:
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.blocked_retries_in_episode = 1
            self.error_sum = 0.0
            self.last_error = 0.0
        # Transition from non-opposing to opposing/spin while blocked: allow at most ONE re-boost retry per episode
        elif self.stiction_state == STICTION_BLOCKED and not was_opposing_or_spin and is_now_opposing_or_spin and self.blocked_retries_in_episode == 0 and abs(target_radps) >= 0.01:
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.blocked_retries_in_episode = 1
            self.error_sum = 0.0
            self.last_error = 0.0
        # Transition back to near-zero: reset to IDLE and clear episode retries
        elif abs(target_radps) < 0.01:
            self.stiction_state = STICTION_IDLE
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
            self.blocked_retries_in_episode = 0
            self.error_sum = 0.0
            self.last_error = 0.0

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

            # For non-pure-spin (straight, rolling arc, or opposing tight arc):
            # Breakout confirmation strictly >= 3 ticks displacement (0.31 mm)
            if not self.is_spin_maneuver and delta_ticks >= 3:
                self.stiction_state = STICTION_KINETIC
            elif not self.is_spin_maneuver:
                self.boost_ticks_count += 1
                if self.boost_ticks_count >= 50: # 500 ms @ 100 Hz
                    self.stiction_state = STICTION_BLOCKED
                    self.last_pwm = 0
                    return 0
                boost_mag = SPIN_STICTION_BOOST_PWM if self.is_opposing_maneuver else STICTION_BOOST_PWM
                boost_pwm = boost_mag if self.target_vel > 0 else -boost_mag
                self.last_pwm = boost_pwm
                return self.last_pwm
            else:
                # For pure spin maneuver:
                self.boost_ticks_count += 1
                if self.boost_ticks_count >= 50: # 500 ms safety shutoff
                    self.stiction_state = STICTION_BLOCKED
                    self.last_pwm = 0
                    return 0
                if self.boost_ticks_count >= SPIN_BREAKOUT_MAX_CYCLES:
                    self.stiction_state = STICTION_KINETIC
                else:
                    boost_mag = SPIN_STICTION_BOOST_PWM
                    boost_pwm = boost_mag if self.target_vel > 0 else -boost_mag
                    self.last_pwm = boost_pwm
                    return self.last_pwm

        if self.stiction_state == STICTION_BLOCKED:
            self.last_pwm = 0
            return 0

        # STICTION_KINETIC calculation
        error = self.target_vel - measured_radps
        self.error_sum += error * dt
        integral_term = max(-150.0, min(150.0, self.error_sum * self.ki))
        derivative = (error - self.last_error) / dt
        self.last_error = error

        breakaway = 75.0 if self.is_spin_maneuver else (self.forward_breakaway * 0.55)
        ff_magnitude = breakaway + (self.kv * abs(self.target_vel))

        if self.stiction_state == STICTION_KINETIC and abs(self.target_vel) >= 0.01:
            if self.is_spin_maneuver:
                min_ff_floor = 94.0 if self.is_forward_rear_wheel else 80.0
            else:
                min_ff_floor = 22.0
            if ff_magnitude < min_ff_floor:
                ff_magnitude = min_ff_floor

        feedforward = (1.0 if self.target_vel > 0 else -1.0) * ff_magnitude
        active_kp = 10.0 if self.is_spin_maneuver else self.kp
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
        for c in self.controllers:
            c.reset()

    def set_targets(self, left_target_radps, right_target_radps, is_spin=False, is_opposing=False):
        m3_fwd_rear = is_spin and (left_target_radps > 0.01)
        m4_fwd_rear = is_spin and (right_target_radps > 0.01)
        self.controllers[0].set_target_velocity(left_target_radps, is_spin, False, is_opposing)
        self.controllers[1].set_target_velocity(right_target_radps, is_spin, False, is_opposing)
        self.controllers[2].set_target_velocity(left_target_radps, is_spin, m3_fwd_rear, is_opposing)
        self.controllers[3].set_target_velocity(right_target_radps, is_spin, m4_fwd_rear, is_opposing)

    def route_command(self, vx, wz):
        ZERO_VELOCITY_EPSILON = 0.005
        is_spin = (abs(vx) <= ZERO_VELOCITY_EPSILON) and (abs(wz) > ZERO_VELOCITY_EPSILON)
        wl, wr = velocity_to_wheels(vx, wz)
        is_opposing = (wl * wr < -0.001)
        self.set_targets(wl, wr, is_spin=is_spin, is_opposing=is_opposing)
        return is_spin, is_opposing, wl, wr

    def update(self, measured_vels, ticks, dt=0.01):
        pwms = [self.controllers[i].update(measured_vels[i], ticks[i], dt) for i in range(4)]
        any_spin_boost = any(c.stiction_state == STICTION_BOOST and c.is_spin_maneuver for c in self.controllers)
        if any_spin_boost:
            lr_moving = abs(measured_vels[2]) >= 0.10
            rr_moving = abs(measured_vels[3]) >= 0.10
            if lr_moving and rr_moving:
                self.spin_sustained_cycles += 1
            else:
                self.spin_sustained_cycles = 0
            max_boost = max(c.boost_ticks_count for c in self.controllers)
            if (max_boost >= 10 and self.spin_sustained_cycles >= 3) or (max_boost >= 20):
                for c in self.controllers:
                    c.force_kinetic()
        else:
            self.spin_sustained_cycles = 0
        return pwms


class TestTightArcAndBlockedProtection(unittest.TestCase):
    def setUp(self):
        self.wc = WheelControllerSim()

    def test_01_pure_spin_breakout_and_sustained_motion(self):
        # Pure spin: vx = 0.0, wz = 0.372
        is_spin, is_opposing, wl, wr = self.wc.route_command(0.0, 0.372)
        self.assertTrue(is_spin)
        self.assertTrue(is_opposing)
        pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
        # Full 102 PWM spin boost
        self.assertEqual(pwms, [-102, 102, -102, 102])

    def test_02_tight_arc_opposing_breakout_boost(self):
        # Tight arc: vx = 0.014, wz = 0.500 -> wl = -2.13, wr = +2.96
        is_spin, is_opposing, wl, wr = self.wc.route_command(0.014, 0.500)
        self.assertFalse(is_spin, "vx > 0.005 is not pure spin")
        self.assertTrue(is_opposing, "Counter-rotating wheels must be classified as opposing")
        self.assertLess(wl, 0)
        self.assertGreater(wr, 0)

        # Initial cycle at zero displacement: receives 102 PWM breakout boost
        pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(pwms, [-102, 102, -102, 102])
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BOOST)

        # After 4 ticks displacement: transitions to KINETIC without needing rear-pair check
        pwms = self.wc.update([-0.5, 0.5, -0.5, 0.5], [4, 4, 4, 4], 0.01)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_KINETIC)
        self.assertEqual(self.wc.controllers[1].stiction_state, STICTION_KINETIC)

    def test_03_ordinary_forward_travel(self):
        # Forward travel: vx = 0.200, wz = 0.0
        is_spin, is_opposing, wl, wr = self.wc.route_command(0.200, 0.0)
        self.assertFalse(is_spin)
        self.assertFalse(is_opposing)
        pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
        # Normal 48 PWM boost, never 102 PWM
        self.assertEqual(pwms, [48, 48, 48, 48])

    def test_04_zero_command_idle_reset(self):
        # Drive first
        self.wc.route_command(0.014, 0.500)
        self.wc.update([0.0]*4, [0]*4, 0.01)
        # Command zero
        is_spin, is_opposing, wl, wr = self.wc.route_command(0.0, 0.0)
        pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(pwms, [0, 0, 0, 0])
        for c in self.wc.controllers:
            self.assertEqual(c.stiction_state, STICTION_IDLE)
            self.assertEqual(c.blocked_retries_in_episode, 0)

    def test_05_blocked_wheel_constant_tight_arc_and_pure_spin(self):
        # Test 5A: Constant tight arc with completely locked wheels (0 ticks)
        self.wc.route_command(0.014, 0.500)
        # Cycles 1 to 49 (490 ms): outputs 102 PWM boost
        for _ in range(49):
            pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
            self.assertEqual(pwms, [-102, 102, -102, 102])
            self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BOOST)

        # Cycle 50 (500 ms timeout): transitions to STICTION_BLOCKED, PWM cuts to 0
        pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(pwms, [0, 0, 0, 0])
        for c in self.wc.controllers:
            self.assertEqual(c.stiction_state, STICTION_BLOCKED)

        # Next 100 cycles (1 second) with constant command: remains strictly 0 PWM
        for _ in range(100):
            pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
            self.assertEqual(pwms, [0, 0, 0, 0])

        # Test 5B: Tracing pure spin behavior with locked wheels (reconciling Point 5):
        # Under pure spin, WheelController forces STICTION_KINETIC at 200 ms (20 cycles)
        self.wc.reset()
        self.wc.route_command(0.0, 0.500)
        # Cycles 1 to 19 (190 ms): outputs 102 PWM boost
        for _ in range(19):
            pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
            self.assertEqual(pwms, [-102, 102, -102, 102])
            self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BOOST)

        # Cycle 20 (200 ms): pure spin hard timeout forces transition to STICTION_KINETIC
        pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_KINETIC)
        # Once in STICTION_KINETIC, feedforward + PID takes over (no 500 ms STICTION_BLOCKED shutdown)
        self.assertNotEqual(pwms, [0, 0, 0, 0])

    def test_06_repeated_transitions_across_classification_boundary(self):
        # Start blocked under straight crawl (vx = 0.029, wz = 0.0)
        self.wc.route_command(0.029, 0.0)
        for _ in range(50):
            self.wc.update([0.0]*4, [0]*4, 0.01)
        # Confirm blocked
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BLOCKED)
        self.assertEqual(self.wc.controllers[0].blocked_retries_in_episode, 0)

        # Transition 1: cross boundary into opposing tight arc (vx = 0.014, wz = 0.500)
        self.wc.route_command(0.014, 0.500)
        # Should allow 1 retry
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BOOST)
        self.assertEqual(self.wc.controllers[0].blocked_retries_in_episode, 1)

        # Wheel remains physically locked: times out again after 500 ms
        for _ in range(50):
            self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BLOCKED)
        self.assertEqual(self.wc.controllers[0].blocked_retries_in_episode, 1)

        # Transitions 2 through 10: repeatedly crossing the boundary back and forth
        for i in range(10):
            if i % 2 == 0:
                self.wc.route_command(0.150, 0.100) # Non-opposing
            else:
                self.wc.route_command(0.014, 0.500) # Opposing
            pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
            # Output MUST remain strictly 0 PWM! Retries consumed!
            self.assertEqual(pwms, [0, 0, 0, 0], f"Boundary bypass occurred on iteration {i}!")
            self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BLOCKED)

    def test_07_small_noisy_commands_around_boundary(self):
        # Start in crawl, get blocked
        self.wc.route_command(0.029, 0.0)
        for _ in range(50):
            self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BLOCKED)

        # Command oscillating around boundary (|vx| ~ 0.1704 * |wz|): wz = 0.300 -> threshold vx = 0.0511
        # Re-arms once
        self.wc.route_command(0.040, 0.300) # Opposing
        for _ in range(50):
            self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BLOCKED)

        # 50 cycles of high-frequency command noise crossing the threshold
        for cycle in range(50):
            vx_noisy = 0.0511 + (0.010 if cycle % 2 == 0 else -0.010)
            self.wc.route_command(vx_noisy, 0.300)
            pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
            self.assertEqual(pwms, [0, 0, 0, 0], f"Noise bypassed protection at cycle {cycle}!")

    def test_08_one_blocked_wheel_while_others_move(self):
        # M1 (LF, index 0) physically jammed (0 ticks); M2, M3, M4 free
        self.wc.route_command(0.014, 0.500)
        
        # Advance 40 ms (4 cycles): M2, M3, M4 register ticks; M1 stays 0
        ticks = [0, 0, 0, 0]
        vels = [0.0, 0.0, 0.0, 0.0]
        for c in range(1, 5):
            ticks = [0, c*2, -c*2, c*2]
            vels = [0.0, 1.0, -1.0, 1.0]
            pwms = self.wc.update(vels, ticks, 0.01)

        # Free wheels broke out to KINETIC
        self.assertEqual(self.wc.controllers[1].stiction_state, STICTION_KINETIC)
        self.assertEqual(self.wc.controllers[2].stiction_state, STICTION_KINETIC)
        self.assertEqual(self.wc.controllers[3].stiction_state, STICTION_KINETIC)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BOOST)

        # Advance to 500 ms: M1 times out into STICTION_BLOCKED
        for _ in range(46):
            ticks[1] += 2; ticks[2] -= 2; ticks[3] += 2
            pwms = self.wc.update(vels, ticks, 0.01)

        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BLOCKED)
        self.assertEqual(pwms[0], 0, "Blocked wheel must output 0 PWM")
        self.assertNotEqual(pwms[1], 0, "Moving wheel M2 must remain powered")
        self.assertNotEqual(pwms[2], 0, "Moving wheel M3 must remain powered")
        self.assertNotEqual(pwms[3], 0, "Moving wheel M4 must remain powered")

    def test_09_arc_to_spin_transition_without_zero_command(self):
        # Wheel blocked during gentle arc (vx = 0.150, wz = 0.200)
        self.wc.route_command(0.150, 0.200)
        for _ in range(50):
            self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BLOCKED)

        # Transition directly to pure spin (vx = 0.0, wz = 0.400) without zero command
        is_spin, is_opposing, wl, wr = self.wc.route_command(0.0, 0.400)
        self.assertTrue(is_spin)
        # Grants the bounded 1 retry
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BOOST)
        pwms = self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(pwms, [-102, 102, -102, 102])

        # At 200 ms (20 cycles), pure spin hard timeout transitions to STICTION_KINETIC
        for _ in range(19):
            self.wc.update([0.0]*4, [0]*4, 0.01)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_KINETIC)

if __name__ == '__main__':
    unittest.main()
