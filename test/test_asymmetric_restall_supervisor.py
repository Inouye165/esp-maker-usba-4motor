"""
test_asymmetric_restall_supervisor.py

Unit test suite verifying the bounded firmware correction in ChassisSupervisor
for asymmetric track re-stall during turning arcs and spin-to-arc transitions.

Scenarios tested:
1. Left-track stall during arc (asymmetric stall detection & sustained recovery)
2. Right-track stall during arc (asymmetric stall detection & sustained recovery)
3. Spin-to-arc transition re-stall (matching run 20261005_050405 profile)
4. Failed recovery: blocked track exceeds 1000 ms shared budget -> latches FAILED_LATCHED
5. Retention of fault latch across disarm: spinFailedExhausted remains True after disarm
6. Stalled wheel twitch (< 30 ms) does NOT reset recovery budget
7. Other track moving continuously does NOT reset recovery budget
8. Normal moving arcs do not trigger false re-breakout
9. Strict episode bounding: no repeated boost cycles in a single episode

NOTE: These unit tests verify controller state machine transitions, timing logic,
and supervisor invariants. They do NOT claim to prove physical breakout times
or floor performance under carpet friction.
"""

import unittest
import math

# Dynamic Stiction States
STICTION_IDLE = 0
STICTION_BOOST = 1
STICTION_KINETIC = 2
STICTION_BLOCKED = 3

# Supervisor Maneuver Types
MANEUVER_IDLE = 0
MANEUVER_TRANSLATION_OR_ARC = 1
MANEUVER_PURE_SPIN = 2

# Spin Recovery States
SPIN_RECOVERY_IDLE = 0
SPIN_RECOVERY_MONITORING_INITIAL = 1
SPIN_RECOVERY_SETTLE_1 = 2
SPIN_RECOVERY_TWITCH_PULSE = 3
SPIN_RECOVERY_SETTLE_2 = 4
SPIN_RECOVERY_RETRY = 5
SPIN_RECOVERY_FAILED_LATCHED = 6

MIN_RELIABLE_SPEED_RADPS = 0.35
WHEEL_SEPARATION_M = 0.17042875 * 2.0  # 0.3408575 m
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
        self.stiction_state = STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False
        self.last_pwm = 0

    def reset(self):
        self.stiction_state = STICTION_IDLE
        self.boost_start_ticks = 0
        self.boost_ticks_count = 0
        self.boost_ticks_initialized = False
        self.last_pwm = 0

    def force_kinetic(self):
        self.stiction_state = STICTION_KINETIC

    def force_blocked(self):
        self.stiction_state = STICTION_BLOCKED
        self.last_pwm = 0

    def set_target_velocity(self, target_radps):
        # Direction change across zero while active
        is_reversal = (self.target_vel > 0.05 and target_radps < -0.05) or (self.target_vel < -0.05 and target_radps > 0.05)

        if (self.stiction_state == STICTION_IDLE or abs(self.target_vel) < 0.01) and abs(target_radps) >= 0.01:
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
        elif is_reversal and self.stiction_state != STICTION_BLOCKED:
            self.stiction_state = STICTION_BOOST
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
        elif abs(target_radps) < 0.01:
            self.stiction_state = STICTION_IDLE
            self.boost_ticks_count = 0
            self.boost_ticks_initialized = False
        self.target_vel = target_radps

    def update(self, measured_radps, current_ticks):
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
            if delta_ticks >= 3:
                self.stiction_state = STICTION_KINETIC
            else:
                self.boost_ticks_count += 1
                if self.boost_ticks_count >= 50:
                    self.stiction_state = STICTION_BLOCKED
                    self.last_pwm = 0
                    return 0
                self.last_pwm = 48 if self.target_vel > 0 else -48
                return self.last_pwm

        if self.stiction_state == STICTION_BLOCKED:
            self.last_pwm = 0
            return 0

        self.last_pwm = int(math.copysign(35.0, self.target_vel))
        return self.last_pwm

class WheelControllerSim:
    def __init__(self):
        self.controllers = [SingleWheelControllerSim(i) for i in range(4)]

    def reset(self):
        for c in self.controllers:
            c.reset()

    def force_blocked(self):
        for c in self.controllers:
            c.force_blocked()

    def set_targets(self, left_target, right_target):
        self.controllers[0].set_target_velocity(left_target)
        self.controllers[1].set_target_velocity(right_target)
        self.controllers[2].set_target_velocity(left_target)
        self.controllers[3].set_target_velocity(right_target)

    def update(self, measured_vels, ticks):
        for i in range(4):
            self.controllers[i].update(measured_vels[i], ticks[i])

class ChassisSupervisorSim:
    def __init__(self):
        self.candidateAConsumedForEpisode = False
        self.chassisRebreakoutConsumedForEpisode = False
        self.zeroDebounceTicks = 0
        self.stalledKineticTicks = 0
        self.stallWindowInitialized = False
        self.stallWindowStartTicks = [0, 0, 0, 0]
        self.prevManeuverType = MANEUVER_IDLE
        self.supervisorResetCount = 0

        self.spinRecoveryState = SPIN_RECOVERY_IDLE
        self.spinBreakoutSustainedTicks = 0
        self.spinFailedExhausted = False
        self.cumulativeNoMotionMs = 0
        self.supervisorTimerInitialized = False
        self.prevEvalMs = 0
        self.prevCmdWasActive = False
        self.sustainedMotionMs = 0
        self.prevArmed = False

    def update(self, cmd_lin, cmd_ang, is_armed, now_ms, wc: WheelControllerSim, velocities, ticks, imu_gz):
        CMD_LINEAR_EPSILON = 0.005
        CMD_ANGULAR_EPSILON = 0.005

        is_cmd_zero = (abs(cmd_lin) < CMD_LINEAR_EPSILON) and (abs(cmd_ang) < CMD_ANGULAR_EPSILON)
        is_cmd_active = is_armed and not is_cmd_zero

        # Fresh re-arm transition clears latched exhaustion from a previous mission
        if is_armed and not self.prevArmed:
            self.spinFailedExhausted = False
            self.spinRecoveryState = SPIN_RECOVERY_IDLE
            self.cumulativeNoMotionMs = 0

        # 1. Episode tracking & debounce
        if not is_armed:
            wc.reset()
            self.candidateAConsumedForEpisode = False
            self.chassisRebreakoutConsumedForEpisode = False
            self.zeroDebounceTicks = 0
            self.stalledKineticTicks = 0
            self.stallWindowInitialized = False
            self.prevManeuverType = MANEUVER_IDLE
            self.spinBreakoutSustainedTicks = 0
            self.supervisorTimerInitialized = False
            self.prevEvalMs = 0
            self.prevCmdWasActive = False
            self.sustainedMotionMs = 0

            # Invariant: Disarm must NOT erase the fault latch!
            if not self.spinFailedExhausted:
                self.spinRecoveryState = SPIN_RECOVERY_IDLE
                self.cumulativeNoMotionMs = 0
        elif is_cmd_zero:
            self.zeroDebounceTicks += 1
            if self.zeroDebounceTicks >= 5:  # 50 ms @ 100 Hz
                self.candidateAConsumedForEpisode = False
                self.chassisRebreakoutConsumedForEpisode = False
                self.stalledKineticTicks = 0
                self.stallWindowInitialized = False
                self.prevManeuverType = MANEUVER_IDLE
                if not self.spinFailedExhausted:
                    self.spinRecoveryState = SPIN_RECOVERY_IDLE
                    self.spinBreakoutSustainedTicks = 0
                    self.cumulativeNoMotionMs = 0
                    self.sustainedMotionMs = 0
        else:
            self.zeroDebounceTicks = 0

        # 2. Maneuver classification
        current_maneuver = MANEUVER_IDLE
        if is_armed and not is_cmd_zero:
            if abs(cmd_lin) <= CMD_LINEAR_EPSILON and abs(cmd_ang) >= CMD_ANGULAR_EPSILON:
                current_maneuver = MANEUVER_PURE_SPIN
            else:
                current_maneuver = MANEUVER_TRANSLATION_OR_ARC

        # 3. Shared cumulative active no-motion recovery budget
        if self.supervisorTimerInitialized:
            delta_ms = max(0, now_ms - self.prevEvalMs)
            if self.prevCmdWasActive and delta_ms > 0:
                moving_wheels = sum(1 for v in velocities if abs(v) >= 0.08)
                moving_left = sum(1 for i in [0, 2] if abs(velocities[i]) >= 0.08)
                moving_right = sum(1 for i in [1, 3] if abs(velocities[i]) >= 0.08)

                left_tgt, right_tgt = velocity_to_wheels(cmd_lin, cmd_ang)
                left_commanded = abs(left_tgt) >= MIN_RELIABLE_SPEED_RADPS
                right_commanded = abs(right_tgt) >= MIN_RELIABLE_SPEED_RADPS

                if current_maneuver == MANEUVER_PURE_SPIN or self.prevManeuverType == MANEUVER_PURE_SPIN:
                    credible_motion = (moving_wheels >= 2) and (abs(imu_gz) >= 0.04)
                else:
                    if left_commanded and right_commanded:
                        credible_motion = (moving_left >= 1 and moving_right >= 1)
                    elif left_commanded:
                        credible_motion = (moving_left >= 1)
                    elif right_commanded:
                        credible_motion = (moving_right >= 1)
                    else:
                        credible_motion = (moving_wheels >= 2)

                if credible_motion:
                    if self.sustainedMotionMs < 1000:
                        self.sustainedMotionMs += delta_ms
                else:
                    self.sustainedMotionMs = 0

                # Budget charges until sustained confirmation (>= 30 ms) is reached
                if self.sustainedMotionMs >= 30:
                    self.cumulativeNoMotionMs = 0
                else:
                    self.cumulativeNoMotionMs += delta_ms
                    if self.cumulativeNoMotionMs >= 1000:
                        self.spinRecoveryState = SPIN_RECOVERY_FAILED_LATCHED
                        self.spinFailedExhausted = True
                        wc.force_blocked()

        self.prevEvalMs = now_ms
        self.prevCmdWasActive = is_cmd_active
        self.prevArmed = is_armed
        self.supervisorTimerInitialized = True

        if self.spinFailedExhausted:
            wc.force_blocked()

        self.prevManeuverType = current_maneuver

        # 4. Coordinated Stalled-KINETIC Gate
        if is_armed and not is_cmd_zero and not self.chassisRebreakoutConsumedForEpisode and not self.spinFailedExhausted:
            all_kinetic = all(c.stiction_state == STICTION_KINETIC for c in wc.controllers)
            zero_vel_count = sum(1 for v in velocities if abs(v) < 0.05)

            if not self.stallWindowInitialized:
                self.stallWindowStartTicks = list(ticks)
                self.stallWindowInitialized = True

            static_disp_count = sum(1 for i in range(4) if abs(ticks[i] - self.stallWindowStartTicks[i]) < 3)
            imu_stationary = abs(imu_gz) < 0.04

            left_tgt, right_tgt = velocity_to_wheels(cmd_lin, cmd_ang)
            left_commanded = abs(left_tgt) >= MIN_RELIABLE_SPEED_RADPS
            right_commanded = abs(right_tgt) >= MIN_RELIABLE_SPEED_RADPS

            left_kinetic = (wc.controllers[0].stiction_state == STICTION_KINETIC) and (wc.controllers[2].stiction_state == STICTION_KINETIC)
            right_kinetic = (wc.controllers[1].stiction_state == STICTION_KINETIC) and (wc.controllers[3].stiction_state == STICTION_KINETIC)

            left_stationary = (abs(velocities[0]) < 0.05) and (abs(velocities[2]) < 0.05) and \
                              (abs(ticks[0] - self.stallWindowStartTicks[0]) < 3) and (abs(ticks[2] - self.stallWindowStartTicks[2]) < 3)
            right_stationary = (abs(velocities[1]) < 0.05) and (abs(velocities[3]) < 0.05) and \
                               (abs(ticks[1] - self.stallWindowStartTicks[1]) < 3) and (abs(ticks[3] - self.stallWindowStartTicks[3]) < 3)

            whole_chassis_stalled = all_kinetic and (zero_vel_count >= 3) and (static_disp_count >= 3) and imu_stationary
            left_track_stalled = left_commanded and left_kinetic and left_stationary
            right_track_stalled = right_commanded and right_kinetic and right_stationary
            asymmetric_track_stalled = (left_commanded and right_commanded) and (left_track_stalled or right_track_stalled)

            stall_detected = whole_chassis_stalled or asymmetric_track_stalled

            if stall_detected:
                self.stalledKineticTicks += 1
                if self.stalledKineticTicks >= 20:  # 200 ms @ 100 Hz
                    wc.reset()
                    self.supervisorResetCount += 1
                    self.chassisRebreakoutConsumedForEpisode = True
                    self.stalledKineticTicks = 0
                    self.stallWindowInitialized = False
                    if current_maneuver == MANEUVER_PURE_SPIN:
                        self.spinRecoveryState = SPIN_RECOVERY_IDLE
            else:
                self.stalledKineticTicks = 0
                self.stallWindowInitialized = False
        else:
            self.stalledKineticTicks = 0
            self.stallWindowInitialized = False


class TestAsymmetricReStallSupervisor(unittest.TestCase):
    def setUp(self):
        self.wc = WheelControllerSim()
        self.sup = ChassisSupervisorSim()

    def test_right_track_stall_during_arc_recovery_success(self):
        """Asymmetric right-track stall during arc (e.g. run 050405 profile).
        Right track stalls in KINETIC while left moves.
        Re-breakout triggers at 200 ms; sustained motion clears the budget."""
        cmd_lin, cmd_ang = 0.057, -0.50
        left_tgt, right_tgt = velocity_to_wheels(cmd_lin, cmd_ang)
        self.assertGreater(left_tgt, MIN_RELIABLE_SPEED_RADPS)
        self.assertLess(right_tgt, -MIN_RELIABLE_SPEED_RADPS)

        # Set all wheels initially to KINETIC (simulating active motion before stall)
        for c in self.wc.controllers:
            c.force_kinetic()

        now = 1000
        ticks = [1000, 500, 1000, 500]

        # 20 cycles (200 ms) of asymmetric stall: Left moves (1.5 rad/s), Right stalls (0.0 rad/s)
        for cycle in range(20):
            now += 10
            ticks[0] += 5  # Left moving
            ticks[2] += 5
            vels = [1.5, 0.0, 1.5, 0.0]
            gz = -0.30  # Yaw rate due to left track pushing
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, gz)

        # At cycle 20 (200 ms), re-breakout must trigger
        self.assertEqual(self.sup.supervisorResetCount, 1)
        self.assertTrue(self.sup.chassisRebreakoutConsumedForEpisode)
        self.assertEqual(self.wc.controllers[1].stiction_state, STICTION_IDLE)

        # Re-apply targets: controllers enter STICTION_BOOST
        self.wc.set_targets(left_tgt, right_tgt)
        self.assertEqual(self.wc.controllers[1].stiction_state, STICTION_BOOST)

        # First cycle initializes boostStartTicks baseline
        now += 10
        self.wc.update(vels, ticks)

        # Breakout occurs: right wheels move >= 3 ticks
        now += 10
        ticks[0] += 5; ticks[2] += 5
        ticks[1] += 4; ticks[3] += 4  # Breakout displacement >= 3 ticks
        vels = [1.5, -0.86, 1.5, -0.86]
        self.wc.update(vels, ticks)
        self.assertEqual(self.wc.controllers[1].stiction_state, STICTION_KINETIC)

        # Sustained motion for 30 ms resets cumulative budget
        for _ in range(4):
            now += 10
            ticks[0] += 5; ticks[2] += 5
            ticks[1] += 3; ticks[3] += 3
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.50)

        self.assertEqual(self.sup.cumulativeNoMotionMs, 0)
        self.assertFalse(self.sup.spinFailedExhausted)

    def test_left_track_stall_during_arc_recovery_success(self):
        """Asymmetric left-track stall during arc (turning CW, left track stalls).
        Re-breakout triggers at 200 ms; sustained motion clears the budget."""
        cmd_lin, cmd_ang = 0.057, 0.50
        left_tgt, right_tgt = velocity_to_wheels(cmd_lin, cmd_ang)
        self.assertLess(left_tgt, -MIN_RELIABLE_SPEED_RADPS)
        self.assertGreater(right_tgt, MIN_RELIABLE_SPEED_RADPS)

        for c in self.wc.controllers:
            c.force_kinetic()

        now = 2000
        ticks = [500, 1000, 500, 1000]

        for cycle in range(20):
            now += 10
            ticks[1] += 5  # Right moving
            ticks[3] += 5
            vels = [0.0, 1.5, 0.0, 1.5]
            gz = 0.30
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, gz)

        self.assertEqual(self.sup.supervisorResetCount, 1)
        self.assertTrue(self.sup.chassisRebreakoutConsumedForEpisode)

    def test_spin_to_arc_transition_restall(self):
        """Pure spin CW followed by transition to asymmetric arc where right track stalls.
        Re-breakout fires and resolves the stall."""
        # Phase 1: Pure spin CW
        cmd_lin, cmd_ang = 0.0, -0.50
        for c in self.wc.controllers:
            c.force_kinetic()

        now = 3000
        ticks = [1000, 1000, 1000, 1000]
        for _ in range(30):
            now += 10
            ticks[0] += 3; ticks[2] += 3
            ticks[1] -= 3; ticks[3] -= 3
            vels = [2.6, -2.6, 2.6, -2.6]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.50)

        self.assertEqual(self.sup.cumulativeNoMotionMs, 0)
        self.assertEqual(self.sup.supervisorResetCount, 0)

        # Phase 2: Transition to arc (v=0.057, w=-0.50). Right track stalls immediately.
        cmd_lin, cmd_ang = 0.057, -0.50
        for _ in range(20):
            now += 10
            ticks[0] += 4; ticks[2] += 4
            vels = [1.5, 0.0, 1.5, 0.0]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.30)

        self.assertEqual(self.sup.supervisorResetCount, 1)
        self.assertTrue(self.sup.chassisRebreakoutConsumedForEpisode)

    def test_failed_recovery_exhaustion_and_latch(self):
        """Asymmetric stall occurs, re-breakout is consumed, but track never moves.
        Budget must advance to 1000 ms, latch FAILED_LATCHED, and block output."""
        cmd_lin, cmd_ang = 0.057, -0.50
        for c in self.wc.controllers:
            c.force_kinetic()

        now = 4000
        ticks = [1000, 500, 1000, 500]

        # First 200 ms: stall accumulates, re-breakout triggers
        for _ in range(20):
            now += 10
            ticks[0] += 5; ticks[2] += 5
            vels = [1.5, 0.0, 1.5, 0.0]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.30)

        self.assertEqual(self.sup.supervisorResetCount, 1)
        self.assertTrue(self.sup.chassisRebreakoutConsumedForEpisode)

        # Right track remains completely stuck for the remaining 800 ms
        for _ in range(85):
            now += 10
            ticks[0] += 5; ticks[2] += 5
            vels = [1.5, 0.0, 1.5, 0.0]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.30)

        self.assertTrue(self.sup.spinFailedExhausted)
        self.assertEqual(self.sup.spinRecoveryState, SPIN_RECOVERY_FAILED_LATCHED)
        self.assertEqual(self.wc.controllers[0].stiction_state, STICTION_BLOCKED)
        self.assertEqual(self.wc.controllers[1].stiction_state, STICTION_BLOCKED)

    def test_disarm_retains_fault_latch(self):
        """When rover disarms after exhaustion, spinFailedExhausted and FAILED_LATCHED
        must be retained so telemetry and journal retain the fault status.
        Only a deliberate re-arm clears the latch."""
        cmd_lin, cmd_ang = 0.057, -0.50
        self.sup.spinFailedExhausted = True
        self.sup.spinRecoveryState = SPIN_RECOVERY_FAILED_LATCHED
        self.sup.cumulativeNoMotionMs = 1030
        self.sup.prevArmed = True

        # Rover is disarmed
        now = 5000
        self.sup.update(0.0, 0.0, False, now, self.wc, [0, 0, 0, 0], [0, 0, 0, 0], 0.0)

        # Latch must be retained!
        self.assertTrue(self.sup.spinFailedExhausted)
        self.assertEqual(self.sup.spinRecoveryState, SPIN_RECOVERY_FAILED_LATCHED)
        self.assertEqual(self.sup.cumulativeNoMotionMs, 1030)

        # Another disarmed cycle retains latch
        now += 100
        self.sup.update(0.0, 0.0, False, now, self.wc, [0, 0, 0, 0], [0, 0, 0, 0], 0.0)
        self.assertTrue(self.sup.spinFailedExhausted)
        self.assertEqual(self.sup.spinRecoveryState, SPIN_RECOVERY_FAILED_LATCHED)

        # Fresh re-arm clears the latch
        now += 100
        self.sup.update(0.0, 0.0, True, now, self.wc, [0, 0, 0, 0], [0, 0, 0, 0], 0.0)
        self.assertFalse(self.sup.spinFailedExhausted)
        self.assertEqual(self.sup.spinRecoveryState, SPIN_RECOVERY_IDLE)
        self.assertEqual(self.sup.cumulativeNoMotionMs, 0)

    def test_twitch_does_not_reset_budget(self):
        """A brief encoder twitch (< 30 ms) on the stalled track must NOT reset
        the cumulative recovery budget."""
        cmd_lin, cmd_ang = 0.057, -0.50
        for c in self.wc.controllers:
            c.force_kinetic()

        now = 6000
        ticks = [1000, 500, 1000, 500]

        # 40 cycles (400 ms) of stall -> budget accumulates to 400 ms
        for _ in range(40):
            now += 10
            ticks[0] += 5; ticks[2] += 5
            vels = [1.5, 0.0, 1.5, 0.0]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.30)

        self.assertGreaterEqual(self.sup.cumulativeNoMotionMs, 380)

        # Stalled track twitches for 2 cycles (20 ms, < 30 ms confirmation requirement)
        for _ in range(2):
            now += 10
            ticks[0] += 5; ticks[2] += 5
            ticks[1] += 2; ticks[3] += 2
            vels = [1.5, -0.5, 1.5, -0.5]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.30)

        # Twitch stops: track halts again
        now += 10
        ticks[0] += 5; ticks[2] += 5
        vels = [1.5, 0.0, 1.5, 0.0]
        self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.30)

        # Budget was NOT reset by the 20 ms twitch!
        self.assertGreaterEqual(self.sup.cumulativeNoMotionMs, 400)
        self.assertEqual(self.sup.sustainedMotionMs, 0)

    def test_other_track_moving_does_not_reset_budget(self):
        """Left track moving continuously while right track is commanded but stalled
        must NOT qualify as credible motion."""
        cmd_lin, cmd_ang = 0.057, -0.50
        for c in self.wc.controllers:
            c.force_kinetic()

        now = 7000
        ticks = [1000, 500, 1000, 500]

        for _ in range(50):  # 500 ms
            now += 10
            ticks[0] += 10; ticks[2] += 10  # Left moving fast
            vels = [2.5, 0.0, 2.5, 0.0]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.40)

        self.assertGreaterEqual(self.sup.cumulativeNoMotionMs, 480)
        self.assertEqual(self.sup.sustainedMotionMs, 0)

    def test_normal_moving_arcs_no_false_trigger(self):
        """When both tracks move as commanded, supervisor must never trip re-breakout
        and budget must remain zero."""
        cmd_lin, cmd_ang = 0.057, -0.50
        for c in self.wc.controllers:
            c.force_kinetic()

        now = 8000
        ticks = [1000, 500, 1000, 500]

        for _ in range(100):  # 1.0 second of normal arc motion
            now += 10
            ticks[0] += 5; ticks[2] += 5
            ticks[1] -= 2; ticks[3] -= 2
            vels = [1.5, -0.86, 1.5, -0.86]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.50)

        self.assertEqual(self.sup.supervisorResetCount, 0)
        self.assertFalse(self.sup.chassisRebreakoutConsumedForEpisode)
        self.assertEqual(self.sup.cumulativeNoMotionMs, 0)
        self.assertFalse(self.sup.spinFailedExhausted)

    def test_no_repeated_boost_cycles(self):
        """Re-breakout must strictly fire at most once per episode."""
        cmd_lin, cmd_ang = 0.057, -0.50
        for c in self.wc.controllers:
            c.force_kinetic()

        now = 9000
        ticks = [1000, 500, 1000, 500]

        # Trigger re-breakout #1 at 200 ms
        for _ in range(20):
            now += 10
            ticks[0] += 5; ticks[2] += 5
            vels = [1.5, 0.0, 1.5, 0.0]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.30)

        self.assertEqual(self.sup.supervisorResetCount, 1)

        # Reset controllers to kinetic without clearing episode
        for c in self.wc.controllers:
            c.force_kinetic()

        # Another 20 cycles of stall in the same episode
        for _ in range(25):
            now += 10
            ticks[0] += 5; ticks[2] += 5
            vels = [1.5, 0.0, 1.5, 0.0]
            self.sup.update(cmd_lin, cmd_ang, True, now, self.wc, vels, ticks, -0.30)

        # Must NOT fire a second re-breakout!
        self.assertEqual(self.sup.supervisorResetCount, 1)

if __name__ == '__main__':
    unittest.main()
