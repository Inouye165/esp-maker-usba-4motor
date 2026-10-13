#pragma once
#include <Arduino.h>
#include "CommandManager.h"
#include "RoverConfig.h"
#include "DifferentialDrive.h"
#ifdef TEST_BASELINE
#include "WheelController_baseline.h"
#else
#include "WheelController.h"
#endif
#include <cmath>
#include <algorithm>

enum ChassisManeuverType {
    MANEUVER_IDLE = 0,
    MANEUVER_TRANSLATION_OR_ARC = 1,
    MANEUVER_PURE_SPIN = 2
};

enum SpinRecoveryState {
    SPIN_RECOVERY_IDLE = 0,
    SPIN_RECOVERY_MONITORING_INITIAL,
    SPIN_RECOVERY_SETTLE_1,
    SPIN_RECOVERY_TWITCH_PULSE,
    SPIN_RECOVERY_SETTLE_2,
    SPIN_RECOVERY_RETRY,
    SPIN_RECOVERY_FAILED_LATCHED
};

struct ChassisSupervisorState {
    bool candidateAConsumedForEpisode = false;
    bool chassisRebreakoutConsumedForEpisode = false;
    uint16_t zeroDebounceTicks = 0;
    uint16_t stalledKineticTicks = 0;
    bool stallWindowInitialized = false;
    int32_t stallWindowStartTicks[4] = {0, 0, 0, 0};
    ChassisManeuverType prevManeuverType = MANEUVER_IDLE;
    int supervisorResetCount = 0;

    // Authoritative Coordinated Spin-Recovery State
    SpinRecoveryState spinRecoveryState = SPIN_RECOVERY_IDLE;
    uint32_t spinStartTimeMs = 0;
    uint32_t spinStateEnteredMs = 0;
    uint32_t retryStartTimeMs = 0;
    int32_t spinStartTicks[4] = {0, 0, 0, 0};
    int32_t twitchStartTicks[4] = {0, 0, 0, 0};
    float requestedAngular = 0.0f;
    uint8_t spinBreakoutSustainedTicks = 0;
    bool spinFailedExhausted = false;
    uint32_t cumulativeNoMotionMs = 0;
    bool supervisorTimerInitialized = false;
    uint32_t prevEvalMs = 0;
    bool prevCmdWasActive = false;
    uint16_t sustainedMotionMs = 0;
    bool prevArmed = false;

    void reset() {
        candidateAConsumedForEpisode = false;
        chassisRebreakoutConsumedForEpisode = false;
        zeroDebounceTicks = 0;
        stalledKineticTicks = 0;
        stallWindowInitialized = false;
        for (int i = 0; i < 4; i++) {
            stallWindowStartTicks[i] = 0;
            spinStartTicks[i] = 0;
            twitchStartTicks[i] = 0;
        }
        prevManeuverType = MANEUVER_IDLE;
        supervisorResetCount = 0;
        spinRecoveryState = SPIN_RECOVERY_IDLE;
        spinStartTimeMs = 0;
        spinStateEnteredMs = 0;
        retryStartTimeMs = 0;
        requestedAngular = 0.0f;
        spinBreakoutSustainedTicks = 0;
        spinFailedExhausted = false;
        cumulativeNoMotionMs = 0;
        supervisorTimerInitialized = false;
        prevEvalMs = 0;
        prevCmdWasActive = false;
        sustainedMotionMs = 0;
        prevArmed = false;
    }
};

extern ChassisSupervisorState g_chassisSupervisorState;

// Production Chassis Stall Supervisor algorithm
template<typename TWc, typename TEnc, typename TImu>
inline void updateChassisStallSupervisorWithState(
    ChassisSupervisorState &state,
    const ChassisCommand &activeCmd,
    bool isArmed,
    uint32_t nowMs,
    TWc &wc,
    const TEnc &enc,
    const TImu &imu
) {
    const float CMD_LINEAR_EPSILON = 0.005f;
    const float CMD_ANGULAR_EPSILON = 0.005f;

    bool isCmdZero = (std::fabs(activeCmd.linearVelocity) < CMD_LINEAR_EPSILON) && 
                     (std::fabs(activeCmd.angularVelocity) < CMD_ANGULAR_EPSILON);
    bool isCmdActive = (isArmed && !isCmdZero);

    // 1. Episode tracking & debounce
    if (isArmed && !state.prevArmed) {
        // Deliberate re-arm transition: clear latched exhaustion from a previous mission
        state.spinFailedExhausted = false;
        state.spinRecoveryState = SPIN_RECOVERY_IDLE;
        state.cumulativeNoMotionMs = 0;
        wc.reset();
    }

    if (!isArmed) {
        // Disarm resets both episode latches, recovery state, and wheel controllers immediately (0 ms)
        wc.reset();
        state.candidateAConsumedForEpisode = false;
        state.chassisRebreakoutConsumedForEpisode = false;
        state.zeroDebounceTicks = 0;
        state.stalledKineticTicks = 0;
        state.stallWindowInitialized = false;
        state.prevManeuverType = MANEUVER_IDLE;
        state.spinBreakoutSustainedTicks = 0;
        state.retryStartTimeMs = 0;
        state.supervisorTimerInitialized = false;
        state.prevEvalMs = 0;
        state.prevCmdWasActive = false;
        state.sustainedMotionMs = 0;

        // Invariant: If recovery is exhausted, disarm must NOT erase the fault latch!
        // Telemetry must continue reporting spinFailedExhausted and FAILED_LATCHED while disarmed.
        // The latch is only cleared on a fresh arming transition (!prevArmed && isArmed).
        if (!state.spinFailedExhausted) {
            state.spinRecoveryState = SPIN_RECOVERY_IDLE;
            state.cumulativeNoMotionMs = 0;
        }
    } else if (isCmdZero) {
        state.zeroDebounceTicks++;
        if (state.zeroDebounceTicks >= 5) { // 50 ms @ 100 Hz
            state.candidateAConsumedForEpisode = false;
            state.chassisRebreakoutConsumedForEpisode = false;
            state.stalledKineticTicks = 0;
            state.stallWindowInitialized = false;
            state.prevManeuverType = MANEUVER_IDLE;
            // Invariant: If recovery is exhausted, near-zero commands while armed MUST NOT clear the latch!
            // Only deliberate disarm (!isArmed) clears spinFailedExhausted and resets cumulativeNoMotionMs.
            if (!state.spinFailedExhausted) {
                state.spinRecoveryState = SPIN_RECOVERY_IDLE;
                state.spinBreakoutSustainedTicks = 0;
                state.retryStartTimeMs = 0;
                state.cumulativeNoMotionMs = 0;
                state.sustainedMotionMs = 0;
            }
        }
    } else {
        state.zeroDebounceTicks = 0;
    }

    // 2. Maneuver classification
    ChassisManeuverType currentManeuver = MANEUVER_IDLE;
    if (isArmed && !isCmdZero) {
        if (std::fabs(activeCmd.linearVelocity) <= CMD_LINEAR_EPSILON && std::fabs(activeCmd.angularVelocity) >= CMD_ANGULAR_EPSILON) {
            currentManeuver = MANEUVER_PURE_SPIN;
        } else {
            currentManeuver = MANEUVER_TRANSLATION_OR_ARC;
        }
    }

    // 3. Shared cumulative active no-motion recovery budget:
    // Uses interval-based accounting: the interval [prevEvalMs, nowMs] that just elapsed
    // is charged according to whether the command was active during that interval.
    // Brief zeros preserve accumulated active effort without erasing it.
    if (state.supervisorTimerInitialized) {
        uint32_t deltaMs = 0;
        if (nowMs >= state.prevEvalMs) {
            deltaMs = nowMs - state.prevEvalMs;
        }

        if (state.prevCmdWasActive && deltaMs > 0) {
            int movingWheels = 0;
            for (int i = 0; i < 4; i++) {
                if (std::fabs(enc.getVelocity(i)) >= 0.08f) {
                    movingWheels++;
                }
            }
            float gz = imu.getData().gz;

            // Sustained, credible motion evidence:
            // Requires wheel rotation corroborated by maneuver intent:
            // 1. Pure spins: >= 2 moving wheels corroborated by gyro yaw rate (|gz| >= 0.04 rad/s).
            // 2. Translation / Turning Arc: Both tracks are evaluated against commanded intent.
            //    If both tracks are commanded (|target| >= MIN_RELIABLE_SPEED_RADPS), credible motion requires
            //    active motion on BOTH tracks (movingLeft >= 1 && movingRight >= 1). A single-track stall
            //    (e.g., left wheels spinning while right wheels stall) does NOT qualify as credible motion,
            //    preventing commanded-side stalls from resetting the no-motion budget!
            int movingLeft = 0;
            int movingRight = 0;
            for (int i = 0; i < 4; i++) {
                if (std::fabs(enc.getVelocity(i)) >= 0.08f) {
                    if (i == 0 || i == 2) {
                        movingLeft++;
                    } else {
                        movingRight++;
                    }
                }
            }

            bool credibleMotion = false;
            if (currentManeuver == MANEUVER_PURE_SPIN || state.prevManeuverType == MANEUVER_PURE_SPIN) {
                credibleMotion = (movingWheels >= 2) && (std::fabs(gz) >= 0.04f);
            } else {
                float leftTarget = 0.0f, rightTarget = 0.0f;
                DifferentialDrive::velocityToWheels(activeCmd.linearVelocity, activeCmd.angularVelocity, leftTarget, rightTarget);
                bool leftCommanded = (std::fabs(leftTarget) >= MIN_RELIABLE_SPEED_RADPS);
                bool rightCommanded = (std::fabs(rightTarget) >= MIN_RELIABLE_SPEED_RADPS);

                if (leftCommanded && rightCommanded) {
                    credibleMotion = (movingLeft >= 1 && movingRight >= 1);
                } else if (leftCommanded) {
                    credibleMotion = (movingLeft >= 1);
                } else if (rightCommanded) {
                    credibleMotion = (movingRight >= 1);
                } else {
                    credibleMotion = (movingWheels >= 2);
                }
            }

            if (credibleMotion) {
                if (state.sustainedMotionMs < 1000) {
                    state.sustainedMotionMs += deltaMs;
                }
            } else {
                state.sustainedMotionMs = 0;
            }

            // Continue charging the budget until motion satisfies the sustained confirmation requirement (>= 30 ms);
            // then reset it.
            if (state.sustainedMotionMs >= 30) {
                state.cumulativeNoMotionMs = 0;
            } else {
                state.cumulativeNoMotionMs += deltaMs;
                if (state.cumulativeNoMotionMs >= 1000) {
                    state.spinRecoveryState = SPIN_RECOVERY_FAILED_LATCHED;
                    state.spinFailedExhausted = true;
                    wc.forceBlocked();
                }
            }
        }
    }

    state.prevEvalMs = nowMs;
    state.prevCmdWasActive = isCmdActive;
    state.prevArmed = isArmed;
    state.supervisorTimerInitialized = true;

    if (state.spinFailedExhausted) {
        wc.forceBlocked();
    }

    // 3. Stalled Arc -> Pure Spin Transition
    if (isArmed && currentManeuver == MANEUVER_PURE_SPIN && state.prevManeuverType == MANEUVER_TRANSLATION_OR_ARC) {
        int stationaryWheels = 0;
        for (int i = 0; i < 4; i++) {
            if (std::fabs(enc.getVelocity(i)) < 0.05f) {
                stationaryWheels++;
            }
        }
        float gz = imu.getData().gz;
        bool isChassisStationary = (stationaryWheels >= 3) && (std::fabs(gz) < 0.04f);

        if (isChassisStationary && !state.chassisRebreakoutConsumedForEpisode && !state.spinFailedExhausted) {
            wc.reset();
            state.supervisorResetCount++;
            state.chassisRebreakoutConsumedForEpisode = true;
            state.stalledKineticTicks = 0;
            state.stallWindowInitialized = false;

            // Reset recovery harness to monitor fresh pure spin
            state.spinRecoveryState = SPIN_RECOVERY_IDLE;
            state.candidateAConsumedForEpisode = false;
#ifdef LOG_SERIAL_PRINTLN
            LOG_SERIAL_PRINTLN("[ChassisSupervisor] Stalled Arc -> Pure Spin: Synchronous 102-PWM reset & re-arm.");
#endif
        }
    }
    state.prevManeuverType = currentManeuver;

    // 4. Coordinated Stalled-KINETIC Gate
    if (isArmed && !isCmdZero && !state.chassisRebreakoutConsumedForEpisode && !state.spinFailedExhausted) {
        bool allKinetic = true;
        for (int i = 0; i < 4; i++) {
            if (wc.getController(i).getStictionState() != STICTION_KINETIC) {
                allKinetic = false;
                break;
            }
        }

        int zeroVelCount = 0;
        for (int i = 0; i < 4; i++) {
            if (std::fabs(enc.getVelocity(i)) < 0.05f) {
                zeroVelCount++;
            }
        }

        if (!state.stallWindowInitialized) {
            for (int i = 0; i < 4; i++) {
                state.stallWindowStartTicks[i] = enc.getTicks(i);
            }
            state.stallWindowInitialized = true;
        }

        int staticDispCount = 0;
        for (int i = 0; i < 4; i++) {
            if (std::abs(enc.getTicks(i) - state.stallWindowStartTicks[i]) < 3) {
                staticDispCount++;
            }
        }

        float gz = imu.getData().gz;
        bool imuStationary = (std::fabs(gz) < 0.04f);

        // Track-level commanded state and kinetic/stationary detection:
        // Canonical indices: Left = M1 (0), M3 (2); Right = M2 (1), M4 (3)
        float leftTarget = 0.0f, rightTarget = 0.0f;
        DifferentialDrive::velocityToWheels(activeCmd.linearVelocity, activeCmd.angularVelocity, leftTarget, rightTarget);
        bool leftCommanded = (std::fabs(leftTarget) >= MIN_RELIABLE_SPEED_RADPS);
        bool rightCommanded = (std::fabs(rightTarget) >= MIN_RELIABLE_SPEED_RADPS);

        bool leftKinetic = (wc.getController(0).getStictionState() == STICTION_KINETIC) &&
                           (wc.getController(2).getStictionState() == STICTION_KINETIC);
        bool rightKinetic = (wc.getController(1).getStictionState() == STICTION_KINETIC) &&
                            (wc.getController(3).getStictionState() == STICTION_KINETIC);

        bool leftStationary = (std::fabs(enc.getVelocity(0)) < 0.05f) &&
                              (std::fabs(enc.getVelocity(2)) < 0.05f) &&
                              (std::abs(enc.getTicks(0) - state.stallWindowStartTicks[0]) < 3) &&
                              (std::abs(enc.getTicks(2) - state.stallWindowStartTicks[2]) < 3);

        bool rightStationary = (std::fabs(enc.getVelocity(1)) < 0.05f) &&
                               (std::fabs(enc.getVelocity(3)) < 0.05f) &&
                               (std::abs(enc.getTicks(1) - state.stallWindowStartTicks[1]) < 3) &&
                               (std::abs(enc.getTicks(3) - state.stallWindowStartTicks[3]) < 3);

        // Condition A: Whole chassis stalled (>= 3 wheels zero velocity & static displacement, IMU stationary)
        bool wholeChassisStalled = allKinetic && (zeroVelCount >= 3) && (staticDispCount >= 3) && imuStationary;

        // Condition B: Asymmetric track stall during commanded motion:
        // When both tracks are commanded, one commanded track is stalled in KINETIC while the other track is moving
        bool leftTrackStalled = leftCommanded && leftKinetic && leftStationary && !rightStationary;
        bool rightTrackStalled = rightCommanded && rightKinetic && rightStationary && !leftStationary;
        bool asymmetricTrackStalled = (leftCommanded && rightCommanded) && (leftTrackStalled || rightTrackStalled);

        bool stallDetected = wholeChassisStalled || asymmetricTrackStalled;

        if (stallDetected) {
            state.stalledKineticTicks++;
            if (state.stalledKineticTicks >= 20) { // 200 ms @ 100 Hz
                wc.reset();
                state.supervisorResetCount++;
                state.chassisRebreakoutConsumedForEpisode = true;
                state.stalledKineticTicks = 0;
                state.stallWindowInitialized = false;

                if (currentManeuver == MANEUVER_PURE_SPIN) {
                    state.spinRecoveryState = SPIN_RECOVERY_IDLE;
                    state.candidateAConsumedForEpisode = false;
                }
#ifdef LOG_SERIAL_PRINTLN
                LOG_SERIAL_PRINTLN("[ChassisSupervisor] Stalled-KINETIC gate tripped (200ms): Synchronous re-breakout armed.");
#endif
            }
        } else {
            state.stalledKineticTicks = 0;
            state.stallWindowInitialized = false;
        }
    } else {
        state.stalledKineticTicks = 0;
        state.stallWindowInitialized = false;
    }
}

template<typename TWc, typename TEnc, typename TImu>
inline void updateChassisStallSupervisor(
    const ChassisCommand &activeCmd,
    bool isArmed,
    uint32_t nowMs,
    TWc &wc,
    const TEnc &enc,
    const TImu &imu
) {
    updateChassisStallSupervisorWithState(g_chassisSupervisorState, activeCmd, isArmed, nowMs, wc, enc, imu);
}

// Production Coordinated Spin-Recovery Routine
template<typename TWc, typename TEnc, typename TImu>
inline void updateSpinRecoveryHarnessWithState(
    ChassisSupervisorState &state,
    const ChassisCommand &activeCmd,
    float &targetLinear,
    float &targetAngular,
    uint32_t nowMs,
    float dt,
    TWc &wheelController,
    const TEnc &encoderManager,
    const TImu &imuManager
) {
    (void)dt;
    // Latched shutdown after exhaustion: continued commands must not renew recovery
    if (state.spinFailedExhausted || state.spinRecoveryState == SPIN_RECOVERY_FAILED_LATCHED) {
        state.spinRecoveryState = SPIN_RECOVERY_FAILED_LATCHED;
        state.spinFailedExhausted = true;
        targetLinear = 0.0f;
        targetAngular = 0.0f;
        wheelController.forceBlocked();
        return;
    }

    const float ZERO_EPSILON = 0.005f;
    bool isCmdZero = (std::fabs(activeCmd.linearVelocity) < ZERO_EPSILON) && 
                     (std::fabs(activeCmd.angularVelocity) < ZERO_EPSILON);
    bool isOperatorPureSpin = (std::fabs(activeCmd.linearVelocity) <= ZERO_EPSILON) && 
                              (std::fabs(activeCmd.angularVelocity) >= ZERO_EPSILON);

    if (isCmdZero) {
        targetLinear = 0.0f;
        targetAngular = 0.0f;
        if (state.zeroDebounceTicks >= 5) {
            state.spinRecoveryState = SPIN_RECOVERY_IDLE;
            state.spinBreakoutSustainedTicks = 0;
        }
        return;
    }

    if (!isOperatorPureSpin) {
        state.spinRecoveryState = SPIN_RECOVERY_IDLE;
        state.spinBreakoutSustainedTicks = 0;
        return;
    }

    // Direction change detection: if active command changes sign relative to requestedAngular during active recovery,
    // abort old recovery attempt immediately and respect current navigation intent without resetting episode budget.
    if (state.spinRecoveryState != SPIN_RECOVERY_IDLE) {
        if (activeCmd.angularVelocity * state.requestedAngular < -0.0001f) {
            state.spinRecoveryState = SPIN_RECOVERY_IDLE;
            // DO NOT reset candidateAConsumedForEpisode on direction change
            state.requestedAngular = activeCmd.angularVelocity;
            state.spinBreakoutSustainedTicks = 0;
            wheelController.reset();
        }
    }

    // Coordinated Pure-Spin Breakout Qualification:
    // Requires BOTH:
    // 1. Sustained IMU yaw rate in the commanded direction (|gz| >= 0.08 rad/s matching cmd sign)
    // 2. Sustained wheel movement from at least 3 of 4 wheels in the commanded direction (|vel| >= 0.10 rad/s)
    // 3. Persisting for at least 3 consecutive control cycles (30 ms @ 100 Hz)
    float cmdSign = (activeCmd.angularVelocity >= 0.0f) ? 1.0f : -1.0f;
    float gz = imuManager.getData().gz;
    bool imuRotating = (gz * cmdSign >= 0.08f);

    int movingWheels = 0;
    for (int i = 0; i < 4; i++) {
        float v = encoderManager.getVelocity(i);
        // Left wheels (0, 2) rotate reverse for CCW (+wz), Right wheels (1, 3) rotate forward for CCW (+wz)
        float expectedSign = (i == 0 || i == 2) ? -cmdSign : cmdSign;
        if (v * expectedSign >= 0.10f) {
            movingWheels++;
        }
    }

    bool genuineChassisBreakout = (movingWheels >= 3) && imuRotating;

    if (genuineChassisBreakout) {
        state.spinBreakoutSustainedTicks++;
        if (state.spinBreakoutSustainedTicks >= 3) { // 30 ms sustained
            wheelController.transitionAllToKinetic();
            state.spinRecoveryState = SPIN_RECOVERY_IDLE;
        }
    } else {
        state.spinBreakoutSustainedTicks = 0;
    }

    switch (state.spinRecoveryState) {
        case SPIN_RECOVERY_IDLE:
            if (!state.candidateAConsumedForEpisode) {
                bool anyBoost = false;
                for (int i = 0; i < 4; i++) {
                    if (wheelController.getController(i).getStictionState() == STICTION_BOOST) {
                        anyBoost = true;
                        break;
                    }
                }
                if (anyBoost) {
                    state.spinRecoveryState = SPIN_RECOVERY_MONITORING_INITIAL;
                    state.spinStartTimeMs = nowMs;
                    state.requestedAngular = activeCmd.angularVelocity;
                    for (int i = 0; i < 4; i++) {
                        state.spinStartTicks[i] = encoderManager.getTicks(i);
                    }
                }
            }
            break;

        case SPIN_RECOVERY_MONITORING_INITIAL: {
            if (genuineChassisBreakout) {
                state.spinRecoveryState = SPIN_RECOVERY_IDLE;
                break;
            }

            // Check for initial pure-spin stall at 350 ms
            if (nowMs - state.spinStartTimeMs >= 350) {
                state.spinRecoveryState = SPIN_RECOVERY_SETTLE_1;
                state.spinStateEnteredMs = nowMs;
                state.candidateAConsumedForEpisode = true; // Independent latch: does NOT consume chassis rebreakout!
                wheelController.reset();
                targetLinear = 0.0f;
                targetAngular = 0.0f;
            }
            break;
        }

        case SPIN_RECOVERY_SETTLE_1: {
            targetLinear = 0.0f;
            targetAngular = 0.0f;
            if (nowMs - state.spinStateEnteredMs >= 30) {
                state.spinRecoveryState = SPIN_RECOVERY_TWITCH_PULSE;
                state.spinStateEnteredMs = nowMs;
                for (int i = 0; i < 4; i++) {
                    state.twitchStartTicks[i] = encoderManager.getTicks(i);
                }
                wheelController.reset();
            }
            break;
        }

        case SPIN_RECOVERY_TWITCH_PULSE: {
            float sign_w = (state.requestedAngular >= 0.0f) ? 1.0f : -1.0f;
            float twitchAngular = -sign_w * MAX_ANGULAR_VELOCITY_RADPS;
            targetLinear = 0.0f;
            targetAngular = twitchAngular;

            // Directional wheel travel in opposite-yaw sense
            int32_t deltaTicks[4];
            deltaTicks[0] = (int32_t)(-sign_w * (encoderManager.getTicks(0) - state.twitchStartTicks[0]));
            deltaTicks[1] = (int32_t)( sign_w * (encoderManager.getTicks(1) - state.twitchStartTicks[1]));
            deltaTicks[2] = (int32_t)(-sign_w * (encoderManager.getTicks(2) - state.twitchStartTicks[2]));
            deltaTicks[3] = (int32_t)( sign_w * (encoderManager.getTicks(3) - state.twitchStartTicks[3]));

            int validCount = 0;
            for (int i = 0; i < 4; i++) {
                if (deltaTicks[i] >= 4) validCount++;
            }

            int32_t sorted[4] = {deltaTicks[0], deltaTicks[1], deltaTicks[2], deltaTicks[3]};
            std::sort(sorted, sorted + 4);
            float medianTicks = (sorted[1] + sorted[2]) / 2.0f;

            bool dispTargetReached = (medianTicks >= 8.0f) && (validCount >= 3);
            bool gyroTriggered     = (std::fabs(imuManager.getData().gz) >= 0.08f);
            bool hardTimeout       = (nowMs - state.spinStateEnteredMs >= 50);

            if (dispTargetReached || gyroTriggered || hardTimeout) {
                state.spinRecoveryState = SPIN_RECOVERY_SETTLE_2;
                state.spinStateEnteredMs = nowMs;
                wheelController.reset();
                targetLinear = 0.0f;
                targetAngular = 0.0f;
            }
            break;
        }

        case SPIN_RECOVERY_SETTLE_2: {
            targetLinear = 0.0f;
            targetAngular = 0.0f;
            if (nowMs - state.spinStateEnteredMs >= 30) {
                state.spinRecoveryState = SPIN_RECOVERY_RETRY;
                state.spinStateEnteredMs = nowMs;
                state.retryStartTimeMs = nowMs;
                wheelController.reset(); // Fresh coordinated spin boost in original direction!
            }
            break;
        }

        case SPIN_RECOVERY_RETRY: {
            if (genuineChassisBreakout) {
                wheelController.transitionAllToKinetic();
                state.spinRecoveryState = SPIN_RECOVERY_IDLE;
                break;
            }

            // Retry duration is bounded: 500 ms timeout for retry attempt
            if (nowMs - state.retryStartTimeMs >= 500) {
                state.spinRecoveryState = SPIN_RECOVERY_FAILED_LATCHED;
                state.spinFailedExhausted = true;
                wheelController.forceBlocked();
                targetLinear = 0.0f;
                targetAngular = 0.0f;
                break;
            }

            targetLinear = 0.0f;
            // Respect reduced speed and current navigation intent:
            targetAngular = activeCmd.angularVelocity;
            break;
        }

        case SPIN_RECOVERY_FAILED_LATCHED: {
            targetLinear = 0.0f;
            targetAngular = 0.0f;
            wheelController.forceBlocked();
            break;
        }
    }
}

template<typename TWc, typename TEnc, typename TImu>
inline void updateSpinRecoveryHarness(
    const ChassisCommand &activeCmd,
    float &targetLinear,
    float &targetAngular,
    uint32_t nowMs,
    float dt,
    TWc &wheelController,
    const TEnc &encoderManager,
    const TImu &imuManager
) {
    updateSpinRecoveryHarnessWithState(g_chassisSupervisorState, activeCmd, targetLinear, targetAngular, nowMs, dt, wheelController, encoderManager, imuManager);
}
