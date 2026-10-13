#include "WheelController.h"
#include "RoverConfig.h"
#include "MotorDriver.h"

SingleWheelController::SingleWheelController()
    : index(0)
    , targetVel(0.0f)
    , measuredVel(0.0f)
    , isSpinManeuver(false)
    , isForwardRearWheel(false)
    , isOpposingManeuver(false)
    , isTurningArc(false)
    , blockedRetriesInEpisode(0)
    , zeroDebounceTicks(0)
    , stallDebounceTicks(0)
    , integralTerm(0.0f)
    , recoveryAssist(0.0f)
    , candidateRecoveryAssist(false)
    , lastError(0.0f)
    , Kp(KP_SPEED)
    , Ki(KI_SPEED)
    , Kd(KD_SPEED)
    , lastPwm(0)
    , reversalDeadtimeTicks(0)
    , stictionState(STICTION_IDLE)
    , boostStartTicks(0)
    , boostTicksCount(0)
    , boostTicksInitialized(false)
    , ultraLowSpeedMode(ULTRA_LOW_MODE_BASELINE) {}

void SingleWheelController::begin(int motorIndex) {
    index = motorIndex;
    reset();
}

void SingleWheelController::setGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void SingleWheelController::setTargetVelocity(float targetRadps, bool isSpin, bool isForwardRear, bool isOpposing, bool isTurning) {
    bool wasOpposingOrSpin = (isSpinManeuver || isOpposingManeuver);
    if (abs(targetRadps) >= 0.01f) {
        isSpinManeuver = isSpin;
        isForwardRearWheel = isForwardRear;
        isOpposingManeuver = isOpposing;
        isTurningArc = isTurning;
    }
    bool isNowOpposingOrSpin = (isSpin || isOpposing);

    // Direction change across zero while active:
    // Require deliberate command (|targetRadps| >= 0.05) rather than micro-noise to qualify as reversal
    bool isReversal = (targetVel > 0.05f && targetRadps < -0.05f) || (targetVel < -0.05f && targetRadps > 0.05f);

    if (abs(targetRadps) < 0.01f) {
        // Transition towards zero: require 5 consecutive cycles (50ms @ 100Hz) to declare sustained zero
        zeroDebounceTicks++;
        if (zeroDebounceTicks >= 5) {
            stictionState = STICTION_IDLE;
            boostTicksCount = 0;
            boostTicksInitialized = false;
            blockedRetriesInEpisode = 0;
            integralTerm = 0.0f;
            recoveryAssist = 0.0f;
            stallDebounceTicks = 0;
            lastError = 0.0f;
            zeroDebounceTicks = 0;
        }
    } else {
        // Active non-zero target command
        zeroDebounceTicks = 0;

        // Non-straight maneuvers isolate recovery assist immediately (0 PWM)
        if (isSpin || isOpposing || isTurning) {
            recoveryAssist = 0.0f;
            stallDebounceTicks = 0;
        }

        // Case 1: Starting fresh from STICTION_IDLE
        if (stictionState == STICTION_IDLE) {
            stictionState = STICTION_BOOST;
            boostTicksCount = 0;
            boostTicksInitialized = false;
            integralTerm = 0.0f;
            recoveryAssist = 0.0f;
            stallDebounceTicks = 0;
            lastError = 0.0f;
        }
        // Case 2: Already in STICTION_BOOST
        else if (stictionState == STICTION_BOOST) {
            if (isReversal) {
                // Reversal while stationary in boost: bound no-motion effort across transitions!
                // Do NOT reset boostTicksCount. Re-initialize displacement baseline.
                boostTicksInitialized = false;
                integralTerm = 0.0f;
                recoveryAssist = 0.0f;
                stallDebounceTicks = 0;
                lastError = 0.0f;
            }
            // If returning from a transient zero (< 50ms), boostTicksCount is preserved as well.
        }
        // Case 3: Reversal while in STICTION_KINETIC (motion was active)
        else if (isReversal && stictionState == STICTION_KINETIC) {
            stictionState = STICTION_BOOST;
            boostTicksCount = 0;
            boostTicksInitialized = false;
            integralTerm = 0.0f;
            recoveryAssist = 0.0f;
            stallDebounceTicks = 0;
            lastError = 0.0f;
        }
        // Case 4: Deliberate direction change while blocked: allow one reversal attempt if commanded speed is substantial (>= 0.15 rad/s)
        else if (isReversal && stictionState == STICTION_BLOCKED && abs(targetRadps) >= 0.15f && blockedRetriesInEpisode == 0) {
            stictionState = STICTION_BOOST;
            boostTicksCount = 0;
            boostTicksInitialized = false;
            blockedRetriesInEpisode = 1;
            integralTerm = 0.0f;
            recoveryAssist = 0.0f;
            stallDebounceTicks = 0;
            lastError = 0.0f;
        }
        // Case 5: Transition from non-opposing to opposing/spin while blocked: allow at most ONE re-boost retry per episode
        else if (stictionState == STICTION_BLOCKED && !wasOpposingOrSpin && isNowOpposingOrSpin && blockedRetriesInEpisode == 0) {
            stictionState = STICTION_BOOST;
            boostTicksCount = 0;
            boostTicksInitialized = false;
            blockedRetriesInEpisode = 1;
            integralTerm = 0.0f;
            recoveryAssist = 0.0f;
            stallDebounceTicks = 0;
            lastError = 0.0f;
        }
        else if (isSpin && abs(targetRadps - targetVel) > 0.5f) {
            // Step change during spin (e.g. cruise to creep transition): clear integral accumulator to avoid negative windup
            integralTerm = 0.0f;
            recoveryAssist = 0.0f;
            stallDebounceTicks = 0;
            lastError = 0.0f;
        }
    }

    targetVel = targetRadps;
}

int SingleWheelController::update(float measuredRadps, int32_t currentTicks, float dt) {
    measuredVel = measuredRadps;
    
    // Target near-zero handling: decay output and reset integrators immediately (0 PWM)
    // Note: Do not clear stictionState or boostTicksCount on single-cycle zero transient;
    // sustained zero (50ms debounce) in setTargetVelocity resets state to STICTION_IDLE.
    if (abs(targetVel) < 0.01f) {
        integralTerm = 0.0f;
        recoveryAssist = 0.0f;
        stallDebounceTicks = 0;
        lastError = 0.0f;
        lastPwm = 0;
        diag.targetVel = 0.0f;
        diag.measuredVel = measuredRadps;
        diag.feedforward = 0.0f;
        diag.pTerm = 0.0f;
        diag.iTerm = 0.0f;
        diag.dTerm = 0.0f;
        diag.recoveryAssist = 0.0f;
        diag.finalPwm = 0;
        diag.stictionState = stictionState;
        return 0;
    }
    
    // Dynamic Stiction State Machine
    if (stictionState == STICTION_BOOST) {
        if (!boostTicksInitialized) {
            boostStartTicks = currentTicks;
            boostTicksInitialized = true;
        }
        
        int32_t deltaTicks = abs(currentTicks - boostStartTicks);
        
        // For non-spin (straight/rolling): breakout confirmation strictly >= 3 ticks displacement (0.31 mm)
        if (!isSpinManeuver && deltaTicks >= 3) {
            stictionState = STICTION_KINETIC;
            // Transition directly into kinetic PID calculation below
        } else if (!isSpinManeuver) {
            boostTicksCount++;
            if (boostTicksCount >= 50) { // 500 ms at 100 Hz
                stictionState = STICTION_BLOCKED;
                blockedRetriesInEpisode = 1;
                lastPwm = 0;
                diag.targetVel = targetVel;
                diag.measuredVel = measuredRadps;
                diag.feedforward = 0.0f;
                diag.pTerm = 0.0f;
                diag.iTerm = 0.0f;
                diag.dTerm = 0.0f;
                diag.finalPwm = 0;
                diag.stictionState = STICTION_BLOCKED;
                return 0;
            }
            
            // While in boost: opposing wheel tight arc uses SPIN_STICTION_BOOST_PWM (102 PWM)
            // Commanded turning arc with lateral floor scrub uses MIN_USABLE_SPIN_PWM (68 PWM)
            // Low-speed turning arc retains STICTION_BOOST_PWM (48 PWM)
            // Ordinary straight forward/reverse startup uses STRAIGHT_STICTION_BOOST_PWM (64 PWM)
            int boostMag = STICTION_BOOST_PWM;
            if (isOpposingManeuver) {
                boostMag = SPIN_STICTION_BOOST_PWM;
            } else if (isTurningArc) {
                if (abs(targetVel) >= MIN_RELIABLE_SPEED_RADPS) {
                    boostMag = MIN_USABLE_SPIN_PWM;
                } else {
                    boostMag = STICTION_BOOST_PWM;
                }
            } else {
                boostMag = STRAIGHT_STICTION_BOOST_PWM;
            }
            int boostPwm = (targetVel > 0.0f) ? boostMag : -boostMag;
            
            // H-bridge polarity protection (50ms deadtime = 5 ticks @ 100Hz)
            if (((lastPwm > 0 && boostPwm < 0) || (lastPwm < 0 && boostPwm > 0)) && lastPwm != 0 && boostPwm != 0) {
                reversalDeadtimeTicks = 5;
            }
            lastPwm = boostPwm;
            
            diag.targetVel = targetVel;
            diag.measuredVel = measuredRadps;
            diag.feedforward = (float)boostPwm;
            diag.pTerm = 0.0f;
            diag.iTerm = 0.0f;
            diag.dTerm = 0.0f;
            diag.finalPwm = (int16_t)boostPwm;
            diag.stictionState = STICTION_BOOST;
            
            if (reversalDeadtimeTicks > 0) {
                reversalDeadtimeTicks--;
                return 0;
            }
            return lastPwm;
        } else {
            // For PURE-SPIN maneuver:
            // Controlled by WheelController rear-pair sustained motion guard or hard 200 ms timeout
            boostTicksCount++;
            if (boostTicksCount >= SPIN_BREAKOUT_MAX_CYCLES) {
                stictionState = STICTION_KINETIC;
                // Transition directly into kinetic PID calculation below
            } else {
                int boostMag = SPIN_STICTION_BOOST_PWM; // Empirically validated 102 PWM breakout boost
                int boostPwm = (targetVel > 0.0f) ? boostMag : -boostMag;
                
                if (((lastPwm > 0 && boostPwm < 0) || (lastPwm < 0 && boostPwm > 0)) && lastPwm != 0 && boostPwm != 0) {
                    reversalDeadtimeTicks = 5;
                }
                lastPwm = boostPwm;
                
                diag.targetVel = targetVel;
                diag.measuredVel = measuredRadps;
                diag.feedforward = (float)boostPwm;
                diag.pTerm = 0.0f;
                diag.iTerm = 0.0f;
                diag.dTerm = 0.0f;
                diag.finalPwm = (int16_t)boostPwm;
                diag.stictionState = STICTION_BOOST;
                
                if (reversalDeadtimeTicks > 0) {
                    reversalDeadtimeTicks--;
                    return 0;
                }
                return lastPwm;
            }
        }
    }
    
    if (stictionState == STICTION_BLOCKED) {
        lastPwm = 0;
        recoveryAssist = 0.0f;
        stallDebounceTicks = 0;
        diag.targetVel = targetVel;
        diag.measuredVel = measuredRadps;
        diag.feedforward = 0.0f;
        diag.pTerm = 0.0f;
        diag.iTerm = 0.0f;
        diag.dTerm = 0.0f;
        diag.recoveryAssist = 0.0f;
        diag.finalPwm = 0;
        diag.stictionState = STICTION_BLOCKED;
        return 0;
    }
    
    // STICTION_KINETIC: Closed-loop PID + Feedforward structure
    float error = targetVel - measuredRadps;
    
    // Check for speed deficit in commanded direction (wheel too slow or stalled)
    bool isCommanded = abs(targetVel) >= MIN_RELIABLE_SPEED_RADPS;
    bool isLagging = ((targetVel > 0.0f && error > 0.0f) || (targetVel < 0.0f && error < 0.0f));
    bool isOverspeed = ((targetVel > 0.0f && error < 0.0f) || (targetVel < 0.0f && error > 0.0f));
    bool isStalledCondition = isCommanded && isLagging && (abs(measuredRadps) < ANTI_STALL_SPEED_THRESHOLD);

    // Straight kinetic motion check:
    bool isStraightKinetic = !isSpinManeuver && !isOpposingManeuver && !isTurningArc && (stictionState == STICTION_KINETIC);
    bool isEligibleStraightRecovery = isStraightKinetic && candidateRecoveryAssist;

    // Candidate stall debounce applies strictly inside eligible ordinary straight KINETIC recovery:
    if (isEligibleStraightRecovery && isStalledCondition) {
        if (stallDebounceTicks < 255) stallDebounceTicks++;
    } else {
        stallDebounceTicks = 0;
    }
    bool isCandidateStalled = isEligibleStraightRecovery && (stallDebounceTicks >= 2);

    // Dynamic integral gain:
    float activeKi = isSpinManeuver ? SPIN_PID_KI : Ki;
    float effectiveError = error;

    if (isEligibleStraightRecovery) {
        if (isCandidateStalled) {
            // Straight kinetic stall recovery:
            // Handled exclusively by separate bounded recovery assist (recoveryAssist), NOT by inflating tracking integral!
            // Ramp recovery assist in commanded direction at 40 PWM/s up to 14 PWM cap:
            if (targetVel > 0.0f) {
                recoveryAssist += RECOVERY_ASSIST_RAMP_RATE_PWPS * dt;
                if (recoveryAssist > RECOVERY_ASSIST_MAX_PWM) {
                    recoveryAssist = RECOVERY_ASSIST_MAX_PWM;
                }
            } else {
                recoveryAssist -= RECOVERY_ASSIST_RAMP_RATE_PWPS * dt;
                if (recoveryAssist < -RECOVERY_ASSIST_MAX_PWM) {
                    recoveryAssist = -RECOVERY_ASSIST_MAX_PWM;
                }
            }
        } else {
            // Once wheel breaks out (|measuredRadps| >= 0.15 rad/s), smoothly decay recovery assist to 0:
            // Slew rate is 30 + 20*abs(error) PWM/s when error opposes targetVel (overspeed):
            float decayRate = RECOVERY_ASSIST_DECAY_RATE_PWPS;
            if (isOverspeed) {
                decayRate += abs(error) * 20.0f;
            }

            float prevAssist = recoveryAssist;
            if (recoveryAssist > 0.0f) {
                recoveryAssist -= decayRate * dt;
                if (recoveryAssist < 0.0f) recoveryAssist = 0.0f;
            } else if (recoveryAssist < 0.0f) {
                recoveryAssist += decayRate * dt;
                if (recoveryAssist > 0.0f) recoveryAssist = 0.0f;
            }
            float assistDecline = abs(prevAssist) - abs(recoveryAssist);

            // DIRECTION-AWARE SEAMLESS RECOVERY-TO-TRACKING HANDOFF:
            // When temporary recovery assist declines after breakout, transfer the declining torque
            // directly into the tracking integral in the commanded direction ONLY when not overspeeding (!isOverspeed).
            // This prevents an abrupt torque drop and ensures the tracking integrator takes over
            // the sustained load before assist disappears, eliminating restall oscillations.
            // If the wheel is already at or above target (isOverspeed), handoff is blocked so
            // temporary recovery assist does not inflate the tracking integral into sustained overspeed.
            bool isUltraLow = (ultraLowSpeedMode != ULTRA_LOW_MODE_BASELINE) && isStraightKinetic && (abs(targetVel) < ULTRA_LOW_SPEED_SCOPE_RADPS);
            if (isUltraLow && assistDecline > 0.0f && !isOverspeed) {
                if (targetVel > 0.0f) {
                    integralTerm += assistDecline;
                } else {
                    integralTerm -= assistDecline;
                }
            }
        }

        // Freeze tracking integration during stall condition AND while recovery assist is active,
        // EXCEPT when the wheel is overspeeding during assist decay (where unwinding is required):
        bool isUltraLow = (ultraLowSpeedMode != ULTRA_LOW_MODE_BASELINE) && isStraightKinetic && (abs(targetVel) < ULTRA_LOW_SPEED_SCOPE_RADPS);
        bool isOverspeedUnwind = isUltraLow && (recoveryAssist != 0.0f) && isOverspeed;
        if (isStalledCondition || (recoveryAssist != 0.0f && !isOverspeedUnwind)) {
            activeKi = 0.0f;
            effectiveError = 0.0f;
        } else {
            // Normal tracking integral resumes once recovery assist is fully discharged (or during overspeed unwinding):
            if (isUltraLow) {
                // In ordinary straight crawl below 0.80 rad/s, apply dedicated STRAIGHT_CRAWL_KI (9.0)
                // across the entire commanded operating range, providing useful closed-loop
                // correction for small persistent velocity errors and eliminating noise-induced regime switching:
                if (isCommanded) {
                    activeKi = STRAIGHT_CRAWL_KI;
                }
            } else {
                if (abs(targetVel) >= 0.01f && abs(targetVel) <= 2.0f && abs(error) >= 0.40f) {
                    activeKi = STRAIGHT_CRAWL_KI;
                } else if (isCommanded && abs(error) >= 0.25f) {
                    activeKi = Ki * 3.0f;
                }
            }
        }
    } else {
        // Non-straight maneuvers (pure spin, opposing arcs, turning arcs) or legacy comparison mode:
        // Isolate straight recovery assist completely (must remain 0)
        recoveryAssist = 0.0f;

        // Original instantaneous stall detection and gains strictly preserved:
        if (isStalledCondition) {
            activeKi = ANTI_STALL_KI;
            const float minStallError = 1.60f;
            if (error > 0.0f && effectiveError < minStallError) {
                effectiveError = minStallError;
            } else if (error < 0.0f && effectiveError > -minStallError) {
                effectiveError = -minStallError;
            }
        } else if (!isSpinManeuver && !isOpposingManeuver && !isTurningArc &&
                   stictionState == STICTION_KINETIC &&
                   abs(targetVel) >= 0.01f && abs(targetVel) <= 2.0f && abs(error) >= 0.40f) {
            activeKi = STRAIGHT_CRAWL_KI;
        } else if (isCommanded && abs(error) >= 0.25f) {
            activeKi = isSpinManeuver ? (isLagging ? LAGGING_KI : SPIN_PID_KI) : (Ki * 3.0f);
        }
    }

    // Actuator total-output saturation handling:
    // Strictly scoped to candidate's eligible straight recovery path.
    // Legacy behavior (unsuppressed integration) is preserved when disabled, during startup boost, or for spins/arcs.
    bool saturatedPositive = (lastPwm >= 255) && (error > 0.0f);
    bool saturatedNegative = (lastPwm <= -255) && (error < 0.0f);
    bool suppressIntegration = isEligibleStraightRecovery && (saturatedPositive || saturatedNegative);

    if (!suppressIntegration) {
        // Integrate gain-weighted error increments to maintain output continuity across gain transitions
        // Fast unwinding of opposing integral windup from acceleration/overshoot (2x rate)
        bool isOpposing = (error < 0.0f && integralTerm > 0.0f) || (error > 0.0f && integralTerm < 0.0f);
        if (isOpposing) {
            integralTerm += activeKi * error * dt * 2.0f;
        } else {
            integralTerm += activeKi * effectiveError * dt;
        }
    }

    // Integral state anti-windup: clamp the integral contribution to [-150, 150]
    integralTerm = constrain(integralTerm, -150.0f, 150.0f);
    
    // Derivative term
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    // Breakaway friction and velocity feedforward magnitude
    float breakaway;
    if (isSpinManeuver) {
        // Dedicated pure-spin kinetic feedforward base (75.0 PWM)
        breakaway = SPIN_KINETIC_KS_PWM;
    } else {
        // In rolling kinetic motion, dynamic friction is ~55% of static breakaway (~22 PWM)
        float staticBreakaway = (targetVel > 0.0f) ? (float)motorCalibrations[index].forwardBreakawayPwm
                                                   : (float)motorCalibrations[index].reverseBreakawayPwm;
        breakaway = staticBreakaway * 0.55f;
    }
    float rawFfMagnitude = breakaway + (motorCalibrations[index].kV * abs(targetVel));
    float ffMagnitude = rawFfMagnitude;

    // In KINETIC state with non-zero target, apply minimum feedforward floor
    if (stictionState == STICTION_KINETIC && abs(targetVel) >= 0.01f) {
        float minFfFloor;
        if (isSpinManeuver) {
            // Forward-driving rear wheel receives 94 PWM floor; others receive 80 PWM floor
            minFfFloor = isForwardRearWheel ? SPIN_FORWARD_REAR_KINETIC_FLOOR : MIN_SPIN_KINETIC_FF_FLOOR;
        } else if (isOpposingManeuver || isTurningArc) {
            // Turning arcs and opposing tight maneuvers retain baseline kinetic floor (22.0 PWM)
            minFfFloor = 22.0f;
        } else {
            // Ordinary straight forward/reverse driving uses dedicated kinetic floor.
            if (ultraLowSpeedMode == ULTRA_LOW_MODE_GAINS_AND_FF && abs(targetVel) < ULTRA_LOW_SPEED_SCOPE_RADPS) {
                if (targetVel >= 0.0f) {
                    // Forward straight ultra-low crawl (unchanged): linearly scales from ULTRA_LOW_SPEED_MIN_FF_FLOOR_FWD (37.0)
                    // up to STRAIGHT_KINETIC_FF_FLOOR (56.0) at 0.80 rad/s, delivering 48.0 PWM at Case 2 (+0.461538 rad/s).
                    minFfFloor = ULTRA_LOW_SPEED_MIN_FF_FLOOR_FWD + (targetVel / ULTRA_LOW_SPEED_SCOPE_RADPS) * (STRAIGHT_KINETIC_FF_FLOOR - ULTRA_LOW_SPEED_MIN_FF_FLOOR_FWD);
                } else {
                    // Reverse straight ultra-low crawl piecewise floor:
                    // 1. For |targetVel| in [0.01, 0.461538] rad/s: constant 41.0 PWM floor.
                    //    Prevents sub-sustaining feedforward (22-38 PWM) at low speeds that would stall and cycle.
                    // 2. For |targetVel| in (0.461538, 0.80) rad/s: continuous linear interpolation from 41.0 PWM to 56.0 PWM.
                    float absTarget = abs(targetVel);
                    if (absTarget <= ULTRA_LOW_SPEED_REVERSE_ANCHOR_RADPS) {
                        minFfFloor = ULTRA_LOW_SPEED_REVERSE_CRAWL_FLOOR_PWM;
                    } else {
                        minFfFloor = ULTRA_LOW_SPEED_REVERSE_CRAWL_FLOOR_PWM +
                            ((absTarget - ULTRA_LOW_SPEED_REVERSE_ANCHOR_RADPS) /
                             (ULTRA_LOW_SPEED_SCOPE_RADPS - ULTRA_LOW_SPEED_REVERSE_ANCHOR_RADPS)) *
                            (STRAIGHT_KINETIC_FF_FLOOR - ULTRA_LOW_SPEED_REVERSE_CRAWL_FLOOR_PWM);
                    }
                }
            } else {
                minFfFloor = STRAIGHT_KINETIC_FF_FLOOR;
            }
        }
        if (ffMagnitude < minFfFloor) {
            ffMagnitude = minFfFloor;
        }
    }
    
    float feedforward = (targetVel > 0.0f ? 1.0f : -1.0f) * ffMagnitude;
    
    // PID Correction (pure spin selects SPIN_PID_KP = 6.0, normal driving retains Kp = 2.2)
    // Full un-deadbanded integral authority: provides immediate closed-loop torque response to tracking error
    float activeKp = isSpinManeuver ? SPIN_PID_KP : Kp;
    float pidCorrection = (activeKp * error) + integralTerm + (Kd * derivative);
    
    // Combine PID correction, Feedforward, and Separate Recovery Assist
    float totalPwm = feedforward + pidCorrection + recoveryAssist;
    int targetPwm = constrain((int)round(totalPwm), -255, 255);
    
    // Populate diagnostic telemetry (read-only, non-interfering)
    diag.targetVel = targetVel;
    diag.measuredVel = measuredRadps;
    diag.feedforward = feedforward;
    diag.pTerm = activeKp * error;
    diag.iTerm = integralTerm;
    diag.dTerm = Kd * derivative;
    diag.recoveryAssist = recoveryAssist;

    diag.finalPwm = (int16_t)targetPwm;
    diag.stictionState = STICTION_KINETIC;

    // Non-blocking H-bridge polarity protection (50ms deadtime = 5 ticks @ 100Hz)
    if (((lastPwm > 0 && targetPwm < 0) || (lastPwm < 0 && targetPwm > 0)) && lastPwm != 0 && targetPwm != 0) {
        reversalDeadtimeTicks = 5;
    }
    
    lastPwm = targetPwm;
    
    if (reversalDeadtimeTicks > 0) {
        reversalDeadtimeTicks--;
        return 0; // Output zero during reversal deadtime
    }
    
    return lastPwm;
}

void SingleWheelController::setDiagPwmOutputs(int basePwm, int trim, int finalPwm) {
    diag.basePwm = (int16_t)basePwm;
    diag.spinSyncTrim = (int16_t)trim;
    diag.finalPwm = (int16_t)finalPwm;
    lastPwm = finalPwm;
}

void SingleWheelController::updateOpenLoop(float measuredRadps, int pwm) {
    measuredVel = measuredRadps;
    lastPwm = pwm;
    recoveryAssist = 0.0f;
    stictionState = STICTION_KINETIC;
    diag.targetVel = 0.0f;
    diag.measuredVel = measuredRadps;
    diag.feedforward = (float)pwm;
    diag.pTerm = 0.0f;
    diag.iTerm = 0.0f;
    diag.dTerm = 0.0f;
    diag.recoveryAssist = 0.0f;
    diag.basePwm = (int16_t)pwm;
    diag.spinSyncTrim = 0;
    diag.finalPwm = (int16_t)pwm;
    diag.stictionState = STICTION_KINETIC;
}

void SingleWheelController::reset() {
    integralTerm = 0.0f;
    recoveryAssist = 0.0f;
    lastError = 0.0f;
    lastPwm = 0;
    reversalDeadtimeTicks = 0;
    stictionState = STICTION_IDLE;
    boostStartTicks = 0;
    boostTicksCount = 0;
    boostTicksInitialized = false;
    isOpposingManeuver = false;
    isTurningArc = false;
    blockedRetriesInEpisode = 0;
    zeroDebounceTicks = 0;
    stallDebounceTicks = 0;
    memset(&diag, 0, sizeof(diag));
}

WheelController::WheelController() 
    : openLoopActive(false)
    , lastOpenLoopPwmMs(0)
    , timeoutDisarmHook(nullptr)
    , spinSustainedVelocityCycles(0)
    , lastSpinSyncTrimLeft(0)
    , lastSpinSyncTrimRight(0)
    , pairErrorLeft(0.0f)
    , pairErrorRight(0.0f)
    , prevTargetLeft(0.0f)
    , prevTargetRight(0.0f) {
    for (int i = 0; i < 4; i++) openLoopPwms[i] = 0;
}

void WheelController::begin() {
    openLoopActive = false;
    lastOpenLoopPwmMs = 0;
    spinSustainedVelocityCycles = 0;
    lastSpinSyncTrimLeft = 0;
    lastSpinSyncTrimRight = 0;
    pairErrorLeft = 0.0f;
    pairErrorRight = 0.0f;
    prevTargetLeft = 0.0f;
    prevTargetRight = 0.0f;
    for (int i = 0; i < 4; i++) {
        openLoopPwms[i] = 0;
        controllers[i].begin(i);
        controllers[i].setCandidateRecoveryAssist(true);
        controllers[i].setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    }
}

void WheelController::setTargets(float leftTargetRadps, float rightTargetRadps, bool isSpin, bool isOpposing, bool isTurning) {
    openLoopActive = false;
    
    // Canonical mapping: Index 0=M1/LF, Index 1=M2/RF, Index 2=M3/LR, Index 3=M4/RR
    // In pure spin:
    // CCW spin (left target < 0, right target > 0): M4/RR (Index 3) is forward-driving rear wheel
    // CW spin (left target > 0, right target < 0):  M3/LR (Index 2) is forward-driving rear wheel
    bool m1_isForwardRear = false;
    bool m2_isForwardRear = false;
    bool m3_isForwardRear = isSpin && (leftTargetRadps > 0.01f);
    bool m4_isForwardRear = isSpin && (rightTargetRadps > 0.01f);

    controllers[0].setTargetVelocity(leftTargetRadps, isSpin, m1_isForwardRear, isOpposing, isTurning);
    controllers[1].setTargetVelocity(rightTargetRadps, isSpin, m2_isForwardRear, isOpposing, isTurning);
    controllers[2].setTargetVelocity(leftTargetRadps, isSpin, m3_isForwardRear, isOpposing, isTurning);
    controllers[3].setTargetVelocity(rightTargetRadps, isSpin, m4_isForwardRear, isOpposing, isTurning);
}

void WheelController::setWheelTarget(int index, float targetRadps, bool isSpin, bool isOpposing, bool isTurning) {
    openLoopActive = false;
    if (index >= 0 && index < 4) {
        bool isForwardRear = isSpin && (index == 2 || index == 3) && (targetRadps > 0.01f);
        controllers[index].setTargetVelocity(targetRadps, isSpin, isForwardRear, isOpposing, isTurning);
    }
}

void WheelController::setOpenLoopPwm(int m1, int m2, int m3, int m4, uint32_t nowMs) {
    openLoopActive = true;
    lastOpenLoopPwmMs = (nowMs != 0) ? nowMs : millis();
    openLoopPwms[0] = m1;
    openLoopPwms[1] = m2;
    openLoopPwms[2] = m3;
    openLoopPwms[3] = m4;
}

void WheelController::clearOpenLoop() {
    openLoopActive = false;
    lastOpenLoopPwmMs = 0;
    for (int i = 0; i < 4; i++) {
        openLoopPwms[i] = 0;
    }
}

void WheelController::update(const float *measuredVelocities, const int32_t *ticks, float dt, MotorDriver &driver, bool isSpinManeuver, bool isGoingStraight, bool encodersFresh, uint32_t nowMs) {
    if (nowMs == 0) {
        nowMs = millis();
    }
    if (openLoopActive) {
        // Enforce timeout using modular subtraction for safe unsigned wraparound
        if ((uint32_t)(nowMs - lastOpenLoopPwmMs) > OPEN_LOOP_PWM_TIMEOUT_MS) {
            clearOpenLoop();
            for (int i = 0; i < 4; i++) {
                driver.setPWM(i, 0);
            }
            reset();
            if (timeoutDisarmHook) {
                timeoutDisarmHook();
            }
            return;
        }
        // If driver was in dynamic braking state, release brake so open-loop PWM can drive
        if (driver.getActuationState() == DrivetrainActuationState::BRAKE) {
            driver.coastAll();
        }
        lastSpinSyncTrimLeft = 0;
        lastSpinSyncTrimRight = 0;
        for (int i = 0; i < 4; i++) {
            driver.setPWM(i, openLoopPwms[i]);
            controllers[i].updateOpenLoop(measuredVelocities[i], openLoopPwms[i]);
        }
        return;
    }

    // 1. Calculate base closed-loop PID + Feedforward PWM outputs first
    int basePwms[4];
    for (int i = 0; i < 4; i++) {
        basePwms[i] = controllers[i].update(measuredVelocities[i], ticks[i], dt);
    }

    // 2. Shared Production Drivetrain Same-Side Wheel Synchronization Trim
    int finalPwms[4];
    for (int i = 0; i < 4; i++) {
        finalPwms[i] = basePwms[i];
    }

    bool balancingEligible = WHEEL_BALANCE_ENABLED && encodersFresh && (driver.getActuationState() != DrivetrainActuationState::BRAKE);

    if (!balancingEligible) {
        lastSpinSyncTrimLeft = 0;
        lastSpinSyncTrimRight = 0;
        pairErrorLeft = 0.0f;
        pairErrorRight = 0.0f;
        prevTargetLeft = 0.0f;
        prevTargetRight = 0.0f;
    } else {
        // --- Left Side Sync Trim (M1 / LF [0] vs M3 / LR [2]) ---
        float t0 = controllers[0].getTarget();
        float t2 = controllers[2].getTarget();
        bool leftReliable = (abs(t0) >= MIN_RELIABLE_SPEED_RADPS) && (abs(t2) >= MIN_RELIABLE_SPEED_RADPS);
        bool leftSameDir = (t0 > 0.0f && t2 > 0.0f) || (t0 < 0.0f && t2 < 0.0f);

        // Reset left trim on direction change or zero command
        if ((t0 > 0.01f && prevTargetLeft < -0.01f) || (t0 < -0.01f && prevTargetLeft > 0.01f) || (abs(t0) < 0.01f)) {
            lastSpinSyncTrimLeft = 0;
            pairErrorLeft = 0.0f;
        }
        prevTargetLeft = t0;

        if (leftReliable && leftSameDir) {
            float v0 = measuredVelocities[0];
            float v2 = measuredVelocities[2];
            pairErrorLeft = abs(v0) - abs(v2);

            float effLeftDiff = 0.0f;
            if (pairErrorLeft > SAME_SIDE_SYNC_DEADBAND) {
                effLeftDiff = pairErrorLeft - SAME_SIDE_SYNC_DEADBAND;
            } else if (pairErrorLeft < -SAME_SIDE_SYNC_DEADBAND) {
                effLeftDiff = pairErrorLeft + SAME_SIDE_SYNC_DEADBAND;
            }

            int rawLeftTrim = (int)round(SAME_SIDE_SYNC_TRIM_GAIN * effLeftDiff);
            rawLeftTrim = constrain(rawLeftTrim, -SAME_SIDE_SYNC_TRIM_MAX, SAME_SIDE_SYNC_TRIM_MAX);

            int leftDelta = rawLeftTrim - lastSpinSyncTrimLeft;
            leftDelta = constrain(leftDelta, -SAME_SIDE_SYNC_SLEW_LIMIT, SAME_SIDE_SYNC_SLEW_LIMIT);
            lastSpinSyncTrimLeft += leftDelta;
        } else {
            lastSpinSyncTrimLeft = 0;
            pairErrorLeft = 0.0f;
        }

        // --- Right Side Sync Trim (M2 / RF [1] vs M4 / RR [3]) ---
        float t1 = controllers[1].getTarget();
        float t3 = controllers[3].getTarget();
        bool rightReliable = (abs(t1) >= MIN_RELIABLE_SPEED_RADPS) && (abs(t3) >= MIN_RELIABLE_SPEED_RADPS);
        bool rightSameDir = (t1 > 0.0f && t3 > 0.0f) || (t1 < 0.0f && t3 < 0.0f);

        // Reset right trim on direction change or zero command
        if ((t1 > 0.01f && prevTargetRight < -0.01f) || (t1 < -0.01f && prevTargetRight > 0.01f) || (abs(t1) < 0.01f)) {
            lastSpinSyncTrimRight = 0;
            pairErrorRight = 0.0f;
        }
        prevTargetRight = t1;

        if (rightReliable && rightSameDir) {
            float v1 = measuredVelocities[1];
            float v3 = measuredVelocities[3];
            pairErrorRight = abs(v1) - abs(v3);

            float effRightDiff = 0.0f;
            if (pairErrorRight > SAME_SIDE_SYNC_DEADBAND) {
                effRightDiff = pairErrorRight - SAME_SIDE_SYNC_DEADBAND;
            } else if (pairErrorRight < -SAME_SIDE_SYNC_DEADBAND) {
                effRightDiff = pairErrorRight + SAME_SIDE_SYNC_DEADBAND;
            }

            int rawRightTrim = (int)round(SAME_SIDE_SYNC_TRIM_GAIN * effRightDiff);
            rawRightTrim = constrain(rawRightTrim, -SAME_SIDE_SYNC_TRIM_MAX, SAME_SIDE_SYNC_TRIM_MAX);

            int rightDelta = rawRightTrim - lastSpinSyncTrimRight;
            rightDelta = constrain(rightDelta, -SAME_SIDE_SYNC_SLEW_LIMIT, SAME_SIDE_SYNC_SLEW_LIMIT);
            lastSpinSyncTrimRight += rightDelta;
        } else {
            lastSpinSyncTrimRight = 0;
            pairErrorRight = 0.0f;
        }
    }

    // Apply push-pull trim to base PWMs and perform final ±255 clamp
    if (lastSpinSyncTrimLeft != 0 || lastSpinSyncTrimRight != 0) {
        float t0 = controllers[0].getTarget();
        float t2 = controllers[2].getTarget();
        int s0 = (t0 > 0.0f) ? 1 : ((t0 < 0.0f) ? -1 : 0);
        int s2 = (t2 > 0.0f) ? 1 : ((t2 < 0.0f) ? -1 : 0);

        finalPwms[0] = constrain(basePwms[0] - s0 * lastSpinSyncTrimLeft, -255, 255);
        finalPwms[2] = constrain(basePwms[2] + s2 * lastSpinSyncTrimLeft, -255, 255);

        float t1 = controllers[1].getTarget();
        float t3 = controllers[3].getTarget();
        int s1 = (t1 > 0.0f) ? 1 : ((t1 < 0.0f) ? -1 : 0);
        int s3 = (t3 > 0.0f) ? 1 : ((t3 < 0.0f) ? -1 : 0);

        finalPwms[1] = constrain(basePwms[1] - s1 * lastSpinSyncTrimRight, -255, 255);
        finalPwms[3] = constrain(basePwms[3] + s3 * lastSpinSyncTrimRight, -255, 255);
    }

    // Usable power floor protection: ensure push-pull trim never reduces kinetic motor power below stall threshold
    if (isSpinManeuver) {
        for (int i = 0; i < 4; i++) {
            float t = controllers[i].getTarget();
            if (abs(t) >= 0.05f && controllers[i].getStictionState() == STICTION_KINETIC) {
                int s_wh = (t > 0.0f) ? 1 : ((t < 0.0f) ? -1 : 0);
                int trim_val = 0;
                if (i == 0) trim_val = -s_wh * lastSpinSyncTrimLeft;
                else if (i == 2) trim_val = +s_wh * lastSpinSyncTrimLeft;
                else if (i == 1) trim_val = -s_wh * lastSpinSyncTrimRight;
                else if (i == 3) trim_val = +s_wh * lastSpinSyncTrimRight;

                bool isThrottled = (t > 0.0f && trim_val < 0) || (t < 0.0f && trim_val > 0);
                int floorPwm = isThrottled ? MIN_USABLE_SPIN_PWM_FASTER : MIN_USABLE_SPIN_PWM;

                if (t > 0.0f && finalPwms[i] < floorPwm) {
                    finalPwms[i] = floorPwm;
                } else if (t < 0.0f && finalPwms[i] > -floorPwm) {
                    finalPwms[i] = -floorPwm;
                }
            }
        }
    }

    // Write final PWM outputs to driver and set diagnostic telemetry
    for (int i = 0; i < 4; i++) {
        if (driver.getActuationState() != DrivetrainActuationState::BRAKE) {
            driver.setPWM(i, finalPwms[i]);
        }

        int trimForWheel = 0;
        float t_wh = controllers[i].getTarget();
        int s_wh = (t_wh > 0.0f) ? 1 : ((t_wh < 0.0f) ? -1 : 0);

        if (i == 0) trimForWheel = -s_wh * lastSpinSyncTrimLeft;
        if (i == 2) trimForWheel = +s_wh * lastSpinSyncTrimLeft;
        if (i == 1) trimForWheel = -s_wh * lastSpinSyncTrimRight;
        if (i == 3) trimForWheel = +s_wh * lastSpinSyncTrimRight;

        controllers[i].setDiagPwmOutputs(basePwms[i], trimForWheel, finalPwms[i]);
    }
    
    // 3. Coordinated Rear-Pair Breakout Guard for Pure Spin:
    bool anySpinBoost = false;
    for (int i = 0; i < 4; i++) {
        if (controllers[i].getStictionState() == STICTION_BOOST) {
            anySpinBoost = true;
            break;
        }
    }
    
    if (isSpinManeuver && anySpinBoost) {
        // Track sustained velocity across BOTH physical rear wheels (Index 2: M3/LR, Index 3: M4/RR)
        bool lr_moving = (abs(measuredVelocities[2]) >= SPIN_BREAKOUT_VELOCITY_THRESHOLD);
        bool rr_moving = (abs(measuredVelocities[3]) >= SPIN_BREAKOUT_VELOCITY_THRESHOLD);
        
        if (lr_moving && rr_moving) {
            spinSustainedVelocityCycles++;
        } else {
            spinSustainedVelocityCycles = 0;
        }
        
        uint16_t maxBoostCycles = 0;
        for (int i = 0; i < 4; i++) {
            if (controllers[i].getBoostTicksCount() > maxBoostCycles) {
                maxBoostCycles = controllers[i].getBoostTicksCount();
            }
        }
        
        // Conservative Breakout Exit Rule:
        // 1. Must satisfy minimum dwell time (>= 100 ms / 10 cycles @ 100 Hz) AND sustained rear motion for >= 3 cycles (30 ms)
        // OR 2. Reach hard maximum timeout (>= 200 ms / 20 cycles @ 100 Hz)
        bool earlyExitOk = (maxBoostCycles >= SPIN_BREAKOUT_MIN_CYCLES) && (spinSustainedVelocityCycles >= SPIN_BREAKOUT_SUSTAINED_CYCLES);
        bool hardTimeoutOk = (maxBoostCycles >= SPIN_BREAKOUT_MAX_CYCLES);
        
        if (earlyExitOk || hardTimeoutOk) {
            transitionAllToKinetic();
        }
    } else {
        spinSustainedVelocityCycles = 0;
    }
}


void WheelController::transitionAllToKinetic() {
    for (int i = 0; i < 4; i++) {
        controllers[i].forceKinetic();
    }
}

void WheelController::forceBlocked() {
    for (int i = 0; i < 4; i++) {
        controllers[i].forceBlocked();
    }
}

void WheelController::setCandidateRecoveryAssist(bool enabled) {
    for (int i = 0; i < 4; i++) {
        controllers[i].setCandidateRecoveryAssist(enabled);
    }
}

void WheelController::reset() {
    openLoopActive = false;
    lastOpenLoopPwmMs = 0;
    spinSustainedVelocityCycles = 0;
    lastSpinSyncTrimLeft = 0;
    lastSpinSyncTrimRight = 0;
    pairErrorLeft = 0.0f;
    pairErrorRight = 0.0f;
    prevTargetLeft = 0.0f;
    prevTargetRight = 0.0f;
    for (int i = 0; i < 4; i++) {
        openLoopPwms[i] = 0;
        controllers[i].reset();
    }
}
