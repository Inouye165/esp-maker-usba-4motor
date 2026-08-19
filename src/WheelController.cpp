#include "WheelController.h"
#include "RoverConfig.h"
#include "MotorDriver.h"

SingleWheelController::SingleWheelController()
    : index(0)
    , targetVel(0.0f)
    , measuredVel(0.0f)
    , isSpinManeuver(false)
    , isForwardRearWheel(false)
    , errorSum(0.0f)
    , lastError(0.0f)
    , Kp(KP_SPEED)
    , Ki(KI_SPEED)
    , Kd(KD_SPEED)
    , lastPwm(0)
    , reversalDeadtimeTicks(0)
    , stictionState(STICTION_IDLE)
    , boostStartTicks(0)
    , boostTicksCount(0)
    , boostTicksInitialized(false) {}

void SingleWheelController::begin(int motorIndex) {
    index = motorIndex;
    reset();
}

void SingleWheelController::setGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void SingleWheelController::setTargetVelocity(float targetRadps, bool isSpin, bool isForwardRear) {
    isSpinManeuver = isSpin;
    isForwardRearWheel = isForwardRear;
    // Transition from near-zero OR from STICTION_IDLE to non-zero: enter STICTION_BOOST
    if ((stictionState == STICTION_IDLE || abs(targetVel) < 0.01f) && abs(targetRadps) >= 0.01f) {
        stictionState = STICTION_BOOST;
        boostTicksCount = 0;
        boostTicksInitialized = false;
        errorSum = 0.0f;
        lastError = 0.0f;
    }
    // Direction change across zero while active: trigger boost in new direction
    else if ((targetVel > 0.01f && targetRadps < -0.01f) || (targetVel < -0.01f && targetRadps > 0.01f)) {
        stictionState = STICTION_BOOST;
        boostTicksCount = 0;
        boostTicksInitialized = false;
        errorSum = 0.0f;
        lastError = 0.0f;
    }
    // Transition back to near-zero: reset to IDLE
    else if (abs(targetRadps) < 0.01f) {
        stictionState = STICTION_IDLE;
        boostTicksCount = 0;
        boostTicksInitialized = false;
        errorSum = 0.0f;
        lastError = 0.0f;
    }
    targetVel = targetRadps;
}

int SingleWheelController::update(float measuredRadps, int32_t currentTicks, float dt) {
    measuredVel = measuredRadps;
    
    // Target near-zero handling: decay output and reset integrators
    if (abs(targetVel) < 0.01f) {
        stictionState = STICTION_IDLE;
        errorSum = 0.0f;
        lastError = 0.0f;
        lastPwm = 0;
        boostTicksCount = 0;
        boostTicksInitialized = false;
        diag.targetVel = 0.0f;
        diag.measuredVel = measuredRadps;
        diag.feedforward = 0.0f;
        diag.pTerm = 0.0f;
        diag.iTerm = 0.0f;
        diag.dTerm = 0.0f;
        diag.finalPwm = 0;
        diag.stictionState = STICTION_IDLE;
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
            
            // While in boost: command boost PWM in target direction (48 for normal)
            int boostMag = STICTION_BOOST_PWM;
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
                if (boostTicksCount >= 50) { // 500 ms safety limit @ 100 Hz
                    stictionState = STICTION_BLOCKED;
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
    
    // STICTION_KINETIC: Closed-loop PID + Feedforward structure
    float error = targetVel - measuredRadps;
    errorSum += error * dt;
    
    // Integral anti-windup: clamp the integral contribution
    float integralTerm = errorSum * Ki;
    integralTerm = constrain(integralTerm, -150.0f, 150.0f);
    
    // Derivative term
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    // Breakaway friction and velocity feedforward magnitude
    float breakaway;
    if (isSpinManeuver) {
        breakaway = SPIN_KINETIC_KS_PWM; // Dedicated pure-spin kinetic feedforward base (75.0 PWM)
    } else {
        breakaway = (targetVel > 0.0f) ? (float)motorCalibrations[index].forwardBreakawayPwm
                                       : (float)motorCalibrations[index].reverseBreakawayPwm;
    }
    float ffMagnitude = breakaway + (motorCalibrations[index].kV * abs(targetVel));

    
    // In KINETIC state with non-zero target, apply minimum feedforward floor
    if (stictionState == STICTION_KINETIC && abs(targetVel) >= 0.01f) {
        float minFfFloor;
        if (isSpinManeuver) {
            // Forward-driving rear wheel receives 94 PWM floor; others receive 80 PWM floor
            minFfFloor = isForwardRearWheel ? SPIN_FORWARD_REAR_KINETIC_FLOOR : MIN_SPIN_KINETIC_FF_FLOOR;
        } else {
            minFfFloor = 45.0f;
        }
        if (ffMagnitude < minFfFloor) {
            ffMagnitude = minFfFloor;
        }
    }
    
    float feedforward = (targetVel > 0.0f ? 1.0f : -1.0f) * ffMagnitude;
    
    // PID Correction (pure spin selects SPIN_PID_KP = 6.0, normal driving retains Kp = 2.2)
    float activeKp = isSpinManeuver ? SPIN_PID_KP : Kp;
    float pidCorrection = (activeKp * error) + integralTerm + (Kd * derivative);
    
    // Combine PID correction and Feedforward
    float totalPwm = feedforward + pidCorrection;
    int targetPwm = constrain((int)round(totalPwm), -255, 255);
    
    // Populate diagnostic telemetry (read-only, non-interfering)
    diag.targetVel = targetVel;
    diag.measuredVel = measuredRadps;
    diag.feedforward = feedforward;
    diag.pTerm = activeKp * error;
    diag.iTerm = integralTerm;
    diag.dTerm = Kd * derivative;

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

void SingleWheelController::updateOpenLoop(float measuredRadps, int pwm) {
    measuredVel = measuredRadps;
    lastPwm = pwm;
    stictionState = STICTION_KINETIC;
    diag.targetVel = 0.0f;
    diag.measuredVel = measuredRadps;
    diag.feedforward = (float)pwm;
    diag.pTerm = 0.0f;
    diag.iTerm = 0.0f;
    diag.dTerm = 0.0f;
    diag.finalPwm = (int16_t)pwm;
    diag.stictionState = STICTION_KINETIC;
}

void SingleWheelController::reset() {
    errorSum = 0.0f;
    lastError = 0.0f;
    lastPwm = 0;
    reversalDeadtimeTicks = 0;
    stictionState = STICTION_IDLE;
    boostStartTicks = 0;
    boostTicksCount = 0;
    boostTicksInitialized = false;
    memset(&diag, 0, sizeof(diag));
}

WheelController::WheelController() 
    : openLoopActive(false)
    , spinSustainedVelocityCycles(0) {
    for (int i = 0; i < 4; i++) openLoopPwms[i] = 0;
}

void WheelController::begin() {
    openLoopActive = false;
    spinSustainedVelocityCycles = 0;
    for (int i = 0; i < 4; i++) {
        openLoopPwms[i] = 0;
        controllers[i].begin(i);
    }
}

void WheelController::setTargets(float leftTargetRadps, float rightTargetRadps, bool isSpin) {
    openLoopActive = false;
    
    // Canonical mapping: Index 0=M1/LF, Index 1=M2/RF, Index 2=M3/LR, Index 3=M4/RR
    // In pure spin:
    // CCW spin (left target < 0, right target > 0): M4/RR (Index 3) is forward-driving rear wheel
    // CW spin (left target > 0, right target < 0):  M3/LR (Index 2) is forward-driving rear wheel
    bool m1_isForwardRear = false;
    bool m2_isForwardRear = false;
    bool m3_isForwardRear = isSpin && (leftTargetRadps > 0.01f);
    bool m4_isForwardRear = isSpin && (rightTargetRadps > 0.01f);

    controllers[0].setTargetVelocity(leftTargetRadps, isSpin, m1_isForwardRear);
    controllers[1].setTargetVelocity(rightTargetRadps, isSpin, m2_isForwardRear);
    controllers[2].setTargetVelocity(leftTargetRadps, isSpin, m3_isForwardRear);
    controllers[3].setTargetVelocity(rightTargetRadps, isSpin, m4_isForwardRear);
}

void WheelController::setWheelTarget(int index, float targetRadps, bool isSpin) {
    openLoopActive = false;
    if (index >= 0 && index < 4) {
        bool isForwardRear = isSpin && (index == 2 || index == 3) && (targetRadps > 0.01f);
        controllers[index].setTargetVelocity(targetRadps, isSpin, isForwardRear);
    }
}

void WheelController::setOpenLoopPwm(int m1, int m2, int m3, int m4) {
    openLoopActive = true;
    openLoopPwms[0] = m1;
    openLoopPwms[1] = m2;
    openLoopPwms[2] = m3;
    openLoopPwms[3] = m4;
}

void WheelController::clearOpenLoop() {
    openLoopActive = false;
    for (int i = 0; i < 4; i++) {
        openLoopPwms[i] = 0;
    }
}

void WheelController::update(const float *measuredVelocities, const int32_t *ticks, float dt, MotorDriver &driver) {
    if (openLoopActive) {
        for (int i = 0; i < 4; i++) {
            driver.setPWM(i, openLoopPwms[i]);
            controllers[i].updateOpenLoop(measuredVelocities[i], openLoopPwms[i]);
        }
        return;
    }

    // 1. Update single wheel controllers first to get current boostTicksCount & state
    for (int i = 0; i < 4; i++) {
        int pwmOutput = controllers[i].update(measuredVelocities[i], ticks[i], dt);
        driver.setPWM(i, pwmOutput);
    }
    
    // 2. Coordinated Rear-Pair Breakout Guard for Pure Spin:
    bool anySpinBoost = false;
    for (int i = 0; i < 4; i++) {
        if (controllers[i].getStictionState() == STICTION_BOOST) {
            anySpinBoost = true;
            break;
        }
    }
    
    if (anySpinBoost) {
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

void WheelController::reset() {
    openLoopActive = false;
    spinSustainedVelocityCycles = 0;
    for (int i = 0; i < 4; i++) {
        openLoopPwms[i] = 0;
        controllers[i].reset();
    }
}

