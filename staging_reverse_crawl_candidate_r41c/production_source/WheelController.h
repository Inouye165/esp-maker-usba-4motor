#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include <Arduino.h>
#include "RoverConfig.h"

enum StictionState : int16_t {
    STICTION_IDLE = 0,
    STICTION_BOOST = 1,
    STICTION_KINETIC = 2,
    STICTION_BLOCKED = 3
};

struct WheelPidDiag {
    float targetVel;
    float measuredVel;
    float feedforward;
    float pTerm;
    float iTerm;
    float dTerm;
    int16_t basePwm;
    int16_t spinSyncTrim;
    int16_t finalPwm;
    int16_t stictionState;
    float recoveryAssist;
};

enum UltraLowSpeedMode : uint8_t {
    ULTRA_LOW_MODE_BASELINE = 0,       // Current firmware
    ULTRA_LOW_MODE_GAINS_ONLY = 1,      // Candidate 2: Ultra-low gains & 2x overspeed unwinding (56 FF)
    ULTRA_LOW_MODE_GAINS_AND_FF = 2     // Candidate 3: Scaled FF (46-56 PWM) + Ultra-low gains
};

class SingleWheelController {
public:
    SingleWheelController();
    void begin(int motorIndex);
    
    void setGains(float kp, float ki, float kd);
    void setTargetVelocity(float targetRadps, bool isSpin = false, bool isForwardRear = false, bool isOpposing = false, bool isTurning = false);
    
    // Update PID and return output PWM value (-255 to 255)
    int update(float measuredRadps, int32_t currentTicks, float dt);
    void setDiagPwmOutputs(int basePwm, int trim, int finalPwm);
    
    void reset();
    
    float getTarget() const { return targetVel; }
    float getMeasured() const { return measuredVel; }
    float getError() const { return lastError; }
    int getPwmOutput() const { return lastPwm; }
    const WheelPidDiag& getDiag() const { return diag; }
    StictionState getStictionState() const { return stictionState; }
    bool isForwardRear() const { return isForwardRearWheel; }
    bool isOpposing() const { return isOpposingManeuver; }
    bool isTurning() const { return isTurningArc; }
    uint8_t getBlockedRetriesInEpisode() const { return blockedRetriesInEpisode; }
    uint16_t getBoostTicksCount() const { return boostTicksCount; }
    void forceKinetic() { stictionState = STICTION_KINETIC; }
    void forceBlocked() {
        stictionState = STICTION_BLOCKED;
        lastPwm = 0;
        blockedRetriesInEpisode = 1;
        recoveryAssist = 0.0f;
        stallDebounceTicks = 0;
    }
    void updateOpenLoop(float measuredRadps, int pwm);
    
    // Candidate Separate Recovery-Assist Controls
    float getRecoveryAssist() const { return recoveryAssist; }
    void setCandidateRecoveryAssist(bool enabled) {
        candidateRecoveryAssist = enabled;
        if (!enabled) {
            recoveryAssist = 0.0f;
            stallDebounceTicks = 0;
        }
    }
    bool isCandidateRecoveryAssist() const { return candidateRecoveryAssist; }

    // Ultra-Low Speed Crawl Controls (Scoped below 0.80 rad/s)
    void setUltraLowSpeedMode(UltraLowSpeedMode mode) { ultraLowSpeedMode = mode; }
    UltraLowSpeedMode getUltraLowSpeedMode() const { return ultraLowSpeedMode; }

private:
    UltraLowSpeedMode ultraLowSpeedMode;
    int index;
    float targetVel;
    float measuredVel;
    bool isSpinManeuver;
    bool isForwardRearWheel;
    bool isOpposingManeuver;
    bool isTurningArc;
    uint8_t blockedRetriesInEpisode;
    uint8_t zeroDebounceTicks;
    uint8_t stallDebounceTicks;
    
    float integralTerm;
    float recoveryAssist;
    bool candidateRecoveryAssist;
    float lastError;
    
    float Kp;
    float Ki;
    float Kd;
    
    int lastPwm;
    int reversalDeadtimeTicks;
    WheelPidDiag diag;

    // Dynamic Stiction State Machine
    StictionState stictionState;
    int32_t boostStartTicks;
    uint16_t boostTicksCount;
    bool boostTicksInitialized;
};

class WheelController {
public:
    WheelController();
    void begin();
    
    // Set target velocities for left and right tracks (rad/s)
    void setTargets(float leftTargetRadps, float rightTargetRadps, bool isSpin = false, bool isOpposing = false, bool isTurning = false);
    
    // Set target velocity for a single wheel (rad/s)
    void setWheelTarget(int index, float targetRadps, bool isSpin = false, bool isOpposing = false, bool isTurning = false);
    
    // Direct open-loop PWM test mode with timeout expiry
    typedef void (*TimeoutDisarmHook)();
    void setTimeoutDisarmHook(TimeoutDisarmHook hook) { timeoutDisarmHook = hook; }
    void setOpenLoopPwm(int m1, int m2, int m3, int m4, uint32_t nowMs = 0);
    void clearOpenLoop();
    bool isOpenLoop() const { return openLoopActive; }
    uint32_t getLastOpenLoopPwmMs() const { return lastOpenLoopPwmMs; }
    
    // Update all 4 wheel speed PID loops and write outputs to MotorDriver
    void update(const float *measuredVelocities, const int32_t *ticks, float dt, class MotorDriver &driver, bool isSpinManeuver, bool isGoingStraight, bool encodersFresh, uint32_t nowMs = 0);
    
    // Get single controller references
    const SingleWheelController& getController(int index) const { return controllers[index]; }
    
    int getSpinSyncTrimLeft() const { return lastSpinSyncTrimLeft; }
    int getSpinSyncTrimRight() const { return lastSpinSyncTrimRight; }
    float getPairErrorLeft() const { return pairErrorLeft; }
    float getPairErrorRight() const { return pairErrorRight; }

    // Coordinated transition of all wheels to STICTION_KINETIC
    void transitionAllToKinetic();
    void forceBlocked();
    void setCandidateRecoveryAssist(bool enabled);
    void setUltraLowSpeedMode(UltraLowSpeedMode mode) {
        for (int i = 0; i < 4; i++) {
            controllers[i].setUltraLowSpeedMode(mode);
        }
    }
    
    void reset();

private:
    SingleWheelController controllers[4];
    bool openLoopActive;
    uint32_t lastOpenLoopPwmMs;
    int openLoopPwms[4];
    TimeoutDisarmHook timeoutDisarmHook;
    uint16_t spinSustainedVelocityCycles;
    int lastSpinSyncTrimLeft;
    int lastSpinSyncTrimRight;
    float pairErrorLeft;
    float pairErrorRight;
    float prevTargetLeft;
    float prevTargetRight;
};

#endif // WHEEL_CONTROLLER_H
