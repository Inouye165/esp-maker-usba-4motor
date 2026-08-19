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
    int16_t finalPwm;
    int16_t stictionState;
};

class SingleWheelController {
public:
    SingleWheelController();
    void begin(int motorIndex);
    
    void setGains(float kp, float ki, float kd);
    void setTargetVelocity(float targetRadps, bool isSpin = false, bool isForwardRear = false);
    
    // Update PID and return output PWM value (-255 to 255)
    int update(float measuredRadps, int32_t currentTicks, float dt);
    
    void reset();
    
    float getTarget() const { return targetVel; }
    float getMeasured() const { return measuredVel; }
    float getError() const { return lastError; }
    int getPwmOutput() const { return lastPwm; }
    const WheelPidDiag& getDiag() const { return diag; }
    StictionState getStictionState() const { return stictionState; }
    bool isForwardRear() const { return isForwardRearWheel; }
    uint16_t getBoostTicksCount() const { return boostTicksCount; }
    void forceKinetic() { stictionState = STICTION_KINETIC; }
    void updateOpenLoop(float measuredRadps, int pwm);

private:
    int index;
    float targetVel;
    float measuredVel;
    bool isSpinManeuver;
    bool isForwardRearWheel;
    
    float errorSum;
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
    void setTargets(float leftTargetRadps, float rightTargetRadps, bool isSpin = false);
    
    // Set target velocity for a single wheel (rad/s)
    void setWheelTarget(int index, float targetRadps, bool isSpin = false);
    
    // Direct open-loop PWM test mode
    void setOpenLoopPwm(int m1, int m2, int m3, int m4);
    void clearOpenLoop();
    bool isOpenLoop() const { return openLoopActive; }
    
    // Update all 4 wheel speed PID loops and write outputs to MotorDriver
    // measuredVelocities: array of 4 rad/s velocities from EncoderManager
    // ticks: array of 4 raw cumulative tick counts from EncoderManager
    // dt: loop duration in seconds
    void update(const float *measuredVelocities, const int32_t *ticks, float dt, class MotorDriver &driver);
    
    // Get single controller references
    const SingleWheelController& getController(int index) const { return controllers[index]; }
    
    // Coordinated transition of all wheels to STICTION_KINETIC
    void transitionAllToKinetic();
    
    void reset();

private:
    SingleWheelController controllers[4];
    bool openLoopActive;
    int openLoopPwms[4];
    uint16_t spinSustainedVelocityCycles;
};

#endif // WHEEL_CONTROLLER_H
