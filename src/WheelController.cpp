#include "WheelController.h"
#include "RoverConfig.h"
#include "MotorDriver.h"

SingleWheelController::SingleWheelController()
    : index(0)
    , targetVel(0.0f)
    , measuredVel(0.0f)
    , errorSum(0.0f)
    , lastError(0.0f)
    , Kp(KP_SPEED)
    , Ki(KI_SPEED)
    , Kd(KD_SPEED)
    , lastPwm(0)
    , reversalDeadtimeTicks(0) {}

void SingleWheelController::begin(int motorIndex) {
    index = motorIndex;
    reset();
}

void SingleWheelController::setGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void SingleWheelController::setTargetVelocity(float targetRadps) {
    targetVel = targetRadps;
}

int SingleWheelController::update(float measuredRadps, float dt) {
    measuredVel = measuredRadps;
    
    // Target near-zero handling: decay output and reset integrators
    if (abs(targetVel) < 0.01f) {
        errorSum = 0.0f;
        lastError = 0.0f;
        lastPwm = 0;
        return 0;
    }
    
    float error = targetVel - measuredRadps;
    errorSum += error * dt;
    
    // Integral anti-windup: clamp the integral contribution
    float integralTerm = errorSum * Ki;
    integralTerm = constrain(integralTerm, -150.0f, 150.0f);
    
    // Derivative term
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    // Breakaway friction and velocity feedforward
    float kS = 0.0f;
    if (targetVel > 0.0f) {
        kS = (float)motorCalibrations[index].forwardBreakawayPwm;
    } else {
        kS = -(float)motorCalibrations[index].reverseBreakawayPwm;
    }
    
    float feedforward = kS + (motorCalibrations[index].kV * targetVel);
    
    // PID Correction
    float pidCorrection = (Kp * error) + integralTerm + (Kd * derivative);
    
    // Combine PID correction and Feedforward
    float totalPwm = feedforward + pidCorrection;
    int targetPwm = constrain((int)totalPwm, -255, 255);
    
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

void SingleWheelController::reset() {
    errorSum = 0.0f;
    lastError = 0.0f;
    lastPwm = 0;
    reversalDeadtimeTicks = 0;
}

WheelController::WheelController() {}

void WheelController::begin() {
    for (int i = 0; i < 4; i++) {
        controllers[i].begin(i);
    }
}

void WheelController::setTargets(float leftTargetRadps, float rightTargetRadps) {
    // M1 (0) & M3 (2) are left side
    controllers[0].setTargetVelocity(leftTargetRadps);
    controllers[2].setTargetVelocity(leftTargetRadps);
    
    // M2 (1) & M4 (3) are right side
    controllers[1].setTargetVelocity(rightTargetRadps);
    controllers[3].setTargetVelocity(rightTargetRadps);
}

void WheelController::setWheelTarget(int index, float targetRadps) {
    if (index >= 0 && index < 4) {
        controllers[index].setTargetVelocity(targetRadps);
    }
}

void WheelController::update(const float *measuredVelocities, float dt, MotorDriver &driver) {
    for (int i = 0; i < 4; i++) {
        int pwmOutput = controllers[i].update(measuredVelocities[i], dt);
        driver.setPWM(i, pwmOutput);
    }
}

void WheelController::reset() {
    for (int i = 0; i < 4; i++) {
        controllers[i].reset();
    }
}
