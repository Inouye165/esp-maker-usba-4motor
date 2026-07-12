#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include <Arduino.h>

class SingleWheelController {
public:
    SingleWheelController();
    void begin(int motorIndex);
    
    void setGains(float kp, float ki, float kd);
    void setTargetVelocity(float targetRadps);
    
    // Update PID and return output PWM value (-255 to 255)
    int update(float measuredRadps, float dt);
    
    void reset();
    
    float getTarget() const { return targetVel; }
    float getMeasured() const { return measuredVel; }
    float getError() const { return lastError; }
    int getPwmOutput() const { return lastPwm; }

private:
    int index;
    float targetVel;
    float measuredVel;
    
    float errorSum;
    float lastError;
    
    float Kp;
    float Ki;
    float Kd;
    
    int lastPwm;
    int reversalDeadtimeTicks;
};

class WheelController {
public:
    WheelController();
    void begin();
    
    // Set target velocities for left and right tracks (rad/s)
    void setTargets(float leftTargetRadps, float rightTargetRadps);
    
    // Set target velocity for a single wheel (rad/s)
    void setWheelTarget(int index, float targetRadps);
    
    // Update all 4 wheel speed PID loops and write outputs to MotorDriver
    // measuredVelocities: array of 4 rad/s velocities from EncoderManager
    // dt: loop duration in seconds
    void update(const float *measuredVelocities, float dt, class MotorDriver &driver);
    
    // Get single controller references
    const SingleWheelController& getController(int index) const { return controllers[index]; }
    
    void reset();

private:
    SingleWheelController controllers[4];
};

#endif // WHEEL_CONTROLLER_H
