#ifndef MOTION_LIMITER_H
#define MOTION_LIMITER_H

#include <Arduino.h>

class SCurveLimiter {
public:
    SCurveLimiter(float maxVel, float maxAcc, float maxDec, float maxJerk);
    void reset(float initialVel);
    float update(float targetVel, float dt);
    float getVelocity() const { return currentVel; }
    float getAcceleration() const { return currentAcc; }

private:
    float maxVelocity;
    float maxAcceleration;
    float maxDeceleration;
    float maxJerk;

    float currentVel;
    float currentAcc;
};

class MotionLimiter {
public:
    MotionLimiter();
    void begin();
    
    // Reset internal state to initial values (used for smooth handoff)
    void reset(float linearVel, float angularVel);
    
    // Update limiter with new targets
    void update(float targetLinear, float targetAngular, float dt);
    
    float getLinearVelocity() const { return linearLimiter.getVelocity(); }
    float getAngularVelocity() const { return angularLimiter.getVelocity(); }
    
    float getLinearAcceleration() const { return linearLimiter.getAcceleration(); }
    float getAngularAcceleration() const { return angularLimiter.getAcceleration(); }

private:
    SCurveLimiter linearLimiter;
    SCurveLimiter angularLimiter;
};

#endif // MOTION_LIMITER_H
