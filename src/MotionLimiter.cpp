#include "MotionLimiter.h"
#include "RoverConfig.h"

static float getSign(float val) {
    if (val > 0.001f) return 1.0f;
    if (val < -0.001f) return -1.0f;
    return 0.0f;
}

SCurveLimiter::SCurveLimiter(float maxVel, float maxAcc, float maxDec, float maxJerk)
    : maxVelocity(maxVel)
    , maxAcceleration(maxAcc)
    , maxDeceleration(maxDec)
    , maxJerk(maxJerk)
    , currentVel(0.0f)
    , currentAcc(0.0f) {}

void SCurveLimiter::reset(float initialVel) {
    currentVel = constrain(initialVel, -maxVelocity, maxVelocity);
    currentAcc = 0.0f;
}

float SCurveLimiter::update(float targetVel, float dt) {
    // 1. Constrain requested target to maximum configured velocity limit
    float target = constrain(targetVel, -maxVelocity, maxVelocity);
    
    // 2. Safe Direction Reversals: force target to zero if we need to cross signs
    if (currentVel > 0.005f && target < 0.0f) {
        target = 0.0f;
    } else if (currentVel < -0.005f && target > 0.0f) {
        target = 0.0f;
    }
    
    // 3. Determine if accelerating or decelerating
    bool decelerating = false;
    if (target == 0.0f || (abs(target) < abs(currentVel) && getSign(target) == getSign(currentVel))) {
        decelerating = true;
    }
    
    float accLimit = decelerating ? maxDeceleration : maxAcceleration;
    
    // 4. Calculate desired acceleration to reach the target velocity in one step
    float desiredAcc = (target - currentVel) / dt;
    desiredAcc = constrain(desiredAcc, -accLimit, accLimit);
    
    // 5. Calculate desired jerk to reach the desired acceleration in one step
    float desiredJerk = (desiredAcc - currentAcc) / dt;
    desiredJerk = constrain(desiredJerk, -maxJerk, maxJerk);
    
    // 6. Integrate jerk to get acceleration
    currentAcc += desiredJerk * dt;
    currentAcc = constrain(currentAcc, -accLimit, accLimit);
    
    // 7. Integrate acceleration to get velocity
    currentVel += currentAcc * dt;
    
    // 8. Prevent overshoot and handle zero-clamping
    if (target >= 0.0f) {
        if (currentVel > target) {
            currentVel = target;
            currentAcc = 0.0f;
        }
    } else {
        if (currentVel < target) {
            currentVel = target;
            currentAcc = 0.0f;
        }
    }
    
    if (target == 0.0f && abs(currentVel) < 0.005f) {
        currentVel = 0.0f;
        currentAcc = 0.0f;
    }
    
    return currentVel;
}

MotionLimiter::MotionLimiter()
    : linearLimiter(MAX_LINEAR_VELOCITY_MPS, MAX_LINEAR_ACCEL_MPS2, MAX_LINEAR_DECEL_MPS2, MAX_LINEAR_JERK_MPS3)
    , angularLimiter(MAX_ANGULAR_VELOCITY_RADPS, MAX_ANGULAR_ACCEL_RADPS2, MAX_ANGULAR_DECEL_RADPS2, MAX_ANGULAR_JERK_RADPS3) {}

void MotionLimiter::begin() {
    linearLimiter.reset(0.0f);
    angularLimiter.reset(0.0f);
}

void MotionLimiter::reset(float linearVel, float angularVel) {
    linearLimiter.reset(linearVel);
    angularLimiter.reset(angularVel);
}

void MotionLimiter::update(float targetLinear, float targetAngular, float dt) {
    linearLimiter.update(targetLinear, dt);
    angularLimiter.update(targetAngular, dt);
}
