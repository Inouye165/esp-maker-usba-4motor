#include "DifferentialDrive.h"
#include "RoverConfig.h"

DifferentialDrive::DifferentialDrive() {}

void DifferentialDrive::velocityToWheels(float linear, float angular, float &leftRadps, float &rightRadps) {
    float leftVelMps = linear - (angular * WHEEL_SEPARATION_M / 2.0f);
    float rightVelMps = linear + (angular * WHEEL_SEPARATION_M / 2.0f);
    
    leftRadps = leftVelMps / WHEEL_RADIUS_M;
    rightRadps = rightVelMps / WHEEL_RADIUS_M;
}

void DifferentialDrive::wheelsToVelocity(float w1_radps, float w2_radps, float w3_radps, float w4_radps, float &linear, float &angular) {
    // Average left and right wheel velocities in rad/s
    float avgLeftRadps = (w1_radps + w3_radps) / 2.0f;
    float avgRightRadps = (w2_radps + w4_radps) / 2.0f;
    
    // Convert to linear wheel velocities (m/s)
    float leftVelMps = avgLeftRadps * WHEEL_RADIUS_M;
    float rightVelMps = avgRightRadps * WHEEL_RADIUS_M;
    
    // Chassis linear and angular velocities
    linear = (leftVelMps + rightVelMps) / 2.0f;
    angular = (rightVelMps - leftVelMps) / WHEEL_SEPARATION_M;
}

void DifferentialDrive::calculateOdometryStep(float w1_radps, float w2_radps, float w3_radps, float w4_radps, float dt, float &deltaDist, float &deltaYaw) {
    if (dt <= 0.0f) {
        deltaDist = 0.0f;
        deltaYaw = 0.0f;
        return;
    }
    
    float linearVel = 0.0f;
    float angularVel = 0.0f;
    
    wheelsToVelocity(w1_radps, w2_radps, w3_radps, w4_radps, linearVel, angularVel);
    
    deltaDist = linearVel * dt;
    deltaYaw = angularVel * dt;
}
