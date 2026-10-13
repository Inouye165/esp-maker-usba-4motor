#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <Arduino.h>

class DifferentialDrive {
public:
    DifferentialDrive();
    
    // Convert chassis target linear (m/s) and angular (rad/s) velocity to left and right wheel velocities (rad/s)
    static void velocityToWheels(float linear, float angular, float &leftRadps, float &rightRadps);
    
    // Convert individual wheel velocities (rad/s) back to chassis linear (m/s) and angular (rad/s) velocity
    static void wheelsToVelocity(float w1_radps, float w2_radps, float w3_radps, float w4_radps, float &linear, float &angular);
    
    // Calculate displacement and yaw change during sample period dt (in seconds)
    static void calculateOdometryStep(float w1_radps, float w2_radps, float w3_radps, float w4_radps, float dt, float &deltaDist, float &deltaYaw);
};

#endif // DIFFERENTIAL_DRIVE_H
