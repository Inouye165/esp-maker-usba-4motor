#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <Arduino.h>
#include <ESP32Encoder.h>

class EncoderManager {
public:
    EncoderManager();
    void begin();
    
    // Read and update encoder counts, positions, and velocities
    // dt: loop sample time in seconds
    void update(float dt);
    
    // Get cumulative tick counts
    int32_t getTicks(int index) const { return (index >= 0 && index < 4) ? currentTicks[index] : 0; }
    
    // Get cumulative wheel position in radians
    float getPosition(int index) const { return (index >= 0 && index < 4) ? wheelPositions[index] : 0.0f; }
    
    // Get filtered wheel velocity in rad/s
    float getVelocity(int index) const { return (index >= 0 && index < 4) ? wheelVelocities[index] : 0.0f; }
    
    // Reset all counts and positions to zero
    void reset();

private:
    ESP32Encoder encoders[4];
    
    int32_t currentTicks[4];
    int32_t prevTicks[4];
    
    float wheelPositions[4];    // in radians
    float wheelVelocities[4];   // in rad/s (filtered)
    
    const float velocityFilterAlpha = 0.35f; // Low-pass filter coefficient
};

#endif // ENCODER_MANAGER_H
