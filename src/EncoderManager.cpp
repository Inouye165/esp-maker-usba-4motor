#include "EncoderManager.h"
#include "RoverConfig.h"

EncoderManager::EncoderManager() {
    for (int i = 0; i < 4; i++) {
        currentTicks[i] = 0;
        prevTicks[i] = 0;
        wheelPositions[i] = 0.0f;
        wheelVelocities[i] = 0.0f;
    }
}

void EncoderManager::begin() {
    // Configure ESP32Encoder options
    ESP32Encoder::useInternalWeakPullResistors = UP;

    // Attach hardware pins to quadrature decoders.
    // Chassis rebuild 2026-10-18: logical index -> board port follows MotorDriver
    // (LF->port 2, RF->port 1, LR->port 4, RR->port 3). Count-direction corrections
    // are applied in update() per verified raised-wheel tests.
    // Logical 0 (Left Front)  -> encoder port E2
    encoders[0].attachFullQuad(E2_A, E2_B);
    encoders[0].setFilter(1023); // Noise glitch filter (max 1023)

    // Logical 1 (Right Front) -> encoder port E1
    encoders[1].attachFullQuad(E1_A, E1_B);
    encoders[1].setFilter(1023);

    // Logical 2 (Left Rear)   -> encoder port E4 (swapped pins per original board layout)
    encoders[2].attachFullQuad(E4_B, E4_A);
    encoders[2].setFilter(1023);

    // Logical 3 (Right Rear)  -> encoder port E3
    encoders[3].attachFullQuad(E3_A, E3_B);
    encoders[3].setFilter(1023);

    reset();
}

void EncoderManager::update(float dt) {
    if (dt <= 0.0f) return;
    lastUpdateMs = millis();
    
    // Read raw counts from hardware counters
    // Mirrored motor mounting: every port's count direction is flipped relative to
    // the original chassis (port 2 was already inverted, so it is now non-inverted).
    int32_t raw0 = (int32_t)encoders[0].getCount();   // LF on port E2
    int32_t raw1 = -(int32_t)encoders[1].getCount();  // RF on port E1
    int32_t raw2 = -(int32_t)encoders[2].getCount();  // LR on port E4
    int32_t raw3 = -(int32_t)encoders[3].getCount();  // RR on port E3
    
    currentTicks[0] = raw0;
    currentTicks[1] = raw1;
    currentTicks[2] = raw2;
    currentTicks[3] = raw3;
    
    for (int i = 0; i < 4; i++) {
        // Calculate delta ticks
        int32_t deltaTicks = currentTicks[i] - prevTicks[i];
        prevTicks[i] = currentTicks[i];
        
        // Cumulative position in radians
        wheelPositions[i] = (currentTicks[i] / TICKS_PER_WHEEL_REV) * (2.0f * PI);
        
        // Instantaneous raw velocity in rad/s
        float rawVel = ((float)deltaTicks / TICKS_PER_WHEEL_REV) * (2.0f * PI) / dt;
        
        // Low-pass noise filtering
        wheelVelocities[i] = (velocityFilterAlpha * rawVel) + ((1.0f - velocityFilterAlpha) * wheelVelocities[i]);
    }
}

void EncoderManager::reset() {
    lastUpdateMs = 0;
    for (int i = 0; i < 4; i++) {
        encoders[i].clearCount();
        currentTicks[i] = 0;
        prevTicks[i] = 0;
        wheelPositions[i] = 0.0f;
        wheelVelocities[i] = 0.0f;
    }
}
