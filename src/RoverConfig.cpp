#include "RoverConfig.h"
#include <Preferences.h>

Preferences preferences;

// Default calibrations:
// SS6625E motor driver board on the Maker Pro generally starts rotating wheels around 40-60 PWM
MotorCalibration motorCalibrations[4] = {
    {45, 45, 12.0f}, // M1
    {45, 45, 12.0f}, // M2
    {45, 45, 12.0f}, // M3
    {45, 45, 12.0f}  // M4
};

void initConfigStorage() {
    // Preferences automatically creates the namespace if it doesn't exist
    preferences.begin("rover-config", false);
    loadCalibrations();
}

void loadCalibrations() {
    // Force uniform breakaway defaults (45 PWM) to prevent asymmetric floor scrubbing
    for (int i = 0; i < 4; i++) {
        motorCalibrations[i].forwardBreakawayPwm = 45;
        motorCalibrations[i].reverseBreakawayPwm = 45;
        motorCalibrations[i].kV = 12.0f;
    }
    
    Serial.println("[Config] Forced uniform breakaway parameters (45 PWM):");
    for (int i = 0; i < 4; i++) {
        Serial.printf("  Motor %d: FWD=%d, REV=%d, kV=%.2f\n", 
            i + 1, motorCalibrations[i].forwardBreakawayPwm, 
            motorCalibrations[i].reverseBreakawayPwm, motorCalibrations[i].kV);
    }
}

void saveCalibrations() {
    // Write M1
    preferences.putInt("m1_fwd_break", motorCalibrations[0].forwardBreakawayPwm);
    preferences.putInt("m1_rev_break", motorCalibrations[0].reverseBreakawayPwm);
    preferences.putFloat("m1_kv", motorCalibrations[0].kV);

    // Write M2
    preferences.putInt("m2_fwd_break", motorCalibrations[1].forwardBreakawayPwm);
    preferences.putInt("m2_rev_break", motorCalibrations[1].reverseBreakawayPwm);
    preferences.putFloat("m2_kv", motorCalibrations[1].kV);

    // Write M3
    preferences.putInt("m3_fwd_break", motorCalibrations[2].forwardBreakawayPwm);
    preferences.putInt("m3_rev_break", motorCalibrations[2].reverseBreakawayPwm);
    preferences.putFloat("m3_kv", motorCalibrations[2].kV);

    // Write M4
    preferences.putInt("m4_fwd_break", motorCalibrations[3].forwardBreakawayPwm);
    preferences.putInt("m4_rev_break", motorCalibrations[3].reverseBreakawayPwm);
    preferences.putFloat("m4_kv", motorCalibrations[3].kV);

    Serial.println("[Config] Saved breakaway parameters to NVS.");
}
