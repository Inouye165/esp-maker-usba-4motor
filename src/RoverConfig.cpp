#include "RoverConfig.h"
#include <Preferences.h>

Preferences preferences;

// Define mutable physical parameters
float WHEEL_DIAMETER_M = 0.065f;
float WHEEL_RADIUS_M = 0.0325f;
float WHEEL_SEPARATION_M = 0.197f; // Geometric baseline: 7.75 inches = 0.19685 m
float LEFT_TRIM = 1.00f;
float RIGHT_TRIM = 1.00f;
float LEFT_TRIM_FWD = 1.00f;
float RIGHT_TRIM_FWD = 1.00f;
float LEFT_TRIM_REV = 1.00f;
float RIGHT_TRIM_REV = 1.00f;
bool USE_UNIFORM_BREAKAWAY = true;

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
    USE_UNIFORM_BREAKAWAY = preferences.getBool("use_uniform", true);

    // Initial default values
    for (int i = 0; i < 4; i++) {
        motorCalibrations[i].forwardBreakawayPwm = 45;
        motorCalibrations[i].reverseBreakawayPwm = 45;
        motorCalibrations[i].kV = 12.0f;
    }

    if (!USE_UNIFORM_BREAKAWAY) {
        // Load custom breakaway calibrations from NVS
        motorCalibrations[0].forwardBreakawayPwm = preferences.getInt("m1_fwd_break", 45);
        motorCalibrations[0].reverseBreakawayPwm = preferences.getInt("m1_rev_break", 45);
        motorCalibrations[0].kV = preferences.getFloat("m1_kv", 12.0f);

        motorCalibrations[1].forwardBreakawayPwm = preferences.getInt("m2_fwd_break", 45);
        motorCalibrations[1].reverseBreakawayPwm = preferences.getInt("m2_rev_break", 45);
        motorCalibrations[1].kV = preferences.getFloat("m2_kv", 12.0f);

        motorCalibrations[2].forwardBreakawayPwm = preferences.getInt("m3_fwd_break", 45);
        motorCalibrations[2].reverseBreakawayPwm = preferences.getInt("m3_rev_break", 45);
        motorCalibrations[2].kV = preferences.getFloat("m3_kv", 12.0f);

        motorCalibrations[3].forwardBreakawayPwm = preferences.getInt("m4_fwd_break", 45);
        motorCalibrations[3].reverseBreakawayPwm = preferences.getInt("m4_rev_break", 45);
        motorCalibrations[3].kV = preferences.getFloat("m4_kv", 12.0f);
    }
    
    Serial.printf("[Config] Loaded breakaway parameters (uniform=%d):\n", USE_UNIFORM_BREAKAWAY);
    for (int i = 0; i < 4; i++) {
        Serial.printf("  Motor %d: FWD=%d, REV=%d, kV=%.2f\n", 
            i + 1, motorCalibrations[i].forwardBreakawayPwm, 
            motorCalibrations[i].reverseBreakawayPwm, motorCalibrations[i].kV);
    }

    // Comprehensive safe diagnostic boot log (M1-M4 effective config audit)
    Serial.println("[Config Diagnostic] Effective Loaded Configuration Audit:");
    const char* drvPolarity[4] = {"NORMAL(in1=1,in2=0)", "INVERTED(in1=0,in2=1)", "NORMAL(in1=1,in2=0)", "NORMAL(in1=1,in2=0)"};
    const char* encPolarity[4] = {"NORMAL(E1_A,E1_B)", "INVERTED(-raw)", "NORMAL(E3_A,E3_B)", "SWAPPED(E4_B,E4_A)"};
    for (int i = 0; i < 4; i++) {
        bool fwdNvs = preferences.isKey((String("m") + (i+1) + "_fwd_break").c_str());
        bool revNvs = preferences.isKey((String("m") + (i+1) + "_rev_break").c_str());
        bool kvNvs  = preferences.isKey((String("m") + (i+1) + "_kv").c_str());
        Serial.printf("  M%d [%s | %s]: fwdBreak=%d (%s), revBreak=%d (%s), kV=%.2f (%s), maxPwm=255\n",
            i + 1, drvPolarity[i], encPolarity[i],
            motorCalibrations[i].forwardBreakawayPwm, fwdNvs ? "NVS" : "DEFAULT",
            motorCalibrations[i].reverseBreakawayPwm, revNvs ? "NVS" : "DEFAULT",
            motorCalibrations[i].kV, kvNvs ? "NVS" : "DEFAULT"
        );
    }
    Serial.printf("  PID Speed Loop: Kp=%.2f, Ki=%.2f, Kd=%.2f (DEFAULT)\n", KP_SPEED, KI_SPEED, KD_SPEED);

    // Load dynamic physical parameters from NVS
    WHEEL_DIAMETER_M = preferences.getFloat("wheel_dia", 0.065f);
    WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0f;
    WHEEL_SEPARATION_M = preferences.getFloat("wheel_sep", 0.197f);
    if (WHEEL_SEPARATION_M < 0.100f || WHEEL_SEPARATION_M > 0.500f) {
        Serial.printf("[Config WARNING] Invalid wheel separation %.4fm loaded from NVS, resetting to default 0.1970m\n", WHEEL_SEPARATION_M);
        WHEEL_SEPARATION_M = 0.197f;
    }
    
    LEFT_TRIM_FWD = preferences.getFloat("left_trim", 1.00f);
    RIGHT_TRIM_FWD = preferences.getFloat("right_trim", 1.00f);
    LEFT_TRIM_REV = preferences.getFloat("left_trim_rev", 1.00f);
    RIGHT_TRIM_REV = preferences.getFloat("right_trim_rev", 1.00f);
    LEFT_TRIM = LEFT_TRIM_FWD;
    RIGHT_TRIM = RIGHT_TRIM_FWD;
    
    bool diaNvs = preferences.isKey("wheel_dia");
    bool sepNvs = preferences.isKey("wheel_sep");
    bool trimFwdNvs = preferences.isKey("left_trim");
    bool trimRevNvs = preferences.isKey("left_trim_rev");

    Serial.printf("[Config] Loaded physical dimensions: diameter=%.4f m (%s), separation=%.4f m (%s)\n", 
                  WHEEL_DIAMETER_M, diaNvs ? "NVS" : "DEFAULT", WHEEL_SEPARATION_M, sepNvs ? "NVS" : "DEFAULT");
    Serial.printf("[Config] Loaded FWD trims (%s): Left=%.4f, Right=%.4f | REV trims (%s): Left=%.4f, Right=%.4f\n", 
                  trimFwdNvs ? "NVS" : "DEFAULT", LEFT_TRIM_FWD, RIGHT_TRIM_FWD, 
                  trimRevNvs ? "NVS" : "DEFAULT", LEFT_TRIM_REV, RIGHT_TRIM_REV);
}

void saveCalibrations() {
    preferences.putBool("use_uniform", false); // Toggles uniform off once breakaway cal is saved

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

void saveTrims(float left, float right) {
    saveTrimsFwd(left, right);
}

void saveTrimsFwd(float left, float right) {
    LEFT_TRIM_FWD = left;
    RIGHT_TRIM_FWD = right;
    preferences.putFloat("left_trim", left);
    preferences.putFloat("right_trim", right);
    Serial.printf("[Config] Saved FWD straight drive trims to NVS: Left=%.4f, Right=%.4f\n", left, right);
}

void saveTrimsRev(float left, float right) {
    LEFT_TRIM_REV = left;
    RIGHT_TRIM_REV = right;
    preferences.putFloat("left_trim_rev", left);
    preferences.putFloat("right_trim_rev", right);
    Serial.printf("[Config] Saved REV straight drive trims to NVS: Left=%.4f, Right=%.4f\n", left, right);
}
