#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <Arduino.h>
#include <cstdio>
#include <cstring>
#include <Preferences.h>

extern Preferences preferences;

// Control Loop Settings
const unsigned long CONTROL_PERIOD_MS = 10; // 100 Hz rate
const float CONTROL_PERIOD_S = 0.01f;

// Pin Mappings: DC Motors
const int M1_IN1 = 27;
const int M1_IN2 = 13;
const int M2_IN1 = 4;
const int M2_IN2 = 2;
const int M3_IN1 = 17;
const int M3_IN2 = 12;
const int M4_IN1 = 14;
const int M4_IN2 = 15;

// Pin Mappings: Quadrature Encoders
const int E1_A = 18;
const int E1_B = 19;
const int E2_A = 5;
const int E2_B = 23;
const int E3_A = 35; // Input only pin
const int E3_B = 36; // Input only pin
const int E4_A = 34; // Input only pin
const int E4_B = 39; // Input only pin

// Physical Parameters
extern float WHEEL_DIAMETER_M;
extern float WHEEL_RADIUS_M;
extern float WHEEL_SEPARATION_M; // Effective skid-steer track width (0.3408575433m)
const float PHYSICAL_WHEEL_SEPARATION_M = 0.197f; // Physical wheel-center distance (0.197m = 7.75 in)
extern float LEFT_TRIM;
extern float RIGHT_TRIM;
extern float LEFT_TRIM_FWD;
extern float RIGHT_TRIM_FWD;
extern float LEFT_TRIM_REV;
extern float RIGHT_TRIM_REV;
void saveTrims(float left, float right);
void saveTrimsFwd(float left, float right);
void saveTrimsRev(float left, float right);
const float TICKS_PER_REV = 1974.1666666667f; // Measured 4-wheel average ticks/revolution (1974.1667)
const float TICKS_PER_WHEEL_REV = 1974.1666666667f;

// Kinematic Motion Constraints (Safe Conservative Defaults for Phase 4)
const float MAX_LINEAR_VELOCITY_MPS = 0.80f;     // High velocity ceiling for floor driving
const float MAX_ANGULAR_VELOCITY_RADPS = 3.50f;   // High angular velocity ceiling for skid turns

const float MAX_LINEAR_ACCEL_MPS2 = 10.0f;       // Instant acceleration
const float MAX_LINEAR_DECEL_MPS2 = 15.0f;       // Instant deceleration
const float MAX_LINEAR_JERK_MPS3 = 100.0f;       // Instant jerk response

const float MAX_ANGULAR_ACCEL_RADPS2 = 20.0f;    // Snappy angular acceleration
const float MAX_ANGULAR_DECEL_RADPS2 = 30.0f;    // Snappy angular deceleration
const float MAX_ANGULAR_JERK_RADPS3 = 200.0f;     // Snappy angular jerk response

// Motor Controller PID Gains
const float KP_SPEED = 2.2f;
const float KI_SPEED = 1.2f;
const float KD_SPEED = 0.05f;
const float SPIN_PID_KP = 6.0f; // Pure-spin proportional gain (6.0), active strictly during pure-spin maneuvers


// Breakaway and Static Friction Compensation Limits (Feedforward Constants)
// Start with initial approximate breakaway values (will be refined by calibration)
struct MotorCalibration {
    int forwardBreakawayPwm;
    int reverseBreakawayPwm;
    float kV; // Velocity Feedforward gain (Duty cycle / (rad/s))
};

extern MotorCalibration motorCalibrations[4];
extern bool USE_UNIFORM_BREAKAWAY;

// Dynamic Stiction Breakout Parameters
const int STICTION_BOOST_PWM = 48;                 // Standard straight/coordinated breakout
const int SPIN_STICTION_BOOST_PWM = 102;           // Empirically validated pure-spin startup breakout PWM (102 PWM)
const uint16_t SPIN_BREAKOUT_MIN_CYCLES = 10;      // Minimum breakout dwell duration: 100 ms @ 100 Hz (10 control cycles)
const uint16_t SPIN_BREAKOUT_MAX_CYCLES = 20;      // Hard max breakout boost duration: 200 ms @ 100 Hz (20 control cycles)
const uint16_t SPIN_BREAKOUT_SUSTAINED_CYCLES = 3; // Consecutive control cycles of sustained velocity required for early exit
const float SPIN_BREAKOUT_VELOCITY_THRESHOLD = 0.10f; // Velocity threshold (rad/s) to qualify sustained rear-wheel motion
const float SPIN_KS_PWM = 58.0f;                   // Empirical pure-spin lateral scrub breakaway base
const float SPIN_KINETIC_KS_PWM = 75.0f;           // Dedicated pure-spin kinetic feedforward base/intercept (75 PWM)
const float MIN_SPIN_KINETIC_FF_FLOOR = 80.0f;     // Pure-spin kinetic feedforward floor baseline for non-forward-rear wheels

const float SPIN_FORWARD_REAR_KINETIC_FLOOR = 94.0f; // Empirically validated kinetic floor for forward-driving rear wheel (94 PWM)
const float SPIN_REVERSE_REAR_KINETIC_FLOOR = 80.0f; // Kinetic floor for reverse-driving rear wheel (80 PWM)
const float SPIN_MANEUVER_EPSILON = 0.005f;        // Linear velocity threshold for pure spin classification (m/s)

// Watchdog communication timeout
const uint32_t WATCHDOG_TIMEOUT_MS = 300; // Controlled deceleration trigger
const uint32_t FAULT_TIMEOUT_MS = 1000;    // Fully disable motor output trigger

// Load/Save calibration settings to NVS
void initConfigStorage();
void loadCalibrations();
void saveCalibrations();

#endif // ROVER_CONFIG_H
