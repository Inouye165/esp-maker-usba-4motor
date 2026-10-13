// test/test_case2_behavioral.cpp
// Focused behavioral validation suite for Rover 1 Case 2 low-speed controller correction.
// Validates:
// 1. Small sustained overspeed and underspeed closed-loop authority (STRAIGHT_CRAWL_KI = 9.0)
// 2. Temporary stall followed by recovery (preventing assist transfer during overspeed)
// 3. Sustained load assist transfer followed by load removal and 2x unwinding
// 4. Forward and reverse signed symmetry
// 5. Immediate zero-command shutdown, output cutoff, and state reset

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cassert>
#include <vector>

#include "Arduino.h"
#include "RoverConfig.h"
#include "MotorDriver.h"
#include "DifferentialDrive.h"
#include "DifferentialDrive.cpp"
#include "CommandManager.h"
#include "CommandManager.cpp"
#include "MotionLimiter.h"
#include "MotionLimiter.cpp"
#include "WheelController.h"
#include "WheelController.cpp"

// --- Hardware Mocks for Native Test Execution ---
uint32_t g_mockMillis = 0;
uint64_t g_mockMicros = 0;
Preferences preferences;

float WHEEL_DIAMETER_M = 0.065f;
float WHEEL_RADIUS_M = 0.0325f;
float WHEEL_SEPARATION_M = 0.3408575433f;
float LEFT_TRIM = 1.00f;
float RIGHT_TRIM = 1.00f;
float LEFT_TRIM_FWD = 1.00f;
float RIGHT_TRIM_FWD = 1.00f;
float LEFT_TRIM_REV = 1.00f;
float RIGHT_TRIM_REV = 1.00f;
bool USE_UNIFORM_BREAKAWAY = true;
bool WHEEL_BALANCE_ENABLED = false;
bool DYNAMIC_BRAKE_ENABLED = false;
uint16_t DYNAMIC_BRAKE_DURATION_MS = 100;
float DYNAMIC_BRAKE_MAX_TRIGGER_MPS = 0.35f;
float DYNAMIC_BRAKE_MAX_TRIGGER_RADPS = 0.80f;

MotorCalibration motorCalibrations[4] = {
    {40, 40, 6.00f},
    {40, 40, 6.00f},
    {40, 40, 6.00f},
    {40, 40, 6.00f}
};

void saveTrims(float, float) {}
void saveTrimsFwd(float, float) {}
void saveTrimsRev(float, float) {}
void initConfigStorage() {}
void loadCalibrations() {}
void saveCalibrations() {}
uint32_t getEspBootCount() { return 1; }
uint32_t getEspResetReason() { return 1; }
uint32_t getEspRtc0ResetReason() { return 1; }
const char* getEspResetReasonString(uint32_t) { return "POWERON_RESET"; }
const char* getEspRtcResetReasonString(uint32_t) { return "POWERON_RESET"; }

// --- Mock MotorDriver implementation ---
MotorDriver::MotorDriver() 
    : currentMode(MotorOutputMode::LOCKED)
    , authorizedMotorIndex(-1)
    , actuationState(DrivetrainActuationState::COAST) {
    for (int i = 0; i < 4; i++) {
        motors[i].in1 = 0;
        motors[i].in2 = 0;
        motors[i].chan1 = 0;
        motors[i].chan2 = 0;
        motors[i].invert = (i == 1);
    }
}

void MotorDriver::begin() {
    currentMode = MotorOutputMode::LOCKED;
    actuationState = DrivetrainActuationState::COAST;
    for (int i = 0; i < 4; i++) motors[i].chan1 = 0;
}

bool MotorDriver::allows(int motorIndex, int) const {
    if (currentMode != MotorOutputMode::NORMAL_DRIVE) return false;
    if (PHASE4A1_NORMAL_DRIVE_OUTPUT_DISABLED) return false;
    return (motorIndex >= 0 && motorIndex < 4);
}

void MotorDriver::setPWM(int index, int pwm) {
    if (index < 0 || index >= 4) return;
    if (actuationState == DrivetrainActuationState::BRAKE) return;
    if (!allows(index, pwm)) {
        motors[index].chan1 = 0;
        return;
    }
    motors[index].chan1 = constrain(pwm, -255, 255);
    actuationState = (pwm != 0) ? DrivetrainActuationState::DRIVE : DrivetrainActuationState::COAST;
}

void MotorDriver::brakeAll() {
    actuationState = DrivetrainActuationState::BRAKE;
    for (int i = 0; i < 4; i++) motors[i].chan1 = 0;
}

void MotorDriver::coastAll() {
    actuationState = DrivetrainActuationState::COAST;
    for (int i = 0; i < 4; i++) motors[i].chan1 = 0;
}

void MotorDriver::emergencyStop() {
    coastAll();
    currentMode = MotorOutputMode::EMERGENCY_STOP;
}

void MotorDriver::setMode(MotorOutputMode mode, int authorizedMotor) {
    currentMode = mode;
    authorizedMotorIndex = authorizedMotor;
    if (mode == MotorOutputMode::LOCKED || mode == MotorOutputMode::EMERGENCY_STOP || mode == MotorOutputMode::FAULTED) {
        coastAll();
    }
}

// ============================================================================
// TEST 1: Small Sustained Overspeed and Underspeed Closed-Loop Correction
// ============================================================================
void test1_SmallSustainedOverspeedAndUnderspeed() {
    std::cout << "[BEHAVIORAL TEST 1] Small Sustained Overspeed and Underspeed Tracking Authority\n";

    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.forceKinetic();

    const float targetSpeed = 0.4615f; // Case 2: +0.015 m/s at 65mm geometry
    wheel.setTargetVelocity(targetSpeed, false, false, false, false);

    const float dt = 0.010f; // 100 Hz
    int32_t ticks = 100;

    // --- Part 1A: Small Persistent Overspeed (+0.12 rad/s error = -0.12 rad/s) ---
    // In baseline with Ki=1.2, this produced only ~0.2 PWM trim over 1.8s.
    // With STRAIGHT_CRAWL_KI=9.0 and 2x opposing unwinding, it actively trims down PWM.
    float overspeedMeas = 0.58f; // ~25% overspeed
    for (int step = 0; step < 50; step++) { // 500 ms
        ticks += 5;
        wheel.update(overspeedMeas, ticks, dt);
    }
    const auto& diagOverspeed = wheel.getDiag();
    std::cout << "  Part 1A (Overspeed +0.12 rad/s for 500ms): iTerm=" << diagOverspeed.iTerm
              << " PWM, finalPwm=" << diagOverspeed.finalPwm
              << ", feedforward=" << diagOverspeed.feedforward << " PWM\n";

    // Rate from zero: STRAIGHT_CRAWL_KI * error * dt = 9.0 * (-0.1185) * 0.50s = -0.533 PWM
    assert(diagOverspeed.iTerm < -0.40f && diagOverspeed.iTerm > -0.70f);
    assert(diagOverspeed.finalPwm < (int)round(diagOverspeed.feedforward));
    std::cout << "  -> Part 1A PASS: Active negative closed-loop trim pulls PWM below feedforward.\n";

    // --- Part 1B: Small Persistent Underspeed (-0.11 rad/s error = +0.11 rad/s) ---
    wheel.reset();
    wheel.forceKinetic();
    wheel.setTargetVelocity(targetSpeed, false, false, false, false);
    ticks = 100;

    float underspeedMeas = 0.35f; // ~24% underspeed
    for (int step = 0; step < 50; step++) { // 500 ms
        ticks += 3;
        wheel.update(underspeedMeas, ticks, dt);
    }
    const auto& diagUnderspeed = wheel.getDiag();
    std::cout << "  Part 1B (Underspeed -0.11 rad/s for 500ms): iTerm=" << diagUnderspeed.iTerm
              << " PWM, finalPwm=" << diagUnderspeed.finalPwm
              << ", feedforward=" << diagUnderspeed.feedforward << " PWM\n";

    // Rate: STRAIGHT_CRAWL_KI * error * dt = 9.0 * (+0.1115) * 0.50s = +0.502 PWM
    assert(diagUnderspeed.iTerm > +0.35f && diagUnderspeed.iTerm < +0.75f);
    assert(diagUnderspeed.finalPwm > (int)round(diagUnderspeed.feedforward));
    std::cout << "  -> Part 1B PASS: Active positive closed-loop trim boosts PWM above feedforward.\n";
}

// ============================================================================
// TEST 2: Temporary Stall Followed by Recovery & Assist-Overspeed Isolation
// ============================================================================
void test2_TemporaryStallFollowedByRecovery() {
    std::cout << "\n[BEHAVIORAL TEST 2] Temporary Stall Followed by Recovery & Assist Isolation\n";

    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.forceKinetic();

    const float targetSpeed = 0.4615f;
    wheel.setTargetVelocity(targetSpeed, false, false, false, false);
    const float dt = 0.010f;
    int32_t ticks = 100;

    // Phase 1: Normal steady-state rolling for 300 ms (steps 0 to 30)
    for (int step = 0; step < 30; step++) {
        ticks += 4;
        wheel.update(targetSpeed, ticks, dt);
    }
    float preStallIntegral = wheel.getDiag().iTerm;
    assert(wheel.getDiag().recoveryAssist == 0.0f);
    std::cout << "  Phase 1 (Pre-stall rolling): iTerm=" << preStallIntegral
              << " PWM, Assist=" << wheel.getDiag().recoveryAssist << " PWM\n";

    // Phase 2: Injected 200 ms stall (steps 30 to 50, ticks frozen, meas=0.0)
    float peakAssist = 0.0f;
    for (int step = 30; step < 50; step++) {
        wheel.update(0.0f, ticks, dt);
        float assist = wheel.getDiag().recoveryAssist;
        if (assist > peakAssist) peakAssist = assist;
        // Tracking integral must remain frozen during stall condition
        assert(wheel.getDiag().iTerm == preStallIntegral);
    }
    std::cout << "  Phase 2 (Stalled 200ms): Peak Assist=" << peakAssist
              << " PWM, iTerm frozen at=" << wheel.getDiag().iTerm << " PWM\n";
    assert(peakAssist >= 6.0f && peakAssist <= 14.0f);

    // Phase 3: Breakout with temporary overspeed (meas = 0.70 rad/s > 0.4615 rad/s)
    // Assist decays to 0. CRITICAL: Assist decline MUST NOT be added to integral because isOverspeed is true!
    for (int step = 50; step < 90; step++) {
        ticks += 7;
        wheel.update(0.70f, ticks, dt);
    }
    const auto& diagPostRecovery = wheel.getDiag();
    std::cout << "  Phase 3 (Breakout into overspeed 0.70 rad/s): Assist="
              << diagPostRecovery.recoveryAssist << " PWM, iTerm=" << diagPostRecovery.iTerm << " PWM\n";

    assert(diagPostRecovery.recoveryAssist == 0.0f);
    // Crucial check: iTerm must NOT have absorbed the 6+ PWM of recovery assist!
    // In fact, because meas=0.70 > 0.4615, iTerm should be <= preStallIntegral (unwinding excess):
    assert(diagPostRecovery.iTerm <= preStallIntegral + 0.05f);
    assert(diagPostRecovery.iTerm < 1.0f); // Did not jump to +6 PWM
    std::cout << "  -> TEST 2 PASS: Temporary recovery assist decayed without contaminating tracking integral.\n";
}

// ============================================================================
// TEST 3: Sustained Load Followed by Load Removal
// ============================================================================
void test3_SustainedLoadFollowedByLoadRemoval() {
    std::cout << "\n[BEHAVIORAL TEST 3] Sustained Load Handoff Followed by Load Removal\n";

    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.forceKinetic();

    const float targetSpeed = 0.4615f;
    wheel.setTargetVelocity(targetSpeed, false, false, false, false);
    const float dt = 0.010f;
    int32_t ticks = 100;

    // Phase 1: Induce stall
    for (int step = 0; step < 25; step++) {
        wheel.update(0.0f, ticks, dt);
    }
    float assistAtBreakout = wheel.getDiag().recoveryAssist;
    assert(assistAtBreakout >= 7.0f);
    std::cout << "  Phase 1 (Stall breakout point): Assist=" << assistAtBreakout << " PWM\n";

    // Phase 2: Wheel breaks out under sustained high rolling friction, running slightly lagging
    // (meas = 0.40 rad/s < 0.4615 rad/s, !isOverspeed).
    // As assist declines, the declining assist torque MUST transfer into integralTerm to maintain rolling!
    for (int step = 25; step < 70; step++) {
        ticks += 4;
        wheel.update(0.40f, ticks, dt);
    }
    const auto& diagUnderLoad = wheel.getDiag();
    std::cout << "  Phase 2 (Lagging under sustained load): Assist=" << diagUnderLoad.recoveryAssist
              << " PWM, iTerm=" << diagUnderLoad.iTerm << " PWM, finalPwm=" << diagUnderLoad.finalPwm << "\n";

    assert(diagUnderLoad.recoveryAssist == 0.0f);
    // Integral must have received the declining assist:
    assert(diagUnderLoad.iTerm >= 6.0f);
    assert(diagUnderLoad.finalPwm >= (int)round(diagUnderLoad.feedforward) + 6);
    std::cout << "  -> Sustained load handoff strictly verified: Integral holds "
              << diagUnderLoad.iTerm << " PWM to sustain motion.\n";

    // Phase 3: Load is removed! Wheel speeds up to 0.65 rad/s (overspeed).
    // Active STRAIGHT_CRAWL_KI must unwind the transferred integral at 2x rate!
    for (int step = 70; step < 400; step++) {
        ticks += 6;
        wheel.update(0.65f, ticks, dt);
    }
    const auto& diagUnwound = wheel.getDiag();
    std::cout << "  Phase 3 (Load removed, overspeed 0.65 rad/s): iTerm unwound to="
              << diagUnwound.iTerm << " PWM, finalPwm=" << diagUnwound.finalPwm << "\n";

    assert(diagUnwound.iTerm < 0.0f); // Unwound all the way past 0 into negative trim
    assert(diagUnwound.finalPwm < (int)round(diagUnwound.feedforward));
    std::cout << "  -> TEST 3 PASS: Smooth recovery under load and rapid 2x unwinding upon load removal verified.\n";
}

// ============================================================================
// TEST 4: Forward and Reverse Signed Symmetry
// ============================================================================
void test4_ForwardAndReverseDirectionAwareFeedforward() {
    std::cout << "\n[BEHAVIORAL TEST 4] Forward and Reverse Direction-Aware Feedforward & Feedback Symmetry\n";

    SingleWheelController w_fwd, w_rev;
    w_fwd.begin(0); w_fwd.setGains(KP_SPEED, KI_SPEED, KD_SPEED); w_fwd.setCandidateRecoveryAssist(true); w_fwd.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF); w_fwd.forceKinetic();
    w_rev.begin(0); w_rev.setGains(KP_SPEED, KI_SPEED, KD_SPEED); w_rev.setCandidateRecoveryAssist(true); w_rev.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF); w_rev.forceKinetic();

    const float spd = 0.4615f;
    w_fwd.setTargetVelocity(+spd, false, false, false, false);
    w_rev.setTargetVelocity(-spd, false, false, false, false);

    const float dt = 0.010f;
    int32_t ticks_fwd = 100, ticks_rev = 100;

    // Apply identical relative trajectory: 10 steps lagging, 10 steps overspeeding
    for (int step = 0; step < 20; step++) {
        float meas_rel = (step < 10) ? 0.30f : 0.60f;
        ticks_fwd += (int32_t)(meas_rel * 10);
        ticks_rev -= (int32_t)(meas_rel * 10);

        int pwm_fwd = w_fwd.update(+meas_rel, ticks_fwd, dt);
        int pwm_rev = w_rev.update(-meas_rel, ticks_rev, dt);

        const auto& d_fwd = w_fwd.getDiag();
        const auto& d_rev = w_rev.getDiag();

        // 1. Forward feedforward unchanged at +47.96 PWM (rounds to 48)
        assert(std::abs(d_fwd.feedforward - 47.96057f) < 0.02f);
        // 2. Reverse feedforward floor is strictly -41.0 PWM
        assert(std::abs(d_rev.feedforward - (-41.0f)) < 0.001f);

        // 3. Error feedback terms remain properly signed
        assert(std::abs(d_fwd.pTerm - (-d_rev.pTerm)) < 0.01f);
        assert(std::abs(d_fwd.iTerm - (-d_rev.iTerm)) < 0.01f);
        assert(std::abs(d_fwd.recoveryAssist - (-d_rev.recoveryAssist)) < 0.01f);

        // 4. Closed-loop PID correction from feedforward is bit-for-bit signed symmetric:
        float corr_fwd = (d_fwd.pTerm + d_fwd.iTerm + d_fwd.recoveryAssist);
        float corr_rev = (d_rev.pTerm + d_rev.iTerm + d_rev.recoveryAssist);
        assert(std::abs(corr_fwd - (-corr_rev)) < 0.01f);
    }

    std::cout << "  -> TEST 4 PASS: Forward unchanged (+48 PWM), Reverse near -41 PWM, feedback regulation verified.\n";
}

// ============================================================================
// TEST 5: Immediate Zero-Command Behavior and Integral Reset
// ============================================================================
void test5_ImmediateZeroCommandAndReset() {
    std::cout << "\n[BEHAVIORAL TEST 5] Immediate Zero-Command Shutdown and Disarm State Invariants\n";

    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.forceKinetic();

    // Run active motion with active integral
    wheel.setTargetVelocity(0.4615f, false, false, false, false);
    for (int step = 0; step < 30; step++) {
        wheel.update(0.35f, 100 + step * 3, 0.01f);
    }
    assert(wheel.getDiag().finalPwm > 0);
    assert(wheel.getDiag().iTerm > 0.0f);

    // Issue IMMEDIATE zero command
    wheel.setTargetVelocity(0.0f, false, false, false, false);
    int zeroPwm = wheel.update(0.10f, 200, 0.01f);
    const auto& diagImmediate = wheel.getDiag();

    std::cout << "  Immediate tick zero command: PWM=" << zeroPwm
              << ", Feedforward=" << diagImmediate.feedforward
              << ", Assist=" << diagImmediate.recoveryAssist
              << ", iTerm=" << diagImmediate.iTerm << "\n";

    // Immediate tick verification: torque, feedforward, assist, and integral MUST be strictly zero
    assert(zeroPwm == 0);
    assert(diagImmediate.finalPwm == 0);
    assert(diagImmediate.feedforward == 0.0f);
    assert(diagImmediate.recoveryAssist == 0.0f);
    assert(diagImmediate.iTerm == 0.0f);

    // Sustained zero command (50ms / 5 ticks debounce in setTargetVelocity)
    for (int step = 0; step < 5; step++) {
        wheel.setTargetVelocity(0.0f, false, false, false, false);
        wheel.update(0.0f, 200, 0.01f);
    }
    const auto& diagSustained = wheel.getDiag();
    std::cout << "  Sustained zero command (50ms): StictionState=" << diagSustained.stictionState << "\n";
    assert(diagSustained.stictionState == STICTION_IDLE);

    // Complete disarm reset
    wheel.reset();
    const auto& diagReset = wheel.getDiag();
    assert(diagReset.finalPwm == 0);
    assert(diagReset.iTerm == 0.0f);
    assert(diagReset.recoveryAssist == 0.0f);
    assert(wheel.getStictionState() == STICTION_IDLE);

    std::cout << "  -> TEST 5 PASS: Zero output confirmed, state IDLE, all torque cleared immediately.\n";
}

int main() {
    std::cout << std::unitbuf;
    std::cout << "=======================================================\n";
    std::cout << "RUNNING CASE 2 FOCUSED BEHAVIORAL VALIDATION SUITE\n";
    std::cout << "=======================================================\n";

    test1_SmallSustainedOverspeedAndUnderspeed();
    test2_TemporaryStallFollowedByRecovery();
    test3_SustainedLoadFollowedByLoadRemoval();
    test4_ForwardAndReverseDirectionAwareFeedforward();
    test5_ImmediateZeroCommandAndReset();

    std::cout << "\n=======================================================\n";
    std::cout << "ALL 5 FOCUSED BEHAVIORAL TESTS PASSED SUCCESSFULLY!\n";
    std::cout << "=======================================================\n";
    return 0;
}
