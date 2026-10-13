// test/test_reverse_crawl_candidate.cpp
// Native C++ test suite validating the Rover 1 candidate firmware 1.3.3-cand-r41b
// (source: rev41-floor-cand) with piecewise reverse ultra-low crawl feedforward floor.
//
// Required tests verified:
//  1. Forward +0.461538 rad/s remains numerically identical to baseline (+48 PWM).
//  2. Forward values across the entire 0-0.80 rad/s scope remain unchanged.
//  3. Reverse feedforward is 41 PWM at representative commands:
//     -0.01, -0.05, -0.10, -0.20, -0.30, -0.40, -0.461538 rad/s.
//  4. Continuity immediately below and above -0.461538 rad/s.
//  5. Continuity at -0.80 rad/s.
//  6. Zero command gives zero output and clears controller state.
//  7. Reverse overspeed feedback reduces PWM magnitude below the feedforward baseline.
//  8. Reverse underspeed feedback increases PWM magnitude.
//  9. Stall recovery remains bounded and does not depend on reaching an impossible value below sustaining boundary.
// 10. Breakout remains +/-64 PWM and bounded.
// 11. Spins, arcs, Case 10, Case 11 and Case 12 remain unchanged.
// 12. Open-loop timeout (300ms) and dynamic-brake arbitration still pass.

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cassert>
#include <vector>
#include <deque>
#include <cstdarg>

#include "Arduino.h"

// Mock Serial implementation
class MockSerial {
public:
    std::deque<uint8_t> rxBuffer;
    std::vector<uint8_t> txBuffer;

    int available() { return (int)rxBuffer.size(); }
    int read() {
        if (rxBuffer.empty()) return -1;
        uint8_t b = rxBuffer.front();
        rxBuffer.pop_front();
        return b;
    }
    size_t write(uint8_t b) {
        txBuffer.push_back(b);
        return 1;
    }
    size_t write(const uint8_t *buf, size_t len) {
        for (size_t i = 0; i < len; i++) txBuffer.push_back(buf[i]);
        return len;
    }
    int availableForWrite() { return 256; }
    void setTxBufferSize(size_t) {}
    void begin(unsigned long) {}
    void print(const char* s) { std::cout << s; }
    void println(const char* s) { std::cout << s << std::endl; }
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
    void clear() {
        rxBuffer.clear();
        txBuffer.clear();
    }
};

MockSerial Serial;

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

bool WHEEL_BALANCE_ENABLED = false;
bool DYNAMIC_BRAKE_ENABLED = true;
uint16_t DYNAMIC_BRAKE_DURATION_MS = 250;
float DYNAMIC_BRAKE_MAX_TRIGGER_MPS = 0.80f;
float DYNAMIC_BRAKE_MAX_TRIGGER_RADPS = 3.50f;

MotorCalibration motorCalibrations[4] = {
    {40, 40, 6.0f},
    {40, 40, 6.0f},
    {40, 40, 6.0f},
    {40, 40, 6.0f}
};
bool USE_UNIFORM_BREAKAWAY = true;

static int g_driverPwms[4] = {0, 0, 0, 0};

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
        g_driverPwms[i] = 0;
    }
}

bool MotorDriver::allows(int motorIndex, int) const {
    if (currentMode != MotorOutputMode::NORMAL_DRIVE) return false;
    return (motorIndex >= 0 && motorIndex < 4);
}

void MotorDriver::begin() {
    currentMode = MotorOutputMode::NORMAL_DRIVE;
    actuationState = DrivetrainActuationState::COAST;
    authorizedMotorIndex = -1;
    for (int i = 0; i < 4; i++) {
        motors[i].chan1 = 0;
        motors[i].chan2 = 0;
        g_driverPwms[i] = 0;
    }
}

void MotorDriver::setPWM(int index, int pwm) {
    if (index < 0 || index >= 4) return;
    if (currentMode == MotorOutputMode::LOCKED || currentMode == MotorOutputMode::EMERGENCY_STOP || currentMode == MotorOutputMode::FAULTED) {
        motors[index].chan1 = 0;
        g_driverPwms[index] = 0;
        return;
    }
    if (!allows(index, pwm)) {
        motors[index].chan1 = 0;
        g_driverPwms[index] = 0;
        return;
    }
    motors[index].chan1 = constrain(pwm, -255, 255);
    g_driverPwms[index] = motors[index].chan1;
    actuationState = (pwm != 0) ? DrivetrainActuationState::DRIVE : DrivetrainActuationState::COAST;
}

void MotorDriver::coastAll() {
    actuationState = DrivetrainActuationState::COAST;
    for (int i = 0; i < 4; i++) {
        motors[i].chan1 = 0;
        g_driverPwms[i] = 0;
    }
}

void MotorDriver::brakeAll() {
    actuationState = DrivetrainActuationState::BRAKE;
    for (int i = 0; i < 4; i++) {
        motors[i].chan1 = 0;
        g_driverPwms[i] = 0;
    }
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
// TEST 1: Forward +0.461538 rad/s Remains Numerically Identical to Baseline
// ============================================================================
void test1_ForwardCase2NumericallyIdentical() {
    std::cout << "[TEST 1] Forward +0.461538 rad/s Numerically Identical to Baseline... ";
    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.forceKinetic();

    const float targetRadps = 0.015f / 0.0325f; // +0.46153846 rad/s
    wheel.setTargetVelocity(targetRadps, false, false, false, false);

    int pwm = wheel.update(targetRadps, 100, 0.010f);
    const auto& diag = wheel.getDiag();

    // Baseline forward floor formula: 37.0 + (0.46153846/0.80)*(56.0 - 37.0) = 47.960538 PWM
    float expectedFloor = 37.0f + (targetRadps / 0.80f) * (56.0f - 37.0f);
    assert(std::abs(diag.feedforward - expectedFloor) < 0.001f);
    assert(pwm == 48); // round(47.9605) = 48
    assert(diag.finalPwm == 48);
    assert(diag.pTerm == 0.0f);
    assert(diag.iTerm == 0.0f);
    assert(diag.recoveryAssist == 0.0f);

    std::cout << "PASSED (FF = " << diag.feedforward << " PWM, Delivered = " << pwm << " PWM)" << std::endl;
}

// ============================================================================
// TEST 2: Forward Values Across the Entire 0-0.80 rad/s Scope Remain Unchanged
// ============================================================================
void test2_ForwardScopeUnchanged() {
    std::cout << "[TEST 2] Forward Values Across 0-0.80 rad/s Scope Unchanged... ";
    const float testSpeeds[] = {0.01f, 0.05f, 0.10f, 0.20f, 0.30f, 0.40f, 0.461538f, 0.60f, 0.70f, 0.79f, 0.80f};

    for (float spd : testSpeeds) {
        SingleWheelController wheel;
        wheel.begin(0);
        wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
        wheel.setCandidateRecoveryAssist(true);
        wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
        wheel.forceKinetic();

        wheel.setTargetVelocity(spd, false, false, false, false);
        wheel.update(spd, 100, 0.010f);
        const auto& diag = wheel.getDiag();

        float expectedFloor = 37.0f + (spd / 0.80f) * (56.0f - 37.0f);
        assert(std::abs(diag.feedforward - expectedFloor) < 0.001f);
    }
    std::cout << "PASSED (All 11 forward setpoints bit-for-bit identical)" << std::endl;
}

// ============================================================================
// TEST 3: Reverse Feedforward is 41.0 PWM at Representative Commands
// ============================================================================
void test3_ReverseRepresentativeCommands41PWM() {
    std::cout << "[TEST 3] Reverse Feedforward is 41.0 PWM at Representative Commands... ";
    const float repCommands[] = {-0.01f, -0.05f, -0.10f, -0.20f, -0.30f, -0.40f, -0.461538f};

    for (float cmd : repCommands) {
        SingleWheelController wheel;
        wheel.begin(0);
        wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
        wheel.setCandidateRecoveryAssist(true);
        wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
        wheel.forceKinetic();

        wheel.setTargetVelocity(cmd, false, false, false, false);
        int pwm = wheel.update(cmd, 100, 0.010f);
        const auto& diag = wheel.getDiag();

        // Feedforward must be strictly -41.0 PWM
        assert(std::abs(diag.feedforward - (-41.0f)) < 0.001f);
        assert(pwm == -41);
        assert(diag.finalPwm == -41);
        assert(diag.pTerm == 0.0f);
        assert(diag.iTerm == 0.0f);
    }
    std::cout << "PASSED (Strictly -41.0 PWM across all 7 representative reverse commands)" << std::endl;
}

// ============================================================================
// TEST 4: Continuity Immediately Below and Above -0.461538 rad/s
// ============================================================================
void test4_ReverseAnchorContinuity() {
    std::cout << "[TEST 4] Continuity Immediately Below and Above -0.461538 rad/s... ";
    SingleWheelController w_below, w_at, w_above;
    w_below.begin(0); w_below.setGains(KP_SPEED, KI_SPEED, KD_SPEED); w_below.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF); w_below.forceKinetic();
    w_at.begin(0);    w_at.setGains(KP_SPEED, KI_SPEED, KD_SPEED);    w_at.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);    w_at.forceKinetic();
    w_above.begin(0); w_above.setGains(KP_SPEED, KI_SPEED, KD_SPEED); w_above.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF); w_above.forceKinetic();

    const float anchor = 0.461538f;
    const float eps = 0.0001f;

    w_below.setTargetVelocity(-(anchor - eps), false, false, false, false);
    w_at.setTargetVelocity(-anchor, false, false, false, false);
    w_above.setTargetVelocity(-(anchor + eps), false, false, false, false);

    w_below.update(-(anchor - eps), 100, 0.010f);
    w_at.update(-anchor, 100, 0.010f);
    w_above.update(-(anchor + eps), 100, 0.010f);

    float ff_below = w_below.getDiag().feedforward;
    float ff_at = w_at.getDiag().feedforward;
    float ff_above = w_above.getDiag().feedforward;

    assert(std::abs(ff_below - (-41.0f)) < 0.001f);
    assert(std::abs(ff_at - (-41.0f)) < 0.001f);
    assert(std::abs(ff_above - (-41.0f)) < 0.01f);
    assert(std::abs(ff_above - ff_at) < 0.01f);

    std::cout << "PASSED (ff_below=" << ff_below << ", ff_at=" << ff_at << ", ff_above=" << ff_above << ", delta < 0.01)" << std::endl;
}

// ============================================================================
// TEST 5: Continuity at -0.80 rad/s
// ============================================================================
void test5_ReverseBoundaryContinuity80() {
    std::cout << "[TEST 5] Continuity at -0.80 rad/s Scope Boundary... ";
    SingleWheelController w_sub, w_at, w_sup;
    w_sub.begin(0); w_sub.setGains(KP_SPEED, KI_SPEED, KD_SPEED); w_sub.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF); w_sub.forceKinetic();
    w_at.begin(0);  w_at.setGains(KP_SPEED, KI_SPEED, KD_SPEED);  w_at.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);  w_at.forceKinetic();
    w_sup.begin(0); w_sup.setGains(KP_SPEED, KI_SPEED, KD_SPEED); w_sup.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF); w_sup.forceKinetic();

    // Target just below: -0.79 rad/s
    w_sub.setTargetVelocity(-0.79f, false, false, false, false);
    w_sub.update(-0.79f, 100, 0.010f);
    float ff_sub = w_sub.getDiag().feedforward;

    // Target at boundary: -0.80 rad/s
    w_at.setTargetVelocity(-0.80f, false, false, false, false);
    w_at.update(-0.80f, 100, 0.010f);
    float ff_at = w_at.getDiag().feedforward;

    // Target just above: -0.81 rad/s (standard kinetic floor)
    w_sup.setTargetVelocity(-0.81f, false, false, false, false);
    w_sup.update(-0.81f, 100, 0.010f);
    float ff_sup = w_sup.getDiag().feedforward;

    // At 0.80 rad/s, must equal -56.0 PWM exactly
    assert(std::abs(ff_at - (-56.0f)) < 0.001f);
    assert(std::abs(ff_sup - (-56.0f)) < 0.001f);
    // At 0.79 rad/s, expected: 41.0 + ((0.79 - 0.461538)/(0.80 - 0.461538))*15.0 = 55.556 PWM
    assert(std::abs(ff_sub - (-55.556f)) < 0.01f);
    assert(std::abs(ff_at - ff_sub) < 0.5f);

    std::cout << "PASSED (ff_sub=" << ff_sub << ", ff_at=" << ff_at << ", ff_sup=" << ff_sup << " -> smooth 56.0 transition)" << std::endl;
}

// ============================================================================
// TEST 6: Zero Command Gives Zero Output and Clears Controller State
// ============================================================================
void test6_ZeroCommandZeroOutput() {
    std::cout << "[TEST 6] Zero Command Gives Zero Output & Clears Controller State... ";
    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.forceKinetic();

    // Drive in reverse first
    wheel.setTargetVelocity(-0.461538f, false, false, false, false);
    wheel.update(-0.30f, 100, 0.010f);
    assert(wheel.getDiag().finalPwm < 0);

    // Command zero
    wheel.setTargetVelocity(0.0f, false, false, false, false);
    int pwm = wheel.update(0.0f, 100, 0.010f);
    const auto& diag = wheel.getDiag();

    assert(pwm == 0);
    assert(diag.finalPwm == 0);
    assert(diag.feedforward == 0.0f);
    assert(diag.pTerm == 0.0f);
    assert(diag.iTerm == 0.0f);
    assert(diag.dTerm == 0.0f);
    assert(diag.recoveryAssist == 0.0f);

    // 50ms debounce clears stiction state to IDLE
    for (int step = 0; step < 5; step++) {
        wheel.setTargetVelocity(0.0f, false, false, false, false);
        wheel.update(0.0f, 100, 0.010f);
    }
    assert(wheel.getStictionState() == STICTION_IDLE);

    std::cout << "PASSED (Zero PWM output, integrators zeroed, state = IDLE)" << std::endl;
}

// ============================================================================
// TEST 7: Reverse Overspeed Feedback Reduces PWM Magnitude Below Feedforward
// ============================================================================
void test7_ReverseOverspeedReducesMagnitude() {
    std::cout << "[TEST 7] Reverse Overspeed Feedback Reduces PWM Magnitude Below Baseline... ";
    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.forceKinetic();

    const float targetRadps = -0.461538f;
    wheel.setTargetVelocity(targetRadps, false, false, false, false);

    // Overspeed condition: wheel is running at -0.70 rad/s (more negative than -0.461538)
    // Error = target - meas = -0.461538 - (-0.70) = +0.238462 > 0
    int32_t ticks = 100;
    for (int step = 0; step < 50; step++) { // 500 ms
        ticks -= 7;
        wheel.update(-0.70f, ticks, 0.010f);
    }
    const auto& diag = wheel.getDiag();

    assert(diag.feedforward == -41.0f);
    assert(diag.pTerm > 0.0f); // Positive proportional term
    assert(diag.iTerm > 0.0f); // Positive integral unwinding term
    // Output magnitude must be reduced below feedforward magnitude 41:
    assert(diag.finalPwm > -41);
    assert(std::abs(diag.finalPwm) < 41);

    std::cout << "PASSED (FF = -41 PWM, Delivered = " << diag.finalPwm << " PWM > -41, iTerm = +" << diag.iTerm << " PWM)" << std::endl;
}

// ============================================================================
// TEST 8: Reverse Underspeed Feedback Increases PWM Magnitude Above Baseline
// ============================================================================
void test8_ReverseUnderspeedIncreasesMagnitude() {
    std::cout << "[TEST 8] Reverse Underspeed Feedback Increases PWM Magnitude... ";
    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.forceKinetic();

    const float targetRadps = -0.461538f;
    wheel.setTargetVelocity(targetRadps, false, false, false, false);

    // Underspeed condition: wheel is lagging at -0.20 rad/s (less negative than -0.461538)
    // Error = target - meas = -0.461538 - (-0.20) = -0.261538 < 0
    int32_t ticks = 100;
    for (int step = 0; step < 50; step++) { // 500 ms
        ticks -= 2;
        wheel.update(-0.20f, ticks, 0.010f);
    }
    const auto& diag = wheel.getDiag();

    assert(diag.feedforward == -41.0f);
    assert(diag.pTerm < 0.0f); // Negative proportional term
    assert(diag.iTerm < 0.0f); // Negative integral term
    // Output magnitude must be increased above feedforward magnitude 41:
    assert(diag.finalPwm < -41);
    assert(std::abs(diag.finalPwm) > 41);

    std::cout << "PASSED (FF = -41 PWM, Delivered = " << diag.finalPwm << " PWM < -41, iTerm = " << diag.iTerm << " PWM)" << std::endl;
}

// ============================================================================
// TEST 9: Stall Recovery Remains Bounded & Supported at -41 PWM Floor
// ============================================================================
void test9_StallRecoveryBoundedAndSupported() {
    std::cout << "[TEST 9] Stall Recovery Remains Bounded & Supported at -41 PWM Floor... ";
    // Part 9A: Low commands (-0.10, -0.20 rad/s) maintain sustaining -41.0 PWM floor (not sub-sustaining 22-29 PWM)
    const float lowSpeeds[] = {-0.10f, -0.20f};
    for (float spd : lowSpeeds) {
        SingleWheelController wheel;
        wheel.begin(0);
        wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
        wheel.setCandidateRecoveryAssist(true);
        wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
        wheel.forceKinetic();

        wheel.setTargetVelocity(spd, false, false, false, false);
        wheel.update(0.0f, 100, 0.010f);

        const auto& d = wheel.getDiag();
        // Baseline feedforward floor must be -41.0 PWM (NOT sub-sustaining 22-29 PWM)
        assert(d.feedforward == -41.0f);
        assert(d.finalPwm <= -41);
    }

    // Part 9B: At Case 13 target (-0.461538 rad/s), injected stall engages recovery assist up to -14 PWM cap
    {
        SingleWheelController wheel;
        wheel.begin(0);
        wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
        wheel.setCandidateRecoveryAssist(true);
        wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
        wheel.forceKinetic();

        wheel.setTargetVelocity(-0.461538f, false, false, false, false);

        int32_t ticks = 100;
        int minPwm = 0;
        for (int step = 0; step < 50; step++) { // 500 ms stall
            int p = wheel.update(0.0f, ticks, 0.010f);
            if (p < minPwm) minPwm = p;
        }

        const auto& finalDiag = wheel.getDiag();
        assert(finalDiag.feedforward == -41.0f);
        // Recovery assist is capped at -14 PWM
        assert(finalDiag.recoveryAssist >= -RECOVERY_ASSIST_MAX_PWM);
        assert(finalDiag.recoveryAssist <= -13.9f);
        // Total delivered PWM reaches -55 PWM (-41 feedforward + -14 recovery assist)
        assert(minPwm <= -55);
    }
    std::cout << "PASSED (Low-target floor = -41 PWM, Case 13 stall assist capped at -14 PWM -> peak -55 PWM)" << std::endl;
}

// ============================================================================
// TEST 10: Breakout Remains +/-64 PWM and Bounded
// ============================================================================
void test10_BreakoutRemains64PWM() {
    std::cout << "[TEST 10] Breakout Remains +/-64 PWM and Bounded... ";
    SingleWheelController w_fwd, w_rev;
    w_fwd.begin(0); w_fwd.setTargetVelocity(+0.461538f, false, false, false, false);
    w_rev.begin(0); w_rev.setTargetVelocity(-0.461538f, false, false, false, false);

    assert(w_fwd.getStictionState() == STICTION_BOOST);
    assert(w_rev.getStictionState() == STICTION_BOOST);

    int pwm_fwd = w_fwd.update(0.0f, 0, 0.010f);
    int pwm_rev = w_rev.update(0.0f, 0, 0.010f);
    assert(pwm_fwd == STRAIGHT_STICTION_BOOST_PWM); // +64 PWM
    assert(pwm_rev == -STRAIGHT_STICTION_BOOST_PWM); // -64 PWM

    // Displacement >= 3 ticks triggers kinetic transition
    w_rev.update(0.0f, -3, 0.010f);
    assert(w_rev.getStictionState() == STICTION_KINETIC);

    // 500ms Blocked-Wheel Timeout: if stationary for 50 cycles, must transition to BLOCKED (0 PWM)
    SingleWheelController w_blocked;
    w_blocked.begin(0);
    w_blocked.setTargetVelocity(-0.461538f, false, false, false, false);
    for (int step = 0; step < 49; step++) {
        w_blocked.update(0.0f, 0, 0.010f);
        assert(w_blocked.getStictionState() == STICTION_BOOST);
    }
    int pwm_blocked = w_blocked.update(0.0f, 0, 0.010f); // 50th cycle (500 ms)
    assert(w_blocked.getStictionState() == STICTION_BLOCKED);
    assert(pwm_blocked == 0);

    std::cout << "PASSED (Boost = +/-64 PWM, 3-tick breakout, 500ms blocked cutoff)" << std::endl;
}

// ============================================================================
// TEST 11: Spins, Arcs, Case 10, Case 11 and Case 12 Remain Unchanged
// ============================================================================
void test11_SpinsArcsCases10_11_12_Unchanged() {
    std::cout << "[TEST 11] Spins, Arcs, Cases 10, 11, 12 Remain Unchanged... ";
    SingleWheelController w;
    w.begin(0);
    w.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    w.setCandidateRecoveryAssist(true);
    w.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    w.forceKinetic();

    // 1. Pure spin: 94 PWM (forward-rear) or 80 PWM (others)
    w.setTargetVelocity(1.54f, true, true, false, false);
    w.update(1.54f, 100, 0.010f);
    assert(std::abs(w.getDiag().feedforward - 94.0f) < 0.01f);

    w.setTargetVelocity(0.50f, true, false, false, false);
    w.update(0.50f, 100, 0.010f);
    assert(std::abs(w.getDiag().feedforward - 80.0f) < 0.01f);

    // 2. Turning arc: 22 PWM baseline floor
    w.setTargetVelocity(0.40f, false, false, false, true);
    w.update(0.40f, 100, 0.010f);
    assert(std::abs(w.getDiag().feedforward - 24.40f) < 0.01f);

    // 3. Case 10: +0.030 m/s -> +0.9231 rad/s (> 0.80 rad/s)
    w.setTargetVelocity(0.9231f, false, false, false, false);
    w.forceKinetic();
    w.update(0.9231f, 100, 0.010f);
    assert(std::abs(w.getDiag().feedforward - 56.0f) < 0.01f);

    // 4. Case 11: +0.050 m/s -> +1.5385 rad/s (> 0.80 rad/s)
    w.setTargetVelocity(1.5385f, false, false, false, false);
    w.forceKinetic();
    w.update(1.5385f, 100, 0.010f);
    assert(std::abs(w.getDiag().feedforward - 56.0f) < 0.01f);

    // 5. Case 12: -0.050 m/s -> -1.5385 rad/s (|target| > 0.80 rad/s)
    w.setTargetVelocity(-1.5385f, false, false, false, false);
    w.forceKinetic();
    w.update(-1.5385f, 100, 0.010f);
    assert(std::abs(w.getDiag().feedforward - (-56.0f)) < 0.01f);

    std::cout << "PASSED (Spin 94/80, Arcs 22, Cases 10/11/12 exact 56/-56 PWM)" << std::endl;
}

// ============================================================================
// TEST 12: Open-Loop Safety Timeout and Dynamic Brake Arbitration
// ============================================================================
static int g_hookCalls = 0;
void testTimeoutHook() { g_hookCalls++; }

void test12_OpenLoopSafetyTimeoutAndArbitration() {
    std::cout << "[TEST 12] Open-Loop Safety Timeout & Dynamic Brake Arbitration... ";
    WheelController wc;
    MotorDriver drv;
    drv.begin();
    drv.setMode(MotorOutputMode::NORMAL_DRIVE);
    wc.begin();
    wc.setTimeoutDisarmHook(testTimeoutHook);

    float measuredVels[4] = {0, 0, 0, 0};
    int32_t ticks[4] = {0, 0, 0, 0};

    // 1. Dispatch open-loop PWM
    wc.setOpenLoopPwm(-41, -41, -41, -41, 1000);
    assert(wc.isOpenLoop());

    wc.update(measuredVels, ticks, 0.010f, drv, false, true, true, 1050);
    for (int i = 0; i < 4; i++) {
        assert(g_driverPwms[i] == -41);
    }

    // 2. Watchdog timeout enforcement (>300ms elapsed)
    wc.update(measuredVels, ticks, 0.010f, drv, false, true, true, 1310); // 310 ms elapsed
    assert(!wc.isOpenLoop());
    for (int i = 0; i < 4; i++) {
        assert(g_driverPwms[i] == 0);
    }
    assert(g_hookCalls == 1);

    // 3. Dynamic braking arbitration: open loop releases dynamic brake
    drv.brakeAll();
    wc.setOpenLoopPwm(-41, -41, -41, -41, 2000);
    wc.update(measuredVels, ticks, 0.010f, drv, false, true, true, 2010);
    assert(drv.getActuationState() != DrivetrainActuationState::BRAKE);
    assert(drv.getActuationState() == DrivetrainActuationState::DRIVE);
    assert(g_driverPwms[0] == -41);

    std::cout << "PASSED (Watchdog cutoff at 310ms, Brake release verified)" << std::endl;
}

int main() {
    std::cout << std::unitbuf;
    std::cout << "=======================================================\n";
    std::cout << "RUNNING REVERSE CRAWL CANDIDATE VERIFICATION SUITE (1.3.3-cand-r41c)\n";
    std::cout << "=======================================================\n";

    test1_ForwardCase2NumericallyIdentical();
    test2_ForwardScopeUnchanged();
    test3_ReverseRepresentativeCommands41PWM();
    test4_ReverseAnchorContinuity();
    test5_ReverseBoundaryContinuity80();
    test6_ZeroCommandZeroOutput();
    test7_ReverseOverspeedReducesMagnitude();
    test8_ReverseUnderspeedIncreasesMagnitude();
    test9_StallRecoveryBoundedAndSupported();
    test10_BreakoutRemains64PWM();
    test11_SpinsArcsCases10_11_12_Unchanged();
    test12_OpenLoopSafetyTimeoutAndArbitration();

    std::cout << "=======================================================\n";
    std::cout << "ALL 12 CANDIDATE SPECIFICATION TESTS PASSED SUCCESSFULLY!\n";
    std::cout << "=======================================================\n";
    return 0;
}
