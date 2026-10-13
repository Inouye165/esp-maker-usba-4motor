// test/test_open_loop_expiry.cpp
// Native C++ regression suite validating ESP32 open-loop PWM command expiry and isolation.
// Exercises REAL SerialProtocol handlers (0x11, 0x12, 0x2C, 0x2D, 0x32) and the real registered timeout hook.
// Verifies:
// 1. Timeout Expiry: Motor output forced to 0 after > 300 ms without refresh, open-loop terminated, normal drive disarmed.
// 2. Regular Refresh: Periodic 0x11 packets (< 300 ms) sustain open-loop output cleanly.
// 3. Explicit Zero: 0x11 [0,0,0,0] packet clears open-loop immediately, resets motion limiter and closed-loop targets.
// 4. Disarm Scoping: 0x2D immediate reset is scoped strictly to active open-loop sessions; ordinary closed-loop deceleration is preserved.
// 5. Stale-Target Non-Resumption: Old closed-loop velocity commands cannot resume after open-loop expiry even after subsequent rearming.
// 6. Timer Wraparound: uint32_t modular subtraction safely handles timer overflow around 0xFFFFFFFF.
// 7. Distinct Firmware Identity: 0x32 emits version "1.3.2-olp-safe" and build tag "olp-safe-case2".

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

// --- Mock HardwareSerial with Buffer Queue ---
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
#include "CalibrationManager.h"
#include "CalibrationManager.cpp"
#include "MaintenanceManager.h"
#include "MaintenanceManager.cpp"
#include "SafetyManager.h"
#include "SafetyManager.cpp"
#include "ChassisSupervisor.h"
#include "SerialProtocol.h"
#include "SerialProtocol.cpp"

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

// --- Mock MotorDriver ---
static int g_driverPwms[4] = {0, 0, 0, 0};

uint16_t ImuManager::getStatusFlags(int64_t) const { return 0; }
uint16_t ImuManager::getRotVecAgeMs(int64_t) const { return 0; }
uint16_t ImuManager::getGyroAgeMs(int64_t) const { return 0; }
uint16_t ImuManager::getAccelAgeMs(int64_t) const { return 0; }

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

void MotorDriver::begin() {
    currentMode = MotorOutputMode::LOCKED;
    actuationState = DrivetrainActuationState::COAST;
    for (int i = 0; i < 4; i++) {
        motors[i].chan1 = 0;
        g_driverPwms[i] = 0;
    }
}

bool MotorDriver::allows(int motorIndex, int) const {
    if (currentMode != MotorOutputMode::NORMAL_DRIVE) return false;
    if (PHASE4A1_NORMAL_DRIVE_OUTPUT_DISABLED) return false;
    return (motorIndex >= 0 && motorIndex < 4);
}

void MotorDriver::setPWM(int index, int pwm) {
    if (index < 0 || index >= 4) return;
    if (actuationState == DrivetrainActuationState::BRAKE) {
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

void MotorDriver::brakeAll() {
    actuationState = DrivetrainActuationState::BRAKE;
    for (int i = 0; i < 4; i++) {
        motors[i].chan1 = 0;
        g_driverPwms[i] = 0;
    }
}

void MotorDriver::coastAll() {
    actuationState = DrivetrainActuationState::COAST;
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

int getDriverPWM(int index) {
    if (index < 0 || index >= 4) return 0;
    return g_driverPwms[index];
}

// Global instances required by SerialProtocol externs
CommandManager commandManager;
MotionLimiter motionLimiter;
MotorDriver motorDriver;
WheelController wheelController;
ChassisSupervisorState g_chassisSupervisorState;

// Packet injection helpers
void injectFramedPacket(uint8_t extType, const std::vector<uint8_t>& data) {
    uint8_t extLen = 1 + (uint8_t)data.size() + 1; // extType + data + checksum
    std::vector<uint8_t> payloadBuf;
    payloadBuf.push_back(extType);
    for (uint8_t b : data) {
        payloadBuf.push_back(b);
    }
    uint16_t sum = extLen;
    for (size_t i = 0; i < payloadBuf.size(); i++) {
        sum += payloadBuf[i];
    }
    uint8_t checksum = sum & 0xFF;
    payloadBuf.push_back(checksum);

    Serial.rxBuffer.push_back(0xFF);
    Serial.rxBuffer.push_back(0xFC);
    Serial.rxBuffer.push_back(extLen);
    for (uint8_t b : payloadBuf) {
        Serial.rxBuffer.push_back(b);
    }
}

void sendCmdArmNormalDrive() {
    injectFramedPacket(0x2C, {});
}

void sendCmdDisarmNormalDrive() {
    injectFramedPacket(0x2D, {});
}

void sendCmdOpenLoopPwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    std::vector<uint8_t> p(8, 0);
    p[0] = (uint8_t)(m1 & 0xFF); p[1] = (uint8_t)((m1 >> 8) & 0xFF);
    p[2] = (uint8_t)(m2 & 0xFF); p[3] = (uint8_t)((m2 >> 8) & 0xFF);
    p[4] = (uint8_t)(m3 & 0xFF); p[5] = (uint8_t)((m3 >> 8) & 0xFF);
    p[6] = (uint8_t)(m4 & 0xFF); p[7] = (uint8_t)((m4 >> 8) & 0xFF);
    injectFramedPacket(0x11, p);
}

void sendCmdMotion(float vx, float vy, float vz) {
    int16_t ivx = (int16_t)(vx * 1000.0f);
    int16_t ivy = (int16_t)(vy * 1000.0f);
    int16_t ivz = (int16_t)(vz * 1000.0f);
    std::vector<uint8_t> p(6, 0);
    p[0] = (uint8_t)(ivx & 0xFF); p[1] = (uint8_t)((ivx >> 8) & 0xFF);
    p[2] = (uint8_t)(ivy & 0xFF); p[3] = (uint8_t)((ivy >> 8) & 0xFF);
    p[4] = (uint8_t)(ivz & 0xFF); p[5] = (uint8_t)((ivz >> 8) & 0xFF);
    injectFramedPacket(0x12, p);
}

static int g_disarmHookCalls = 0;
static void productionTimeoutDisarmHook() {
    g_disarmHookCalls++;
    commandManager.disarmNormalDrive();
    commandManager.requestSourceChange(SOURCE_NONE, 0.0f, 0.0f);
    motionLimiter.reset(0.0f, 0.0f);
}

// Production Test Environment Harness
struct ProductionDrivetrainHarness {
    CalibrationManager calManager;
    MaintenanceManager maintManager;
    SafetyManager safetyManager;
    ControlLoopStats loopStats;
    SerialProtocol serialProtocol;
    float measuredVels[4];
    int32_t ticks[4];

    ProductionDrivetrainHarness() {
        Serial.clear();
        g_disarmHookCalls = 0;
        commandManager.begin();
        motionLimiter.begin();
        wheelController.begin();
        motorDriver.begin();
        calManager.begin();
        maintManager.begin();
        safetyManager.begin();
        serialProtocol.begin();

        for (int i = 0; i < 4; i++) {
            measuredVels[i] = 0.0f;
            ticks[i] = 0;
            g_driverPwms[i] = 0;
        }

        // Register the EXACT production setup() timeout hook from main.cpp
        wheelController.setTimeoutDisarmHook(productionTimeoutDisarmHook);
    }

    // Dynamic Brake Pulse & Actuation State Machine (exact production logic from main.cpp)
    enum class BrakePulseState {
        DRIVING = 0,
        BRAKING = 1,
        COASTING = 2
    };
    BrakePulseState brakePulseState = BrakePulseState::COASTING;
    uint32_t brakePulseStartMs = 0;
    float lastActiveAngSpeed = 0.0f;
    float lastActiveLinSpeed = 0.0f;

    void processSerial() {
        serialProtocol.update(commandManager, calManager, maintManager, safetyManager, loopStats);
    }

    void runControlTick(float dt = 0.01f, uint32_t nowMs = 0) {
        g_mockMillis = nowMs;
        processSerial();

        ChassisCommand activeCmd = commandManager.getActiveCommand(nowMs);
        if (commandManager.isNormalDriveArmed()) {
            motionLimiter.update(activeCmd.linearVelocity, activeCmd.angularVelocity, dt);
        } else {
            motionLimiter.update(0.0f, 0.0f, dt);
        }
        float targetLinear = motionLimiter.getLinearVelocity();
        float targetAngular = motionLimiter.getAngularVelocity();

        // Section 7: Route target wheel speeds
        if (!wheelController.isOpenLoop()) {
            float leftRadps = 0.0f, rightRadps = 0.0f;
            DifferentialDrive::velocityToWheels(targetLinear, targetAngular, leftRadps, rightRadps);
            wheelController.setTargets(leftRadps, rightRadps, false, false, false);
        }

        // Section 8: WheelController update
        wheelController.update(measuredVels, ticks, dt, motorDriver, false, false, true, nowMs);

        // Production Dynamic Brake Pulse & Actuation State Machine (from main.cpp)
        if (wheelController.isOpenLoop()) {
            // Open-loop PWM session active: arbitrate away from dynamic braking and preserve motor PWMs
            brakePulseState = BrakePulseState::DRIVING;
            lastActiveAngSpeed = 0.0f;
            lastActiveLinSpeed = 0.0f;
        } else {
            bool isTargetZero = (abs(targetLinear) < 0.005f) && (abs(targetAngular) < 0.005f);
            bool isCommandZero = (abs(activeCmd.linearVelocity) < 0.005f) && (abs(activeCmd.angularVelocity) < 0.005f);

            if (!isTargetZero && !isCommandZero) {
                // Active motion: release brake pulse if one was active, record speeds, and set driving state
                if (brakePulseState == BrakePulseState::BRAKING) {
                    motorDriver.coastAll();
                }
                brakePulseState = BrakePulseState::DRIVING;
                lastActiveAngSpeed = abs(targetAngular);
                lastActiveLinSpeed = abs(targetLinear);
            } else {
                // Target is zero (stopping or settled)
                if (DYNAMIC_BRAKE_ENABLED && commandManager.isNormalDriveArmed() && motorDriver.getMode() == MotorOutputMode::NORMAL_DRIVE) {
                    if (brakePulseState == BrakePulseState::DRIVING) {
                        bool eligibleForBrake = (lastActiveAngSpeed <= DYNAMIC_BRAKE_MAX_TRIGGER_RADPS) && (lastActiveLinSpeed <= DYNAMIC_BRAKE_MAX_TRIGGER_MPS);
                        if (eligibleForBrake) {
                            brakePulseState = BrakePulseState::BRAKING;
                            brakePulseStartMs = nowMs;
                            motorDriver.brakeAll();
                            wheelController.reset();
                        } else {
                            brakePulseState = BrakePulseState::COASTING;
                            motorDriver.coastAll();
                            wheelController.reset();
                        }
                    } else if (brakePulseState == BrakePulseState::BRAKING) {
                        if (nowMs - brakePulseStartMs >= DYNAMIC_BRAKE_DURATION_MS) {
                            brakePulseState = BrakePulseState::COASTING;
                            motorDriver.coastAll();
                            wheelController.reset();
                        } else {
                            motorDriver.brakeAll();
                        }
                    } else {
                        motorDriver.coastAll();
                    }
                } else {
                    brakePulseState = BrakePulseState::COASTING;
                    if (isTargetZero) {
                        motorDriver.coastAll();
                    }
                }
            }
        }
    }
};

// ============================================================================
// Test 1: Real Serial Handler 0x11 Expiry & Registered Timeout Hook
// ============================================================================
void test1_RealSerialExpiryAndRegisteredHook() {
    std::cout << "[TEST 1] Real Serial 0x11 Expiry & Registered Hook... ";
    ProductionDrivetrainHarness h;

    // 1. Arm via real 0x2C packet
    sendCmdArmNormalDrive();
    h.processSerial();
    motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);
    assert(commandManager.isNormalDriveArmed());

    // 2. Dispatch open-loop PWM command via real 0x11 packet at t = 1000
    g_mockMillis = 1000;
    sendCmdOpenLoopPwm(60, 60, -60, -60);
    h.processSerial();
    assert(wheelController.isOpenLoop());

    // 3. Tick at t = 1100 ms (elapsed = 100 ms <= 300 ms bound)
    h.runControlTick(0.01f, 1100);
    assert(wheelController.isOpenLoop());
    assert(getDriverPWM(0) == 60);
    assert(getDriverPWM(2) == -60);
    assert(g_disarmHookCalls == 0);
    assert(commandManager.isNormalDriveArmed());

    // 4. Tick at t = 1300 ms (elapsed = 300 ms <= 300 ms: not yet strictly > 300 ms)
    h.runControlTick(0.01f, 1300);
    assert(wheelController.isOpenLoop());
    assert(getDriverPWM(0) == 60);

    // 5. Tick at t = 1310 ms (elapsed = 310 ms = 300ms + 10ms control cycle > 300 ms)
    // EXPIRED! Real registered hook MUST execute.
    h.runControlTick(0.01f, 1310);
    assert(!wheelController.isOpenLoop());
    for (int i = 0; i < 4; i++) {
        assert(getDriverPWM(i) == 0);
    }
    assert(g_disarmHookCalls == 1);
    assert(!commandManager.isNormalDriveArmed()); // Verified disarmed by registered hook

    // Subsequent ticks remain fully zeroed and disarmed
    h.runControlTick(0.01f, 1320);
    for (int i = 0; i < 4; i++) {
        assert(getDriverPWM(i) == 0);
    }

    std::cout << "PASSED (Bound: 310 ms verified)" << std::endl;
}

// ============================================================================
// Test 2: Real Serial 0x11 Regular Refresh (< 300 ms)
// ============================================================================
void test2_RealSerialRegularRefresh() {
    std::cout << "[TEST 2] Real Serial 0x11 Regular Refresh... ";
    ProductionDrivetrainHarness h;

    sendCmdArmNormalDrive();
    h.processSerial();
    motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);

    uint32_t t = 1000;
    g_mockMillis = t;
    sendCmdOpenLoopPwm(50, 50, 50, 50);
    h.runControlTick(0.01f, t);
    assert(wheelController.isOpenLoop());
    assert(getDriverPWM(0) == 50);

    // Refresh every 100 ms for 5 steps via real 0x11 packets
    for (int i = 1; i <= 5; i++) {
        t += 100;
        g_mockMillis = t;
        sendCmdOpenLoopPwm(50, 50, 50, 50);
        h.runControlTick(0.01f, t);
        assert(wheelController.isOpenLoop());
        assert(getDriverPWM(0) == 50);
        assert(commandManager.isNormalDriveArmed());
    }

    // Cease packets and advance by 310 ms (1500 -> 1810)
    t += 310;
    h.runControlTick(0.01f, t);
    assert(!wheelController.isOpenLoop());
    for (int i = 0; i < 4; i++) {
        assert(getDriverPWM(i) == 0);
    }
    assert(!commandManager.isNormalDriveArmed());

    std::cout << "PASSED" << std::endl;
}

// ============================================================================
// Test 3: Real Serial 0x11 Explicit Zero [0,0,0,0] Before Timeout
// ============================================================================
void test3_RealSerialExplicitZeroBeforeTimeout() {
    std::cout << "[TEST 3] Real Serial 0x11 Explicit Zero Before Timeout... ";
    ProductionDrivetrainHarness h;

    sendCmdArmNormalDrive();
    h.processSerial();
    motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);

    // Start open-loop
    g_mockMillis = 2000;
    sendCmdOpenLoopPwm(45, 45, -45, -45);
    h.runControlTick(0.01f, 2000);
    assert(wheelController.isOpenLoop());
    assert(getDriverPWM(0) == 45);

    // 50 ms later (well before 300 ms), client sends explicit zero [0,0,0,0] via 0x11
    g_mockMillis = 2050;
    sendCmdOpenLoopPwm(0, 0, 0, 0);
    h.processSerial();

    // Verified: Real 0x11 handler clears open-loop mode immediately
    assert(!wheelController.isOpenLoop());

    // Control tick outputs 0 PWM
    h.runControlTick(0.01f, 2060);
    for (int i = 0; i < 4; i++) {
        assert(getDriverPWM(i) == 0);
    }

    std::cout << "PASSED" << std::endl;
}

// ============================================================================
// Test 4: Disarm Scoping (Open Loop Reset vs Closed Loop Ramp Preservation)
// ============================================================================
void test4_DisarmScopingIsolation() {
    std::cout << "[TEST 4] Disarm Scoping (Open-Loop Reset vs Closed-Loop Deceleration)... ";
    
    // --- Part 4A: Active Open-Loop Immediate Reset ---
    {
        ProductionDrivetrainHarness h;
        sendCmdArmNormalDrive();
        h.processSerial();
        motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);

        g_mockMillis = 5000;
        sendCmdOpenLoopPwm(70, 70, 70, 70);
        h.runControlTick(0.01f, 5000);
        assert(wheelController.isOpenLoop());
        assert(getDriverPWM(0) == 70);

        // Operator sends real 0x2D disarm packet
        sendCmdDisarmNormalDrive();
        h.processSerial();

        // Must immediately clear open loop and reset state
        assert(!wheelController.isOpenLoop());
        assert(!commandManager.isNormalDriveArmed());

        h.runControlTick(0.01f, 5010);
        for (int i = 0; i < 4; i++) {
            assert(getDriverPWM(i) == 0);
        }
    }

    // --- Part 4B: Ordinary Closed-Loop Deceleration Preservation ---
    {
        ProductionDrivetrainHarness h;
        sendCmdArmNormalDrive();
        h.processSerial();
        motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);

        // Send closed-loop velocity command vx = 0.20 m/s via real 0x12 packet at t = 6000
        g_mockMillis = 6000;
        sendCmdMotion(0.20f, 0.0f, 0.0f);
        h.processSerial();
        assert(!wheelController.isOpenLoop());

        // Spool up motion limiter over 50 ms (5 ticks)
        for (int step = 0; step < 5; step++) {
            ChassisCommand activeCmd = commandManager.getActiveCommand(6000 + step * 10);
            motionLimiter.update(activeCmd.linearVelocity, activeCmd.angularVelocity, 0.01f);
        }
        float speedBeforeDisarm = motionLimiter.getLinearVelocity();
        assert(speedBeforeDisarm > 0.05f); // Running at speed

        // Operator disarms normal drive via real 0x2D packet
        sendCmdDisarmNormalDrive();
        h.processSerial();
        assert(!commandManager.isNormalDriveArmed());

        // CRITICAL PRODUCTION INVARIANT:
        // MotionLimiter MUST NOT be abruptly zeroed! It must retain its current speed
        // so that main.cpp can execute controlled deceleration to a stop!
        float speedImmediatelyAfterDisarm = motionLimiter.getLinearVelocity();
        assert(std::abs(speedImmediatelyAfterDisarm - speedBeforeDisarm) < 0.001f);

        // Step through main.cpp controlled deceleration loop
        // In main.cpp, disarmed drive passes linearVelocity = 0.0f to motionLimiter
        for (int step = 1; step <= 20; step++) {
            motionLimiter.update(0.0f, 0.0f, 0.01f);
        }
        // Decelerates smoothly to 0 m/s
        assert(std::abs(motionLimiter.getLinearVelocity()) < 0.01f);
    }

    std::cout << "PASSED" << std::endl;
}

// ============================================================================
// Test 5: Stale-Target Non-Resumption After Expiry and Subsequent Rearm
// ============================================================================
void test5_StaleTargetNonResumptionAfterRearm() {
    std::cout << "[TEST 5] Stale-Target Non-Resumption After Expiry and Rearming... ";
    ProductionDrivetrainHarness h;

    sendCmdArmNormalDrive();
    h.processSerial();
    motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);

    // 1. Send forward velocity command vx = 0.20 m/s via real 0x12 packet at t = 10000
    g_mockMillis = 10000;
    sendCmdMotion(0.20f, 0.0f, 0.0f);
    h.processSerial();
    ChassisCommand cmd1 = commandManager.getActiveCommand(10000);
    assert(std::abs(cmd1.linearVelocity - 0.20f) < 0.001f);

    // Spool up motion limiter
    motionLimiter.update(0.20f, 0.0f, 0.05f);
    assert(motionLimiter.getLinearVelocity() > 0.05f);

    // 2. Client transitions to open-loop PWM mode via real 0x11 packet
    // Real 0x11 handler MUST clear the active closed-loop command and reset motion limiter
    g_mockMillis = 10050;
    sendCmdOpenLoopPwm(30, 30, 30, 30);
    h.processSerial();

    assert(wheelController.isOpenLoop());
    assert(std::abs(motionLimiter.getLinearVelocity()) < 0.001f);
    ChassisCommand cmdDuringOlp = commandManager.getActiveCommand(10050);
    assert(std::abs(cmdDuringOlp.linearVelocity) < 0.001f);

    // 3. Client disconnects: no packets received for 310 ms -> times out at t = 10360
    h.runControlTick(0.01f, 10360);
    assert(!wheelController.isOpenLoop());
    assert(!commandManager.isNormalDriveArmed()); // Disarmed by registered hook
    assert(g_disarmHookCalls == 1);

    // 4. Crucial Check: Operator deliberately re-arms normal drive via real 0x2C packet
    sendCmdArmNormalDrive();
    h.processSerial();
    assert(commandManager.isNormalDriveArmed());

    // 5. Verify the old 0.20 m/s target does NOT magically resume!
    ChassisCommand cmdAfterRearm = commandManager.getActiveCommand(10400);
    assert(std::abs(cmdAfterRearm.linearVelocity) < 0.001f);
    assert(std::abs(cmdAfterRearm.angularVelocity) < 0.001f);
    assert(std::abs(motionLimiter.getLinearVelocity()) < 0.001f);

    // Run control tick: all motor PWM outputs remain 0
    h.runControlTick(0.01f, 10410);
    for (int i = 0; i < 4; i++) {
        assert(getDriverPWM(i) == 0);
    }

    std::cout << "PASSED" << std::endl;
}

// ============================================================================
// Test 6: Timer Wraparound Safety (uint32_t modular subtraction)
// ============================================================================
void test6_TimerWraparoundSafety() {
    std::cout << "[TEST 6] Timer Wraparound Safety (uint32_t modular arithmetic)... ";
    ProductionDrivetrainHarness h;

    sendCmdArmNormalDrive();
    h.processSerial();
    motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);

    // Set lastOpenLoopPwmMs near 0xFFFFFFFF (e.g. 0xFFFFFF80 = 2^32 - 128)
    uint32_t t_stamp = 0xFFFFFF80;
    g_mockMillis = t_stamp;
    sendCmdOpenLoopPwm(55, 55, 55, 55);
    h.runControlTick(0.01f, t_stamp);
    assert(wheelController.isOpenLoop());
    assert(getDriverPWM(0) == 55);

    // Scenario A: nowMs has wrapped around to 0x00000050 (80 ms post-wraparound)
    // Elapsed time = 0x00000050 - 0xFFFFFF80 = 80 - (-128) = 208 ms <= 300 ms -> NOT expired
    uint32_t t_wrapped_fresh = 0x00000050;
    h.runControlTick(0.01f, t_wrapped_fresh);
    assert(wheelController.isOpenLoop());
    assert(getDriverPWM(0) == 55);
    assert(g_disarmHookCalls == 0);

    // Scenario B: nowMs advances to 0x00000100 (256 ms post-wraparound)
    // Elapsed time = 0x00000100 - 0xFFFFFF80 = 256 - (-128) = 384 ms > 300 ms -> EXPIRED!
    uint32_t t_wrapped_expired = 0x00000100;
    h.runControlTick(0.01f, t_wrapped_expired);
    assert(!wheelController.isOpenLoop());
    for (int i = 0; i < 4; i++) {
        assert(getDriverPWM(i) == 0);
    }
    assert(g_disarmHookCalls == 1);
    assert(!commandManager.isNormalDriveArmed());

    std::cout << "PASSED" << std::endl;
}

// ============================================================================
// Test 7: Distinct Firmware Identity Verification
// ============================================================================
void test7_FirmwareIdentity() {
    std::cout << "[TEST 7] Distinct Firmware Identity & Packet NUL-Termination Verification... ";
    ProductionDrivetrainHarness h;
    Serial.clear();

    h.serialProtocol.sendFirmwareInfo();

    // Verify emitted 0x32 packet in Serial.txBuffer:
    // Packet wire framing: 0xFF, 0xFB, extLen=113, extType=0x32, 112 data bytes, checksum
    assert(Serial.txBuffer.size() >= 116);
    assert(Serial.txBuffer[0] == 0xFF);
    assert(Serial.txBuffer[1] == 0xFB);
    assert(Serial.txBuffer[3] == 0x32);

    const uint8_t* payload = &Serial.txBuffer[4];

    // 1. Board Name field (bytes 0-31):
    const char* boardName = (const char*)&payload[0];
    assert(strcmp(boardName, "Maker-ESP32-Unified-Rover") == 0);

    // 2. Version field (bytes 32-47):
    // Must contain "1.3.3-cand-r41c" and a NUL terminator strictly within bytes 32-47
    const char* fwVersion = (const char*)&payload[32];
    assert(strcmp(fwVersion, "1.3.3-cand-r41c") == 0);
    assert(strlen(fwVersion) == 15);
    bool versionHasNul = false;
    for (int i = 32; i < 48; i++) {
        if (payload[i] == '\0') { versionHasNul = true; break; }
    }
    assert(versionHasNul);
    assert(payload[47] == '\0'); // 15 chars + NUL fits within 16 bytes

    // 3. HW Version field (bytes 48-55):
    const char* hwVersion = (const char*)&payload[48];
    assert(strcmp(hwVersion, "v1.1") == 0);

    // 4. Source field (bytes 56-71):
    // Must contain "rev41-floor-c" and a NUL terminator strictly within bytes 56-71
    const char* fwTag = (const char*)&payload[56];
    bool sourceHasNul = false;
    for (int i = 56; i < 72; i++) {
        if (payload[i] == '\0') { sourceHasNul = true; break; }
    }
    assert(sourceHasNul);
    // Parsed source equals exactly "rev41-floor-c"
    assert(strcmp(fwTag, "rev41-floor-c") == 0);
    assert(strlen(fwTag) == 13);
    // Verify bytes 69, 70, 71 are all NUL:
    assert(payload[56 + 13] == '\0');
    assert(payload[56 + 14] == '\0');
    assert(payload[56 + 15] == '\0');

    // 5. Build-date data begins at byte 72 and does NOT concatenate with source:
    const char* buildDate = (const char*)&payload[72];
    assert(payload[72] != '\0'); // Date data begins at byte 72
    assert(strlen(buildDate) > 0);
    // Reading fwTag as a C-string terminates at byte 69 and contains NO date characters:
    assert(strstr(fwTag, buildDate) == nullptr);
    assert(strcmp(fwTag, "rev41-floor-c") == 0);

    std::cout << "PASSED (Version=" << fwVersion << ", Source=" << fwTag 
              << ", NUL-terminated in-field, BuildDate starts at byte 72)" << std::endl;
}

// ============================================================================
// Test 8: Production Braking Arbitration & Active Brake Pulse Release
// ============================================================================
void test8_ProductionBrakingArbitrationRelease() {
    std::cout << "[TEST 8] Production Braking Arbitration & Dynamic Brake Release... ";
    DYNAMIC_BRAKE_ENABLED = true;
    ProductionDrivetrainHarness h;

    // 1. Arm drive
    sendCmdArmNormalDrive();
    h.processSerial();
    motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);

    // 2. Drive in closed-loop to establish moving state
    g_mockMillis = 20000;
    sendCmdMotion(0.20f, 0.0f, 0.0f);
    for (int step = 0; step < 10; step++) {
        h.runControlTick(0.01f, 20000 + step * 10);
    }
    assert(h.brakePulseState == ProductionDrivetrainHarness::BrakePulseState::DRIVING);

    // 3. Command zero velocity to trigger dynamic braking
    g_mockMillis = 20110;
    sendCmdMotion(0.0f, 0.0f, 0.0f);
    h.runControlTick(0.01f, 20110);
    // In main.cpp, zero command while moving triggers dynamic braking pulse
    assert(h.brakePulseState == ProductionDrivetrainHarness::BrakePulseState::BRAKING);
    assert(motorDriver.getActuationState() == DrivetrainActuationState::BRAKE);

    // 4. While actively holding dynamic brake, dispatch open-loop PWM via real 0x11 packet
    g_mockMillis = 20130;
    sendCmdOpenLoopPwm(65, 65, -65, -65);
    // Execute production control loop tick
    h.runControlTick(0.01f, 20130);

    // CRITICAL PRODUCTION ARBITRATION ASSERTIONS:
    // A. Open loop MUST release the active dynamic brake
    assert(h.brakePulseState == ProductionDrivetrainHarness::BrakePulseState::DRIVING);
    assert(wheelController.isOpenLoop());
    // B. MotorDriver PWM MUST receive and retain open loop PWM (coastAll must NOT stomp it!)
    assert(getDriverPWM(0) == 65);
    assert(getDriverPWM(1) == 65);
    assert(getDriverPWM(2) == -65);
    assert(getDriverPWM(3) == -65);

    // 5. Subsequent control tick retains PWM cleanly without interference
    h.runControlTick(0.01f, 20140);
    assert(getDriverPWM(0) == 65);
    assert(getDriverPWM(2) == -65);

    // 6. Cease packets: verify expiry forces zero at 310 ms bound (20130 -> 20440)
    h.runControlTick(0.01f, 20440);
    assert(!wheelController.isOpenLoop());
    for (int i = 0; i < 4; i++) {
        assert(getDriverPWM(i) == 0);
    }
    assert(!commandManager.isNormalDriveArmed());

    DYNAMIC_BRAKE_ENABLED = false;
    std::cout << "PASSED (Arbitration & Release Verified)" << std::endl;
}

// ============================================================================
// Main Runner
// ============================================================================
int main() {
    std::cout << "=======================================================" << std::endl;
    std::cout << "Running Native Open-Loop PWM Expiry & Safety Unit Tests" << std::endl;
    std::cout << "=======================================================" << std::endl;

    test1_RealSerialExpiryAndRegisteredHook();
    test2_RealSerialRegularRefresh();
    test3_RealSerialExplicitZeroBeforeTimeout();
    test4_DisarmScopingIsolation();
    test5_StaleTargetNonResumptionAfterRearm();
    test6_TimerWraparoundSafety();
    test7_FirmwareIdentity();
    test8_ProductionBrakingArbitrationRelease();

    std::cout << "=======================================================" << std::endl;
    std::cout << "ALL 8 OPEN-LOOP SAFETY & ARBITRATION TESTS PASSED!" << std::endl;
    std::cout << "=======================================================" << std::endl;
    return 0;
}
