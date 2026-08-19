#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <algorithm>
#include <cstring>

// Unit Test Suite for ESP32 Spin-Specific Stiction/Kinetic Assistance Layer with Conservative Breakout Guard

enum StictionState : int16_t {
    STICTION_IDLE = 0,
    STICTION_BOOST = 1,
    STICTION_KINETIC = 2,
    STICTION_BLOCKED = 3
};

static const int STICTION_BOOST_PWM = 48;
static const int SPIN_STICTION_BOOST_PWM = 102;
static const uint16_t SPIN_BREAKOUT_MIN_CYCLES = 10;
static const uint16_t SPIN_BREAKOUT_MAX_CYCLES = 20;
static const uint16_t SPIN_BREAKOUT_SUSTAINED_CYCLES = 3;
static const float SPIN_BREAKOUT_VELOCITY_THRESHOLD = 0.10f;
static const float SPIN_KS_PWM = 58.0f;
static const float SPIN_KINETIC_KS_PWM = 75.0f;
static const float MIN_SPIN_KINETIC_FF_FLOOR = 80.0f;
static const float SPIN_FORWARD_REAR_KINETIC_FLOOR = 94.0f;
static const float SPIN_REVERSE_REAR_KINETIC_FLOOR = 80.0f;
static const float SPIN_PID_KP = 6.0f;


struct SingleWheelControllerMock {
    int index = 0;
    float targetVel = 0.0f;
    float measuredVel = 0.0f;
    bool isSpinManeuver = false;
    bool isForwardRearWheel = false;
    int lastPwm = 0;
    StictionState stictionState = STICTION_IDLE;
    uint16_t boostTicksCount = 0;
    bool boostTicksInitialized = false;
    int32_t boostStartTicks = 0;

    float Kp = 2.2f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float errorSum = 0.0f;
    float lastError = 0.0f;
    float kV = 6.0f;
    float forwardBreakawayPwm = 40.0f;

    void begin(int i) { index = i; reset(); }
    StictionState getStictionState() const { return stictionState; }
    bool isForwardRear() const { return isForwardRearWheel; }
    uint16_t getBoostTicksCount() const { return boostTicksCount; }
    void forceKinetic() { stictionState = STICTION_KINETIC; }

    void setTargetVelocity(float targetRadps, bool isSpin = false, bool isForwardRear = false) {
        isSpinManeuver = isSpin;
        isForwardRearWheel = isForwardRear;
        if ((stictionState == STICTION_IDLE || std::abs(targetVel) < 0.01f) && std::abs(targetRadps) >= 0.01f) {
            stictionState = STICTION_BOOST;
            boostTicksCount = 0;
            boostTicksInitialized = false;
        } else if ((targetVel > 0.01f && targetRadps < -0.01f) || (targetVel < -0.01f && targetRadps > 0.01f)) {
            stictionState = STICTION_BOOST;
            boostTicksCount = 0;
            boostTicksInitialized = false;
        } else if (std::abs(targetRadps) < 0.01f) {
            stictionState = STICTION_IDLE;
            boostTicksCount = 0;
            boostTicksInitialized = false;
        }
        targetVel = targetRadps;
    }

    int update(float measuredRadps, int32_t currentTicks, float dt = 0.01f) {
        measuredVel = measuredRadps;
        if (std::abs(targetVel) < 0.01f) {
            stictionState = STICTION_IDLE;
            lastPwm = 0;
            return 0;
        }

        if (stictionState == STICTION_BOOST) {
            if (!boostTicksInitialized) {
                boostStartTicks = currentTicks;
                boostTicksInitialized = true;
            }
            int32_t deltaTicks = std::abs(currentTicks - boostStartTicks);
            if (!isSpinManeuver && deltaTicks >= 3) {
                stictionState = STICTION_KINETIC;
            } else if (!isSpinManeuver) {
                boostTicksCount++;
                if (boostTicksCount >= 50) {
                    stictionState = STICTION_BLOCKED;
                    lastPwm = 0;
                    return 0;
                }
                int boostPwm = (targetVel > 0.0f) ? STICTION_BOOST_PWM : -STICTION_BOOST_PWM;
                lastPwm = boostPwm;
                return lastPwm;
            } else {
                boostTicksCount++;
                if (boostTicksCount >= SPIN_BREAKOUT_MAX_CYCLES) {
                    stictionState = STICTION_KINETIC;
                } else {
                    if (boostTicksCount >= 50) {
                        stictionState = STICTION_BLOCKED;
                        lastPwm = 0;
                        return 0;
                    }
                    int boostMag = SPIN_STICTION_BOOST_PWM; // 102 PWM
                    int boostPwm = (targetVel > 0.0f) ? boostMag : -boostMag;
                    lastPwm = boostPwm;
                    return lastPwm;
                }
            }
        }

        if (stictionState == STICTION_BLOCKED) {
            lastPwm = 0;
            return 0;
        }

        float error = targetVel - measuredRadps;
        errorSum += error * dt;
        float integralTerm = std::max(-150.0f, std::min(150.0f, errorSum * Ki));
        float derivative = (error - lastError) / dt;
        lastError = error;

        float breakaway = isSpinManeuver ? SPIN_KINETIC_KS_PWM : forwardBreakawayPwm;
        float ffMagnitude = breakaway + (kV * std::abs(targetVel));


        if (stictionState == STICTION_KINETIC && std::abs(targetVel) >= 0.01f) {
            float minFfFloor;
            if (isSpinManeuver) {
                minFfFloor = isForwardRearWheel ? SPIN_FORWARD_REAR_KINETIC_FLOOR : MIN_SPIN_KINETIC_FF_FLOOR;
            } else {
                minFfFloor = 45.0f;
            }
            if (ffMagnitude < minFfFloor) {
                ffMagnitude = minFfFloor;
            }
        }

        float feedforward = (targetVel > 0.0f ? 1.0f : -1.0f) * ffMagnitude;
        float pidCorrection = (Kp * error) + integralTerm + (Kd * derivative);
        float totalPwm = feedforward + pidCorrection;
        lastPwm = std::max(-255, std::min(255, (int)std::round(totalPwm)));
        return lastPwm;
    }

    void reset() {
        errorSum = 0.0f;
        lastError = 0.0f;
        lastPwm = 0;
        stictionState = STICTION_IDLE;
        boostStartTicks = 0;
        boostTicksCount = 0;
        boostTicksInitialized = false;
    }
};

struct WheelControllerMock {
    SingleWheelControllerMock controllers[4];
    uint16_t spinSustainedVelocityCycles = 0;

    void begin() {
        spinSustainedVelocityCycles = 0;
        for (int i = 0; i < 4; i++) controllers[i].begin(i);
    }

    void setTargets(float leftTargetRadps, float rightTargetRadps, bool isSpin) {
        bool m1_isForwardRear = false;
        bool m2_isForwardRear = false;
        bool m3_isForwardRear = isSpin && (leftTargetRadps > 0.01f);
        bool m4_isForwardRear = isSpin && (rightTargetRadps > 0.01f);

        controllers[0].setTargetVelocity(leftTargetRadps, isSpin, m1_isForwardRear);
        controllers[1].setTargetVelocity(rightTargetRadps, isSpin, m2_isForwardRear);
        controllers[2].setTargetVelocity(leftTargetRadps, isSpin, m3_isForwardRear);
        controllers[3].setTargetVelocity(rightTargetRadps, isSpin, m4_isForwardRear);
    }

    void update(const float *measuredVelocities, const int32_t *ticks, float dt) {
        bool anySpinBoost = false;
        for (int i = 0; i < 4; i++) {
            if (controllers[i].getStictionState() == STICTION_BOOST) {
                anySpinBoost = true;
                break;
            }
        }

        if (anySpinBoost) {
            bool lr_moving = (std::abs(measuredVelocities[2]) >= SPIN_BREAKOUT_VELOCITY_THRESHOLD);
            bool rr_moving = (std::abs(measuredVelocities[3]) >= SPIN_BREAKOUT_VELOCITY_THRESHOLD);

            if (lr_moving && rr_moving) {
                spinSustainedVelocityCycles++;
            } else {
                spinSustainedVelocityCycles = 0;
            }

            uint16_t maxBoostCycles = 0;
            for (int i = 0; i < 4; i++) {
                if (controllers[i].getBoostTicksCount() > maxBoostCycles) {
                    maxBoostCycles = controllers[i].getBoostTicksCount();
                }
            }

            bool earlyExitOk = (maxBoostCycles >= SPIN_BREAKOUT_MIN_CYCLES) && (spinSustainedVelocityCycles >= SPIN_BREAKOUT_SUSTAINED_CYCLES);
            bool hardTimeoutOk = (maxBoostCycles >= SPIN_BREAKOUT_MAX_CYCLES);

            if (earlyExitOk || hardTimeoutOk) {
                for (int i = 0; i < 4; i++) controllers[i].forceKinetic();
            }
        } else {
            spinSustainedVelocityCycles = 0;
        }

        for (int i = 0; i < 4; i++) {
            controllers[i].update(measuredVelocities[i], ticks[i], dt);
        }
    }

    void reset() {
        spinSustainedVelocityCycles = 0;
        for (int i = 0; i < 4; i++) controllers[i].reset();
    }
};

int main() {
    std::cout << "=======================================================\n";
    std::cout << "RUNNING C++ UNIT TESTS FOR CONSERVATIVE BREAKOUT GUARD\n";
    std::cout << "=======================================================\n";

    WheelControllerMock wc;
    wc.begin();

    // -----------------------------------------------------------------
    // TEST 1: CCW Direction Selection (M4/RR is forward-driving rear wheel)
    // -----------------------------------------------------------------
    std::cout << "[TEST 1] CCW Direction Selection... ";
    wc.setTargets(-2.0f, +2.0f, true);
    assert(!wc.controllers[0].isForwardRear());
    assert(!wc.controllers[1].isForwardRear());
    assert(!wc.controllers[2].isForwardRear());
    assert(wc.controllers[3].isForwardRear());
    std::cout << "PASSED!\n";

    // -----------------------------------------------------------------
    // TEST 2: CW Direction Selection (M3/LR is forward-driving rear wheel)
    // -----------------------------------------------------------------
    std::cout << "[TEST 2] CW Direction Selection... ";
    wc.setTargets(+2.0f, -2.0f, true);
    assert(!wc.controllers[0].isForwardRear());
    assert(!wc.controllers[1].isForwardRear());
    assert(wc.controllers[2].isForwardRear());
    assert(!wc.controllers[3].isForwardRear());
    std::cout << "PASSED!\n";

    // -----------------------------------------------------------------
    // TEST 3: Breakout Startup Boost Magnitude (102 PWM)
    // -----------------------------------------------------------------
    std::cout << "[TEST 3] Breakout Startup Boost Magnitude (102 PWM)... ";
    wc.reset();
    wc.setTargets(-2.0f, +2.0f, true);
    float measuredZero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int32_t ticksZero[4] = {0, 0, 0, 0};
    wc.update(measuredZero, ticksZero, 0.01f);

    assert(wc.controllers[0].lastPwm == -102);
    assert(wc.controllers[1].lastPwm == +102);
    assert(wc.controllers[2].lastPwm == -102);
    assert(wc.controllers[3].lastPwm == +102);
    std::cout << "PASSED!\n";

    // -----------------------------------------------------------------
    // TEST 4: Backlash / Twitch & Min Dwell Protection (< 100 ms)
    // -----------------------------------------------------------------
    std::cout << "[TEST 4] Backlash Twitch & Min Dwell Protection (<100 ms)... ";
    wc.reset();
    wc.setTargets(-2.0f, +2.0f, true);
    // Cycle 1: 5 ticks twitch displacement with 0.5 rad/s velocity
    float measuredTwitch[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    int32_t ticksTwitch[4] = {5, 5, 5, 5};
    wc.update(measuredTwitch, ticksTwitch, 0.01f);
    assert(wc.controllers[3].getStictionState() == STICTION_BOOST);

    // Cycles 2 to 8: sustained motion present, but min dwell (10 cycles / 100 ms) not reached
    for (int c = 2; c <= 8; c++) {
        wc.update(measuredTwitch, ticksTwitch, 0.01f);
    }
    assert(wc.controllers[3].getStictionState() == STICTION_BOOST);
    assert(wc.controllers[3].lastPwm == +102);
    std::cout << "PASSED!\n";

    // -----------------------------------------------------------------
    // TEST 5: Sustained Rear Motion Early Exit After Dwell
    // -----------------------------------------------------------------
    std::cout << "[TEST 5] Sustained Rear Motion Early Exit After Dwell... ";
    wc.reset();
    wc.setTargets(-2.0f, +2.0f, true);
    // Cycles 1 to 7: zero motion
    for (int c = 1; c <= 7; c++) {
        wc.update(measuredZero, ticksZero, 0.01f);
    }
    assert(wc.controllers[3].getStictionState() == STICTION_BOOST);

    // Cycles 8, 9, 10: sustained rear motion >= 0.10 rad/s on BOTH rear wheels (M3/LR & M4/RR)
    float measuredRearMotion[4] = {0.0f, 0.0f, -0.2f, 0.2f};
    for (int c = 8; c <= 10; c++) {
        wc.update(measuredRearMotion, ticksZero, 0.01f);
    }

    // After cycle 10 (100 ms dwell) + 3 sustained velocity cycles -> transitions to STICTION_KINETIC
    assert(wc.controllers[3].getStictionState() == STICTION_KINETIC);
    std::cout << "PASSED!\n";

    // -----------------------------------------------------------------
    // TEST 6: Hard 200 ms Timeout Fallback
    // -----------------------------------------------------------------
    std::cout << "[TEST 6] Hard 200 ms Timeout Fallback... ";
    wc.reset();
    wc.setTargets(-2.0f, +2.0f, true);
    // 19 cycles without sustained motion
    for (int c = 1; c <= 19; c++) {
        wc.update(measuredZero, ticksZero, 0.01f);
    }
    assert(wc.controllers[3].getStictionState() == STICTION_BOOST);

    // Cycle 20 (200 ms hard timeout) -> forces STICTION_KINETIC
    wc.update(measuredZero, ticksZero, 0.01f);
    assert(wc.controllers[3].getStictionState() == STICTION_KINETIC);
    std::cout << "PASSED!\n";

    // -----------------------------------------------------------------
    // TEST 7: Zero Command & Disarm Immediate Cancellation
    // -----------------------------------------------------------------
    std::cout << "[TEST 7] Zero Command & Disarm Immediate Cancellation... ";
    wc.reset();
    wc.setTargets(-2.0f, +2.0f, true);
    wc.update(measuredZero, ticksZero, 0.01f);
    assert(wc.controllers[3].getStictionState() == STICTION_BOOST);

    wc.setTargets(0.0f, 0.0f, false);
    wc.update(measuredZero, ticksZero, 0.01f);
    for (int i = 0; i < 4; i++) {
        assert(wc.controllers[i].getStictionState() == STICTION_IDLE);
        assert(wc.controllers[i].lastPwm == 0);
    }
    std::cout << "PASSED!\n";

    // -----------------------------------------------------------------
    // TEST 9: SPIN_KINETIC_KS_PWM = 75.0 Feedforward Base (100.2 PWM at 4.195 rad/s)
    // -----------------------------------------------------------------
    std::cout << "[TEST 9] SPIN_KINETIC_KS_PWM = 75.0 Feedforward Base... ";
    wc.reset();
    wc.setTargets(-4.195f, +4.195f, true); // Pure spin at nominal 4.195 rad/s
    for (int i = 0; i < 4; i++) wc.controllers[i].stictionState = STICTION_KINETIC;
    
    // Measured velocity matches target (error = 0) -> output is pure feedforward
    float measuredMatch[4] = {-4.195f, +4.195f, -4.195f, +4.195f};
    wc.update(measuredMatch, ticksZero, 0.01f);
    
    // FF = 75.0 + (6.0 * 4.195) = 100.17 PWM -> ~100 PWM
    assert(wc.controllers[3].lastPwm == +100);
    assert(wc.controllers[1].lastPwm == +100);
    assert(wc.controllers[0].lastPwm == -100);
    assert(wc.controllers[2].lastPwm == -100);

    // Verify low-speed forward-rear floor (94 PWM) still works for target = 1.0 rad/s
    wc.setTargets(-1.0f, +1.0f, true);
    for (int i = 0; i < 4; i++) wc.controllers[i].stictionState = STICTION_KINETIC;
    float measuredMatch1[4] = {-1.0f, +1.0f, -1.0f, +1.0f};
    wc.update(measuredMatch1, ticksZero, 0.01f);
    
    // For M4 forward-rear: FF = 75 + 6 = 81 PWM, clamped to 94 floor -> +94 PWM
    assert(wc.controllers[3].lastPwm == +94);
    // For M1 (ordinary spin wheel): FF = 75 + 6 = 81 PWM (above 80 floor) -> -81 PWM
    assert(wc.controllers[0].lastPwm == -81);

    // Switch to straight driving -> must use normal feedforward (40 + 6*4.195 = 65 PWM)
    wc.setTargets(+4.195f, +4.195f, false); // Straight driving
    for (int i = 0; i < 4; i++) wc.controllers[i].stictionState = STICTION_KINETIC;
    float measuredMatchStraight[4] = {4.195f, 4.195f, 4.195f, 4.195f};
    wc.update(measuredMatchStraight, ticksZero, 0.01f);
    
    // Normal feedforward before PID = 65.17 PWM
    assert(wc.controllers[3].lastPwm == +65);
    std::cout << "PASSED!\n";

    std::cout << "=======================================================\n";
    std::cout << "ALL BREAKOUT GUARD UNIT TESTS PASSED SUCCESSFULLY!\n";
    std::cout << "=======================================================\n";

    return 0;
}


