#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <algorithm>
#include <cstring>

// Stiction state enum
enum StictionState : int16_t {
    STICTION_IDLE = 0,
    STICTION_BOOST = 1,
    STICTION_KINETIC = 2,
    STICTION_BLOCKED = 3
};

struct ChassisCommand {
    float linearVelocity;
    float angularVelocity;
};

// Simulation mocks
struct MockSingleWheelController {
    float targetVel = 0.0f;
    float measuredVel = 0.0f;
    int lastPwm = 0;
    StictionState stictionState = STICTION_IDLE;
    uint16_t boostTicksCount = 0;
    bool boostTicksInitialized = false;
    int32_t boostStartTicks = 0;
    bool isSpinManeuver = false;

    StictionState getStictionState() const { return stictionState; }

    void setTargetVelocity(float targetRadps, bool isSpin) {
        isSpinManeuver = isSpin;
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

    int update(float measuredRadps, int32_t currentTicks, float dt) {
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
            int32_t delta = std::abs(currentTicks - boostStartTicks);
            if (delta >= 3) {
                stictionState = STICTION_KINETIC;
            } else {
                boostTicksCount++;
                if (boostTicksCount >= 50) { // 500 ms @ 100 Hz
                    stictionState = STICTION_BLOCKED;
                    lastPwm = 0;
                    return 0;
                }
                int boostMag = isSpinManeuver ? 58 : 48;
                lastPwm = (targetVel > 0.0f) ? boostMag : -boostMag;
                return lastPwm;
            }
        }
        if (stictionState == STICTION_BLOCKED) {
            lastPwm = 0;
            return 0;
        }
        // STICTION_KINETIC
        float ff = (targetVel > 0.0f ? 1.0f : -1.0f) * 45.0f;
        lastPwm = (int)ff;
        return lastPwm;
    }

    void reset() {
        lastPwm = 0;
        stictionState = STICTION_IDLE;
        boostStartTicks = 0;
        boostTicksCount = 0;
        boostTicksInitialized = false;
    }
};

struct MockWheelController {
    MockSingleWheelController controllers[4];
    void reset() {
        for (int i = 0; i < 4; i++) controllers[i].reset();
    }
    void setTargets(float leftRadps, float rightRadps, bool isSpin) {
        controllers[0].setTargetVelocity(leftRadps, isSpin);
        controllers[1].setTargetVelocity(rightRadps, isSpin);
        controllers[2].setTargetVelocity(leftRadps, isSpin);
        controllers[3].setTargetVelocity(rightRadps, isSpin);
    }
};

// Candidate A State
enum SpinRecoveryState {
    SPIN_RECOVERY_IDLE = 0,
    SPIN_RECOVERY_MONITORING_INITIAL,
    SPIN_RECOVERY_SETTLE_1,
    SPIN_RECOVERY_TWITCH_PULSE,
    SPIN_RECOVERY_SETTLE_2,
    SPIN_RECOVERY_RETRY
};

class ChassisSupervisorHarness {
public:
    MockWheelController wheelCtrl;
    ChassisCommand activeCmd = {0.0f, 0.0f};
    bool isArmed = true;

    int32_t currentTicks[4] = {0, 0, 0, 0};
    float measuredVel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float imuGz = 0.0f;

    // Supervisor state
    bool s_rebreakoutConsumedForEpisode = false;
    uint32_t s_zeroDebounceTicks = 0;
    uint32_t s_stalledKineticTicks = 0;
    int32_t s_stallWindowStartTicks[4] = {0, 0, 0, 0};
    bool s_stallWindowInitialized = false;

    enum ChassisManeuverType {
        MANEUVER_IDLE = 0,
        MANEUVER_TRANSLATION_OR_ARC = 1,
        MANEUVER_PURE_SPIN = 2
    };
    ChassisManeuverType s_prevManeuverType = MANEUVER_IDLE;

    // Candidate A state
    SpinRecoveryState s_spinRecoveryState = SPIN_RECOVERY_IDLE;
    uint32_t s_spinStartTimeMs = 0;
    uint32_t s_spinStateEnteredMs = 0;
    int32_t s_spinStartTicks[4] = {0, 0, 0, 0};
    int32_t s_twitchStartTicks[4] = {0, 0, 0, 0};
    float s_requestedAngular = 0.0f;
    bool s_recoveryAttemptedThisEpisode = false;

    int rebreakoutEventsCount = 0;
    int candidateATwitchCount = 0;

    void updateSpinRecovery(float &targetLinear, float &targetAngular, uint32_t nowMs) {
        const float ZERO_EPSILON = 0.005f;
        bool isOperatorPureSpin = (std::abs(activeCmd.linearVelocity) <= ZERO_EPSILON) && 
                                  (std::abs(activeCmd.angularVelocity) >= ZERO_EPSILON);

        if (!isOperatorPureSpin) {
            s_spinRecoveryState = SPIN_RECOVERY_IDLE;
            s_recoveryAttemptedThisEpisode = false;
            return;
        }

        switch (s_spinRecoveryState) {
            case SPIN_RECOVERY_IDLE:
                if (!s_recoveryAttemptedThisEpisode) {
                    s_spinRecoveryState = SPIN_RECOVERY_MONITORING_INITIAL;
                    s_spinStartTimeMs = nowMs;
                    s_requestedAngular = activeCmd.angularVelocity;
                    for (int i = 0; i < 4; i++) s_spinStartTicks[i] = currentTicks[i];
                }
                break;

            case SPIN_RECOVERY_MONITORING_INITIAL: {
                int movingWheels = 0;
                for (int i = 0; i < 4; i++) {
                    if (std::abs(currentTicks[i] - s_spinStartTicks[i]) >= 3) movingWheels++;
                }
                if (movingWheels == 4 || std::abs(imuGz) >= 0.08f) {
                    s_spinRecoveryState = SPIN_RECOVERY_IDLE;
                    break;
                }
                if (nowMs - s_spinStartTimeMs >= 350) {
                    s_spinRecoveryState = SPIN_RECOVERY_SETTLE_1;
                    s_spinStateEnteredMs = nowMs;
                    s_recoveryAttemptedThisEpisode = true;
                    wheelCtrl.reset();
                    targetLinear = 0.0f;
                    targetAngular = 0.0f;
                }
                break;
            }

            case SPIN_RECOVERY_SETTLE_1:
                targetLinear = 0.0f;
                targetAngular = 0.0f;
                if (nowMs - s_spinStateEnteredMs >= 30) {
                    s_spinRecoveryState = SPIN_RECOVERY_TWITCH_PULSE;
                    s_spinStateEnteredMs = nowMs;
                    for (int i = 0; i < 4; i++) s_twitchStartTicks[i] = currentTicks[i];
                    wheelCtrl.reset();
                    candidateATwitchCount++;
                }
                break;

            case SPIN_RECOVERY_TWITCH_PULSE: {
                float sign_w = (s_requestedAngular >= 0.0f) ? 1.0f : -1.0f;
                targetLinear = 0.0f;
                targetAngular = -sign_w * 0.8f;
                if (nowMs - s_spinStateEnteredMs >= 50) {
                    s_spinRecoveryState = SPIN_RECOVERY_SETTLE_2;
                    s_spinStateEnteredMs = nowMs;
                    wheelCtrl.reset();
                    targetLinear = 0.0f;
                    targetAngular = 0.0f;
                }
                break;
            }

            case SPIN_RECOVERY_SETTLE_2:
                targetLinear = 0.0f;
                targetAngular = 0.0f;
                if (nowMs - s_spinStateEnteredMs >= 30) {
                    s_spinRecoveryState = SPIN_RECOVERY_RETRY;
                    wheelCtrl.reset();
                }
                break;

            case SPIN_RECOVERY_RETRY:
                targetLinear = 0.0f;
                targetAngular = s_requestedAngular;
                break;
        }
    }

    void runCycle(uint32_t nowMs, float dt = 0.01f) {
        const float CMD_LINEAR_EPSILON = 0.005f;
        const float CMD_ANGULAR_EPSILON = 0.005f;

        bool isCmdZero = (std::abs(activeCmd.linearVelocity) < CMD_LINEAR_EPSILON) && 
                         (std::abs(activeCmd.angularVelocity) < CMD_ANGULAR_EPSILON);

        // 1. Episode tracking & debounce
        if (!isArmed) {
            s_rebreakoutConsumedForEpisode = false;
            s_zeroDebounceTicks = 0;
            s_stalledKineticTicks = 0;
            s_stallWindowInitialized = false;
            s_prevManeuverType = MANEUVER_IDLE;
        } else if (isCmdZero) {
            s_zeroDebounceTicks++;
            if (s_zeroDebounceTicks >= 5) { // 50 ms @ 100 Hz
                s_rebreakoutConsumedForEpisode = false;
                s_stalledKineticTicks = 0;
                s_stallWindowInitialized = false;
                s_prevManeuverType = MANEUVER_IDLE;
            }
        } else {
            s_zeroDebounceTicks = 0;
        }

        // 2. Maneuver classification
        ChassisManeuverType currentManeuver = MANEUVER_IDLE;
        if (isArmed && !isCmdZero) {
            if (std::abs(activeCmd.linearVelocity) <= CMD_LINEAR_EPSILON && std::abs(activeCmd.angularVelocity) >= CMD_ANGULAR_EPSILON) {
                currentManeuver = MANEUVER_PURE_SPIN;
            } else {
                currentManeuver = MANEUVER_TRANSLATION_OR_ARC;
            }
        }

        // 3. Stalled Arc -> Pure Spin Transition
        if (isArmed && currentManeuver == MANEUVER_PURE_SPIN && s_prevManeuverType == MANEUVER_TRANSLATION_OR_ARC) {
            int stationaryWheels = 0;
            for (int i = 0; i < 4; i++) {
                if (std::abs(measuredVel[i]) < 0.05f) stationaryWheels++;
            }
            bool isChassisStationary = (stationaryWheels >= 3) && (std::abs(imuGz) < 0.04f);

            if (isChassisStationary && !s_rebreakoutConsumedForEpisode) {
                wheelCtrl.reset();
                s_rebreakoutConsumedForEpisode = true;
                s_stalledKineticTicks = 0;
                s_stallWindowInitialized = false;
                s_spinRecoveryState = SPIN_RECOVERY_IDLE;
                s_recoveryAttemptedThisEpisode = false;
                rebreakoutEventsCount++;
            }
        }
        s_prevManeuverType = currentManeuver;

        // 4. Coordinated Stalled-KINETIC Gate
        if (isArmed && !isCmdZero && !s_rebreakoutConsumedForEpisode) {
            bool allKinetic = true;
            for (int i = 0; i < 4; i++) {
                if (wheelCtrl.controllers[i].getStictionState() != STICTION_KINETIC) {
                    allKinetic = false;
                    break;
                }
            }

            int zeroVelCount = 0;
            for (int i = 0; i < 4; i++) {
                if (std::abs(measuredVel[i]) < 0.05f) zeroVelCount++;
            }

            if (!s_stallWindowInitialized) {
                for (int i = 0; i < 4; i++) s_stallWindowStartTicks[i] = currentTicks[i];
                s_stallWindowInitialized = true;
            }

            int staticDispCount = 0;
            for (int i = 0; i < 4; i++) {
                if (std::abs(currentTicks[i] - s_stallWindowStartTicks[i]) < 3) staticDispCount++;
            }

            bool imuStationary = (std::abs(imuGz) < 0.04f);
            bool chassisStalled = allKinetic && (zeroVelCount >= 3) && (staticDispCount >= 3) && imuStationary;

            if (chassisStalled) {
                s_stalledKineticTicks++;
                if (s_stalledKineticTicks >= 20) { // 200 ms @ 100 Hz
                    wheelCtrl.reset();
                    s_rebreakoutConsumedForEpisode = true;
                    s_stalledKineticTicks = 0;
                    s_stallWindowInitialized = false;
                    rebreakoutEventsCount++;

                    if (currentManeuver == MANEUVER_PURE_SPIN) {
                        s_spinRecoveryState = SPIN_RECOVERY_IDLE;
                        s_recoveryAttemptedThisEpisode = false;
                    }
                }
            } else {
                s_stalledKineticTicks = 0;
                s_stallWindowInitialized = false;
            }
        } else {
            s_stalledKineticTicks = 0;
            s_stallWindowInitialized = false;
        }

        // Apply Candidate-A overrides
        float targetLinear = activeCmd.linearVelocity;
        float targetAngular = activeCmd.angularVelocity;
        updateSpinRecovery(targetLinear, targetAngular, nowMs);

        // Kinematics
        float W_eff = 0.340858f;
        float R = 0.0325f;
        float vL = targetLinear - (targetAngular * W_eff / 2.0f);
        float vR = targetLinear + (targetAngular * W_eff / 2.0f);
        bool isSpin = (std::abs(targetLinear) <= CMD_LINEAR_EPSILON) && (std::abs(targetAngular) >= CMD_ANGULAR_EPSILON);

        wheelCtrl.setTargets(vL / R, vR / R, isSpin);

        for (int i = 0; i < 4; i++) {
            wheelCtrl.controllers[i].update(measuredVel[i], currentTicks[i], dt);
        }
    }
};

void runAllTests() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  RUNNING CHASSIS STALL RECOVERY STATE-MACHINE TEST HARNESS     " << std::endl;
    std::cout << "=================================================================" << std::endl;

    // Test 1: KINETIC + real movement -> no recovery
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.12f, 0.0f};
        for (int i = 0; i < 30; i++) {
            for (int w = 0; w < 4; w++) {
                h.currentTicks[w] += 10;
                h.measuredVel[w] = 3.5f;
            }
            h.imuGz = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 0);
        for (int w = 0; w < 4; w++) assert(h.wheelCtrl.controllers[w].getStictionState() == STICTION_KINETIC);
        std::cout << "[PASS] Test 1: KINETIC + real movement -> no recovery." << std::endl;
    }

    // Test 2: One slow/stalled wheel only -> no chassis recovery
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.12f, 0.0f};
        // Break stiction first
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        // Wheel 0 stalls, but wheels 1, 2, 3 keep moving
        for (int i = 5; i < 35; i++) {
            h.measuredVel[0] = 0.0f;
            for (int w = 1; w < 4; w++) {
                h.currentTicks[w] += 10;
                h.measuredVel[w] = 3.5f;
            }
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 0);
        std::cout << "[PASS] Test 2: One slow/stalled wheel only -> no chassis recovery." << std::endl;
    }

    // Test 3: Three stationary wheels but IMU rotating -> no recovery
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.0f, 0.5f};
        // Break stiction
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += (w%2==0?-5:5);
            h.runCycle(i * 10);
        }
        // Wheels stationary but IMU gyro reads 0.15 rad/s (skidding/inertial rotation)
        for (int i = 5; i < 35; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.imuGz = 0.15f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 0);
        std::cout << "[PASS] Test 3: Three stationary wheels but IMU rotating -> no recovery." << std::endl;
    }

    // Test 4: Full accepted stall gate for <200 ms -> no recovery
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.06f, 0.0f};
        // Enter KINETIC
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        // Stall for 180 ms (18 cycles)
        for (int i = 5; i < 23; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.imuGz = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 0);
        std::cout << "[PASS] Test 4: Full accepted stall gate for <200 ms -> no recovery." << std::endl;
    }

    // Test 5: Full gate continuously >=200 ms -> exactly one recovery
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.06f, 0.0f};
        // Enter KINETIC
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        // Stall for 210 ms (21 cycles)
        for (int i = 5; i < 26; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.imuGz = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 1);
        assert(h.s_rebreakoutConsumedForEpisode == true);
        std::cout << "[PASS] Test 5: Full gate continuously >=200 ms -> exactly one recovery." << std::endl;
    }

    // Test 6: Successful movement after recovery does NOT re-arm the episode
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.06f, 0.0f};
        // Enter KINETIC and stall 200 ms
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        for (int i = 5; i < 26; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 1);
        // Breakout succeeds, moves for 500 ms
        for (int i = 26; i < 76; i++) {
            for (int w = 0; w < 4; w++) {
                h.currentTicks[w] += 10;
                h.measuredVel[w] = 2.0f;
            }
            h.runCycle(i * 10);
        }
        assert(h.s_rebreakoutConsumedForEpisode == true); // Remains latched!
        std::cout << "[PASS] Test 6: Successful movement after recovery does NOT re-arm the episode." << std::endl;
    }

    // Test 7: Later stall in same continuous episode -> no second recovery
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.06f, 0.0f};
        // Initial stall -> recovery (1)
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        for (int i = 5; i < 26; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 1);
        // Resumes for 200 ms
        for (int i = 26; i < 46; i++) {
            for (int w = 0; w < 4; w++) {
                h.currentTicks[w] += 10;
                h.measuredVel[w] = 2.0f;
            }
            h.runCycle(i * 10);
        }
        // Stalls again for 500 ms in same continuous command
        for (int i = 46; i < 96; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 1); // Still exactly 1! No second recovery.
        std::cout << "[PASS] Test 7: Later stall in same continuous episode -> no second recovery." << std::endl;
    }

    // Test 8: Arc -> pure spin while already stalled and recovery unused -> synchronized 58-PWM fresh spin breakout
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.04f, 0.72f}; // Arc command
        // Enter KINETIC
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        // Stall during arc for 100 ms
        for (int i = 5; i < 15; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 0);
        // Command transitions to pure spin (vx = 0.0, wz = 0.52)
        h.activeCmd = {0.0f, 0.52f};
        h.runCycle(150);
        assert(h.rebreakoutEventsCount == 1);
        assert(h.wheelCtrl.controllers[0].getStictionState() == STICTION_BOOST);
        assert(std::abs(h.wheelCtrl.controllers[0].lastPwm) == 58); // 58 PWM Pure Spin Boost!
        std::cout << "[PASS] Test 8: Arc -> pure spin while stalled & recovery unused -> 58-PWM spin breakout." << std::endl;
    }

    // Test 9: Arc -> spin -> arc -> spin after recovery consumed -> no second recovery
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.04f, 0.72f};
        // Enter KINETIC and stall -> consume recovery
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        for (int i = 5; i < 26; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 1);

        // Oscillate arc -> spin -> arc -> spin while stalled
        h.activeCmd = {0.0f, 0.52f};
        h.runCycle(270);
        h.activeCmd = {0.04f, 0.72f};
        h.runCycle(280);
        h.activeCmd = {0.0f, 0.52f};
        h.runCycle(290);
        assert(h.rebreakoutEventsCount == 1); // Strictly 1!
        std::cout << "[PASS] Test 9: Arc -> spin -> arc -> spin after recovery consumed -> no second recovery." << std::endl;
    }

    // Test 10: True zero command >=50 ms -> new episode may recover once again
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.06f, 0.0f};
        // Consume recovery in episode 1
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        for (int i = 5; i < 26; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 1);

        // Command zero for 60 ms (6 cycles)
        h.activeCmd = {0.0f, 0.0f};
        for (int i = 26; i < 32; i++) h.runCycle(i * 10);
        assert(h.s_rebreakoutConsumedForEpisode == false); // Reset!

        // Episode 2 starts
        h.activeCmd = {0.06f, 0.0f};
        for (int i = 32; i < 37; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        for (int i = 37; i < 58; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.rebreakoutEventsCount == 2); // Allowed in fresh episode!
        std::cout << "[PASS] Test 10: True zero command >=50 ms -> new episode may recover once again." << std::endl;
    }

    // Test 11: Disarm -> immediate reset
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.06f, 0.0f};
        // Consume recovery
        for (int i = 0; i < 5; i++) {
            for (int w = 0; w < 4; w++) h.currentTicks[w] += 5;
            h.runCycle(i * 10);
        }
        for (int i = 5; i < 26; i++) {
            for (int w = 0; w < 4; w++) h.measuredVel[w] = 0.0f;
            h.runCycle(i * 10);
        }
        assert(h.s_rebreakoutConsumedForEpisode == true);
        // Disarm
        h.isArmed = false;
        h.runCycle(270);
        assert(h.s_rebreakoutConsumedForEpisode == false); // Immediate reset on 0 ms!
        std::cout << "[PASS] Test 11: Disarm -> immediate reset." << std::endl;
    }

    // Test 12: Candidate-A initial/twitch/retry timers cannot prematurely trigger BLOCKED
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.0f, 0.52f}; // Pure spin
        // Monitor for 350 ms without wheel ticks -> Candidate-A twitch triggers at 380 ms
        for (int i = 0; i < 40; i++) {
            h.runCycle(i * 10);
        }
        assert(h.candidateATwitchCount == 1);
        for (int w = 0; w < 4; w++) {
            assert(h.wheelCtrl.controllers[w].getStictionState() != STICTION_BLOCKED);
        }
        std::cout << "[PASS] Test 12: Candidate-A initial/twitch/retry timers cannot prematurely trigger BLOCKED." << std::endl;
    }

    // Test 13: Failed final retry reaches BLOCKED and 0 PWM
    {
        ChassisSupervisorHarness h;
        h.activeCmd = {0.0f, 0.52f};
        // Candidate-A triggers twitch at 380 ms, retry starts at 460 ms, stalls for 500 ms in retry (up to 1000 ms)
        for (int i = 0; i < 110; i++) {
            h.runCycle(i * 10);
        }
        for (int w = 0; w < 4; w++) {
            assert(h.wheelCtrl.controllers[w].getStictionState() == STICTION_BLOCKED);
            assert(h.wheelCtrl.controllers[w].lastPwm == 0);
        }
        std::cout << "[PASS] Test 13: Failed final retry reaches BLOCKED and 0 PWM." << std::endl;
    }

    std::cout << "\n>>> ALL 13 TEST CASES PASSED PERFECTLY! <<<\n" << std::endl;
}

int main() {
    runAllTests();
    return 0;
}
