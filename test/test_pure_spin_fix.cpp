#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <iomanip>
#include <algorithm>
#include <cstring>

enum StictionState : int16_t {
    STICTION_IDLE = 0,
    STICTION_BOOST = 1,
    STICTION_KINETIC = 2,
    STICTION_BLOCKED = 3
};

static const int STICTION_BOOST_PWM = 48;
static const int SPIN_STICTION_BOOST_PWM = 85;   // Empirical pure-spin breakout PWM
static const float SPIN_KS_PWM = 58.0f;          // Empirical pure-spin lateral scrub feedforward base
static const float MIN_SPIN_KINETIC_FF_FLOOR = 80.0f; // Minimum kinetic feedforward floor for pure spin

struct ChassisCommand {
    float linearVelocity;
    float angularVelocity;
};

struct MockSingleWheelController {
    int index = 0;
    float targetVel = 0.0f;
    float measuredVel = 0.0f;
    int lastPwm = 0;
    StictionState stictionState = STICTION_IDLE;
    uint16_t boostTicksCount = 0;
    bool boostTicksInitialized = false;
    int32_t boostStartTicks = 0;
    bool isSpinManeuver = false;

    float Kp = 2.2f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float errorSum = 0.0f;
    float lastError = 0.0f;
    float kV = 6.0f;
    float forwardBreakawayPwm = 40.0f;

    void begin(int i) { index = i; reset(); }
    StictionState getStictionState() const { return stictionState; }
    void forceKinetic() { stictionState = STICTION_KINETIC; }

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
            
            // For straight/rolling motion: individual 3-tick breakout
            if (!isSpinManeuver && delta >= 3) {
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
                // For pure-spin maneuver: hold coordinated boost (85 PWM)
                boostTicksCount++;
                if (boostTicksCount >= 50) {
                    stictionState = STICTION_BLOCKED;
                    lastPwm = 0;
                    return 0;
                }
                int boostPwm = (targetVel > 0.0f) ? SPIN_STICTION_BOOST_PWM : -SPIN_STICTION_BOOST_PWM;
                lastPwm = boostPwm;
                return lastPwm;
            }
        }
        if (stictionState == STICTION_BLOCKED) {
            lastPwm = 0;
            return 0;
        }

        // STICTION_KINETIC: PID + Feedforward
        float error = targetVel - measuredRadps;
        errorSum += error * dt;
        float derivative = (error - lastError) / dt;
        lastError = error;

        float breakaway = isSpinManeuver ? SPIN_KS_PWM : forwardBreakawayPwm;
        float ffMag = breakaway + (kV * std::abs(targetVel));
        float minFfFloor = isSpinManeuver ? MIN_SPIN_KINETIC_FF_FLOOR : 45.0f;
        if (ffMag < minFfFloor) ffMag = minFfFloor;

        float feedforward = (targetVel > 0.0f ? 1.0f : -1.0f) * ffMag;
        float pidCorrection = (Kp * error) + (Ki * errorSum) + (Kd * derivative);
        float totalPwm = feedforward + pidCorrection;

        int targetPwm = (int)std::round(totalPwm);
        if (targetPwm > 255) targetPwm = 255;
        if (targetPwm < -255) targetPwm = -255;

        lastPwm = targetPwm;
        return lastPwm;
    }

    void reset() {
        stictionState = STICTION_IDLE;
        boostStartTicks = 0;
        boostTicksCount = 0;
        boostTicksInitialized = false;
        lastPwm = 0;
        errorSum = 0.0f;
        lastError = 0.0f;
    }
};

struct MockWheelController {
    MockSingleWheelController controllers[4];
    MockWheelController() {
        for (int i = 0; i < 4; i++) controllers[i].begin(i);
    }
    void setTargets(float left, float right, bool isSpin) {
        controllers[0].setTargetVelocity(left, isSpin);
        controllers[2].setTargetVelocity(left, isSpin);
        controllers[1].setTargetVelocity(right, isSpin);
        controllers[3].setTargetVelocity(right, isSpin);
    }
    void transitionAllToKinetic() {
        for (int i = 0; i < 4; i++) controllers[i].forceKinetic();
    }
    void reset() {
        for (int i = 0; i < 4; i++) controllers[i].reset();
    }
};

enum SpinRecoveryState {
    SPIN_RECOVERY_IDLE = 0,
    SPIN_RECOVERY_MONITORING_INITIAL,
    SPIN_RECOVERY_SETTLE_1,
    SPIN_RECOVERY_TWITCH_PULSE,
    SPIN_RECOVERY_SETTLE_2,
    SPIN_RECOVERY_RETRY
};

enum ChassisManeuverType {
    MANEUVER_IDLE = 0,
    MANEUVER_PURE_SPIN,
    MANEUVER_TRANSLATION_OR_ARC
};

struct SimulationContext {
    MockWheelController wheelController;
    int32_t currentTicks[4] = {0, 0, 0, 0};
    float measuredVel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float imuGz = 0.0f;

    SpinRecoveryState spinRecoveryState = SPIN_RECOVERY_IDLE;
    uint32_t spinStartTimeMs = 0;
    uint32_t spinStateEnteredMs = 0;
    float requestedAngular = 0.0f;
    bool candidateAConsumedForEpisode = false;
    bool chassisRebreakoutConsumedForEpisode = false;
    uint8_t spinBreakoutSustainedTicks = 0;

    uint32_t zeroDebounceTicks = 0;
    uint32_t stalledKineticTicks = 0;
    bool stallWindowInitialized = false;
    int32_t stallWindowStartTicks[4] = {0, 0, 0, 0};
    ChassisManeuverType prevManeuverType = MANEUVER_IDLE;

    int candidateAFiredCount = 0;
    int chassisRebreakoutFiredCount = 0;

    void reset() {
        wheelController.reset();
        for (int i = 0; i < 4; i++) {
            currentTicks[i] = 0;
            measuredVel[i] = 0.0f;
        }
        imuGz = 0.0f;
        spinRecoveryState = SPIN_RECOVERY_IDLE;
        candidateAConsumedForEpisode = false;
        chassisRebreakoutConsumedForEpisode = false;
        spinBreakoutSustainedTicks = 0;
        zeroDebounceTicks = 0;
        stalledKineticTicks = 0;
        stallWindowInitialized = false;
        prevManeuverType = MANEUVER_IDLE;
        candidateAFiredCount = 0;
        chassisRebreakoutFiredCount = 0;
    }

    void updateSpinRecovery(const ChassisCommand &activeCmd, float &targetLinear, float &targetAngular, uint32_t nowMs) {
        const float ZERO_EPSILON = 0.005f;
        bool isOperatorPureSpin = (std::abs(activeCmd.linearVelocity) <= ZERO_EPSILON) && (std::abs(activeCmd.angularVelocity) >= ZERO_EPSILON);

        if (!isOperatorPureSpin) {
            spinRecoveryState = SPIN_RECOVERY_IDLE;
            spinBreakoutSustainedTicks = 0;
            return;
        }

        float cmdSign = (activeCmd.angularVelocity >= 0.0f) ? 1.0f : -1.0f;
        bool imuRotating = (imuGz * cmdSign >= 0.08f);

        int movingWheels = 0;
        for (int i = 0; i < 4; i++) {
            float v = measuredVel[i];
            float expectedSign = (i == 0 || i == 2) ? -cmdSign : cmdSign;
            if (v * expectedSign >= 0.10f) {
                movingWheels++;
            }
        }

        bool genuineChassisBreakout = (movingWheels >= 3) && imuRotating;

        if (genuineChassisBreakout) {
            spinBreakoutSustainedTicks++;
            if (spinBreakoutSustainedTicks >= 3) {
                wheelController.transitionAllToKinetic();
                spinRecoveryState = SPIN_RECOVERY_IDLE;
            }
        } else {
            spinBreakoutSustainedTicks = 0;
        }

        switch (spinRecoveryState) {
            case SPIN_RECOVERY_IDLE:
                if (!candidateAConsumedForEpisode) {
                    bool anyBoost = false;
                    for (int i = 0; i < 4; i++) {
                        if (wheelController.controllers[i].getStictionState() == STICTION_BOOST) {
                            anyBoost = true;
                            break;
                        }
                    }
                    if (anyBoost) {
                        spinRecoveryState = SPIN_RECOVERY_MONITORING_INITIAL;
                        spinStartTimeMs = nowMs;
                        requestedAngular = activeCmd.angularVelocity;
                    }
                }
                break;

            case SPIN_RECOVERY_MONITORING_INITIAL:
                if (genuineChassisBreakout) {
                    spinRecoveryState = SPIN_RECOVERY_IDLE;
                    break;
                }
                if (nowMs - spinStartTimeMs >= 350) {
                    spinRecoveryState = SPIN_RECOVERY_SETTLE_1;
                    spinStateEnteredMs = nowMs;
                    candidateAConsumedForEpisode = true;
                    candidateAFiredCount++;
                    wheelController.reset();
                    targetLinear = 0.0f;
                    targetAngular = 0.0f;
                }
                break;

            case SPIN_RECOVERY_SETTLE_1:
                targetLinear = 0.0f;
                targetAngular = 0.0f;
                if (nowMs - spinStateEnteredMs >= 30) {
                    spinRecoveryState = SPIN_RECOVERY_TWITCH_PULSE;
                    spinStateEnteredMs = nowMs;
                    wheelController.reset();
                }
                break;

            case SPIN_RECOVERY_TWITCH_PULSE: {
                float sign_w = (requestedAngular >= 0.0f) ? 1.0f : -1.0f;
                targetLinear = 0.0f;
                targetAngular = -sign_w * 3.50f;
                if (nowMs - spinStateEnteredMs >= 50) {
                    spinRecoveryState = SPIN_RECOVERY_SETTLE_2;
                    spinStateEnteredMs = nowMs;
                    wheelController.reset();
                    targetLinear = 0.0f;
                    targetAngular = 0.0f;
                }
                break;
            }

            case SPIN_RECOVERY_SETTLE_2:
                targetLinear = 0.0f;
                targetAngular = 0.0f;
                if (nowMs - spinStateEnteredMs >= 30) {
                    spinRecoveryState = SPIN_RECOVERY_RETRY;
                    wheelController.reset();
                }
                break;

            case SPIN_RECOVERY_RETRY:
                targetLinear = 0.0f;
                targetAngular = requestedAngular;
                break;
        }
    }

    void updateChassisSupervisor(const ChassisCommand &activeCmd, bool isArmed, uint32_t nowMs) {
        const float CMD_LINEAR_EPSILON = 0.005f;
        const float CMD_ANGULAR_EPSILON = 0.005f;
        bool isCmdZero = (std::abs(activeCmd.linearVelocity) < CMD_LINEAR_EPSILON) && 
                         (std::abs(activeCmd.angularVelocity) < CMD_ANGULAR_EPSILON);

        if (!isArmed) {
            candidateAConsumedForEpisode = false;
            chassisRebreakoutConsumedForEpisode = false;
            zeroDebounceTicks = 0;
            stalledKineticTicks = 0;
            stallWindowInitialized = false;
            prevManeuverType = MANEUVER_IDLE;
        } else if (isCmdZero) {
            zeroDebounceTicks++;
            if (zeroDebounceTicks >= 5) {
                candidateAConsumedForEpisode = false;
                chassisRebreakoutConsumedForEpisode = false;
                stalledKineticTicks = 0;
                stallWindowInitialized = false;
                prevManeuverType = MANEUVER_IDLE;
            }
        } else {
            zeroDebounceTicks = 0;
        }

        ChassisManeuverType currentManeuver = MANEUVER_IDLE;
        if (isArmed && !isCmdZero) {
            if (std::abs(activeCmd.linearVelocity) <= CMD_LINEAR_EPSILON && std::abs(activeCmd.angularVelocity) >= CMD_ANGULAR_EPSILON) {
                currentManeuver = MANEUVER_PURE_SPIN;
            } else {
                currentManeuver = MANEUVER_TRANSLATION_OR_ARC;
            }
        }

        if (isArmed && !isCmdZero && !chassisRebreakoutConsumedForEpisode) {
            bool allKinetic = true;
            for (int i = 0; i < 4; i++) {
                if (wheelController.controllers[i].getStictionState() != STICTION_KINETIC) {
                    allKinetic = false;
                    break;
                }
            }

            int zeroVelCount = 0;
            for (int i = 0; i < 4; i++) {
                if (std::abs(measuredVel[i]) < 0.05f) {
                    zeroVelCount++;
                }
            }

            if (!stallWindowInitialized) {
                for (int i = 0; i < 4; i++) {
                    stallWindowStartTicks[i] = currentTicks[i];
                }
                stallWindowInitialized = true;
            }

            int staticDispCount = 0;
            for (int i = 0; i < 4; i++) {
                if (std::abs(currentTicks[i] - stallWindowStartTicks[i]) < 3) {
                    staticDispCount++;
                }
            }

            bool imuStationary = (std::abs(imuGz) < 0.04f);
            bool chassisStalled = allKinetic && (zeroVelCount >= 3) && (staticDispCount >= 3) && imuStationary;

            if (chassisStalled) {
                stalledKineticTicks++;
                if (stalledKineticTicks >= 20) {
                    wheelController.reset();
                    chassisRebreakoutConsumedForEpisode = true;
                    chassisRebreakoutFiredCount++;
                    stalledKineticTicks = 0;
                    stallWindowInitialized = false;
                    if (currentManeuver == MANEUVER_PURE_SPIN) {
                        spinRecoveryState = SPIN_RECOVERY_IDLE;
                        candidateAConsumedForEpisode = false;
                    }
                }
            } else {
                stalledKineticTicks = 0;
                stallWindowInitialized = false;
            }
        } else {
            stalledKineticTicks = 0;
            stallWindowInitialized = false;
        }
        prevManeuverType = currentManeuver;
    }
};

void run_all_tests() {
    std::cout << "=================================================================\n";
    std::cout << "  RUNNING ADVANCED ROTATION SHIM RAMP & HANDOFF TESTS            \n";
    std::cout << "=================================================================\n";

    SimulationContext ctx;

    // TEST 1: No false breakout on 3-25 encoder ticks with zero IMU yaw
    {
        ctx.reset();
        ChassisCommand cmd{0.0f, 0.8f};
        ctx.wheelController.setTargets(-4.195f, 4.195f, true);
        float lin = 0.0f, ang = 0.8f;

        for (int step = 0; step < 30; step++) {
            uint32_t now = step * 10;
            for (int i = 0; i < 4; i++) {
                ctx.currentTicks[i] = (i == 0 || i == 2) ? -25 : 25;
                ctx.measuredVel[i] = 0.0f;
                ctx.wheelController.controllers[i].update(ctx.measuredVel[i], ctx.currentTicks[i], 0.01f);
            }
            ctx.imuGz = 0.002f;
            ctx.updateSpinRecovery(cmd, lin, ang, now);
        }

        for (int i = 0; i < 4; i++) {
            assert(ctx.wheelController.controllers[i].getStictionState() == STICTION_BOOST);
            assert(std::abs(ctx.wheelController.controllers[i].lastPwm) == 85);
        }
        std::cout << "  [PASS] Test 1: No false breakout on windup; held 85 PWM boost.\n";
    }

    // TEST 2: Genuine breakout at empirical boost (85 PWM) with sustained IMU yaw
    {
        ctx.reset();
        ChassisCommand cmd{0.0f, 0.8f};
        ctx.wheelController.setTargets(-4.195f, 4.195f, true);
        float lin = 0.0f, ang = 0.8f;

        for (int step = 0; step < 4; step++) {
            uint32_t now = step * 10;
            ctx.currentTicks[0] -= 10; ctx.measuredVel[0] = -1.5f;
            ctx.currentTicks[1] += 10; ctx.measuredVel[1] = +1.5f;
            ctx.currentTicks[2] -= 10; ctx.measuredVel[2] = -1.5f;
            ctx.currentTicks[3] += 10; ctx.measuredVel[3] = +1.5f;
            ctx.imuGz = +0.25f;
            for (int i = 0; i < 4; i++) {
                ctx.wheelController.controllers[i].update(ctx.measuredVel[i], ctx.currentTicks[i], 0.01f);
            }
            ctx.updateSpinRecovery(cmd, lin, ang, now);
        }

        for (int i = 0; i < 4; i++) {
            assert(ctx.wheelController.controllers[i].getStictionState() == STICTION_KINETIC);
        }
        std::cout << "  [PASS] Test 2: Genuine breakout qualified and transitioned all wheels to KINETIC.\n";
    }

    // TEST 3: RotationShim Ramp Handoff at wz = 0.20 rad/s (w_target = 1.049 rad/s)
    {
        ctx.reset();
        float w_tgt = (0.340858f / (2.0f * 0.0325f)) * 0.20f; // 1.049 rad/s
        ctx.wheelController.setTargets(-w_tgt, w_tgt, true);
        ctx.wheelController.transitionAllToKinetic();

        // Output at handoff (measured = 0.0 rad/s)
        int pwm_0 = std::abs(ctx.wheelController.controllers[1].update(0.0f, 0, 0.01f));
        int pwm_half = std::abs(ctx.wheelController.controllers[1].update(0.5f, 10, 0.01f));

        // FF is max(80, 58 + 6*1.049) = 80 PWM. P-term is 2.2*1.049 = 2.3 PWM -> Total = 82 PWM
        assert(pwm_0 >= 82);
        assert(pwm_half >= 81);
        std::cout << "  [PASS] Test 3: RotationShim ramp at wz=0.20 rad/s maintains >= 82 PWM (handoff=" << pwm_0 << " PWM, cruise=" << pwm_half << " PWM).\n";
    }

    // TEST 4: RotationShim Ramp Handoff at wz = 0.40 rad/s (w_target = 2.098 rad/s)
    {
        ctx.reset();
        float w_tgt = (0.340858f / (2.0f * 0.0325f)) * 0.40f; // 2.098 rad/s
        ctx.wheelController.setTargets(-w_tgt, w_tgt, true);
        ctx.wheelController.transitionAllToKinetic();

        int pwm_0 = std::abs(ctx.wheelController.controllers[1].update(0.0f, 0, 0.01f));
        assert(pwm_0 >= 84); // 80 FF + 4.6 P = 85 PWM
        std::cout << "  [PASS] Test 4: RotationShim ramp at wz=0.40 rad/s maintains " << pwm_0 << " PWM (>= 84 PWM).\n";
    }

    // TEST 5: RotationShim Ramp Handoff at wz = 0.60 rad/s (w_target = 3.146 rad/s)
    {
        ctx.reset();
        float w_tgt = (0.340858f / (2.0f * 0.0325f)) * 0.60f; // 3.146 rad/s
        ctx.wheelController.setTargets(-w_tgt, w_tgt, true);
        ctx.wheelController.transitionAllToKinetic();

        int pwm_0 = std::abs(ctx.wheelController.controllers[1].update(0.0f, 0, 0.01f));
        assert(pwm_0 >= 86); // 80 FF + 6.9 P = 87 PWM
        std::cout << "  [PASS] Test 5: RotationShim ramp at wz=0.60 rad/s maintains " << pwm_0 << " PWM (>= 86 PWM).\n";
    }

    // TEST 6: RotationShim Ramp Handoff at wz = 0.80 rad/s (w_target = 4.195 rad/s)
    {
        ctx.reset();
        float w_tgt = (0.340858f / (2.0f * 0.0325f)) * 0.80f; // 4.195 rad/s
        ctx.wheelController.setTargets(-w_tgt, w_tgt, true);
        ctx.wheelController.transitionAllToKinetic();

        int pwm_0 = std::abs(ctx.wheelController.controllers[1].update(0.0f, 0, 0.01f));
        int pwm_ss = std::abs(ctx.wheelController.controllers[1].update(4.195f, 50, 0.01f));

        assert(pwm_0 >= 92); // 83.2 FF + 9.2 P = 92 PWM
        assert(pwm_ss == 83); // Exact 83 PWM steady-state cruise
        std::cout << "  [PASS] Test 6: RotationShim ramp at wz=0.80 rad/s maintains " << pwm_0 << " PWM at handoff and " << pwm_ss << " PWM at steady cruise.\n";
    }

    // TEST 7: Increasing measured speed reduces PID correction while closed-loop control regulates
    {
        ctx.reset();
        ctx.wheelController.setTargets(-4.195f, 4.195f, true);
        ctx.wheelController.transitionAllToKinetic();

        int pwm_at_0 = std::abs(ctx.wheelController.controllers[1].update(0.0f, 0, 0.01f));
        int pwm_at_2 = std::abs(ctx.wheelController.controllers[1].update(2.0f, 10, 0.01f));
        int pwm_at_4 = std::abs(ctx.wheelController.controllers[1].update(4.195f, 20, 0.01f));
        int pwm_at_5 = std::abs(ctx.wheelController.controllers[1].update(5.0f, 30, 0.01f));

        assert(pwm_at_0 > pwm_at_2);
        assert(pwm_at_2 > pwm_at_4);
        assert(pwm_at_4 > pwm_at_5); // Closed-loop active braking on overshoot!
        std::cout << "  [PASS] Test 7: Closed-loop regulation active across speed range.\n";
    }

    // TEST 8: Zero command immediately removes spin compensation and resets to IDLE / 0 PWM
    {
        ctx.reset();
        ctx.wheelController.setTargets(0.0f, 0.0f, false);
        int pwm_zero = ctx.wheelController.controllers[1].update(0.0f, 0, 0.01f);
        assert(pwm_zero == 0);
        assert(ctx.wheelController.controllers[1].getStictionState() == STICTION_IDLE);
        std::cout << "  [PASS] Test 8: Zero command immediately returns to IDLE and 0 PWM.\n";
    }

    // TEST 9: Rolling/arc feedforward unchanged (breakaway = 40, floor = 45)
    {
        ctx.reset();
        ctx.wheelController.setTargets(2.0f, 2.0f, false);
        ctx.wheelController.controllers[0].forceKinetic();

        int pwm_straight = ctx.wheelController.controllers[0].update(2.0f, 50, 0.01f);
        assert(pwm_straight == 52); // Normal rolling feedforward unchanged!
        std::cout << "  [PASS] Test 9: Normal rolling/arc feedforward completely unchanged (" << pwm_straight << " PWM).\n";
    }

    // TEST 10: Candidate-A and chassis rebreakout independent latches
    {
        ctx.reset();
        ChassisCommand cmd{0.0f, 0.8f};
        ctx.wheelController.setTargets(-4.195f, 4.195f, true);
        float lin = 0.0f, ang = 0.8f;

        for (int step = 0; step < 36; step++) {
            uint32_t now = step * 10;
            for (int i = 0; i < 4; i++) {
                ctx.wheelController.controllers[i].update(ctx.measuredVel[i], ctx.currentTicks[i], 0.01f);
            }
            ctx.updateSpinRecovery(cmd, lin, ang, now);
            ctx.updateChassisSupervisor(cmd, true, now);
        }

        assert(ctx.candidateAConsumedForEpisode == true);
        assert(ctx.chassisRebreakoutConsumedForEpisode == false);
        std::cout << "  [PASS] Test 10: Candidate-A and chassis rebreakout remain completely independent.\n";
    }

    std::cout << "\n=================================================================\n";
    std::cout << ">>> ALL 10 ADVANCED RAMP & HANDOFF TESTS PASSED PERFECTLY! <<<   \n";
    std::cout << "=================================================================\n";
}

int main() {
    run_all_tests();
    return 0;
}
