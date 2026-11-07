#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>

#define RUNNER_MODE "VALIDATE_COHERENT_CORRECTION"
#define RUNNER_DESCRIPTION "Validation of Nominal Tracking, High Resistance, 66 PWM Disturbance, Negative Trim, Sustained Rolling & Load Removal"

#include "Arduino.h"
#include "RoverConfig.h"
#include "MotorDriver.h"

float WHEEL_DIAMETER_M = 0.065f;
float WHEEL_RADIUS_M = 0.0325f;
bool WHEEL_BALANCE_ENABLED = false;

MotorCalibration motorCalibrations[4] = {
    {40, 40, 6.00f},
    {40, 40, 6.00f},
    {40, 40, 6.00f},
    {40, 40, 6.00f}
};

void MotorDriver::setPWM(int motorIndex, int pwm) {}
MotorDriver::MotorDriver() : currentMode(MotorOutputMode::LOCKED), authorizedMotorIndex(-1), actuationState(DrivetrainActuationState::COAST) {}
void MotorDriver::begin() {}
bool MotorDriver::allows(int motorIndex, int) const { return true; }

#include "../src/WheelController.h"
#include "../src/WheelController.cpp"

struct ValidationMetrics {
    std::string scenarioName;
    float activeCutoffMm;
    float finalStoppedMm;
    float peakSpeedRadps;
    float cutoffSpeedRadps;
    int restalls;
    int cutoffFinalPwm;
    float cutoffFfPwm;
    float cutoffPterm;
    float cutoffIterm;
    float peakAssist;
    float cutoffAssist;
};

ValidationMetrics runScenario(
    const std::string& name,
    float targetRadps,
    float activeDurS,
    float totalDurS,
    float baseFricPwm,
    float disturbancePwm,
    bool injectDisturbance,
    float distStartS = 0.50f,
    float distDurS = 0.20f,
    float initialIntegralTrim = 0.0f,
    bool dropLoadMidway = false,
    float dropLoadTimeS = 1.00f,
    float droppedLoadFricPwm = 43.5f,
    float plantDamping = 7.5f
) {
    SingleWheelController wheel;
    wheel.begin(0);
    wheel.setGains(KP_SPEED, KI_SPEED, KD_SPEED);
    wheel.setCandidateRecoveryAssist(true);
    wheel.setUltraLowSpeedMode(ULTRA_LOW_MODE_GAINS_AND_FF);
    wheel.setTargetVelocity(targetRadps, false, false, false, false);

    float dt = 0.010f;
    int totalSteps = (int)(totalDurS / dt);
    int cutoffStep = (int)(activeDurS / dt);
    int distStartStep = (int)(distStartS / dt);
    int distEndStep = distStartStep + (int)(distDurS / dt);
    int dropLoadStep = (int)(dropLoadTimeS / dt);

    float trueVel = 0.0f;
    float trueTheta = 0.0f;
    int32_t lastReportedTicks = 0;

    // Seed initial integral trim if requested (e.g. from prior smooth rolling)
    if (std::abs(initialIntegralTrim) > 0.01f) {
        // Run mock overspeed to naturally wind the integral to target trim
        for (int i = 0; i < 40; i++) {
            float seedMeas = (targetRadps > 0.0f) ? (targetRadps + 0.35f) : (targetRadps - 0.35f);
            int32_t ticks = (int32_t)(trueTheta * TICKS_PER_REV / (2.0f * (float)M_PI)) + (i + 1) * 8;
            wheel.update(seedMeas, ticks, dt);
        }
    }

    ValidationMetrics m;
    m.scenarioName = name;
    m.restalls = 0;
    m.peakAssist = 0.0f;
    m.peakSpeedRadps = 0.0f;
    bool wasMoving = false;

    for (int step = 0; step < totalSteps; step++) {
        float t = step * dt;
        if (step == cutoffStep) {
            wheel.setTargetVelocity(0.0f, false, false, false, false);
            m.activeCutoffMm = (trueTheta * WHEEL_RADIUS_M) * 1000.0f;
            m.cutoffSpeedRadps = trueVel;
            m.cutoffFinalPwm = wheel.getDiag().finalPwm;
            m.cutoffFfPwm = wheel.getDiag().feedforward;
            m.cutoffPterm = wheel.getDiag().pTerm;
            m.cutoffIterm = wheel.getDiag().iTerm;
            m.cutoffAssist = wheel.getDiag().recoveryAssist;
        }

        float currentFric = baseFricPwm;
        if (dropLoadMidway && step >= dropLoadStep) {
            currentFric = droppedLoadFricPwm;
        }

        bool inDisturbance = injectDisturbance && (step >= distStartStep && step < distEndStep);
        float currentBreakaway = inDisturbance ? disturbancePwm : (currentFric + 5.0f);
        float opposingTorque = inDisturbance ? disturbancePwm : currentFric;

        int32_t simulatedTicks = (int32_t)(trueTheta * TICKS_PER_REV / (2.0f * (float)M_PI));
        float measuredVel = (simulatedTicks - lastReportedTicks) * (2.0f * (float)M_PI) / (TICKS_PER_REV * dt);
        lastReportedTicks = simulatedTicks;

        int pwm = wheel.update(measuredVel, simulatedTicks, dt);
        const auto& diag = wheel.getDiag();

        if (std::abs(diag.recoveryAssist) > std::abs(m.peakAssist)) {
            m.peakAssist = diag.recoveryAssist;
        }
        if (std::abs(trueVel) > std::abs(m.peakSpeedRadps)) {
            m.peakSpeedRadps = std::abs(trueVel);
        }

        // Plant dynamics
        float signCmd = (targetRadps >= 0.0f) ? 1.0f : -1.0f;
        float appliedPwm = (float)pwm * signCmd;
        float J = 0.25f;
        float b = plantDamping;

        if (std::abs(trueVel) < 0.02f) {
            if (appliedPwm >= currentBreakaway) {
                float netTorque = appliedPwm - opposingTorque;
                trueVel += (signCmd * netTorque / J) * dt;
            } else {
                trueVel = 0.0f;
            }
        } else {
            float netTorque = appliedPwm - opposingTorque - (b * std::abs(trueVel));
            trueVel += (signCmd * netTorque / J) * dt;
            if (signCmd > 0.0f && trueVel < 0.0f) trueVel = 0.0f;
            if (signCmd < 0.0f && trueVel > 0.0f) trueVel = 0.0f;
        }

        trueTheta += trueVel * dt;

        bool nowMoving = (std::abs(trueVel) > 0.05f);
        if (wasMoving && !nowMoving && (!injectDisturbance || step > distEndStep + 10) && step > 20 && step < cutoffStep) {
            m.restalls++;
        }
        wasMoving = nowMoving;
    }

    if (cutoffStep >= totalSteps) {
        m.activeCutoffMm = (trueTheta * WHEEL_RADIUS_M) * 1000.0f;
    }
    m.finalStoppedMm = (trueTheta * WHEEL_RADIUS_M) * 1000.0f;
    return m;
}

int main() {
    float targetCase2 = 0.4615f; // 0.015 m/s at 65mm geometry
    float activeDur = 1.75f;
    float totalDur = 2.20f;

    std::vector<ValidationMetrics> results;

    // 1. Nominal Tracking (43.5 PWM rolling friction, calibrated plant)
    results.push_back(runScenario("1. Nominal Tracking (43.5 PWM)", targetCase2, activeDur, totalDur, 43.5f, 50.0f, false));

    // 2. High Resistance (55 PWM rolling friction)
    results.push_back(runScenario("2. High Resistance (55 PWM)", targetCase2, activeDur, totalDur, 55.0f, 60.0f, false));

    // 3. 66 PWM Disturbance (55 PWM carpet + 66 PWM disturbance for 200ms)
    results.push_back(runScenario("3. 66 PWM Disturbance (at 55 PWM)", targetCase2, activeDur, totalDur, 55.0f, 66.0f, true, 0.50f, 0.20f));

    // 4. Negative Prior Trim (-4.9 PWM) + 66 PWM Disturbance
    results.push_back(runScenario("4. Neg Prior Trim + 66 PWM Dist", targetCase2, activeDur, totalDur, 55.0f, 66.0f, true, 0.50f, 0.20f, -4.9f));

    // 5. Sustained Rolling After Assist Disappears (checked over extended 2.5s duration)
    results.push_back(runScenario("5. Sustained Rolling (Post-Assist)", targetCase2, 2.50f, 2.80f, 55.0f, 66.0f, true, 0.40f, 0.20f));

    // 6. Removal of Extra Load (55 PWM drops to 43.5 PWM at 1.0s)
    results.push_back(runScenario("6. Extra Load Removal (55->43.5 PWM)", targetCase2, activeDur, totalDur, 55.0f, 66.0f, true, 0.40f, 0.20f, 0.0f, true, 1.00f, 43.5f));

    // 7. Reverse Nominal Tracking (-0.015 m/s)
    results.push_back(runScenario("7. Reverse Nominal (-0.015 m/s)", -targetCase2, activeDur, totalDur, 43.5f, 50.0f, false));

    // 8. Reverse 66 PWM Disturbance (-0.015 m/s)
    results.push_back(runScenario("8. Reverse 66 PWM Disturbance", -targetCase2, activeDur, totalDur, 55.0f, 66.0f, true, 0.50f, 0.20f));

    std::cout << "\n========================================================================================================================\n";
    std::cout << "ROVER 1 COHERENT CORRECTION VALIDATION MATRIX (PRODUCTION C++ WHEEL CONTROLLER)\n";
    std::cout << "========================================================================================================================\n";
    std::cout << std::left << std::setw(36) << "Scenario"
              << std::right << std::setw(11) << "Cutoff(mm)"
              << std::setw(11) << "Stop(mm)"
              << std::setw(12) << "PeakSpd"
              << std::setw(12) << "CutoffSpd"
              << std::setw(9)  << "Restalls"
              << std::setw(10) << "Tot PWM"
              << std::setw(8)  << "FF"
              << std::setw(8)  << "P"
              << std::setw(8)  << "I"
              << std::setw(9)  << "PkAssist"
              << "\n";
    std::cout << "------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& r : results) {
        std::cout << std::left << std::setw(36) << r.scenarioName
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(11) << r.activeCutoffMm
                  << std::setw(11) << r.finalStoppedMm
                  << std::setprecision(3)
                  << std::setw(12) << r.peakSpeedRadps
                  << std::setw(12) << r.cutoffSpeedRadps
                  << std::setw(9)  << r.restalls
                  << std::setw(10) << r.cutoffFinalPwm
                  << std::setprecision(1)
                  << std::setw(8)  << r.cutoffFfPwm
                  << std::setw(8)  << r.cutoffPterm
                  << std::setw(8)  << r.cutoffIterm
                  << std::setw(9)  << r.peakAssist
                  << "\n";
    }
    std::cout << "========================================================================================================================\n";
    return 0;
}
