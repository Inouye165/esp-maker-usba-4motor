#include "CommandManager.h"
#include "RoverConfig.h"

CommandManager::CommandManager() 
    : activeSource(SOURCE_NONE)
    , lastCmdReceivedMs(0)
    , eStopLatched(false)
    , timedOut(false)
    , normalDriveArmed(false) {
    currentCmd = {0.0f, 0.0f, SOURCE_NONE, 0, false};
}

void CommandManager::begin() {
    clearEmergencyStop();
    normalDriveArmed = false;
}

void CommandManager::setCommand(float linear, float angular, CommandSource source, bool estop) {
    if (estop) {
        triggerEmergencyStop();
        return;
    }
    
    if (eStopLatched) return;
    
    // Safety guard: reject movement commands if disarmed
    if (!normalDriveArmed && (source == SOURCE_WEB_JOYSTICK || source == SOURCE_USB_SERIAL || source == SOURCE_ROS)) {
        return;
    }
    
    uint32_t now = millis();
    
    // Auto-take ownership if the current source is NONE or has timed out
    if (activeSource == SOURCE_NONE || timedOut || (now - lastCmdReceivedMs > FAULT_TIMEOUT_MS)) {
        activeSource = source;
        timedOut = false;
    }
    
    // Accept commands only from the active source
    if (source == activeSource) {
        // Enforce conservative safety limits at the boundary
        currentCmd.linearVelocity = constrain(linear, -MAX_LINEAR_VELOCITY_MPS, MAX_LINEAR_VELOCITY_MPS);
        currentCmd.angularVelocity = constrain(angular, -MAX_ANGULAR_VELOCITY_RADPS, MAX_ANGULAR_VELOCITY_RADPS);
        currentCmd.source = source;
        currentCmd.receivedAtMs = now;
        lastCmdReceivedMs = now;
        timedOut = false;
    }
}

bool CommandManager::armNormalDrive() {
    if (eStopLatched || timedOut) return false;
    normalDriveArmed = true;
    currentCmd.linearVelocity = 0.0f;
    currentCmd.angularVelocity = 0.0f;
    currentCmd.source = SOURCE_NONE; // Begin with zero requested velocity
    return true;
}

void CommandManager::disarmNormalDrive() {
    normalDriveArmed = false;
    currentCmd.linearVelocity = 0.0f;
    currentCmd.angularVelocity = 0.0f;
    currentCmd.source = SOURCE_NONE;
}

void CommandManager::triggerEmergencyStop() {
    eStopLatched = true;
    normalDriveArmed = false; // Emergency stop disarms normal drive
    currentCmd.linearVelocity = 0.0f;
    currentCmd.angularVelocity = 0.0f;
    currentCmd.emergencyStop = true;
}

void CommandManager::clearEmergencyStop() {
    eStopLatched = false;
    currentCmd.emergencyStop = false;
    timedOut = false;
    lastCmdReceivedMs = millis();
}

bool CommandManager::requestSourceChange(CommandSource newSource, float currentLinear, float currentAngular) {
    if (eStopLatched) return false;
    
    activeSource = newSource;
    uint32_t now = millis();
    currentCmd.linearVelocity = currentLinear;
    currentCmd.angularVelocity = currentAngular;
    currentCmd.source = newSource;
    currentCmd.receivedAtMs = now;
    lastCmdReceivedMs = now;
    timedOut = false;
    return true;
}

ChassisCommand CommandManager::getActiveCommand(uint32_t nowMs) {
    ChassisCommand cmd = currentCmd;
    
    if (eStopLatched) {
        cmd.linearVelocity = 0.0f;
        cmd.angularVelocity = 0.0f;
        cmd.emergencyStop = true;
        return cmd;
    }
    
    // Watchdog check
    if (activeSource != SOURCE_NONE && activeSource != SOURCE_CALIBRATION) {
        uint32_t elapsed = nowMs - lastCmdReceivedMs;
        if (elapsed > FAULT_TIMEOUT_MS) {
            timedOut = true;
            normalDriveArmed = false; // Watchdog timeout disarms normal drive
            activeSource = SOURCE_NONE;
            currentCmd.linearVelocity = 0.0f;
            currentCmd.angularVelocity = 0.0f;
            cmd.linearVelocity = 0.0f;
            cmd.angularVelocity = 0.0f;
        } else if (elapsed > WATCHDOG_TIMEOUT_MS) {
            // Temporary communication loss: Soft stop (decelerate target to 0)
            cmd.linearVelocity = 0.0f;
            cmd.angularVelocity = 0.0f;
        }
    } else {
        // If source is NONE, ensure targets are 0
        cmd.linearVelocity = 0.0f;
        cmd.angularVelocity = 0.0f;
    }
    
    return cmd;
}

void CommandManager::resetWatchdog() {
    lastCmdReceivedMs = millis();
    timedOut = false;
}
