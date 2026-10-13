#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <Arduino.h>

enum CommandSource {
    SOURCE_NONE,
    SOURCE_WEB_JOYSTICK,
    SOURCE_USB_SERIAL,
    SOURCE_ROS,
    SOURCE_POSITION,
    SOURCE_CALIBRATION
};

struct ChassisCommand {
    float linearVelocity;  // m/s
    float angularVelocity; // rad/s
    CommandSource source;
    uint32_t receivedAtMs;
    bool emergencyStop;
};

class CommandManager {
public:
    CommandManager();
    void begin();
    
    // Set command from a source (with optional 2-bit clearance mask: 0x01=FWD_OK, 0x02=REV_OK)
    void setCommand(float linear, float angular, CommandSource source, bool estop = false, uint8_t clearanceMask = 0x00);
    
    // Directional clearance authorization (fail-closed if stale >500ms)
    uint8_t getClearanceMask() const;
    uint32_t getClearanceAgeMs() const;
    
    // Trigger an emergency stop immediately
    void triggerEmergencyStop();
    void clearEmergencyStop();
    bool isEmergencyStopped() const { return eStopLatched; }
    
    // Arm/Disarm normal drive
    bool armNormalDrive();
    void disarmNormalDrive();
    bool isNormalDriveArmed() const { return normalDriveArmed; }
    
    // Explicitly set/request active control source
    bool requestSourceChange(CommandSource newSource, float currentLinear, float currentAngular);
    CommandSource getActiveSource() const { return activeSource; }
    uint32_t getLastCmdReceivedMs() const { return lastCmdReceivedMs; }
    float getRequestedLinearVelocity() const { return currentCmd.linearVelocity; }
    float getRequestedAngularVelocity() const { return currentCmd.angularVelocity; }
    
    // Fetch command to execute, executing watchdog timeout checks
    ChassisCommand getActiveCommand(uint32_t nowMs);
    
    // Reset watchdog/timeout state
    void resetWatchdog();
    bool isTimedOut() const { return timedOut; }

private:
    ChassisCommand currentCmd;
    CommandSource activeSource;
    uint32_t lastCmdReceivedMs;
    uint8_t currentClearanceMask;
    uint32_t lastClearanceReceivedMs;
    bool eStopLatched;
    bool timedOut;
    bool normalDriveArmed;
};

#endif // COMMAND_MANAGER_H
