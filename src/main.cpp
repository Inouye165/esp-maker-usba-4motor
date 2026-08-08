#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// Brownout workaround headers
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Unified Controller Components
#include "RoverConfig.h"
#include "MotorDriver.h"
#include "EncoderManager.h"
#include "CommandManager.h"
#include "MotionLimiter.h"
#include "DifferentialDrive.h"
#include "WheelController.h"
#include "SafetyManager.h"
#include "SerialProtocol.h"
#include "CalibrationManager.h"
#include "MaintenanceManager.h"
#include "ImuManager.h"

// Onboard RGB LEDs Configuration
#define RGB_PIN 16
#define NUM_LEDS 4

// Initialize NeoPixel strip
Adafruit_NeoPixel strip(NUM_LEDS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// Unified control component instances
MotorDriver motorDriver;
EncoderManager encoderManager;
CommandManager commandManager;
MotionLimiter motionLimiter;
WheelController wheelController;
SafetyManager safetyManager;
SerialProtocol serialProtocol;
CalibrationManager calManager;
MaintenanceManager maintenanceManager;
ImuManager imuManager;

ControlLoopStats loopStats = {0, 9999999, 0, 0, 0, 0};

unsigned long lastControlTime = 0;

// Structure to track individual LED colors
struct RGBColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Default colors: Pulsing Green/Blue on boot
RGBColor leds[NUM_LEDS] = {
  {0, 255, 0},
  {0, 0, 255},
  {0, 255, 0},
  {0, 0, 255}
};

// Set a single LED color
void setLED(int index, uint8_t r, uint8_t g, uint8_t b) {
  if (index >= 0 && index < NUM_LEDS) {
    leds[index] = {r, g, b};
    strip.setPixelColor(index, strip.Color(r, g, b));
    strip.show();
  }
}

// Set all LEDs to a single color
void setAllLEDs(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = {r, g, b};
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void setup() {
  // Disable brownout detector to prevent low-voltage reset loop
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Initialize Serial & Protocol
  serialProtocol.begin();
  delay(1000);
  
  // Definitive firmware identification serial print
  Serial.println("\n=================================");
  Serial.println("Firmware Name: Maker-ESP32-Unified-Rover");
  Serial.println("Firmware Version: 1.0.0-phase1");
  Serial.println("Protocol Version: v1.1");
  Serial.println("Source Identifier: refactor-p1-cleanup");
  Serial.print("Build Timestamp: ");
  Serial.println(__DATE__ " " __TIME__);
  Serial.println("Hardware Target: Maker-ESP32-Pro");
  Serial.println("=================================");

  // Initialize NVS configurations
  initConfigStorage();

  // Initialize Managers
  motorDriver.begin();
  encoderManager.begin();
  commandManager.begin();
  motionLimiter.begin();
  wheelController.begin();
  safetyManager.begin();
  calManager.begin();
  imuManager.begin(21, 22, 0x4B);

  // Initialize NeoPixels
  strip.begin();
  strip.setBrightness(128); // Moderate brightness
  setLED(0, 66, 133, 244);  // Google Blue
  setLED(1, 234, 67, 53);   // Google Red
  setLED(2, 251, 188, 5);   // Google Yellow
  setLED(3, 52, 168, 83);   // Google Green
  
  lastControlTime = millis();
}

void runMotionControlLoop() {
  int64_t startTimeUs = esp_timer_get_time();
  
  unsigned long now = millis();
  float dt = (now - lastControlTime) / 1000.0f;
  
  // Guard against extreme dt values during boot or overflow
  if (dt <= 0.0f || dt > 0.1f) {
    dt = CONTROL_PERIOD_S;
  }
  lastControlTime = now;
  
  // 1. Read hardware encoders and update ticks and filtered speeds
  encoderManager.update(dt);
  
  // 2. Fetch ticks for calibration manager
  int32_t currentTicks[4];
  for (int i = 0; i < 4; i++) {
    currentTicks[i] = encoderManager.getTicks(i);
  }
  
  // 3. Process breakaway calibration if active (replaces main loop)
  if (calManager.getState() != CAL_IDLE) {
    calManager.update(currentTicks, motorDriver);
    // Measure calibration run control loop execution time
    int64_t endTimeUs = esp_timer_get_time();
    uint32_t durationUs = (uint32_t)(endTimeUs - startTimeUs);
    
    // Update stats
    loopStats.lastDurationUs = durationUs;
    loopStats.minDurationUs = min(loopStats.minDurationUs, durationUs);
    loopStats.maxDurationUs = max(loopStats.maxDurationUs, durationUs);
    loopStats.totalIterations++;
    loopStats.avgDurationUs = (uint32_t)(((uint64_t)loopStats.avgDurationUs * (loopStats.totalIterations - 1) + durationUs) / loopStats.totalIterations);
    if (durationUs > 10000) {
      loopStats.missedDeadlines++;
    }
    return;
  }
  
  // 3b. Process maintenance mode if active (replaces main loop)
  if (maintenanceManager.isActive()) {
    maintenanceManager.update(motorDriver);
    int64_t endTimeUs = esp_timer_get_time();
    uint32_t durationUs = (uint32_t)(endTimeUs - startTimeUs);
    
    // Update stats
    loopStats.lastDurationUs = durationUs;
    loopStats.minDurationUs = min(loopStats.minDurationUs, durationUs);
    loopStats.maxDurationUs = max(loopStats.maxDurationUs, durationUs);
    loopStats.totalIterations++;
    loopStats.avgDurationUs = (uint32_t)(((uint64_t)loopStats.avgDurationUs * (loopStats.totalIterations - 1) + durationUs) / loopStats.totalIterations);
    if (durationUs > 10000) {
      loopStats.missedDeadlines++;
    }
    return;
  }
  
  // 4. Check watchdog and fetch active command target (linear, angular)
  ChassisCommand activeCmd;
  if (commandManager.isNormalDriveArmed()) {
      activeCmd = commandManager.getActiveCommand(now);
  } else {
      activeCmd.linearVelocity = 0.0f;
      activeCmd.angularVelocity = 0.0f;
      activeCmd.source = SOURCE_NONE;
  }
  
  // Handle controlled stop transition on disarm
  if (!commandManager.isNormalDriveArmed() && motorDriver.getMode() == MotorOutputMode::NORMAL_DRIVE) {
      bool stopped = (abs(motionLimiter.getLinearVelocity()) < 0.01f) && 
                     (abs(motionLimiter.getAngularVelocity()) < 0.01f);
      if (stopped) {
          motorDriver.setMode(MotorOutputMode::LOCKED);
          Serial.println("[Command] Rover stopped. Normal drive transitioned to LOCKED.");
      }
  }
  
  // 5. Update Motion Limiter (S-curve acceleration profile)
  motionLimiter.update(activeCmd.linearVelocity, activeCmd.angularVelocity, dt);
  
  float targetLinear = motionLimiter.getLinearVelocity();
  float targetAngular = motionLimiter.getAngularVelocity();
  
  // 6. Kinematic Translation: target (v, w) to individual left/right targets (rad/s)
  float leftTargetRadps = 0.0f;
  float rightTargetRadps = 0.0f;
  DifferentialDrive::velocityToWheels(targetLinear, targetAngular, leftTargetRadps, rightTargetRadps);
  
  // Apply straight drive trims dynamically based on direction
  if (targetLinear >= 0.0f) {
      LEFT_TRIM = LEFT_TRIM_FWD;
      RIGHT_TRIM = RIGHT_TRIM_FWD;
  } else {
      LEFT_TRIM = LEFT_TRIM_REV;
      RIGHT_TRIM = RIGHT_TRIM_REV;
  }
  leftTargetRadps *= LEFT_TRIM;
  rightTargetRadps *= RIGHT_TRIM;
  
  // 7. Route target wheel speeds to controllers with active straight-line synchronization
  static int32_t syncStartTicks[4] = {0, 0, 0, 0};
  static bool wasGoingStraight = false;
  
  bool isGoingStraight = (activeCmd.linearVelocity != 0.0f) && (activeCmd.angularVelocity == 0.0f);
  
  if (isGoingStraight) {
      if (!wasGoingStraight) {
          wasGoingStraight = true;
          for (int i = 0; i < 4; i++) {
              syncStartTicks[i] = encoderManager.getTicks(i);
          }
      }
      
      int32_t relTicks[4];
      for (int i = 0; i < 4; i++) {
          relTicks[i] = encoderManager.getTicks(i) - syncStartTicks[i];
      }
      
      float avgTicks = (relTicks[0] + relTicks[1] + relTicks[2] + relTicks[3]) / 4.0f;
      const float K_SYNC = 0.005f; // rad/s target correction per tick error
      
      float target0 = leftTargetRadps - (relTicks[0] - avgTicks) * K_SYNC;
      float target2 = leftTargetRadps - (relTicks[2] - avgTicks) * K_SYNC;
      float target1 = rightTargetRadps - (relTicks[1] - avgTicks) * K_SYNC;
      float target3 = rightTargetRadps - (relTicks[3] - avgTicks) * K_SYNC;
      
      wheelController.setWheelTarget(0, target0);
      wheelController.setWheelTarget(1, target1);
      wheelController.setWheelTarget(2, target2);
      wheelController.setWheelTarget(3, target3);
  } else {
      wasGoingStraight = false;
      wheelController.setTargets(leftTargetRadps, rightTargetRadps);
  }
  
  // 8. Run PI Speed loops and update motor PWMs
  float measuredVels[4];
  float targetVels[4];
  int pwmOutputs[4];
  
  for (int i = 0; i < 4; i++) {
    measuredVels[i] = encoderManager.getVelocity(i);
    targetVels[i] = wheelController.getController(i).getTarget();
  }
  
  // Run closed-loop PID and driver output update
  wheelController.update(measuredVels, dt, motorDriver);
  
  for (int i = 0; i < 4; i++) {
    pwmOutputs[i] = wheelController.getController(i).getPwmOutput();
  }
  
  // 9. Evaluate safety checks (stall / encoder faults)
  uint32_t faults = safetyManager.update(targetVels, measuredVels, pwmOutputs, dt, motorDriver);
  if (faults != FAULT_NONE) {
    motorDriver.emergencyStop();
    wheelController.reset();
  }
  
  // End loop timing calculation
  int64_t endTimeUs = esp_timer_get_time();
  uint32_t durationUs = (uint32_t)(endTimeUs - startTimeUs);
  
  // Update control loop timing statistics
  loopStats.lastDurationUs = durationUs;
  loopStats.minDurationUs = min(loopStats.minDurationUs, durationUs);
  loopStats.maxDurationUs = max(loopStats.maxDurationUs, durationUs);
  loopStats.totalIterations++;
  loopStats.avgDurationUs = (uint32_t)(((uint64_t)loopStats.avgDurationUs * (loopStats.totalIterations - 1) + durationUs) / loopStats.totalIterations);
  if (durationUs > 10000) {
    loopStats.missedDeadlines++;
  }
}

void loop() {
  // Non-blocking update of BNO08x IMU reports
  imuManager.update();

  // Non-blocking parse of incoming USB/ROS serial packets
  serialProtocol.update(commandManager, calManager, maintenanceManager, safetyManager, loopStats);

  // Truthful 100 Hz (10,000 us) control loop scheduling via esp_timer_get_time()
  static uint64_t scheduledStartUs = 0;
  uint64_t nowUs = esp_timer_get_time();

  if (scheduledStartUs == 0) {
    scheduledStartUs = nowUs;
  }

  if (nowUs >= scheduledStartUs + 10000) {
    // Advance scheduledStartUs to target scheduled start time for this period
    scheduledStartUs += 10000;

    // True start lateness offset relative to target scheduled start time (0us when on deadline)
    uint32_t latenessUs = (uint32_t)(nowUs - scheduledStartUs);
    loopStats.lastStartLatenessUs = latenessUs;
    if (latenessUs > loopStats.maxStartLatenessUs) {
      loopStats.maxStartLatenessUs = latenessUs;
    }

    uint32_t periodsElapsed = (latenessUs / 10000) + 1;

    // Missed 100 Hz periods: if lateness >= 10,000 us, missed = periodsElapsed - 1
    if (periodsElapsed > 1) {
      uint32_t missedThisTick = periodsElapsed - 1;
      loopStats.missedControlPeriods += missedThisTick;
      if (missedThisTick > loopStats.maxConsecutiveMissedPeriods) {
        loopStats.maxConsecutiveMissedPeriods = missedThisTick;
      }
      // Advance scheduledStartUs by missed periods so absolute schedule phase is preserved
      scheduledStartUs += (uint64_t)missedThisTick * 10000;
    }

    // Execute motor/safety control loop EXACTLY ONCE per tick (no catch-up bursts)
    runMotionControlLoop();
  }
  
  unsigned long now = millis();

#if !IMU_DIAGNOSTIC_MODE
  // Production 50 Hz IMU telemetry packet dispatch (fixed 20ms period, no catch-up bursts)
  static uint32_t lastImuTime = 0;
  constexpr uint32_t IMU_PERIOD_MS = 20;

  uint32_t elapsedImu = now - lastImuTime;
  if (elapsedImu >= IMU_PERIOD_MS) {
    uint32_t periodsElapsed = elapsedImu / IMU_PERIOD_MS;
    lastImuTime += periodsElapsed * IMU_PERIOD_MS;
    serialProtocol.sendImuTelemetry(imuManager);
  }

  // Periodic bulk telemetry stream update at 20Hz (50ms)
  static unsigned long lastTelemetryTime = 0;
  if (now - lastTelemetryTime >= 50) {
    lastTelemetryTime = now;
    
    int32_t ticks[4];
    for (int i = 0; i < 4; i++) {
      ticks[i] = encoderManager.getTicks(i);
    }
    
    float linearVel = 0.0f;
    float angularVel = 0.0f;
    DifferentialDrive::wheelsToVelocity(
      encoderManager.getVelocity(0),
      encoderManager.getVelocity(1),
      encoderManager.getVelocity(2),
      encoderManager.getVelocity(3),
      linearVel, angularVel
    );
    
    // Broadcast bulk telemetry to Serial Port
    serialProtocol.sendTelemetry(ticks, 0.0f, angularVel, calManager, maintenanceManager, loopStats, safetyManager.getFaults());
  }
#endif

  // Brief yield
  delay(1);
}
