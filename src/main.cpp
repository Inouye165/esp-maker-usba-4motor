#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// Brownout workaround headers
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Onboard RGB LEDs Configuration
#define RGB_PIN 16
#define NUM_LEDS 4

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// Fallback credentials in case they aren't supplied by build script
#ifndef WIFI_SSID
  #define WIFI_SSID "Dobby"
#endif
#ifndef WIFI_PASS
  #define WIFI_PASS "sanmina-1"
#endif

// PWM Configuration Macros for ESP32 Arduino Core 2.x and 3.x
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define setupPWM(pin, freq, res, chan) ledcAttachChannel(pin, freq, res, chan)
  #define writePWM(pin, chan, val)       ledcWrite(pin, val)
#else
  #define setupPWM(pin, freq, res, chan) { ledcSetup(chan, freq, res); ledcAttachPin(pin, chan); }
  #define writePWM(pin, chan, val)       ledcWrite(chan, val)
#endif

// Initialize WebServer on Port 80
WebServer server(80);

// Initialize UDP for Auto-Discovery Beacons
WiFiUDP udp;
const int udpPort = 3000;
unsigned long lastBeaconTime = 0;
const unsigned long beaconInterval = 2000; // 2 seconds

// Initialize NeoPixel strip
Adafruit_NeoPixel strip(NUM_LEDS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// Initialize SSD1306 OLED Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

// Structure to track Motor Configuration
struct Motor {
  int in1;
  int in2;
  int chan1;
  int chan2;
  int currentSpeed; // Range: -120 to 120
};

// Motor pin mappings (Toshiba Driver)
Motor motors[4] = {
  {27, 13, 0, 1, 0}, // M1
  {4, 2, 2, 3, 0},   // M2
  {17, 12, 4, 5, 0}, // M3 (switch must be set to Motor)
  {14, 15, 6, 7, 0}  // M4 (switch must be set to Motor)
};

// Safety Constants
const int MAX_SAFE_SPEED = 255; // Support up to full 100% PWM
int currentSpeedLimit = 191;    // Default safety limit set to 75% PWM (191)
unsigned long lastMotorCmdTime = 0;
bool motorsActive = false;

// Safe motor timed testing state
unsigned long testEndTime = 0;
bool isTestModeActive = false;

// Update OLED screen with local connection status
void refreshOLEDDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Header
  display.setCursor(0, 0);
  display.println(" Maker-ESP32 Robot");
  display.println("=====================");
  display.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    display.print("WiFi: Connected\n");
    display.print("SSID: ");
    display.println(WIFI_SSID);
    display.print("IP:   ");
    display.println(WiFi.localIP().toString());
    display.print("RSSI: ");
    display.print(WiFi.RSSI());
    display.println(" dBm");
  } else {
    display.print("WiFi: Disconnected\n");
    display.println("Connecting...");
  }
  display.display();
}

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

// Control motor speed safely
void setMotorSpeed(int index, int speed) {
  if (index < 0 || index >= 4) return;
  
  // Enforce safety limit
  int targetSpeed = constrain(speed, -currentSpeedLimit, currentSpeedLimit);
  int prevSpeed = motors[index].currentSpeed;
  
  // Safe direction transition: if reversing, stop the motor and wait 50ms
  if (((prevSpeed > 0 && targetSpeed < 0) || (prevSpeed < 0 && targetSpeed > 0)) && prevSpeed != 0 && targetSpeed != 0) {
    writePWM(motors[index].in1, motors[index].chan1, 0);
    writePWM(motors[index].in2, motors[index].chan2, 0);
    delay(50);
  }
  
  motors[index].currentSpeed = targetSpeed;

  // Reverse both sides to correct hardware direction mismatch.
  // Left side (index 0, 2) is inverted; Right side (index 1, 3) is normal.
  int outputSpeed = targetSpeed;
  if (index == 0 || index == 2) {
    outputSpeed = -outputSpeed;
  }
  
  if (outputSpeed > 0) {
    // Forward: PWM on IN1, Ground on IN2
    writePWM(motors[index].in1, motors[index].chan1, outputSpeed);
    writePWM(motors[index].in2, motors[index].chan2, 0);
  } else if (outputSpeed < 0) {
    // Reverse: Ground on IN1, PWM on IN2
    writePWM(motors[index].in1, motors[index].chan1, 0);
    writePWM(motors[index].in2, motors[index].chan2, -outputSpeed);
  } else {
    // Stop: Ground on both
    writePWM(motors[index].in1, motors[index].chan1, 0);
    writePWM(motors[index].in2, motors[index].chan2, 0);
  }
  
  // Check if any motor is still active
  bool active = false;
  for (int i = 0; i < 4; i++) {
    if (motors[i].currentSpeed != 0) {
      active = true;
      break;
    }
  }
  motorsActive = active;
  lastMotorCmdTime = millis();
}

// Stop all motors immediately
void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    setMotorSpeed(i, 0);
  }
  motorsActive = false;
}

// GET /api/led?index=0&r=255&g=0&b=0 (or hex=ff0000)
void handleSetLED() {
  int index = -1; // Default: all
  if (server.hasArg("index")) {
    index = server.arg("index").toInt();
  }
  
  uint8_t r = 0, g = 0, b = 0;
  bool hasColor = false;
  
  if (server.hasArg("hex")) {
    String hexVal = server.arg("hex");
    if (hexVal.startsWith("#")) hexVal = hexVal.substring(1);
    long colorVal = strtol(hexVal.c_str(), NULL, 16);
    r = (colorVal >> 16) & 0xFF;
    g = (colorVal >> 8) & 0xFF;
    b = colorVal & 0xFF;
    hasColor = true;
  } else if (server.hasArg("r") && server.hasArg("g") && server.hasArg("b")) {
    r = constrain(server.arg("r").toInt(), 0, 255);
    g = constrain(server.arg("g").toInt(), 0, 255);
    b = constrain(server.arg("b").toInt(), 0, 255);
    hasColor = true;
  }
  
  if (hasColor) {
    if (index >= 0 && index < NUM_LEDS) {
      setLED(index, r, g, b);
      Serial.printf("LED[%d] set to R:%d G:%d B:%d\n", index, r, g, b);
    } else {
      setAllLEDs(r, g, b);
      Serial.printf("All LEDs set to R:%d G:%d B:%d\n", r, g, b);
    }
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing color fields (hex or r,g,b)\"}");
  }
}

// GET /api/leds?colors=ff0000,00ff00,0000ff,ffffff
void handleSetMultipleLEDs() {
  if (server.hasArg("colors")) {
    String colorsStr = server.arg("colors");
    int count = 0;
    int startIdx = 0;
    
    while (count < NUM_LEDS) {
      int commaIdx = colorsStr.indexOf(',', startIdx);
      String hexColor;
      if (commaIdx == -1) {
        hexColor = colorsStr.substring(startIdx);
      } else {
        hexColor = colorsStr.substring(startIdx, commaIdx);
      }
      hexColor.trim();
      if (hexColor.startsWith("#")) hexColor = hexColor.substring(1);
      
      long colorVal = strtol(hexColor.c_str(), NULL, 16);
      uint8_t r = (colorVal >> 16) & 0xFF;
      uint8_t g = (colorVal >> 8) & 0xFF;
      uint8_t b = colorVal & 0xFF;
      
      leds[count] = {r, g, b};
      strip.setPixelColor(count, strip.Color(r, g, b));
      
      count++;
      if (commaIdx == -1) break;
      startIdx = commaIdx + 1;
    }
    strip.show();
    Serial.println("Multiple LEDs updated via batch command.");
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing 'colors' parameter\"}");
  }
}

// GET /api/motor?index=[0-3]&speed=[-120..120]
void handleSetMotor() {
  if (server.hasArg("index") && server.hasArg("speed")) {
    int index = server.arg("index").toInt();
    int speed = server.arg("speed").toInt();
    
    setMotorSpeed(index, speed);
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing 'index' or 'speed'\"}");
  }
}

// GET /api/drive?x=X&y=Y
void handleDrive() {
  if (server.hasArg("x") && server.hasArg("y")) {
    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();
    
    // Map joystick coordinates to left and right tracks (x is inverted to correct turning direction)
    float leftPower = y - x;
    float rightPower = y + x;
    
    leftPower = constrain(leftPower, -1.0f, 1.0f);
    rightPower = constrain(rightPower, -1.0f, 1.0f);
    
    // Drive Left: M1 and M3
    // Drive Right: M2 and M4
    setMotorSpeed(0, (int)(leftPower * currentSpeedLimit));  // M1
    setMotorSpeed(2, (int)(leftPower * currentSpeedLimit));  // M3
    setMotorSpeed(1, (int)(rightPower * currentSpeedLimit)); // M2
    setMotorSpeed(3, (int)(rightPower * currentSpeedLimit)); // M4
    
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing x or y\"}");
  }
}

// GET /api/speed?val=VAL
void handleSpeedLimit() {
  if (server.hasArg("val")) {
    int val = server.arg("val").toInt();
    currentSpeedLimit = constrain(val, 40, MAX_SAFE_SPEED);
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing val\"}");
  }
}

// GET /api/test_motor?motor=[0-3|left|right|both]&dir=[forward|reverse]&pwm=[val]&duration=[ms]
void handleTestMotor() {
  if (server.hasArg("motor") && server.hasArg("dir") && server.hasArg("pwm") && server.hasArg("duration")) {
    String motor = server.arg("motor");
    String dir = server.arg("dir");
    int pwm = server.arg("pwm").toInt();
    int duration = server.arg("duration").toInt();
    
    // Safety caps for test mode: PWM capped at 80 (very safe low PWM) and duration at 1500ms
    pwm = constrain(pwm, 0, 80);
    duration = constrain(duration, 0, 1500);
    
    int speed = (dir == "forward") ? pwm : -pwm;
    
    // Stop all motors first
    stopAllMotors();
    delay(50);
    
    if (motor == "0") {
      setMotorSpeed(0, speed);
    } else if (motor == "1") {
      setMotorSpeed(1, speed);
    } else if (motor == "2") {
      setMotorSpeed(2, speed);
    } else if (motor == "3") {
      setMotorSpeed(3, speed);
    } else if (motor == "left") {
      setMotorSpeed(0, speed);
      setMotorSpeed(2, speed);
    } else if (motor == "right") {
      setMotorSpeed(1, speed);
      setMotorSpeed(3, speed);
    } else if (motor == "both") {
      setMotorSpeed(0, speed);
      setMotorSpeed(1, speed);
      setMotorSpeed(2, speed);
      setMotorSpeed(3, speed);
    }
    
    // Set timer for timed test expiration
    testEndTime = millis() + duration;
    isTestModeActive = true;
    
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing parameters\"}");
  }
}

// GET /api/stop
void handleStop() {
  stopAllMotors();
  isTestModeActive = false;
  Serial.println("All motors stopped via API command.");
  server.send(200, "application/json", "{\"status\":\"success\"}");
}

// GET /api/status
void handleGetStatus() {
  String json = "{";
  json += "\"ssid\":\"" + String(WIFI_SSID) + "\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  
  // LED status
  json += "\"leds\":[";
  for (int i = 0; i < NUM_LEDS; i++) {
    json += "{\"r\":" + String(leds[i].r) + ",\"g\":" + String(leds[i].g) + ",\"b\":" + String(leds[i].b) + "}";
    if (i < NUM_LEDS - 1) json += ",";
  }
  json += "],";
  
  // Motor status
  json += "\"motors\":[";
  for (int i = 0; i < 4; i++) {
    json += String(motors[i].currentSpeed);
    if (i < 3) json += ",";
  }
  json += "]}";
  
  server.send(200, "application/json", json);
}

// HTML Dashboard content
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Maker-ESP32 Board Dashboard</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg: #0b0d19;
      --card-bg: #111424;
      --text: #f0f3f8;
      --text-muted: #79809c;
      --primary: #5850ec;
      --primary-hover: #4f46e5;
      --green: #10b981;
      --red: #ef4444;
      --red-hover: #dc2626;
      --border: rgba(255, 255, 255, 0.04);
      --led-off: #24293f;
    }
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Outfit', sans-serif;
    }
    body {
      background-color: var(--bg);
      color: var(--text);
      display: flex;
      justify-content: center;
      align-items: center;
      min-height: 100vh;
      padding: 15px;
    }
    .container {
      width: 100%;
      max-width: 600px;
      background: var(--card-bg);
      border: 1px solid var(--border);
      border-radius: 28px;
      padding: 25px;
      box-shadow: 0 25px 60px rgba(0,0,0,0.5);
    }
    header {
      text-align: center;
      margin-bottom: 20px;
    }
    header h1 {
      font-size: 24px;
      font-weight: 700;
      letter-spacing: -0.5px;
      margin-bottom: 4px;
    }
    header p {
      color: var(--text-muted);
      font-size: 13px;
    }
    .status-badge {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      background: rgba(16, 185, 129, 0.08);
      color: var(--green);
      padding: 4px 12px;
      border-radius: 50px;
      font-size: 12px;
      font-weight: 600;
      margin-top: 10px;
    }
    .status-dot {
      width: 6px;
      height: 6px;
      background-color: var(--green);
      border-radius: 50%;
      box-shadow: 0 0 10px var(--green);
      animation: pulse 2.5s infinite;
    }
    .info-bar {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-bottom: 25px;
      background: rgba(255,255,255,0.01);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 12px;
    }
    .info-item {
      text-align: center;
    }
    .info-item .label {
      color: var(--text-muted);
      font-size: 10px;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      margin-bottom: 3px;
    }
    .info-item .value {
      font-size: 13px;
      font-weight: 600;
    }
    .tabs-nav {
      display: flex;
      background: rgba(0,0,0,0.2);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 4px;
      margin-bottom: 25px;
      gap: 4px;
    }
    .tab-nav-btn {
      flex: 1;
      background: transparent;
      border: none;
      color: var(--text-muted);
      font-weight: 600;
      font-size: 13px;
      height: 38px;
      border-radius: 10px;
      cursor: pointer;
      transition: all 0.2s;
    }
    .tab-nav-btn.active {
      background: var(--primary);
      color: #fff;
      box-shadow: 0 4px 10px rgba(88, 80, 236, 0.2);
    }
    .tab-content {
      display: none;
    }
    .tab-content.active {
      display: block;
    }
    .leds-container {
      background: rgba(0,0,0,0.15);
      border-radius: 20px;
      padding: 20px;
      border: 1px solid var(--border);
    }
    .led-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 12px;
      margin-bottom: 25px;
    }
    .led-slot {
      display: flex;
      flex-direction: column;
      align-items: center;
      background: rgba(255, 255, 255, 0.02);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 12px 6px;
      cursor: pointer;
      transition: all 0.2s;
    }
    .led-slot.selected {
      border-color: var(--primary);
      background: rgba(88, 80, 236, 0.04);
    }
    .led-index-label {
      font-size: 11px;
      font-weight: 600;
      color: var(--text-muted);
      margin-bottom: 8px;
    }
    .led-bulb {
      width: 36px;
      height: 36px;
      border-radius: 50%;
      background-color: var(--led-off);
      box-shadow: inset 0 2px 5px rgba(0,0,0,0.5);
      transition: all 0.3s cubic-bezier(0.16, 1, 0.3, 1);
    }
    .control-panel {
      background: rgba(255, 255, 255, 0.02);
      border: 1px solid var(--border);
      border-radius: 20px;
      padding: 20px;
    }
    .panel-header {
      font-size: 14px;
      font-weight: 600;
      margin-bottom: 15px;
      text-align: center;
      color: var(--text-muted);
    }
    .color-presets {
      display: grid;
      grid-template-columns: repeat(5, 1fr);
      gap: 8px;
      margin-bottom: 20px;
    }
    .preset-btn {
      border: none;
      height: 38px;
      border-radius: 10px;
      cursor: pointer;
      font-weight: 600;
      font-size: 12px;
      color: #fff;
      transition: all 0.2s;
    }
    .preset-btn:active { transform: scale(0.95); }
    .pre-red { background-color: #ef4444; }
    .pre-green { background-color: #10b981; }
    .pre-blue { background-color: #3b82f6; }
    .pre-purple { background-color: #8b5cf6; }
    .pre-off { background-color: #374151; }
    .custom-section {
      display: flex;
      align-items: center;
      justify-content: space-between;
      border-top: 1px dashed var(--border);
      padding-top: 15px;
    }
    .custom-section label {
      font-size: 13px;
      font-weight: 500;
      color: var(--text-muted);
    }
    .picker-container {
      position: relative;
      width: 44px;
      height: 44px;
      border-radius: 50%;
      overflow: hidden;
      border: 2px solid rgba(255, 255, 255, 0.15);
    }
    .picker-container input[type="color"] {
      position: absolute;
      top: -10px;
      left: -10px;
      width: 64px;
      height: 64px;
      cursor: pointer;
      border: none;
      background: none;
    }
    .batch-section {
      display: flex;
      gap: 10px;
      margin-top: 15px;
    }
    .batch-btn {
      flex: 1;
      height: 40px;
      border-radius: 12px;
      border: 1px solid var(--border);
      background: rgba(255,255,255,0.03);
      color: var(--text);
      font-weight: 500;
      font-size: 13px;
      cursor: pointer;
      transition: all 0.2s;
    }
    .batch-btn:hover { background: rgba(255,255,255,0.06); }
    .batch-btn.primary-action { background: var(--primary); border: none; color: #fff; }
    .batch-btn.primary-action:hover { background: var(--primary-hover); }
    .motors-container {
      background: rgba(0,0,0,0.15);
      border-radius: 20px;
      padding: 20px;
      border: 1px solid var(--border);
    }
    .motor-row {
      background: rgba(255, 255, 255, 0.01);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 15px;
      margin-bottom: 12px;
    }
    .motor-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 10px;
    }
    .motor-title {
      font-size: 14px;
      font-weight: 600;
    }
    .motor-speed-badge {
      font-size: 12px;
      font-weight: 700;
      background: rgba(88, 80, 236, 0.1);
      color: #908bf9;
      padding: 2px 8px;
      border-radius: 6px;
      min-width: 45px;
      text-align: center;
    }
    .motor-controls {
      display: flex;
      align-items: center;
      gap: 15px;
    }
    .motor-slider-wrapper {
      flex: 1;
      display: flex;
      align-items: center;
    }
    .m-slider {
      width: 100%;
      height: 6px;
      border-radius: 5px;
      background: #1d2138;
      outline: none;
      -webkit-appearance: none;
      appearance: none;
    }
    .m-slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: var(--primary);
      cursor: pointer;
      box-shadow: 0 0 8px var(--primary);
      transition: background 0.15s;
    }
    .m-slider::-webkit-slider-thumb:hover {
      background: #4f46e5;
    }
    .motor-action-btn {
      background: rgba(255,255,255,0.03);
      border: 1px solid var(--border);
      color: var(--text);
      font-size: 11px;
      font-weight: 600;
      padding: 6px 10px;
      border-radius: 8px;
      cursor: pointer;
      transition: all 0.2s;
      min-width: 50px;
    }
    .motor-action-btn:hover {
      background: rgba(255,255,255,0.08);
    }
    .estop-btn {
      width: 100%;
      height: 46px;
      background-color: var(--red);
      color: #fff;
      border: none;
      border-radius: 14px;
      font-weight: 700;
      font-size: 15px;
      cursor: pointer;
      box-shadow: 0 4px 15px rgba(239, 68, 68, 0.25);
      transition: all 0.2s;
      margin-top: 10px;
    }
    .estop-btn:hover { background-color: var(--red-hover); }
    .estop-btn:active { transform: scale(0.98); }
    footer {
      text-align: center;
      font-size: 11px;
      color: var(--text-muted);
      margin-top: 20px;
    }
    @keyframes pulse {
      0% { opacity: 0.5; box-shadow: 0 0 4px var(--green); }
      50% { opacity: 1; box-shadow: 0 0 12px var(--green); }
      100% { opacity: 0.5; box-shadow: 0 0 4px var(--green); }
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>Maker-ESP32 Control Panel</h1>
      <p>Robotics & Actuator Development Interface</p>
      <div class="status-badge">
        <div class="status-dot"></div>
        <span>CONNECTED</span>
      </div>
    </header>
    
    <div class="info-bar">
      <div class="info-item">
        <div class="label">SSID</div>
        <div class="value" id="val-ssid">...</div>
      </div>
      <div class="info-item">
        <div class="label">Signal</div>
        <div class="value" id="val-rssi">...</div>
      </div>
      <div class="info-item">
        <div class="label">Local IP</div>
        <div class="value" id="val-ip">...</div>
      </div>
    </div>

    <!-- Navigation Tabs -->
    <div class="tabs-nav">
      <button class="tab-nav-btn active" id="btn-leds" onclick="switchTab('leds')">💡 LEDs</button>
      <button class="tab-nav-btn" id="btn-test" onclick="switchTab('test')">🛡️ Safe Test</button>
      <button class="tab-nav-btn" id="btn-motors" onclick="switchTab('motors')">⚙️ Sliders</button>
    </div>
    
    <!-- LED CONTROL TAB -->
    <div class="tab-content active" id="tab-leds">
      <div class="leds-container">
        <div class="led-grid">
          <div class="led-slot selected" id="slot-0" onclick="selectLED(0)">
            <div class="led-index-label">LED 0</div>
            <div class="led-bulb" id="bulb-0"></div>
          </div>
          <div class="led-slot" id="slot-1" onclick="selectLED(1)">
            <div class="led-index-label">LED 1</div>
            <div class="led-bulb" id="bulb-1"></div>
          </div>
          <div class="led-slot" id="slot-2" onclick="selectLED(2)">
            <div class="led-index-label">LED 2</div>
            <div class="led-bulb" id="bulb-2"></div>
          </div>
          <div class="led-slot" id="slot-3" onclick="selectLED(3)">
            <div class="led-index-label">LED 3</div>
            <div class="led-bulb" id="bulb-3"></div>
          </div>
        </div>
        
        <div class="control-panel">
          <div class="panel-header" id="control-label">Control LED 0</div>
          
          <div class="color-presets">
            <button class="preset-btn pre-red" onclick="applyPreset(255, 0, 0)">Red</button>
            <button class="preset-btn pre-green" onclick="applyPreset(0, 255, 0)">Green</button>
            <button class="preset-btn pre-blue" onclick="applyPreset(0, 0, 255)">Blue</button>
            <button class="preset-btn pre-purple" onclick="applyPreset(139, 92, 246)">Purple</button>
            <button class="preset-btn pre-off" onclick="applyPreset(0, 0, 0)">Off</button>
          </div>
          
          <div class="custom-section">
            <label>Custom Color Picker</label>
            <div class="picker-container">
              <input type="color" id="picker" oninput="applyHex(this.value)" value="#ff0000">
            </div>
          </div>
        </div>
        
        <div class="batch-section">
          <button class="batch-btn" onclick="applyAll(0, 0, 0)">All Off</button>
          <button class="batch-btn primary-action" onclick="applyAll(255, 0, 0)">All Red</button>
        </div>
      </div>
    </div>

    <!-- SAFE TEST LAB TAB -->
    <div class="tab-content" id="tab-test">
      <div class="motors-container" style="background: rgba(0,0,0,0.15); border-radius: 20px; padding: 20px; border: 1px solid var(--border); display: flex; flex-direction: column; gap: 15px;">
        <h3 style="font-size: 15px; color: var(--primary); text-transform: uppercase; border-bottom: 1px solid var(--border); padding-bottom: 8px; font-weight: bold;">🛡️ Safe Motor Verification</h3>
        <p style="font-size: 12px; color: var(--text-muted); line-height: 1.4;">
          <strong>Instructions:</strong> Elevate the tank tracks off the ground before testing.
          Pulse each test below at a safe, low voltage (capped at 50 PWM out of 255) and verify correct direction.
        </p>

        <div style="background: rgba(255,255,255,0.01); border: 1px solid var(--border); border-radius: 14px; padding: 12px; display: flex; flex-direction: column; gap: 10px;">
          <!-- Left Forward -->
          <div style="display: flex; align-items: center; justify-content: space-between;">
            <button class="motor-action-btn" style="flex: 1; max-width: 155px;" onclick="runTestPulse('left', 'forward')">⚡ Pulse Left Fwd</button>
            <label style="display: flex; align-items: center; gap: 6px; font-size: 13px; cursor: pointer; user-select: none;">
              <input type="checkbox" id="chk-lf" onchange="checkVerificationUnlock()"> Pass
            </label>
          </div>
          <!-- Left Reverse -->
          <div style="display: flex; align-items: center; justify-content: space-between;">
            <button class="motor-action-btn" style="flex: 1; max-width: 155px;" onclick="runTestPulse('left', 'reverse')">⚡ Pulse Left Rev</button>
            <label style="display: flex; align-items: center; gap: 6px; font-size: 13px; cursor: pointer; user-select: none;">
              <input type="checkbox" id="chk-lr" onchange="checkVerificationUnlock()"> Pass
            </label>
          </div>
          <!-- Right Forward -->
          <div style="display: flex; align-items: center; justify-content: space-between;">
            <button class="motor-action-btn" style="flex: 1; max-width: 155px;" onclick="runTestPulse('right', 'forward')">⚡ Pulse Right Fwd</button>
            <label style="display: flex; align-items: center; gap: 6px; font-size: 13px; cursor: pointer; user-select: none;">
              <input type="checkbox" id="chk-rf" onchange="checkVerificationUnlock()"> Pass
            </label>
          </div>
          <!-- Right Reverse -->
          <div style="display: flex; align-items: center; justify-content: space-between;">
            <button class="motor-action-btn" style="flex: 1; max-width: 155px;" onclick="runTestPulse('right', 'reverse')">⚡ Pulse Right Rev</button>
            <label style="display: flex; align-items: center; gap: 6px; font-size: 13px; cursor: pointer; user-select: none;">
              <input type="checkbox" id="chk-rr" onchange="checkVerificationUnlock()"> Pass
            </label>
          </div>
        </div>

        <button id="btn-unlock" class="estop-btn" style="background-color: var(--green); box-shadow: 0 4px 15px rgba(16, 185, 129, 0.25); cursor: not-allowed; opacity: 0.5; margin-top: 5px;" onclick="unlockManualSliders()" disabled>🔓 Unlock manual controls</button>
      </div>

      <!-- WIRING DIAGRAM PANEL -->
      <div class="motors-container" style="margin-top: 15px; background: rgba(0,0,0,0.2); border-radius: 20px; padding: 20px; border: 1px solid var(--border);">
        <h3 style="font-size: 14px; color: var(--green); text-transform: uppercase; border-bottom: 1px solid var(--border); padding-bottom: 8px; margin-bottom: 10px; font-weight: bold;">🔌 ESP Board Wiring Map</h3>
        <div style="font-family: monospace; font-size: 11px; color: var(--text-muted); line-height: 1.5; white-space: pre-wrap; background: #070911; padding: 12px; border-radius: 8px; border: 1px solid rgba(255,255,255,0.02);">
[M1] GPIO 27, 13  ===>  Left Front Motor
[M3] GPIO 17, 12  ===>  Left Rear Motor
[M2] GPIO 04, 02  ===>  Right Front Motor
[M4] GPIO 14, 15  ===>  Right Rear Motor
----------------------------------------
[RGB] GPIO 16     ===>  4 Onboard NeoPixels
[OLED] GPIO 22(SCL), 21(SDA) ===> Display
        </div>
      </div>
    </div>

    <!-- MOTOR TEST TAB -->
    <div class="tab-content" id="tab-motors" style="position: relative;">
      <!-- Sliders Lockout Overlay -->
      <div id="motors-lockout" style="position: absolute; top: 0; left: 0; right: 0; bottom: 0; background: rgba(17, 20, 36, 0.96); z-index: 10; display: flex; flex-direction: column; align-items: center; justify-content: center; border-radius: 20px; text-align: center; padding: 20px;">
        <span style="font-size: 40px; margin-bottom: 12px; filter: drop-shadow(0 0 10px rgba(0,0,0,0.3));">🔒</span>
        <h3 style="color: var(--red); margin-bottom: 6px; font-weight: bold; letter-spacing: 0.5px;">MANUAL SLIDERS LOCKED</h3>
        <p style="font-size: 13px; color: var(--text-muted); max-width: 80%; line-height: 1.4;">You must complete and verify the safety checklist in the <strong>Safe Test</strong> tab to unlock full manual speed sliders.</p>
      </div>

      <div class="motors-container">
        <!-- Motor 1 -->
        <div class="motor-row">
          <div class="motor-header">
            <span class="motor-title">Motor 1 (M1) <span style="font-size: 11px; color: var(--text-muted);">(GPIO 27, 13)</span></span>
            <span class="motor-speed-badge" id="motor-val-0">0</span>
          </div>
          <div class="motor-controls">
            <button class="motor-action-btn" onclick="triggerMotor(0, -80)">Rev</button>
            <div class="motor-slider-wrapper">
              <input type="range" min="-120" max="120" value="0" class="m-slider" id="motor-slider-0"
                     oninput="handleMotorSlider(0, this.value)"
                     onmouseup="resetMotorSlider(0)"
                     ontouchend="resetMotorSlider(0)">
            </div>
            <button class="motor-action-btn" onclick="triggerMotor(0, 80)">Fwd</button>
          </div>
        </div>

        <!-- Motor 2 -->
        <div class="motor-row">
          <div class="motor-header">
            <span class="motor-title">Motor 2 (M2) <span style="font-size: 11px; color: var(--text-muted);">(GPIO 4, 2)</span></span>
            <span class="motor-speed-badge" id="motor-val-1">0</span>
          </div>
          <div class="motor-controls">
            <button class="motor-action-btn" onclick="triggerMotor(1, -80)">Rev</button>
            <div class="motor-slider-wrapper">
              <input type="range" min="-120" max="120" value="0" class="m-slider" id="motor-slider-1"
                     oninput="handleMotorSlider(1, this.value)"
                     onmouseup="resetMotorSlider(1)"
                     ontouchend="resetMotorSlider(1)">
            </div>
            <button class="motor-action-btn" onclick="triggerMotor(1, 80)">Fwd</button>
          </div>
        </div>

        <!-- Motor 3 -->
        <div class="motor-row">
          <div class="motor-header">
            <span class="motor-title">Motor 3 (M3) <span style="font-size: 11px; color: var(--text-muted);">(GPIO 17, 12)</span></span>
            <span class="motor-speed-badge" id="motor-val-2">0</span>
          </div>
          <div class="motor-controls">
            <button class="motor-action-btn" onclick="triggerMotor(2, -80)">Rev</button>
            <div class="motor-slider-wrapper">
              <input type="range" min="-120" max="120" value="0" class="m-slider" id="motor-slider-2"
                     oninput="handleMotorSlider(2, this.value)"
                     onmouseup="resetMotorSlider(2)"
                     ontouchend="resetMotorSlider(2)">
            </div>
            <button class="motor-action-btn" onclick="triggerMotor(2, 80)">Fwd</button>
          </div>
        </div>

        <!-- Motor 4 -->
        <div class="motor-row">
          <div class="motor-header">
            <span class="motor-title">Motor 4 (M4) <span style="font-size: 11px; color: var(--text-muted);">(GPIO 14, 15)</span></span>
            <span class="motor-speed-badge" id="motor-val-3">0</span>
          </div>
          <div class="motor-controls">
            <button class="motor-action-btn" onclick="triggerMotor(3, -80)">Rev</button>
            <div class="motor-slider-wrapper">
              <input type="range" min="-120" max="120" value="0" class="m-slider" id="motor-slider-3"
                     oninput="handleMotorSlider(3, this.value)"
                     onmouseup="resetMotorSlider(3)"
                     ontouchend="resetMotorSlider(3)">
            </div>
            <button class="motor-action-btn" onclick="triggerMotor(3, 80)">Fwd</button>
          </div>
        </div>

        <!-- Emergency Stop -->
        <button class="estop-btn" onclick="emergencyStop()">🛑 EMERGENCY STOP ALL</button>
      </div>
    </div>
    
    <footer>
      MAX PWM safety limit constrained to 120 (47% power)
    </footer>
  </div>
  
  <script>
    let selectedIndex = 0;
    let lastMotorTime = 0;
    let isSlidersUnlocked = false;
    
    function switchTab(tabId) {
      document.getElementById('tab-leds').classList.toggle('active', tabId === 'leds');
      document.getElementById('tab-test').classList.toggle('active', tabId === 'test');
      document.getElementById('tab-motors').classList.toggle('active', tabId === 'motors');
      document.getElementById('btn-leds').classList.toggle('active', tabId === 'leds');
      document.getElementById('btn-test').classList.toggle('active', tabId === 'test');
      document.getElementById('btn-motors').classList.toggle('active', tabId === 'motors');
      
      if (tabId !== 'motors' && tabId !== 'test') {
        emergencyStop();
      }
    }

    function selectLED(idx) {
      selectedIndex = idx;
      for (let i = 0; i < 4; i++) {
        document.getElementById(`slot-${i}`).classList.toggle('selected', i === idx);
      }
      document.getElementById('control-label').innerText = `Control LED ${idx}`;
      
      const bulb = document.getElementById(`bulb-${idx}`);
      const rgb = window.getComputedStyle(bulb).backgroundColor;
      const hex = rgbToHex(rgb);
      document.getElementById('picker').value = hex;
    }
    
    function rgbToHex(rgb) {
      const match = rgb.match(/^rgb\((\d+),\s*(\d+),\s*(\d+)\)$/);
      if (!match) return '#ffffff';
      const r = parseInt(match[1]).toString(16).padStart(2, '0');
      const g = parseInt(match[2]).toString(16).padStart(2, '0');
      const b = parseInt(match[3]).toString(16).padStart(2, '0');
      return `#${r}${g}${b}`;
    }
    
    async function applyPreset(r, g, b) {
      try {
        await fetch(`/api/led?index=${selectedIndex}&r=${r}&g=${g}&b=${b}`);
        updateStatus();
      } catch (err) { console.error('API Error:', err); }
    }
    
    async function applyHex(hex) {
      try {
        await fetch(`/api/led?index=${selectedIndex}&hex=${hex.substring(1)}`);
        updateStatus();
      } catch (err) { console.error('API Error:', err); }
    }
    
    async function applyAll(r, g, b) {
      try {
        await fetch(`/api/led?r=${r}&g=${g}&b=${b}`);
        updateStatus();
      } catch (err) { console.error('API Error:', err); }
    }

    async function sendMotorSpeed(index, speed) {
      const now = Date.now();
      if (speed === 0 || now - lastMotorTime > 100) {
        lastMotorTime = now;
        try {
          await fetch(`/api/motor?index=${index}&speed=${speed}`);
        } catch (err) {
          console.error('Motor API Error:', err);
        }
      }
    }

    function handleMotorSlider(index, value) {
      document.getElementById(`motor-val-${index}`).innerText = `${value > 0 ? '+' : ''}${value}`;
      sendMotorSpeed(index, parseInt(value));
    }

    function resetMotorSlider(index) {
      const slider = document.getElementById(`motor-slider-${index}`);
      slider.value = 0;
      document.getElementById(`motor-val-${index}`).innerText = '0';
      sendMotorSpeed(index, 0);
    }

    async function triggerMotor(index, speed) {
      const slider = document.getElementById(`motor-slider-${index}`);
      slider.value = speed;
      document.getElementById(`motor-val-${index}`).innerText = `${speed > 0 ? '+' : ''}${speed}`;
      try {
        await fetch(`/api/motor?index=${index}&speed=${speed}`);
      } catch (err) {
        console.error('Motor API Error:', err);
      }
    }

    async function runTestPulse(motor, dir) {
      try {
        // Capped at safe PWM=50 and duration=500ms
        await fetch(`/api/test_motor?motor=${motor}&dir=${dir}&pwm=50&duration=500`);
      } catch (err) {
        console.error('Pulse Test API Error:', err);
      }
    }

    function checkVerificationUnlock() {
      const lf = document.getElementById('chk-lf').checked;
      const lr = document.getElementById('chk-lr').checked;
      const rf = document.getElementById('chk-rf').checked;
      const rr = document.getElementById('chk-rr').checked;
      
      const btn = document.getElementById('btn-unlock');
      if (lf && lr && rf && rr) {
        btn.disabled = false;
        btn.style.opacity = '1';
        btn.style.cursor = 'pointer';
      } else {
        btn.disabled = true;
        btn.style.opacity = '0.5';
        btn.style.cursor = 'not-allowed';
      }
    }

    function unlockManualSliders() {
      isSlidersUnlocked = true;
      document.getElementById('motors-lockout').style.display = 'none';
      switchTab('motors');
    }

    async function emergencyStop() {
      try {
        await fetch('/api/stop');
        for (let i = 0; i < 4; i++) {
          document.getElementById(`motor-slider-${i}`).value = 0;
          document.getElementById(`motor-val-${i}`).innerText = '0';
        }
        updateStatus();
      } catch (err) {
        console.error('Stop API Error:', err);
      }
    }
    
    async function updateStatus() {
      try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        document.getElementById('val-ssid').innerText = data.ssid;
        document.getElementById('val-rssi').innerText = data.rssi + ' dBm';
        document.getElementById('val-ip').innerText = data.ip;
        
        for (let i = 0; i < 4; i++) {
          const led = data.leds[i];
          const colorString = `rgb(${led.r}, ${led.g}, ${led.b})`;
          const bulb = document.getElementById(`bulb-${i}`);
          bulb.style.backgroundColor = colorString;
          
          if (led.r === 0 && led.g === 0 && led.b === 0) {
            bulb.style.boxShadow = 'none';
          } else {
            bulb.style.boxShadow = `0 0 15px ${colorString}`;
          }
        }

        for (let i = 0; i < 4; i++) {
          const slider = document.getElementById(`motor-slider-${i}`);
          if (slider.value == 0 && data.motors) {
            const activeSpeed = data.motors[i];
            document.getElementById(`motor-val-${i}`).innerText = `${activeSpeed > 0 ? '+' : ''}${activeSpeed}`;
          }
        }
      } catch (err) {
        console.error('Status Sync failed:', err);
      }
    }
    
    updateStatus();
    setInterval(updateStatus, 3000);
  </script>
</body>
</html>
)rawliteral";

// Serve root webpage
void handleRoot() {
  server.send_P(200, "text/html", HTML_CONTENT);
}

void setup() {
  // Disable brownout detector to prevent low-voltage reset loop
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Initialize Serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=================================");
  Serial.println("Maker-ESP32 Main Firmware Project");
  Serial.println("=================================");

  // Initialize I2C bus
  Wire.begin(21, 22);

  // Initialize SSD1306 OLED Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 OLED allocation failed or not connected."));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Maker-ESP32 Booting...");
    display.println("Initializing OLED...");
    display.display();
    delay(500);
  }

  // Initialize NeoPixels
  strip.begin();
  strip.setBrightness(128); // Moderate brightness
  // Soft pulsing Google-themed startup transition
  setLED(0, 66, 133, 244);  // Google Blue
  setLED(1, 234, 67, 53);   // Google Red
  setLED(2, 251, 188, 5);   // Google Yellow
  setLED(3, 52, 168, 83);   // Google Green
  Serial.println("Onboard RGB LEDs initialized to Google theme colors.");

  // Initialize DC Motors
  for (int i = 0; i < 4; i++) {
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
    digitalWrite(motors[i].in1, LOW);
    digitalWrite(motors[i].in2, LOW);
    
    // Attach pins to LEDC channels (1000Hz frequency, 8-bit resolution)
    setupPWM(motors[i].in1, 1000, 8, motors[i].chan1);
    setupPWM(motors[i].in2, 1000, 8, motors[i].chan2);
    
    // Verify duty cycle starts at 0
    writePWM(motors[i].in1, motors[i].chan1, 0);
    writePWM(motors[i].in2, motors[i].chan2, 0);
  }
  Serial.println("Motors configured and safety stopped.");

  // Connect to WiFi
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Maker-ESP32 Connecting");
  display.print("SSID: ");
  display.println(WIFI_SSID);
  display.display();

  // Set WiFi Tx Power lower to prevent high current spikes that crash the power rail
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Maker-ESP32 Connecting");
    display.print("SSID: ");
    display.println(WIFI_SSID);
    display.print("Retries: ");
    display.print(retry);
    display.println("/30");
    display.display();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    refreshOLEDDisplay();
  } else {
    Serial.println("\nWiFi Connection failed! Starting in standalone mode.");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("WiFi: Failed!");
    display.println("Running Offline.");
    display.display();
  }

  // Setup HTTP Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/led", HTTP_GET, handleSetLED);
  server.on("/api/leds", HTTP_GET, handleSetMultipleLEDs);
  server.on("/api/motor", HTTP_GET, handleSetMotor);
  server.on("/api/drive", HTTP_GET, handleDrive);
  server.on("/api/speed", HTTP_GET, handleSpeedLimit);
  server.on("/api/test_motor", HTTP_GET, handleTestMotor);
  server.on("/api/stop", HTTP_GET, handleStop);
  server.on("/api/status", HTTP_GET, handleGetStatus);

  // Start the Server
  server.begin();
  Serial.println("WebServer started on port 80.");
}

void loop() {
  server.handleClient();

  // Watchdog safety check: if motors are active and no command was received for 2 seconds (2000ms), stop them
  if (motorsActive && !isTestModeActive && (millis() - lastMotorCmdTime > 2000)) {
    Serial.println("Watchdog: No motor command received for 2000ms. Safety stopping all motors.");
    stopAllMotors();
  }

  // Safe motor timed test expiration check
  if (isTestModeActive && (millis() >= testEndTime)) {
    Serial.println("Test mode completed. Safety stopping motors.");
    stopAllMotors();
    isTestModeActive = false;
  }

  // Periodic UDP broadcast beacon for server autodiscovery
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBeaconTime >= beaconInterval) {
      lastBeaconTime = currentMillis;

      IPAddress ip = WiFi.localIP();
      IPAddress broadcastIP(255, 255, 255, 255);
      
      String beaconMsg = "{\"device\":\"maker-esp32\",\"ip\":\"" + ip.toString() + "\",\"ssid\":\"" + String(WIFI_SSID) + "\",\"sensors\":{";
      
      // Include motor speed telemetry
      beaconMsg += "\"motors\":[";
      for (int i = 0; i < 4; i++) {
        beaconMsg += String(motors[i].currentSpeed);
        if (i < 3) beaconMsg += ",";
      }
      beaconMsg += "],";
      
      // Include LED states
      beaconMsg += "\"leds\":[";
      for (int i = 0; i < NUM_LEDS; i++) {
        beaconMsg += "{\"r\":" + String(leds[i].r) + ",\"g\":" + String(leds[i].g) + ",\"b\":" + String(leds[i].b) + "}";
        if (i < NUM_LEDS - 1) beaconMsg += ",";
      }
      beaconMsg += "]";
      beaconMsg += "}}";

      udp.beginPacket(broadcastIP, udpPort);
      udp.print(beaconMsg);
      udp.endPacket();
      
      // Periodically refresh OLED display
      refreshOLEDDisplay();
    }
  }

  delay(2);
}
