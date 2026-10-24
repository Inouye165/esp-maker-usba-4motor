#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Lonely Binary ESP32-S3 N16R8 onboard WS2812 RGB LED pins across board revisions
static constexpr int LED_PIN_PRIMARY   = 48;
static constexpr int LED_PIN_SECONDARY = 38;
static constexpr int NUM_PIXELS        = 1;
static constexpr uint8_t LED_BRIGHTNESS = 64;

// Timeout without RPi5 heartbeat before reverting to Solid Red (RPi5 off/disconnected)
static constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 1500;

Adafruit_NeoPixel pixel48(NUM_PIXELS, LED_PIN_PRIMARY, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel38(NUM_PIXELS, LED_PIN_SECONDARY, NEO_GRB + NEO_KHZ800);

enum class LedState : uint8_t {
  RPI_OFF_RED      = 0,
  RPI_ON_GREEN     = 1,
  ROVER_MOVING_BLUE = 2
};

static LedState currentState = LedState::RPI_OFF_RED;
static unsigned long lastHeartbeatMs = 0;
static bool hasHeartbeat = false;
static unsigned long lastRefreshMs = 0;

static const char* stateName(LedState state) {
  switch (state) {
    case LedState::RPI_ON_GREEN:      return "GREEN";
    case LedState::ROVER_MOVING_BLUE: return "BLUE";
    case LedState::RPI_OFF_RED:
    default:                          return "RED";
  }
}

static void applyLedColor(LedState state) {
  uint8_t r = 0, g = 0, b = 0;
  switch (state) {
    case LedState::RPI_ON_GREEN:
      r = 0; g = 255; b = 0;
      break;
    case LedState::ROVER_MOVING_BLUE:
      r = 0; g = 0; b = 255;
      break;
    case LedState::RPI_OFF_RED:
    default:
      r = 255; g = 0; b = 0;
      break;
  }

  pixel48.setPixelColor(0, pixel48.Color(r, g, b));
  pixel48.show();

  pixel38.setPixelColor(0, pixel38.Color(r, g, b));
  pixel38.show();
}

static void setState(LedState nextState) {
  if (currentState != nextState) {
    currentState = nextState;
    applyLedColor(currentState);
    Serial.printf("[ESP32S3_LED] State -> %s (uptime=%lu ms)\n", stateName(currentState), millis());
  }
}

static void handleCommandLine(const String& rawLine) {
  String cmd = rawLine;
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) {
    return;
  }

  unsigned long now = millis();
  if (cmd == "G" || cmd == "GREEN" || cmd == "ON" || cmd == "IDLE") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    setState(LedState::RPI_ON_GREEN);
  } else if (cmd == "B" || cmd == "BLUE" || cmd == "MOVING" || cmd == "MOVE") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    setState(LedState::ROVER_MOVING_BLUE);
  } else if (cmd == "R" || cmd == "RED" || cmd == "OFF" || cmd == "SHUTDOWN") {
    hasHeartbeat = false;
    setState(LedState::RPI_OFF_RED);
  } else if (cmd == "?" || cmd == "STATUS") {
    Serial.printf("[ESP32S3_LED] STATUS=%s HEARTBEAT=%d AGE_MS=%lu UPTIME_MS=%lu\n",
                  stateName(currentState),
                  hasHeartbeat ? 1 : 0,
                  hasHeartbeat ? (now - lastHeartbeatMs) : 0UL,
                  now);
  }
}

void setup() {
  Serial.begin(115200);

  pixel48.begin();
  pixel48.setBrightness(LED_BRIGHTNESS);
  pixel38.begin();
  pixel38.setBrightness(LED_BRIGHTNESS);

  // Default to Solid Red until RPi5 heartbeat arrives
  currentState = LedState::RPI_OFF_RED;
  applyLedColor(currentState);
  lastRefreshMs = millis();
}

void loop() {
  static String rxBuf;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) {
        handleCommandLine(rxBuf);
        rxBuf = "";
      }
    } else if (rxBuf.length() < 64) {
      rxBuf += c;
    } else {
      rxBuf = "";
    }
  }

  unsigned long now = millis();

  // Revert to Solid Red if RPi5 heartbeat stops arriving (RPi5 powered off / halted / service stopped)
  if (hasHeartbeat && (now - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
    hasHeartbeat = false;
    setState(LedState::RPI_OFF_RED);
  }

  // Periodic refresh every 250ms to guard against any power-glitch pixel reset
  if (now - lastRefreshMs >= 250) {
    lastRefreshMs = now;
    applyLedColor(currentState);
  }

  delay(5);
}
