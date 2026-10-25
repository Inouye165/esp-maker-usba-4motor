#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Lonely Binary ESP32-S3 N16R8 onboard WS2812 RGB LED pins across board revisions
static constexpr int LED_PIN_PRIMARY   = 48;
static constexpr int LED_PIN_SECONDARY = 38;
static constexpr int NUM_PIXELS        = 1;
static constexpr uint8_t LED_BRIGHTNESS = 64;

// GPIO UART connection to Raspberry Pi 5 header (Pi GPIO14 TX -> ESP GPIO18 RX, Pi GPIO15 RX <- ESP GPIO17 TX)
static constexpr int UART_RX_PIN = 18;
static constexpr int UART_TX_PIN = 17;
static constexpr unsigned long UART_BAUD = 115200;

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
static bool uartTxEnabled = false;
static unsigned long lastRefreshMs = 0;

static void setUartTxDriven(bool enable) {
  if (enable && !uartTxEnabled) {
    Serial1.setPins(UART_RX_PIN, UART_TX_PIN);
    uartTxEnabled = true;
  } else if (!enable && uartTxEnabled) {
    // Detach TX pin and place GPIO17 in high-Z INPUT so powered ESP32-S3 never backfeeds an unpowered RPi5
    Serial1.setPins(UART_RX_PIN, -1);
    pinMode(UART_TX_PIN, INPUT);
    uartTxEnabled = false;
  }
}

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
    if (uartTxEnabled) {
      Serial1.printf("[ESP32S3_LED] State -> %s (uptime=%lu ms)\n", stateName(currentState), millis());
    }
  }
}

static void handleCommandLine(const String& rawLine, Stream& replyStream, bool isGpioUart) {
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
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    setState(LedState::RPI_ON_GREEN);
  } else if (cmd == "B" || cmd == "BLUE" || cmd == "MOVING" || cmd == "MOVE") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    setState(LedState::ROVER_MOVING_BLUE);
  } else if (cmd == "R" || cmd == "RED" || cmd == "OFF" || cmd == "SHUTDOWN") {
    hasHeartbeat = false;
    setUartTxDriven(false);
    setState(LedState::RPI_OFF_RED);
  } else if (cmd == "?" || cmd == "STATUS") {
    if (!isGpioUart || uartTxEnabled) {
      replyStream.printf("[ESP32S3_LED] STATUS=%s HEARTBEAT=%d AGE_MS=%lu UPTIME_MS=%lu\n",
                         stateName(currentState),
                         hasHeartbeat ? 1 : 0,
                         hasHeartbeat ? (now - lastHeartbeatMs) : 0UL,
                         now);
    }
  }
}

void setup() {
  // Do not block if USB cable is unplugged (ARDUINO_USB_CDC_ON_BOOT=1)
  Serial.setTxTimeoutMs(0);
  Serial.begin(UART_BAUD);

  // Initialize GPIO17 in high-Z INPUT until RPi5 proves it is powered by sending a heartbeat on GPIO18
  pinMode(UART_TX_PIN, INPUT);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, -1);
  uartTxEnabled = false;

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
  static String usbRxBuf;
  static String uartRxBuf;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbRxBuf.length() > 0) {
        handleCommandLine(usbRxBuf, Serial, false);
        usbRxBuf = "";
      }
    } else if (usbRxBuf.length() < 64) {
      usbRxBuf += c;
    } else {
      usbRxBuf = "";
    }
  }

  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (uartRxBuf.length() > 0) {
        handleCommandLine(uartRxBuf, Serial1, true);
        uartRxBuf = "";
      }
    } else if (uartRxBuf.length() < 64) {
      uartRxBuf += c;
    } else {
      uartRxBuf = "";
    }
  }

  unsigned long now = millis();

  // Revert to Solid Red and release GPIO17 TX to high-Z if RPi5 heartbeat stops arriving
  if (hasHeartbeat && (now - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
    hasHeartbeat = false;
    setUartTxDriven(false);
    setState(LedState::RPI_OFF_RED);
  }

  // Periodic refresh every 250ms to guard against any power-glitch pixel reset
  if (now - lastRefreshMs >= 250) {
    lastRefreshMs = now;
    applyLedColor(currentState);
  }

  delay(5);
}
