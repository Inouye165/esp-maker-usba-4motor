# ESP Maker Actuator Controller (esp-maker-usba-4motor)

This repository contains the embedded C++ firmware (PlatformIO) for the main actuator controller board of the Robot Tank. Based on the **NULLLAB Maker-ESP32 / Maker-ESP32-Pro** (Espressif ESP32-WROOM-32E module), it drives 4 DC motors/tracks, hosts an on-board SSD1306 OLED screen for status displays, manages addressable NeoPixel LEDs, and runs an HTTP server alongside a UDP autodiscovery beacon.

---

## 📐 Project Architecture & Two-Repository System

The rover operates on a **two-repository architecture**:

| Repository | Local Path | Deployment Target | Role & Purpose |
| :--- | :--- | :--- | :--- |
| **`esp-maker-usba-4motor`** *(This Repo)* | `C:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor` | Maker USB32 Pro / Maker-ESP32 Pro Board | Embedded C++ PlatformIO firmware: motor control, encoder acquisition, closed-loop speed control, serial protocol, watchdog, arming, e-stop, and low-level safety. |
| **`yahboom-encoder`** | `C:\Users\Ron\electronic_projects\yahboom-encoder` | Raspberry Pi 5 (`/home/ron/yahboom-encoder`) | Host software: Cockpit API, LiDAR sidecar, ROS 2 Jazzy Docker stack, encoder odometry, TF transforms, Foxglove bridge, diagnostics, future SLAM/Nav2. |

> [!IMPORTANT]
> **Legacy Repository Naming Clarification**:
> `yahboom-encoder` is a legacy repository and folder name. The rover **does not use the old Yahboom motor-driver board**. The active motor controller is the **Maker USB32 Pro / Maker-ESP32 Pro**. The repository name and RPi path are intentionally left unchanged until the rover is fully operational.

### System Data Flow
```
Maker ESP32 firmware
 → USB serial (/dev/rover-esp32)
 → Raspberry Pi rover-server.service
 → read-only host APIs (http://127.0.0.1:3000)
 → ROS 2 Docker nodes
 → /odom, /scan, /tf and diagnostics
 → Foxglove now, SLAM/Nav2 later
```

### Hardware Ownership & Safety Boundaries
- **`rover-server.service`**: Exclusively owns `/dev/rover-esp32` (serial telemetry & commands).
- **`rover-lidar.service`**: Exclusively owns `/dev/rover-lidar` (LiDAR telemetry).
- **ROS 2 Docker Container**: Has **no `/dev` mounts** and reads read-only host APIs.
- **Foxglove**: Visualization only.
- **`/cmd_vel`**: Not currently enabled in ROS 2.
- **Arming Safety**: The rover must remain **disarmed** during software-only development and testing.

---

The board acts as the central physical controller in the tank cockpit ecosystem:
- **UDP Beacons**: Automatically broadcasts discovery packets to the Node.js proxy server on UDP port `3000` every 2 seconds.
- **HTTP REST API**: Exposes endpoints for real-time motor actuation and LED configuration.
- **Embedded Web Dashboard**: Hosts a direct, browser-based user control interface as a fallback/alternative UI directly on port `80`.

---

## Technical Specifications

| Component / Feature | Pin Assignment / Details |
| ------------------- | ---------------------------------------------------- |
| **Core Module**     | ESP32-WROOM-32E (2.4 GHz Wi-Fi client only)         |
| **DC Motor 1 (M1)** | IN1: `GPIO 27`, IN2: `GPIO 13` (Left Track Front)    |
| **DC Motor 2 (M2)** | IN1: `GPIO 4`, IN2: `GPIO 2` (Right Track Front)     |
| **DC Motor 3 (M3)** | IN1: `GPIO 17`, IN2: `GPIO 12` (Left Track Rear)*    |
| **DC Motor 4 (M4)** | IN1: `GPIO 14`, IN2: `GPIO 15` (Right Track Rear)*   |
| **NeoPixel RGB**    | `GPIO 16` (Controls 4 on-board NeoPixels)           |
| **OLED Display**    | SSD1306 via I2C (SDA/SCL)                            |

*\*Note: For M3 and M4 (GPIOs 12, 14, 15, 17) to function as motors, the hardware switches on the Maker Board must be toggled to the **Motor** position rather than the **IO** position.*

---

## Features & Safety Controls

1. **UDP Auto-Discovery**: Periodically transmits JSON discovery packets payload (IP, SSID, RSSI, motor telemetry, and LED status) to UDP port `3000` to allow the Cockpit backend to find it automatically.
2. **WiFi Fallback Scheme**: Connects as a client to `Pumpkinpie` (primary) or `Dobby` (secondary fallback) with AP mode disabled for safety.
3. **SSID Lockout**: Refuses API commands with `HTTP 403 Forbidden` if it detects it has connected to an unverified Wi-Fi network.
4. **OLED Telemetry HUD**: Displays local connection status, WiFi SSID, IP address, RSSI, and active statuses.
5. **Safe Motor Direction Transitions**: When switching direction from forward to reverse (or vice versa), the controller halts the motor and inserts a 50ms safety deadtime to protect the driver chip from high current spikes.
6. **Safety Speed Caps**: Enforces speed limits (default 75% duty cycle, up to 100%) and features automatic safety timeouts that turn off the motors if control commands stop arriving.

---

## REST API Reference

All requests return JSON responses.

### 1. Drive Controller
- **Route**: `GET /api/drive?x=[float]&y=[float]`
- **Description**: Standard differential steering calculation. Maps joystick coordinates `x` (-1.0 to 1.0) and `y` (-1.0 to 1.0) to left tracks (M1, M3) and right tracks (M2, M4) based on current safety speed limits.

### 2. Individual Motor Speed
- **Route**: `GET /api/motor?index=[0-3]&speed=[-255..255]`
- **Description**: Sets the raw speed of a specific motor directly (index 0=M1, 1=M2, 2=M3, 3=M4).

### 3. Safe Motor Verification Test
- **Route**: `GET /api/test_motor?motor=[0-3|left|right|both]&dir=[forward|reverse]&pwm=[0-80]&duration=[ms]`
- **Description**: Used by the cockpit's pre-drive safety checklist. Sends a timed test pulse. For safety, PWM is strictly capped at `80` (low speed) and duration is capped at `1500ms`.

### 4. Adjust Speed Limits
- **Route**: `GET /api/speed?val=[40-255]`
- **Description**: Changes the safety speed limit (PWM duty cycle limit) used by the drive controller.

### 5. Emergency Stop
- **Route**: `GET /api/stop`
- **Description**: Instantly cuts power to all four motors.

### 6. Control NeoPixel LED
- **Route**: `GET /api/led?index=[0-3]&r=[0-255]&g=[0-255]&b=[0-255]` or `GET /api/led?index=[0-3]&hex=[hex_string]`
- **Description**: Controls a single RGB LED. If no `index` is passed, updates all 4 LEDs.

### 7. Batch NeoPixel Colors
- **Route**: `GET /api/leds?colors=[c1],[c2],[c3],[c4]`
- **Description**: Batch configures the colors of all 4 LEDs simultaneously with comma-separated hex codes (e.g. `/api/leds?colors=ff0000,00ff00,0000ff,ffffff`).

### 8. Fetch Telemetry Status
- **Route**: `GET /api/status`
- **Description**: Returns the real-time IP, SSID, RSSI, motor speeds, and LED states.

---

## Flashing & Setup Instructions

### Pre-requisites
1. Install **VS Code** with the **PlatformIO** extension.
2. Create a `.env` file in this directory with your local credentials:
   ```env
   WIFI_SSID=YourWifiName
   WIFI_PASS=YourWifiPassword
   ```

### Uploading to the Board
1. Connect the Maker-ESP32 board via USB (it usually enumerates on `COM18`).
2. Run the PlatformIO build and upload task:
   ```bash
   # Build the project
   pio run
   
   # Upload firmware
   pio run --target upload
   ```
3. Use the serial monitor (baudrate `115200`) to observe connection logs and see the assigned IP address.
