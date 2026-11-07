# Rover 1 — Maker-ESP32-Pro Motor Firmware, ESP32-S3 ToF Bench & Exact-Motion Test Suite (`esp-maker-usba-4motor`)

Authoritative firmware and physical-test repository for **Rover 1** (4-motor differential-drive wheel rover).

For full operating rules, chronological test ledgers, current session handoff, and open tasks, see:
- [`AGENTS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/AGENTS.md) — Hardware reference, safety boundaries, and agent workflow rules.
- [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) — Active firmware build, roadmap, reusable test catalog, test ledger, and Section 6 Daily Engineering Log.
- [`HANDOFF.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/HANDOFF.md) & [`TODO.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/TODO.md) — Current session handoff state and prioritized next steps.
- [`docs/EXACT_MOTION_CONTRACT.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/EXACT_MOTION_CONTRACT.md) & [`docs/REUSABLE_PHYSICAL_TEST_FRAMEWORK.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/REUSABLE_PHYSICAL_TEST_FRAMEWORK.md) — Canonical closed-loop motion controllers and CLI harness.

---

## 1. Two-Repository Architecture

| Repository | Local Path | Deployment Target | Role & Ownership |
| :--- | :--- | :--- | :--- |
| **`esp-maker-usba-4motor`** *(This Repo)* | `c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor` | **Maker-ESP32-Pro** (`/dev/rover-esp32`) & **ESP32-S3 N16R8** (`/dev/ttyAMA0` / OTA) | 1. Maker-ESP32-Pro 4-motor closed-loop C++ firmware (`src/`, `env:esp32dev`).<br>2. Lonely Binary ESP32-S3 status LED + 8-channel `TCA9548A` ToF sensor bench (`esp32s3_status_led/`).<br>3. Reusable closed-loop physical test suite (`tools/rover_tests/`). |
| **`yahboom-encoder`** *(Companion Host Repo)* | `c:\Users\Ron\electronic_projects\yahboom-encoder` | **Raspberry Pi 5** (`ron@10.0.0.246:/home/ron/yahboom-encoder`) | Legacy repo name (the old Yahboom driver board is **not** used). Hosts Cockpit Server (`:3000`), Rover Ground Control HUD (`public/hud.html`), LiDAR sidecar (`:3002`), and ROS 2 Jazzy / Nav2 Docker stack (`ros2/`). |

### System Data & Control Flow
```text
Maker-ESP32-Pro (1.3.4-remap-r03)  <── USB Serial v1.1 (/dev/rover-esp32, 115200) ──>  rover-server.service (:3000)
ESP32-S3 + TCA9548A ToF (1.2.4)    <── GPIO UART0 (/dev/ttyAMA0, 115200) ───────────>  esp32s3-status-led.service -> /tmp/esp32s3_tof_state.json
RPLiDAR C1                         <── USB Serial (/dev/rover-lidar, 460800) ───────>  rover-lidar.service (:3002)
Geekworm X1206 UPS (MAX17043)      <── RPi5 I2C-1 (0x36) ───────────────────────────>  geekworm_battery.py -> rover-server.service (:3000)
                                                                                              │
                                                                                              ▼
                                                                         Docker: rover-ros2 (ROS 2 Jazzy + Nav2)
                                                                         ├── rover_encoder_odometry (/odom, TF odom->base_link)
                                                                         ├── rover_imu_bridge       (/imu/data, /imu/game_rotation_vector)
                                                                         ├── rover_lidar_bridge     (/scan)
                                                                         ├── rover_tof_bridge       (/tof/front_left/*, /tof/front_right/*)
                                                                         ├── rover_cmd_vel_bridge   (/cmd_vel -> POST /api/cmd_vel)
                                                                         └── rover_nav_bridge       (:3010 Nav2 + AMCL + Guarded HOME init)
```

---

## 2. Hardware & Pin Mapping (Verified on Rebuilt Chassis, 2026-10-18)

### A. Primary Motor Controller — NULLLAB Maker-ESP32-Pro (`ESP32-WROOM-32E`)
- **Active Verified Firmware**: **`1.3.4-remap-r03`** (`source: rev42-remap-c`, commit `a2e3d43`).
- **Serial Interface**: Binary framed USB serial protocol `v1.1` ([`src/SerialProtocol.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/SerialProtocol.cpp)) at `115200` baud over CH340 (`1a86:7523`, `/dev/rover-esp32` → `/dev/ttyUSB0`). *(Note: Wi-Fi and HTTP are not used in active firmware.)*
- **Power Isolation**: Logic powered via RPi5 USB from the **Geekworm X1206 UPS**; motor driver `VMOT` powered by a dedicated **3S LiPo** via the Maker board's physical power switch.
- **Hardware DIP Switches**: M3/M4 switches (`GPIOs 12, 14, 15, 17`) must be set to **Motor** (not IO).
- **Motor & Quadrature Encoder Remap** ([`src/MotorDriver.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/MotorDriver.cpp), [`src/EncoderManager.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/EncoderManager.cpp)):

| Logical Wheel Index | Telemetry Key | Rover Corner | Physical Board Motor Port | Motor Pins & Polarity | Physical Encoder Port | Encoder Pins & Sign |
| :---: | :---: | :--- | :---: | :--- | :---: | :--- |
| **`0`** | `m1` | **Left Front (LF)** | **Port M2** | `IN1=4, IN2=2` (`invert=false`) | **E2** | `A=5, B=23` (`+`) |
| **`1`** | `m2` | **Right Front (RF)** | **Port M1** | `IN1=27, IN2=13` (`invert=true`) | **E1** | `A=18, B=19` (`-`) |
| **`2`** | `m3` | **Left Rear (LR)** | **Port M4** | `IN1=14, IN2=15` (`invert=true`) | **E4** | `B=39, A=34` (`-`) |
| **`3`** | `m4` | **Right Rear (RR)** | **Port M3** | `IN1=17, IN2=12` (`invert=true`) | **E3** | `A=35, B=36` (`-`) |

- **Onboard Peripherals**:
  - **IMU**: Adafruit **BNO08x** on I2C (`SDA=GPIO 21, SCL=GPIO 22`, addr `0x4B`), running `SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC` (`0x08`) at `50 Hz` ([`src/ImuManager.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/ImuManager.cpp)).
  - **NeoPixels**: 4x addressable RGB LEDs on `GPIO 16` ([`src/main.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/main.cpp)).
- **Calibrated Kinematics**:
  - Effective wheel diameter: **`0.06695 m`** (radius `0.033475 m`; firmware nominal `0.065 m`), `1974.1667` ticks/rev.
  - Effective differential track width: **`0.3408575433 m`** (physical separation `0.197 m`).
  - Calibrated stopping advances: **`0.0135 m` (`13.5 mm`)** linear (`LinearApproachController`), **`0.7°`** angular (`AngularApproachController`).

### B. Auxiliary Sensor & Status LED Controller — Lonely Binary ESP32-S3 N16R8 (`esp32s3_status_led/`)
- **Active Firmware**: **`1.2.4-tof-mux`** (`1.2.3-tof-mux` currently running on hardware; `1.2.4-tof-mux` compiled in repo for next `--ota` flash). See [`esp32s3_status_led/README.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/README.md).
- **Primary Data Connection**: RPi5 40-pin header GPIO UART0 (`/dev/ttyAMA0`, `115200` baud: `RPi5 GPIO 14 TXD0 -> ESP32-S3 GPIO 18 RX`, `RPi5 GPIO 15 RXD0 <- ESP32-S3 GPIO 17 TX` with high-Z unpowered-Pi backfeed protection).
- **I2C Multiplexer (`TCA9548A`, addr `0x70`)**: `SDA=GPIO 8, SCL=GPIO 9` at `200 kHz` (`150 ms` timeout):
  - **`CH 0` (`front_right_l7cx`)**: Front-Right **`VL53L7CX`** 8×8 ToF (`0x29`, angled $-8^\circ$ outward, `rot="90_ccw"`).
  - **`CH 1` (`front_left_l7cx`)**: Front-Left **`VL53L7CX`** 8×8 ToF (`0x29`, angled $+8^\circ$ outward, `rot="90_ccw"`).
  - **`CH 2` (`unassigned`)**: Single-zone **`VL53L4CD`** (`0x29`, monitored in `/api/tof-sensors` and `/sensors.html`; excluded from ROS 2 and HUD radar until mounted/mapped).

---

## 3. Build, Test & Deployment Commands

### A. Local Build & Software Verification
```powershell
# 1. Compile Maker-ESP32-Pro Motor Firmware (env:esp32dev)
pio run -e esp32dev

# 2. Compile ESP32-S3 Status LED + ToF Multiplexer Firmware (env:esp32-s3-devkitc-1)
pio run -d esp32s3_status_led

# 3. Run Native C++ Controller Regression Suite (23 Canonical Tests: 1-17, 19-24)
python scripts/run_native_suite.py

# 4. Run Python Unit Tests (Non-Motion)
pytest test/test_rover_test_linear.py test/test_rover_test_turn.py
```

### B. Remote Deployment to Rover 1 (`ron@10.0.0.246`)
```powershell
# Pre-flash check / dry-run for Maker-ESP32-Pro (no flash write)
python deploy_firmware.py --check-only
python deploy_firmware.py --dry-run

# Flash Maker-ESP32-Pro motor firmware (requires explicit operator authorization)
python deploy_firmware.py

# Wireless OTA update for ESP32-S3 ToF/Status LED firmware (USB unplugged)
python esp32s3_status_led/deploy_esp32s3.py --ota
```
