# Rover 1 Agent Instructions (`AGENTS.md`)

Authoritative operating instructions, hardware reference, workflow rules, and safety boundaries for AI coding agents working in the **Rover 1** firmware and test repository (`esp-maker-usba-4motor`).

---

## 1. Project Identity & Rover 1 Build

- **Rover 1 Only**: This repository (`esp-maker-usba-4motor`) is exclusively for **Rover 1** (4-motor differential-drive rover). Keep **Rover 2** hardware, IP addresses, and configurations strictly separate.
- **Authoritative References** (consult and maintain rather than duplicating tables):
  - [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) — Active firmware build, roadmap, reusable test catalog, chronological test ledger, and **Section 6 Daily Engineering & Activity Log**.
  - [`docs/HANDOFF_REBUILT_CHASSIS_CALIBRATION_20261018.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/HANDOFF_REBUILT_CHASSIS_CALIBRATION_20261018.md) — Rebuilt chassis port remap, UPS power isolation validation, and exact-motion calibration baselines.
  - [`docs/EXACT_MOTION_CONTRACT.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/EXACT_MOTION_CONTRACT.md) — Mandatory controllers and motion profiles for physical movement.
  - [`docs/REUSABLE_PHYSICAL_TEST_FRAMEWORK.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/REUSABLE_PHYSICAL_TEST_FRAMEWORK.md) & [`docs/GOLD_STANDARD_HOME_TEST.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/GOLD_STANDARD_HOME_TEST.md) — Canonical test harnesses.

### Two-Repository Architecture & Host Connections
| Component | Path / Target | Role & Ownership |
| :--- | :--- | :--- |
| **Firmware Repo (This Repo)** | `c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor` | PlatformIO C++ firmware for NULLLAB Maker-ESP32-Pro (`env:esp32dev`) + `tools/rover_tests` exact-motion test suite. |
| **Companion Host Repo** | `c:\Users\Ron\electronic_projects\yahboom-encoder` → `/home/ron/yahboom-encoder` on RPi5 | Legacy repo name (Yahboom driver board is **not** used). Hosts Cockpit API (`:3000`), Nav Bridge (`:3010`), LiDAR sidecar, and ROS 2 Jazzy Docker stack. |
| **Raspberry Pi 5 Host** | `ron@10.0.0.246` (SSH key auth) | Primary onboard compute. `rover-server.service` exclusively owns `/dev/rover-esp32`; `rover-lidar.service` exclusively owns `/dev/rover-lidar`. |

### Installed Hardware (Verified)
- **Power Architecture**:
  - **Compute & Logic Rail**: **Geekworm X1206 UPS** (1S4P 21700 lithium cells mounted beneath RPi5; MAX17043 fuel gauge on RPi5 I2C `0x36`) powers the Raspberry Pi 5 and USB-connected ESP32 boards, isolating logic from motor load sag.
  - **Motor Rail**: Dedicated **3S LiPo** pack supplies motor power (`VMOT`) on the Maker-ESP32-Pro (controlled by the board's physical power switch).
- **Primary Motor Controller**: **NULLLAB Maker-ESP32-Pro** (`ESP32-WROOM-32E`), connected to RPi5 via USB CH340 (`1a86:7523`, `/dev/rover-esp32` → `/dev/ttyUSB0`, `115200` baud).
  - **Active Verified Firmware**: **`1.3.4-remap-r03`** (`source: rev42-remap-c`, commit `a2e3d43`).
  - **Hardware Switches**: Maker board DIP switches for M3/M4 (`GPIOs 12, 14, 15, 17`) must be set to **Motor** (not IO).
  - **Onboard NeoPixels**: 4x addressable RGB LEDs on `GPIO 16` ([`src/main.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/main.cpp)).
- **Motor & Quadrature Encoder Mapping (Rebuilt Chassis, 2026-10-18)**:
  - All 4 motors are mounted mirrored relative to the original chassis; physical wiring is intentionally left as-built and remapped in [`src/MotorDriver.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/MotorDriver.cpp) and [`src/EncoderManager.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/EncoderManager.cpp):
    - **Logical 0 — Left Front (LF)**: Motor Port **M2** (`IN1=4, IN2=2`, `invert=false`) | Encoder **E2** (`A=5, B=23`, sign `+`)
    - **Logical 1 — Right Front (RF)**: Motor Port **M1** (`IN1=27, IN2=13`, `invert=true`) | Encoder **E1** (`A=18, B=19`, sign `-`)
    - **Logical 2 — Left Rear (LR)**: Motor Port **M4** (`IN1=14, IN2=15`, `invert=true`) | Encoder **E4** (`B=39, A=34`, sign `-`)
    - **Logical 3 — Right Rear (RR)**: Motor Port **M3** (`IN1=17, IN2=12`, `invert=true`) | Encoder **E3** (`A=35, B=36`, sign `-`)
  - **Calibrated Kinematics**: Effective wheel diameter `0.06695 m` (radius `0.033475 m`; firmware nominal `0.065 m`), `1974.1667` ticks/rev, effective differential track width `0.3408575433 m` (physical separation `0.197 m`).
- **Sensors & RPi5 Peripherals**:
  - **IMU**: Adafruit **BNO08x** on Maker-ESP32-Pro I2C (`SDA=GPIO 21, SCL=GPIO 22`, addr `0x4B`), running `SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC` (`0x08`) at 50 Hz. *Note*: Maker board peripheral power switch must be ON at ESP32 boot for `ImuManager::begin()` to initialize.
  - **LiDAR**: **RPLiDAR C1** (CP2102N `10c4:ea60`, `/dev/rover-lidar`, owned by `rover-lidar.service`). *(Unverified / disconnected during 2026-10-24 USB check — verify physical USB connection prior to LiDAR/Nav2 runs.)*
  - **RPi5 PCIe / AI**: ASM1182e PCIe switch with **WD_BLACK SN7100 NVMe SSD** (`0001:03:00.0`) and **Hailo-8 M.2 AI Processor** (`0001:04:00.0`, `/dev/hailo0`, managed in `D3hot` / `ASPM L1` idle power save by `hailo-powersave.service`).
  - **Camera**: FPV camera stream via Cockpit (`/api/camera`; exact camera hardware/mount details unverified in this repo).

### Additional Auxiliary Board — Lonely Binary ESP32-S3 N16R8
- **Current Physical Installation**:
  - Seated in its supplied **terminal breakout board** (treat the breakout board as the current installation; replacing it is a future task).
  - Powered directly by 5V from the **Geekworm X1206 UPS** (`JST 5V`), sharing common ground with the RPi5.
  - **Primary Data Connection (GPIO UART0)**: Wired between the RPi5 40-pin header and the ESP32-S3 breakout terminals at `115200` baud (`/dev/ttyAMA0` enabled via `dtoverlay=uart0-pi5`, serial login console disabled):
    - **RPi5 `GPIO 14 / TXD0` (Pin 8)** $\to$ **ESP32-S3 `GPIO 18 / RX`**
    - **RPi5 `GPIO 15 / RXD0` (Pin 10)** $\leftarrow$ **ESP32-S3 `GPIO 17 / TX`** *(kept in high-Z `INPUT` whenever RPi5 heartbeat is absent so the always-powered ESP32-S3 never backfeeds an unpowered RPi5)*.
  - **I2C Multiplexer (`TCA9548A` / `PCA9548A`, addr `0x70`)**: Wired to the ESP32-S3 terminal breakout (`VIN -> 3V3`, `GND -> GND`, `SDA -> GPIO 8`, `SCL -> GPIO 9` at `400 kHz`; `RST` and `A0–A2` unconnected) for up to 8 downstream ST Time-of-Flight (`0x29`) sensors (`2 × VL53L7CX` wide-field 8×8 + `4 × VL53L4CD` downward cliff + `2` spare channels; `CH 0` and `CH 1` currently host the two `VL53L7CX` 8×8 sensors, live at `http://10.0.0.246:3000/sensors.html`).
  - **Wireless OTA Updates (Station Mode `WIFI_STA`)**: Associates non-blockingly with home Wi-Fi (`rover1-esp32s3`, port `3232`, `default_16MB.csv` dual `6.25 MB` `app0`/`app1` slots) for authenticated `ArduinoOTA` updates (`python esp32s3_status_led/deploy_esp32s3.py --ota`) with USB unplugged; see [`esp32s3_status_led/README.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/README.md).
  - **Optional Flashing/Debug Connection**: Native Espressif USB JTAG/serial debug port labeled **USB** (`303a:1001`, MAC `58:e6:c5:72:3f:fc`, `16MB` Quad Flash / `8MB` PSRAM, `/dev/ttyACM0`); unplugged during normal operation and used only for `--usb` recovery.
- **Critical Separation Rule**:
  - **Never confuse the Lonely Binary ESP32-S3 N16R8 (`/dev/ttyAMA0` or `/dev/ttyACM0`, `303a:1001`) with the Maker-ESP32-Pro motor controller (`/dev/rover-esp32` → `/dev/ttyUSB0`, `1a86:7523`).**
  - **Do not modify or flash the Maker-ESP32-Pro** when experimenting with the ESP32-S3.
- **Active ESP32-S3 Firmware & Host Service** (`esp32s3_status_led/`):
  - Isolated PlatformIO project in [`esp32s3_status_led/`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/platformio.ini) (`env:esp32-s3-devkitc-1`), deployed via [`esp32s3_status_led/deploy_esp32s3.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/deploy_esp32s3.py) with RPi5 systemd service `esp32s3-status-led.service` ([`esp32s3_status_led/rpi5_status_led_daemon.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/rpi5_status_led_daemon.py)).
  - Drives the onboard RGB LED (`NEO_GRB`, verified working over both GPIO UART0 `/dev/ttyAMA0` and USB on `2026-10-25`): **solid green** when the RPi5 is on (stationary/idle), **solid blue** when the rover is moving, and **solid red** when the RPi5 is off/halted or heartbeats stop (`> 1.5 s`).

---

## 2. Starting Work & Branch Discipline

1. **Inspect Git State First**: Before starting any new task, run `git status` and identify the current branch and recent commits.
2. **Report Uncommitted Work**: If there are uncommitted or untracked changes, tell Ron what they are and whether they appear related to the new task.
3. **Stop & Ask When Dirty**: Before starting a different task or creating/switching branches with uncommitted work, ask whether to finish and commit the current work or knowingly set it aside.
4. **No Silent Git Actions**: Never silently stash, discard, overwrite, move, or commit existing work.
5. **Branch Per Task**:
   - Once the working state is resolved, use a descriptive task branch (e.g., `docs/...`, `feat/...`, `fix/...`).
   - Continue an appropriate existing task branch when resuming that same task; do not create a new branch for every prompt.
   - Do not implement changes directly on `main` (the default branch).
6. **Git Checkpoint Rule (Commit Before Changes)**:
   - Always ensure the current working baseline is committed before making firmware, controller, or configuration edits so regressions can be cleanly reverted (`git revert` / `git checkout`).
7. **Preserve Concurrent Edits**: Preserve changes made by Ron or another agent. If overlapping edits are detected, stop editing the affected files immediately and explain the conflict.
8. **Restricted Operations**: Do not use destructive Git commands (`git reset --hard`, `git clean -fd`, `git checkout -- .`), force-push, merge to `main`, or flash firmware without explicit authorization from Ron.
9. **Secret Hygiene**: Never commit `.env` files, credentials, operator tokens, private keys, or sensitive diagnostic output.

---

## 3. Build, Deployment & Verification

Do not invent build or test commands. Use only the verified commands below and separate local compilation/testing from RPi5 deployment and physical execution.

### A. Local Build & Software Verification (Windows / WSL)
- **Compile Maker-ESP32-Pro Firmware**:
  ```powershell
  pio run -e esp32dev
  ```
  *(Target defined in [`platformio.ini`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/platformio.ini): `platform = espressif32@6.7.0`, `board = esp32dev`, `framework = arduino`. Note: [`README.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/README.md) contains legacy Wi-Fi/HTTP notes; active firmware [`src/main.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/main.cpp) uses binary USB serial protocol `v1.1`. No separate linter is configured.)*
- **Run Python Unit Tests (Non-Motion)**:
  ```powershell
  pytest
  ```
- **Run Native C++ Controller Regression Suite (23 Canonical Tests: 1–17, 19–24)**:
  ```powershell
  python scripts/run_native_suite.py
  ```
  *(Recompile native test binaries in WSL/Linux when C++ control code changes via `bash test/build_and_run_native_tests.sh`.)*

### B. Remote Deployment & Flashing (Through Raspberry Pi 5 `10.0.0.246`)
- **Pre-Flash Validation / Dry-Run (No Flash Write)**:
  ```powershell
  python deploy_firmware.py --check-only
  python deploy_firmware.py --dry-run
  ```
- **Flash Maker-ESP32-Pro Firmware (Requires Explicit Operator Authorization)**:
  ```powershell
  python deploy_firmware.py
  ```
  *(Compiles locally, verifies CH340 `1a86:7523` on `/dev/rover-esp32`, checks `EXPECTED_FIRMWARE_SHA256`, stops `rover-server.service`, backs up 4 MB flash via `esptool`, flashes, restarts `rover-server.service`, and verifies telemetry.)*
- **Deploy Companion Cockpit / Host Files to RPi5**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\yahboom-encoder
  $env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py
  ```

### C. Canonical Physical & Acceptance Tests (Require Explicit Operator Approval)
- **Suite A — Low-Speed Floor & Stiction Characterization** (on RPi5):
  ```bash
  cd /home/ron/yahboom-encoder && python3 scripts/characterize_floor_speeds.py --case <CASE_NUM>
  ```
- **Suite B — Closed-Loop Exact Motion (`tools/rover_tests`)**:
  ```powershell
  # Dry-run (no motion)
  python -m tools.rover_tests.cli turn --degrees 90 --direction cw --dry-run
  # Physical turns, linear translations, and compound out-and-back
  python -m tools.rover_tests.cli turn --degrees 90 --direction cw --trials 1 --inter-trial-approval
  python -m tools.rover_tests.cli linear --distance 1.0 --direction forward --trials 1 --inter-trial-approval
  python -m tools.rover_tests.cli out-and-back --distance 1.0 [--final-turn]
  ```
- **Suite C — Gold Standard Autonomous HOME Test** (on RPi5):
  ```bash
  ./tools/run_gold_standard_home_test.sh --dry-run
  ./tools/run_gold_standard_home_test.sh --test-safety
  ./tools/run_gold_standard_home_test.sh --execute
  ```
- **Suite D — Raised-Wheel Single-Motor Maintenance Check** (on RPi5, wheels clear of floor):
  ```bash
  curl -s -X POST http://127.0.0.1:3000/api/maintenance/run_test -H 'Content-Type: application/json' \
    -d '{"safetyAck":true,"motorIndex":0,"direction":"forward","output":60,"durationSec":2.0}'
  ```

### Verification Discipline
- Run checks proportionate to the changes made; do not repeat unrelated physical calibration or broaden testing without a specific reason.
- Report accurately what passed, what failed, and what could not be verified.

---

## 4. Working Style

- **Read Before Editing**: Inspect [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) and the relevant source/test files before proposing or making changes.
- **Focused Scope**: Make minimal, targeted changes that directly address the requested task. Avoid unrelated refactoring, speculative features, and unnecessary dependencies.
- **Protect Validated Calibrations**: Preserve known-good calibration constants (e.g., `1.3.4-remap-r03` port mapping, `102 PWM` spin breakout, `41.0 PWM` reverse crawl floor, `0.06695 m` effective wheel diameter, `0.3408575433 m` track width, `0.0135 m` linear stopping advance, `0.7°` angular stopping advance) unless the task explicitly requires changing them.
- **Plain-Language Explanations**: Explain significant technical findings and design decisions clearly and concisely.
- **Keep Status Docs Current**: Update [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) (including the Test Ledger and Section 6 Daily Engineering Log) and handoff documentation whenever behavior, hardware, test results, or operating procedures change.

---

## 5. Rover Safety & Exact Motion Contract

1. **Read-Only Hardware Diagnosis First**: Begin hardware or connectivity troubleshooting with non-intrusive inspection (`lsusb`, `udevadm`, `systemctl status`, `journalctl`, `/api/status`).
2. **Identify Serial Devices Before Opening**: Always verify USB VID:PID and device symlinks before opening a serial port—opening a serial device (DTR/RTS toggle) can reset an ESP32 controller mid-operation.
3. **Explicit Authorization Gate**: Never command physical motion, arm motors, flash firmware, or alter power/control wiring without Ron's explicit approval.
4. **Separate Operator vs. Agent Actions**: Clearly distinguish steps the agent performs in software from physical actions Ron must perform (flipping motor power switches, raising wheels, measuring floor marks).
5. **Never Bypass Safety Gates**: Never disable or bypass emergency stops (`EMERGENCY_STOP`), deadman timers, watchdogs (`300 ms` firmware / `500 ms` host), stall protection, non-magnetic IMU checks, or the 3-zero arming handshake.
6. **Exact Motion Contract** ([`docs/EXACT_MOTION_CONTRACT.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/EXACT_MOTION_CONTRACT.md)):
   - Never write ad-hoc one-off physical motion scripts.
   - **Exact Linear Distance**: Must use `LinearApproachController` ($0.20\text{ m/s}$ cruise $\to 0.05\text{ m/s}$ creep at $0.15\text{ m}$, stopping advance $0.0135\text{ m}$, effective wheel diameter $0.06695\text{ m}$). Never cut directly from full speed.
   - **Exact In-Place Turns**: Must use `AngularApproachController` ($0.80\text{ rad/s}$ cruise $\to 0.20\text{ rad/s}$ creep at $30^\circ$, stopping advance $0.7^\circ$, `SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC`). Never use magnetic orientation indoors.
   - **Terminal Safe State**: Every trial, script, and abort path must leave the rover **stopped, disarmed (`armed: false`), and locked (`Mode 0 / LOCKED`)**.
7. **Truthful Acceptance Reporting**: Never claim physical acceptance based solely on software unit tests, simulations, or dry runs.

---

## 6. Completing Work

- **Completion Summary**: Summarize what changed, why it changed, which verification checks were run (and their exact results), and any unresolved issues or risks.
- **Git State Report**: Report the active branch name and whether any modified or untracked files remain uncommitted.
- **Authorized, Scoped Commits**: When authorized to commit, stage and commit only files pertinent to the task with a clear commit message.
- **Outstanding Verification**: Do not declare a task complete if required build, test, or physical verification is still pending—state the exact remaining step clearly.
