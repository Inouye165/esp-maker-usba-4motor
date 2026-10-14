# Project Rules, Repository Reference & Motion Contract

- **Master Project Status & Test Ledger**: Always consult and maintain [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) in the repository root for active firmware build version, roadmap steps, the reusable test catalog, and the chronological test results ledger.
- **Git Checkpoint Rule (Commit Before Changes)**:
  - **ALWAYS** commit the current working baseline (`git commit`) **before** making any firmware, controller, or configuration changes, so that any change that degrades performance can be cleanly reverted (`git revert` / `git checkout`).
  - Record the active git commit hash and firmware version in [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) whenever physical tests are executed.
- **Reusable Tests Only**:
  - Never write ad-hoc one-off physical motion scripts. Always use the parameterized tests listed in [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) (`scripts/characterize_floor_speeds.py --case <N>`, `python -m tools.rover_tests.cli turn|linear`, or `./tools/run_gold_standard_home_test.sh`).
- **Exact Motion Contract**: All physical rover movement, exact distance translation, and turning must strictly adhere to [`docs/EXACT_MOTION_CONTRACT.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/EXACT_MOTION_CONTRACT.md).
  - Use `LinearApproachController` for exact distance ($0.20\text{ m/s} \to 0.05\text{ m/s}$ creep at $0.15\text{ m}$, wheel diameter $0.06695\text{ m}$). Never cut from full speed.
  - Use `AngularApproachController` for exact turns ($0.80\text{ rad/s} \to 0.20\text{ rad/s}$ creep at $30^\circ$, `SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC`). Never use magnetic orientation.
  - Require explicit operator approval from Ron before physical motion.
  - Always finish stopped, disarmed, and locked.

- **Two-Repository Architecture & Network Configuration**:
  - **Firmware Repo**: `esp-maker-usba-4motor` (`c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor`) — NULLLAB Maker-ESP32-Pro firmware (PlatformIO), currently on verified build `1.3.3-cand-r41c`.
  - **Host / Cockpit Repo**: `yahboom-encoder` (`c:\Users\Ron\electronic_projects\yahboom-encoder`, deployed to `/home/ron/yahboom-encoder` on RPi5) — Cockpit API (`:3000`), Nav Bridge (`:3010`), and ROS 2 Jazzy stack.
  - **Raspberry Pi 5 Host**: `10.0.0.246` on user `ron` (SSH key authentication).
  - **Maker ESP32 Flashing**: The Maker ESP32 board is connected directly to the Raspberry Pi 5 (`/dev/ttyUSB0` / `/dev/rover-esp32`). It must be flashed remotely *through* the Raspberry Pi 5 rather than directly from Windows.

### Flashing & Deployment Instructions

#### 1. Flash Maker Board ESP32 (through RPi5)
The ESP32 connected to RPi5's `/dev/ttyUSB0` is flashed remotely from the Windows machine using the local `deploy_firmware.py` script. The script compiles PlatformIO locally, stops the remote cockpit service, copies the binary, flashes it over SSH using `esptool.py` on the Pi, and restarts the cockpit service.
- **Run Command**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor
  python deploy_firmware.py
  ```
- **Credentials**: Loaded automatically from `.env` (`ROVER_PI_HOST="rover"` or `10.0.0.246`).

#### 2. Deploy Cockpit Interface to RPi5
The cockpit server files and public folder pages are deployed using the `deploy_yahboom.py` Python script.
- **Run Command**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\yahboom-encoder
  $env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py
  ```
