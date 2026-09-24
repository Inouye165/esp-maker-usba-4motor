# Project Rules & Motion Contract

- **Exact Motion Contract**: All physical rover movement, exact distance translation, and turning must strictly adhere to [`docs/EXACT_MOTION_CONTRACT.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/EXACT_MOTION_CONTRACT.md).
  - Use `LinearApproachController` for exact distance ($0.20\text{ m/s} \to 0.05\text{ m/s}$ creep at $0.15\text{ m}$, wheel diameter $0.06695\text{ m}$). Never cut from full speed.
  - Use `AngularApproachController` for exact turns ($0.80\text{ rad/s} \to 0.20\text{ rad/s}$ creep at $30^\circ$, `SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC`). Never use magnetic orientation.
  - Require explicit operator approval from Ron before physical motion.
  - Always finish stopped, disarmed, and locked.

- **Maker ESP32 Flashing**: The Maker ESP32 board is connected directly to the Raspberry Pi 5. It must be compiled/flashed remotely *through* the Raspberry Pi 5 (via USB connection on `/dev/ttyUSB0` or `/dev/ttyACM0`) rather than flashed directly from the Windows PC host.
- **Network Configuration**: The RPi5 sits at `10.0.0.246` on user `ron` (uses SSH key / hash authentication).

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
