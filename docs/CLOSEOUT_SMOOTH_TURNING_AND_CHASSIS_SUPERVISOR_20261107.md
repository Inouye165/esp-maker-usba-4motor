# Rover 1 — Smooth Progressive Manual Turning & Chassis Supervisor Motion Evidence Closeout

**Date**: 2026-11-07  
**Author**: Antigravity & Ron Inouye  
**Target Rover**: Rover 1 (4-motor differential-drive rover)  
**Host Repository (`yahboom-encoder`) Head**: [`caad519`](https://github.com/Inouye165/rover-rpi5-encoder/commit/caad519) (`feat/smooth-progressive-manual-turning`)  
**Firmware Repository (`esp-maker-usba-4motor`) Head**: [`012e641`](https://github.com/Inouye165/esp-maker-usba-4motor/commit/012e641) (`fix/chassis-supervisor-pure-spin-motion-evidence`)  
**Flashed Firmware SHA-256**: `7f808b16df0e6d8ec6e7206b307178453b514ebf67bf22057207c873cc17bba0`  

---

## 1. Executive Summary

This task resolved two distinct issues reported by Ron during manual Xbox controller driving:
1. **Abrupt Turn Initiation Near Stick Center**: Forward and reverse driving felt smooth and progressive, but steering felt discontinuous and turned too fast immediately upon leaving stick center.
2. **Unexpected Motor Shutdown During Sustained Turns**: When holding a steady turn for several seconds, motor output abruptly ceased and the rover stopped turning.

Following an end-to-end trace (gamepad driver $\to$ HUD WebSocket stream $\to$ host velocity shaping & rate limiting $\to$ binary serial protocol $\to$ firmware kinematics $\to$ wheel PID/stiction $\to$ chassis stall supervisor), we recorded synchronized millisecond telemetry on bench blocks, pinpointed the root causes across both repositories, deployed mathematically sound fixes preserving all safety invariants, and proved that sustained turns execute without interruption. Ron subsequently confirmed: **"its working."**

---

## 2. Root Cause Analysis: Why It Was Broken

### A. Host-Side Discontinuous Turn Jump (35% Minimum Speed)
In [`server.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js) (around former lines 2761–2768), linear throttle mapped quadratically ($y^2$), but angular steering implemented a piecewise threshold:
```javascript
// Legacy abrupt turn curve
const MIN_COEFF = 0.35;
let turnScaled = 0;
if (Math.abs(stickX) >= DEADZONE) {
  const norm = (Math.abs(stickX) - DEADZONE) / (1.0 - DEADZONE);
  turnScaled = Math.sign(stickX) * (MIN_COEFF + (1.0 - MIN_COEFF) * norm);
}
```
The moment steering input crossed the deadzone, `turnScaled` stepped instantaneously to **`0.35` (35% of max angular speed)**. Under normal maximums ($3.0\text{ rad/s}$), this commanded an immediate jump to **$1.05\text{ rad/s}$**; under floor-testing limits ($0.90\text{ rad/s}$), it jumped to **$0.315\text{ rad/s}$**.

### B. Compounding Double-Deadband & HUD Throttle Scaling
Steering input suffered from nested deadbands:
1. **Gamepad Deadband**: [`public/hud-gamepad.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-gamepad.js) applied `GP_DEADBAND = 0.12`, remapping raw stick deflection $[0.12, 1.0] \to [0.0, 1.0]$.
2. **HUD Throttle Attenuation**: [`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js) scaled input by the HUD throttle slider ($x_{\text{sent}} = x \cdot \text{throttle}$). If the operator drove at 50% throttle, a $0.15$ stick output became $0.075$ in transit.
3. **Host Server Deadzone**: [`server.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js) evaluated `DEADZONE = 0.10`.
Because `0.075 < 0.10`, the command was discarded as zero. To produce even the smallest turn command on a 100% throttle slider, the physical stick had to exceed $0.12 + 0.10 \times (1 - 0.12) = \mathbf{20.8\%}$ **stick deflection**. This large dead zone encouraged the operator to push the stick harder, at which point the output entered the steep part of the curve.

### C. Firmware Pure-Spin Stiction Compensation
In [`WheelController.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/WheelController.cpp), overcoming horizontal tire scrubbing on high-friction floors during pure spins strictly requires `SPIN_STICTION_BOOST_PWM = 102 PWM` for at least 100 ms, followed by an `80 PWM` kinetic floor. When the operator crossed the 20.8% threshold, the sudden step into `102 PWM` accentuated the feeling of abruptness.

### D. Unavailable IMU Rotation Evidence & Faulty Stall Supervision
1. **Hardware Unresponsiveness**: The Adafruit BNO08x IMU on Maker-ESP32-Pro I2C (`SDA=21, SCL=22`) did not ACK on I2C address `0x4B` or `0x4A` at boot, leaving `imuManager.isInitialized() == false` and gyro yaw rate permanently at $gz = 0.0\text{ rad/s}$.
2. **Supervisor Pure-Spin Stall Gate**: In [`ChassisSupervisor.h`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/ChassisSupervisor.h), pure spin recovery previously strictly required gyro yaw rate ($|gz| \ge 0.04\text{ rad/s}$ and $gz \cdot \text{cmdSign} \ge 0.08\text{ rad/s}$) to evidence chassis motion.
3. **Recorded Failure at +960 ms**: Because $gz$ remained $0.0$, the supervisor concluded the chassis was physically jammed. In baseline recording [`turn_diagnostic_2026-11-07T17-57-02-922Z.json`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/reports/turn_diagnostics/turn_diagnostic_2026-11-07T17-57-02-922Z.json), at **$+960\text{ ms}$** of active turning, `cumulativeNoMotionMs` reached $1,000\text{ ms}$, latched `SPIN_RECOVERY_FAILED_LATCHED`, flagged `spinFailedExhausted=true`, and called `wheelController.forceBlocked()`, **cutting motor PWM to 0**.
4. **No ESP Reboot**: Telemetry confirmed `bootCount: 1`, `resetReason: POWERON_RESET`, and uninterrupted uptime (> 5.2 hours). The microcontroller never rebooted; the shutdown was purely algorithmic firmware stall latching.
5. **Undeployed Initial Host Changes**: During early testing, initial host-side changes were committed on git but the RPi5 `rover-server.service` was still running legacy PID `2cf2162`, so browser reloads did not execute the updated code until the service was pulled and restarted.

---

## 3. What Changed & Technical Implementation

### A. Continuous Progressive Turn Mapping & Angular Slew Rate Limiter
In [`yahboom-encoder/server.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js) (commit `4babde9`):
1. **Continuous Quadratic Shaping (`computeManualJoystickVelocity`)**: Replaced the 35% jump with a continuous quadratic curve starting at $0.0\text{ rad/s}$ immediately after the deadzone:
   $$\text{normalizedStick} = \frac{|\text{stickX}| - \text{DEADZONE}}{1.0 - \text{DEADZONE}}, \quad \text{scaledStick} = (\text{normalizedStick})^2$$
   Small inputs produce micro-turn commands ($< 0.05\text{ rad/s}$), while full stick deflection smoothly reaches maximum velocity ($3.0\text{ rad/s}$).
2. **Host-Side Angular Slew Rate Limiter (`stepManualAngularVelocity`)**: Bounded acceleration to $2.00\text{ rad/s}^2$ and deceleration/reversal to $3.00\text{ rad/s}^2$ in the 20 Hz keepalive loop, guaranteeing that sudden stick deflections ramp progressively.
3. **Immediate Safety Bypass**: E-stop, disarm, deadman release, watchdog expiration, and Front Safety Zone overrides ($30\text{ mm}$ stop / $75\text{ mm}$ crawl) bypass the ramp immediately on tick 1 ($0\text{ ms}$).
4. **Mirroring in HUD**: [`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js) `estimateCommandedVelocity` was updated to mirror the identical continuous quadratic curve.

### B. Scoped Host Fault Recovery to Active Autonomy
In [`server.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js) (commit `18c423a`):
- Line 271 previously disarmed the rover if `spinFailedExhausted` was received while `status.armed == true`.
- Scoped `handleDrivetrainRecoveryFault` strictly to active autonomy modes (`ROS_AUTONOMY`, `autoCalibState.active`), preventing host-side disarms during manual joystick operation.

### C. Sound Two-Tier Chassis Motion Evidence Architecture
In [`esp-maker-usba-4motor/src/ChassisSupervisor.h`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/ChassisSupervisor.h) (commit `951f15b`):
- **Tier 1 (Healthy IMU)**: When `imu.isInitialized() && !imu.inResetRecovery() && imu.isGyroValid()`, gyro yaw rate is **strictly required** ($|gz| \ge 0.04\text{ rad/s}$). If wheels spin while the chassis is wedged against an obstacle or high-centered ($gz = 0$), `ChassisSupervisor` latches `FAILED_LATCHED` at $1,000\text{ ms}$ and cuts PWM, preserving autonomous navigation safety!
- **Tier 2 (Degraded / Bench Fallback)**: When the IMU is uninitialized, offline, or recovering, the supervisor falls back to **opposing track encoder motion** (`movingLeft >= 1 && movingRight >= 1`). This allows bench tests and degraded manual operation to sustain turns without tripping stall timers.

### D. Bounded Non-Blocking IMU Probing & Runtime Retry
In [`src/ImuManager.h`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/ImuManager.h), [`src/ImuManager.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/ImuManager.cpp), and [`src/main.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/main.cpp) (commits `951f15b`, `ed2273e`, `012e641`):
1. **Added `isGyroValid()`**: Validates that the IMU is initialized, out of reset recovery, and gyro sample age is $\le 100\text{ ms}$.
2. **I2C Bus Clear Sequence**: Toggles SCL 16 times in `begin()` to release any slave holding SDA low following an uncoordinated MCU reset.
3. **Fast Non-Blocking Presence Check**: Sets `Wire.setTimeOut(10)` and evaluates `Wire.beginTransmission(addr); Wire.endTransmission() == 0` for both `0x4B` and `0x4A`. If no ACK is received, it immediately skips the heavy `_bno.begin_I2C()` initialization (which otherwise blocks for hundreds of milliseconds and delayed serial processing).
4. **Disarmed Runtime Retry**: Retries initialization non-blockingly every 5 seconds only when disarmed and stationary.

### E. Diagnostic & Regression Tooling
1. **Synchronized Diagnostic Recorder**: [`scripts/record_turn_diagnostic.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/scripts/record_turn_diagnostic.js) captures timestamped joystick commands, serial frames, firmware states, PID metrics, and Front Safety Zone data over WebSocket.
2. **Native C++ Drivetrain Tests**: Added Sub-tests **9N** (sustained pure spin with offline IMU) and **9O** (healthy IMU wheel slip stall shutdown) to [`test/test_wheel_controller_native.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/test/test_wheel_controller_native.cpp).
3. **Host Unit & Integration Suite**: Added [`test/test_manual_turn_smoothing.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/test/test_manual_turn_smoothing.js) and [`test/test_cockpit_recovery_fault.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/test/test_cockpit_recovery_fault.js).

---

## 4. Verification & Empirical Evidence

### A. Test Suite Results by Revision
| Repository | Commit | Test Suite | Scope & Result |
| :--- | :--- | :--- | :--- |
| `esp-maker-usba-4motor` | `951f15b` | Native C++ Suite (`test_patched_native`) | **23/23 PASSED** (Tests 1–17, 19–24, including Sub-tests 9N and 9O). |
| `esp-maker-usba-4motor` | `951f15b` | Python Unit Tests (`pytest`) | **311/311 PASSED** (111.63s). |
| `yahboom-encoder` | `4babde9` | `npm test` (`test_manual_turn_smoothing.js`) | **10/10 unit tests, 8/8 integration tests PASSED**. |
| `yahboom-encoder` | `18c423a` | Cockpit Fault Test (`test_cockpit_recovery_fault.js`) | **All 4 fault-handling scenarios PASSED**. |
| `yahboom-encoder` | `caad519` | Full Host Test Suite (`npm test`) | **All LiDAR, HUD Cockpit, Xbox Gamepad, and Smoothing tests PASSED**. |

### B. Synchronized Diagnostic Run Recordings
1. **Baseline Failure (Pre-Fix)**:
   - File: [`reports/turn_diagnostics/turn_diagnostic_2026-11-07T17-57-02-922Z.json`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/reports/turn_diagnostics/turn_diagnostic_2026-11-07T17-57-02-922Z.json)
   - Result: Steady turn requested $\to$ IMU uninitialized ($gz = 0.0$) $\to$ `spinRecoveryState: FAILED_LATCHED` at $+960\text{ ms}$ $\to$ `forceBlocked()` cut PWM to 0.
2. **Post-Deployment Verification Run 1 (0.40 rad/s for 5.0 s)**:
   - File: [`reports/turn_diagnostics/turn_diagnostic_2026-11-07T18-32-43-854Z.json`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/reports/turn_diagnostics/turn_diagnostic_2026-11-07T18-32-43-854Z.json)
   - Result: At $+0.214\text{s}$, transitioned to `MONITORING_INITIAL`. At $+0.264\text{s}$ (50 ms), opposing encoder motion verified and state returned to `IDLE` with `noMotionBudgetMs: 0`. Completed the full 5.0 seconds with zero stall timeouts, zero output cuts, and zero host disarms.
3. **Post-Deployment Verification Run 2 (0.15 rad/s for 5.0 s)**:
   - File: [`reports/turn_diagnostics/turn_diagnostic_2026-11-07T18-32-52-624Z.json`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/reports/turn_diagnostics/turn_diagnostic_2026-11-07T18-32-52-624Z.json)
   - Result: Continuous slow turn at $0.15\text{ rad/s}$ held for 5.0 seconds with zero stops. Terminal safe state verified: `armed: false`, `mode: 0 (LOCKED)`.

### C. Operator Confirmation
Following deployment and raised-wheel verification, Ron tested manual steering directly and confirmed:
> **"its working."**

---

## 5. Important Remaining Limitations

> [!WARNING]
> **IMU Hardware Remains Offline / Unresponsive on I2C**:
> Telemetry confirms the Adafruit BNO08x sensor on Maker-ESP32-Pro I2C (`SDA=21, SCL=22`) did not respond to I2C bus probes on address `0x4B` or `0x4A` (`hardwareInitialized: false`, `gyroValid: false`).
> 
> The firmware's Tier 2 encoder fallback enables smooth manual driving and bench verification in this degraded state. **However, encoder motion does not prove actual chassis rotation and does NOT establish autonomous navigation acceptance.**
> 
> Before conducting autonomous Nav2 missions or AMCL localization runs requiring rotational dead-reckoning, physical inspection of the BNO08x wiring, power supply, or hardware replacement is required.

---

## 6. Repository State Summary

- **Firmware Repository (`esp-maker-usba-4motor`)**:
  - Branch: `fix/chassis-supervisor-pure-spin-motion-evidence`
  - Head Commit: [`012e641`](https://github.com/Inouye165/esp-maker-usba-4motor/commit/012e641)
  - Remote: Matches `origin/fix/chassis-supervisor-pure-spin-motion-evidence`
  - Flashed & Running on ESP32: Verified SHA-256 `7f808b16df0e6d8ec6e7206b307178453b514ebf67bf22057207c873cc17bba0`
  - Working Tree: Clean (`backups/` git-ignored).
- **Host Repository (`yahboom-encoder`)**:
  - Branch: `feat/smooth-progressive-manual-turning`
  - Head Commit: [`caad519`](https://github.com/Inouye165/rover-rpi5-encoder/commit/caad519)
  - Remote: Matches `origin/feat/smooth-progressive-manual-turning`
  - Running on RPi5: `rover-server.service` active and verified at commit `caad519`.
  - Working Tree: Clean (untracked diagnostic reports preserved).
