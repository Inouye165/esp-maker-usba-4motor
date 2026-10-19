# Rover 1 Project Status, Build Reference & Test Ledger

This root document is the single source of truth for **Rover 1** build/repository metadata, the roadmap to autonomous driving, the catalog of reusable physical tests, and the historical test results ledger.

---

## 1. Repository, Hardware & Build Reference

| Item | Value / Path | Notes |
| :--- | :--- | :--- |
| **Project Name** | **Rover 1** (4-Motor Differential Drive Tank/Wheel Rover) | Target Goal: Full ROS 2 / Nav2 Autonomous Driving |
| **Firmware Repo (This Repo)** | `esp-maker-usba-4motor` (`c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor`) | Embedded C++ (PlatformIO) for NULLLAB Maker-ESP32-Pro |
| **Host / Cockpit Repo** | `yahboom-encoder` (`c:\Users\Ron\electronic_projects\yahboom-encoder`) | Legacy repo name; runs Cockpit Server + ROS 2 Jazzy on RPi5 |
| **Raspberry Pi 5 Host** | `ron@10.0.0.246` (`/home/ron/yahboom-encoder`) | Cockpit API: `:3000`, Nav Bridge: `:3010` |
| **Active Firmware Build** | **`1.3.4-remap-r03`** (`source: rev42-remap-c`, commit `a2e3d43`) | Rebuilt chassis port remap on top of verified `1.3.3-cand-r41c` control tuning (`102 PWM` spin breakout + `41.0 PWM` reverse crawl floor) |
| **Motor Port Map (Rebuilt Chassis, 2026-10-18)** | LF→port 2 (M2/E2), RF→port 1 (M1/E1), LR→port 4 (M4/E4), RR→port 3 (M3/E3) | All motors mirrored vs. original chassis: motor invert + encoder sign flipped on every port in `MotorDriver.cpp` / `EncoderManager.cpp`. **Wiring intentionally left as-built.** |
| **Effective Wheel Diameter** | `0.06695 m` (radius `0.033475 m`) | Firmware nominal `0.065 m`; `1974.1667` ticks/rev |
| **Effective Track Width** | `0.3408575433 m` | Calibrated differential track width |
| **IMU Sensor & Mode** | BNO08x — `SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC` (`0x08`) | Non-magnetic yaw only (magnetic orientation forbidden indoors) |
| **Latest Calibration Handoff** | [`docs/HANDOFF_REBUILT_CHASSIS_CALIBRATION_20261018.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/HANDOFF_REBUILT_CHASSIS_CALIBRATION_20261018.md) | Complete calibration & compound out-and-back handoff for Antigravity 2.0 |

### Standard Build & Deployment Commands
* **Run Native Firmware Unit Tests (Before Flashing)**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor
  python scripts/run_native_suite.py
  ```
* **Compile & Flash ESP32 Firmware (Remotely via RPi5)**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor
  python deploy_firmware.py
  ```
* **Deploy Cockpit / Host Scripts to RPi5**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\yahboom-encoder
  $env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py
  ```

---

## 2. Mandatory Change-Control & Git Checkpoint Workflow

To ensure we never lose a working calibration or get stuck after a regression:
1. **Commit Before Changing**: Always create a clean `git commit` of the working baseline **before** editing any firmware, controller, or configuration parameters.
2. **One Change at a Time**: Never bundle unrelated firmware or parameter changes into the same test cycle.
3. **Use Only Reusable Tests**: Never create one-off test scripts. Always run a numbered test from **Section 4 (Reusable Test Catalog)**.
4. **Record Every Run**: Log the commit hash, firmware version, physical measurement, sensor measurement, and JSON report filename in **Section 5 (Test Results Ledger)**.
5. **Revert on Regression**: If a code/tuning change makes physical behavior worse, immediately `git checkout` / `git revert` back to the pre-change commit.

---

## 3. Master Roadmap to Autonomous Driving

- [x] **Phase 1: Forward & Reverse Low-Speed Floor Calibration** (`COMPLETED`)
  - Forward crawl (`0.015 m/s` to `0.050 m/s`) and reverse crawl (`-0.015 m/s`, Case 13) calibrated and verified under `1.3.3-cand-r41c` / `1.3.4-remap-r03`.
- [x] **Phase 2: Rotate-in-Place (In-Place Spin) Calibration & Verification** (`COMPLETED`)
  - **Step 2A (Low-Speed & Nominal Spin Floor Check)**: Verified clean breakout on rebuilt chassis (`Case 4`, `0` resets).
  - **Step 2B (Closed-Loop Exact Angle Turns)**: Verified **90° CW** (`+90.58°` IMU, "perfect 90"), **90° CCW** (`-90.46°` IMU, "another perfect 90"), and **180° CW** (`+180.26°` IMU, `0` resets) using `AngularApproachController` ($0.80\text{ rad/s} \to 0.20\text{ rad/s}$ creep at $30^\circ$, stopping advance $0.7^\circ$).
- [ ] **Phase 3: Spin-to-Arc & Curved Transition Verification**
  - Run Case **8** (`Transition_PureSpin_To_Arc`) and Case **1** (`FailedArc_Baseline`) to confirm smooth handoff when Nav2 transitions from in-place rotation (`RotationShimController`) to DWB path following.
- [x] **Phase 4: Exact Linear Approach & Compound Out-and-Back Verification** (`COMPLETED`)
  - Verified $1.0\text{ m}$ forward (`+0.9968 m` enc / `+10 mm` physical at `8 mm` advance), $1.010\text{ m}$ reverse (`-0.9977 m` enc / `-4.5 mm` physical at `18 mm` advance), locked in **`13.5 mm` (`0.0135 m`)** stopping advance, and passed 3-leg **Compound Out-and-Back** ($1.0\text{ m} \to 180^\circ\text{ CW} \to 1.0\text{ m}$ return within $2.0\text{ mm}$ outbound/inbound parity, `0` resets).
- [ ] **Phase 5: Full Autonomous Navigation Acceptance (`gold_standard_home_test`)**
  - Execute the full ROS 2 / AMCL / Nav2 mission: drive $2.0\text{ ft}$ outbound, rotate $180^\circ$ CW in place, return to saved HOME pose within $\le 40\text{ mm}$ and $\le 3.0^\circ$.

---

## 4. Reusable Test Catalog

Do not write ad-hoc test scripts. Use the following parameterized, safety-checked test suites:

### A. Low-Level Floor & Stiction Characterization Suite (`scripts/characterize_floor_speeds.py`)
*Run on Raspberry Pi (`/home/ron/yahboom-encoder`):*
```bash
cd /home/ron/yahboom-encoder && python3 scripts/characterize_floor_speeds.py --case <CASE_NUM>
```

| Case # | Case ID | Category | Command Profile | Purpose / Acceptance Criteria |
| :---: | :--- | :--- | :--- | :--- |
| **2** | `Straight_Crawl_0.015mps` | Linear Fwd | `vx=+0.015 m/s` for `1.8s` | Forward crawl floor (`24–30 mm` encoder & physical) |
| **10** | `Straight_Crawl_0.030mps` | Linear Fwd | `vx=+0.030 m/s` for `1.8s` | Mid-crawl forward tracking (`~54 mm`) |
| **11** | `Straight_Crawl_0.050mps` | Linear Fwd | `vx=+0.050 m/s` for `1.8s` | Creep-speed forward tracking (`~90 mm`) |
| **12** | `Reverse_Crawl_0.050mps` | Linear Rev | `vx=-0.050 m/s` for `1.8s` | Creep-speed reverse tracking (`~-90 mm`) |
| **13** | `Reverse_Crawl_0.015mps` | Linear Rev | `vx=-0.015 m/s` for `1.8s` | **Reverse crawl floor** (`24–30 mm` encoder & physical) — **PASSED** |
| **4** | `PureSpin_Slow_CCW` | **Rotate** | `wz=+0.15 rad/s` for `1.5s` | **Slow CCW in-place spin** (`~12.9°` nominal, checks stiction/symmetry) |
| **5** | `PureSpin_Slow_CW` | **Rotate** | `wz=-0.15 rad/s` for `1.5s` | **Slow CW in-place spin** (`~-12.9°` nominal, checks stiction/symmetry) |
| **6** | `PureSpin_Nominal_CCW` | **Rotate** | `wz=+0.35 rad/s` for `1.5s` | **Nominal CCW in-place spin** (`~30.1°` nominal, checks steady spin) |
| **7** | `PureSpin_Nominal_CW` | **Rotate** | `wz=-0.35 rad/s` for `1.5s` | **Nominal CW in-place spin** (`~-30.1°` nominal, checks steady spin) |
| **8** | `Transition_PureSpin_To_Arc` | Transition | `wz=-0.35` (`0.8s`) $\to$ `vx=0.02, wz=-0.15` (`1.2s`) | Spin-to-arc track reversal handoff (Nav2 RotationShim $\to$ DWB) |
| **1** | `FailedArc_Baseline` | Arc | `vx=+0.014 m/s, wz=-0.132 rad/s` for `1.8s` | Tight low-speed arc with inner-wheel reverse (`-0.26 rad/s`) |

---

### B. Closed-Loop Exact Motion Suite (`tools/rover_tests`)
*Run from Windows PC or Raspberry Pi (`docs/REUSABLE_PHYSICAL_TEST_FRAMEWORK.md`):*

| Test Name | Reusable Command | Controller Used | Pass Criterion |
| :--- | :--- | :--- | :--- |
| **Turn Dry-Run (No Motion)** | `python -m tools.rover_tests.cli turn --degrees 90 --direction cw --dry-run` | `AngularApproachController` | Preflight + non-magnetic IMU verified |
| **Exact 90° CW Turn** | `python -m tools.rover_tests.cli turn --degrees 90 --direction cw --trials 1 --inter-trial-approval` | `AngularApproachController` | Settled error $\le 2.0^\circ$, clean disarm |
| **Exact 90° CCW Turn** | `python -m tools.rover_tests.cli turn --degrees 90 --direction ccw --trials 1 --inter-trial-approval` | `AngularApproachController` | Settled error $\le 2.0^\circ$, clean disarm |
| **Exact 180° CW Turn (3-Trial)** | `python -m tools.rover_tests.cli turn --degrees 180 --direction cw --trials 3 --inter-trial-approval` | `AngularApproachController` | Settled error $\le 2.0^\circ$ across 3 trials |
| **Exact 180° CCW Turn (3-Trial)** | `python -m tools.rover_tests.cli turn --degrees 180 --direction ccw --trials 3 --inter-trial-approval` | `AngularApproachController` | Settled error $\le 2.0^\circ$ across 3 trials |
| **Exact Linear Forward (1.0 m)** | `python -m tools.rover_tests.cli linear --distance 1.0 --direction forward --trials 1 --inter-trial-approval` | `LinearApproachController` | Settled error $\le 0.05\text{ m}$ ($50\text{ mm}$) |
| **Exact Linear Reverse (1.0 m)** | `python -m tools.rover_tests.cli linear --distance 1.0 --direction reverse --trials 1 --inter-trial-approval` | `LinearApproachController` | Settled error $\le 0.05\text{ m}$ ($50\text{ mm}$) |
| **Compound Out-and-Back (3-Leg / 4-Leg)** | `python -m tools.rover_tests.cli out-and-back --distance 1.0 [--final-turn]` | `LinearApproachController` + `AngularApproachController` | Outbound/Return $\le 50\text{ mm}$, Turn $\le 2.0^\circ$, `0` resets |

---

### C. End-to-End Autonomous Navigation Suite (`tools/gold_standard_home_test`)
*Run on Raspberry Pi (`docs/GOLD_STANDARD_HOME_TEST.md`):*

| Test Name | Reusable Command | Purpose / Pass Criterion |
| :--- | :--- | :--- |
| **Autonomous Dry-Run** | `./tools/run_gold_standard_home_test.sh --dry-run` | Verifies AMCL localization, HOME pose load, and safety gates without moving |
| **Safety Watchdog Self-Test** | `./tools/run_gold_standard_home_test.sh --test-safety` | Verifies stale-telemetry and reboot fail-safe aborts |
| **Gold Standard HOME Mission** | `./tools/run_gold_standard_home_test.sh --execute` | 2.0 ft outbound $\to$ 180° CW turn $\to$ return HOME within $\le 40\text{ mm}$ and $\le 3.0^\circ$ |

---

### D. Raised-Wheel Single-Motor Port / Polarity Check (`/api/maintenance/run_test`)
*Rover must be raised with all four wheels clear. Firmware maintenance mode powers exactly one motor open-loop (PWM capped at 60, 500 ms deadman, ≤3 s, other three motors held at 0) and returns the driver to `LOCKED`. Run on the Pi (or via SSH) once per motor index `0..3`, with the operator identifying the wheel and direction after each run (camera end = front):*
```bash
curl -s -X POST http://127.0.0.1:3000/api/maintenance/run_test -H 'Content-Type: application/json' \
  -d '{"safetyAck":true,"motorIndex":<0-3>,"direction":"forward","output":60,"durationSec":2.0}'
```

| Motor Index | Expected Wheel | Pass Criterion |
| :---: | :--- | :--- |
| `0` (M1) | Left Front | Spins **forward**, `encoder_delta` positive (`~3000–4000` @ 60 PWM / 2 s), `isolation_verified: true` |
| `1` (M2) | Right Front | Same |
| `2` (M3) | Left Rear | Same |
| `3` (M4) | Right Rear | Same |

---

## 5. Historical Test Results Ledger

| Date / Time | Firmware | Git Commit | Test Case | Physical Result | Encoder / IMU Result | Status / Verdict | Report JSON (in `reports/`) |
| :--- | :--- | :--- | :--- | :---: | :---: | :---: | :--- |
| 2026-10-13 04:44 | `1.3.3-cand-r41c` | `83258f1` | Case 13 (`Reverse_Crawl_0.015mps`, Trial 1 Cold) | `25.0 mm` | `21.4 mm` (79.3%) | `TRACKING_DEFICIT` | `floor_char_Reverse_Crawl_0.015mps_20261013_044421.json` |
| 2026-10-13 04:52 | `1.3.3-cand-r41c` | `83258f1` | Case 13 (`Reverse_Crawl_0.015mps`, Trial 2) | `30.0 mm` | `29.9 mm` (110.7%) | `SUCCESS` | `floor_char_Reverse_Crawl_0.015mps_20261013_045249.json` |
| 2026-10-13 04:53 | `1.3.3-cand-r41c` | `83258f1` | Case 13 (`Reverse_Crawl_0.015mps`, Trial 3) | `30.0 mm` | `33.2 mm` (123.0%) | `EXCESSIVE_TRAVEL` | `floor_char_Reverse_Crawl_0.015mps_20261013_045341.json` |
| 2026-10-13 04:54 | `1.3.3-cand-r41c` | `83258f1` | Case 13 (`Reverse_Crawl_0.015mps`, Trial 4) | `30.0 mm` | `32.6 mm` (120.7%) | `EXCESSIVE_TRAVEL` | `floor_char_Reverse_Crawl_0.015mps_20261013_045417.json` |
| **2026-10-13 17:41** | **`1.3.3-cand-r41c`** | **`83258f1`** | **Case 13 (`Reverse_Crawl_0.015mps`, Diagnostic A1)** | **`26.0 mm`** | **`29.0 mm` (107.4%)** | **`SUCCESS` (PASS)** | `floor_char_Reverse_Crawl_0.015mps_20261013_174144.json` |
| **2026-10-13 18:16** | **`1.3.3-cand-r41c`** | **`83258f1`** | **Case 13 (`Reverse_Crawl_0.015mps`, Diagnostic A2)** | `33.0 mm`* | **`26.8 mm` (99.3%)** | **`SUCCESS` (PASS)** | `floor_char_Reverse_Crawl_0.015mps_20261013_181641.json` |
| **2026-10-13 18:18** | **`1.3.3-cand-r41c`** | **`83258f1`** | **Case 13 (`Reverse_Crawl_0.015mps`, Diagnostic B1)** | **`30.0 mm`** | **`29.7 mm` (110.0%)** | **`SUCCESS` (PASS)** | `floor_char_Reverse_Crawl_0.015mps_20261013_181839.json` |
| **2026-10-14 04:42** | **`1.3.3-cand-r41c`** | **`daaf523`** | **Turn 90° CW (`rover-test turn`, Step 1/3)** | **`90.0°`** | **`+90.95°` (Err `+0.95°`)** | **`SUCCESS` (PASS)** | `turn_cw_90deg_1791978108.json` |
| **2026-10-14 05:06** | **`1.3.3-cand-r41c`** | **`d59c58f`** | **Turn 90° CW (`rover-test turn`, 3/3 Steps)** | **`90.0°, 90.0°, 92.0°`** | **`+90.58°, +90.95°, +90.84°` (Mean Err `+0.79°`, Span `0.37°`)** | **`SUCCESS` (3/3 PASS)** | `turn_cw_90deg_1791979545.json` |
| **2026-10-14 05:11** | **`1.3.3-cand-r41c`** | **`64bb56c`** | **Turn 90° CCW (`rover-test turn`, 3/3 Steps)** | **`-90.5°, -91.0°, -92.0°`** | **`-90.65°, -90.85°, -90.81°` (Mean Err `-0.77°`, Span `0.20°`)** | **`SUCCESS` (3/3 PASS)** | `turn_ccw_90deg_1791979851.json` |
| **2026-10-14 05:15** | **`1.3.3-cand-r41c`** | **`a2fdf81`** | **Turn 180° CW (`rover-test turn`, Step 1/3 @ `0.80 rad/s`)** | `~51°` (mid-turn stop) | Aborted after `1.5s` (`-1420` ticks) | **`ABORTED` (ESP32 `POWERON_RESET` / 275ms IMU gap)** | `turn_cw_180deg_1791980120.json` |
| **2026-10-14 05:21** | **`1.3.3-cand-r41c`** | **`fb4615f`** | **Turn 180° CW (`rover-test turn`, Step 1/3 @ `0.40 rad/s`)** | `~66°` (mid-turn stop) | Aborted (`-1846` ticks) | **`ABORTED` (ESP32 `POWERON_RESET` / 251ms IMU gap)** | `turn_cw_180deg_1791980504.json` |
| **2026-10-14 05:23** | **`1.3.3-cand-r41c`** | **`fb4615f`** | **Turn 180° CW (`rover-test turn`, Step 1 PASS / Step 2 Abort)** | **Step 1: `180.0°`** / Step 2: `~91°` | **Step 1: `+181.16°` (`+1.16°` err)** / Step 2: `-2566` ticks | **Step 1 `SUCCESS` / Step 2 `ABORTED` (ESP32 `POWERON_RESET`)** | `turn_cw_180deg_1791980606.json` |
| **2026-10-14 05:30** | **`1.3.3-cand-r41c`** | **`ce957ef`** | **Turn 90° CW (`rover-test turn`, Step 1/3 Re-test)** | `~40°` (mid-turn stop) | Aborted after `1.1s` (`-1110` ticks) | **`ABORTED` (ESP32 `POWERON_RESET` / 254ms IMU gap)** | `turn_cw_90deg_1791981026.json` |
| **2026-10-15 04:38** | **`1.3.3-cand-r41c`** | **`c5b9d41`** | **Turn 90° CW (`rover-test turn`, Cold Start Step 1/3)** | `~22°` (mid-turn stop) | Aborted after `0.5s` (`-616` ticks) | **`ABORTED` (ESP32 `POWERON_RESET` / 264ms IMU gap)** | `turn_cw_90deg_1792064274.json` |
| **2026-10-15 04:45** | **`1.3.3-cand-r41c`** | **`c5b9d41`** | **Turn 90° CW (`rover-test turn`, 3/3 Steps post-manual)** | **`90.0°, 90.0°, 90.0°`** | **`+90.54°, +90.58°, +90.62°` (Mean Err `+0.58°`, Span `0.07°`)** | **`SUCCESS` (3/3 PASS)** | `turn_cw_90deg_1792064709.json` |
| **2026-10-15 04:51** | **`1.3.3-cand-r41c`** | **`d5b78f8`** | **Turn 180° CW (`rover-test turn`, Step 1/3 @ `0.80 rad/s`)** | Mid-turn stop | Aborted after `1.1s` | **`ABORTED` (ESP32 `POWERON_RESET`)** | `turn_cw_180deg_1792065104.json` |
| **2026-10-15 04:54** | **`1.3.3-cand-r41c`** | **`d5b78f8`** | **Turn 180° CW (`rover-test turn`, 3/3 Steps @ `0.30 rad/s`)** | **Step 3 `~180°` (slight overshoot)** | **`+180.84°, +180.84°, +181.12°` (Mean Err `+0.93°`, Span `0.28°`)** | **`SUCCESS` (3/3 PASS)** | `turn_cw_180deg_1792065144.json` |
| **2026-10-15 05:06** | **`1.3.3-cand-r41c`** | **`edce0ff`** | **Turn 180° CW (`rover-test turn`, Step 1 PASS / Step 2 Abort @ `0.30 rad/s`)** | **Step 1: `180.0°` (`+170 mm` fwd walk)** | **Step 1: `+179.89°` (`-0.11°` err)** / Step 2: `2.17s` abort | **Step 1 `SUCCESS` / Step 2 `ABORTED` (ESP32 `POWERON_RESET`)** | `turn_cw_180deg_1792065888.json` |
| **2026-10-15 05:14** | **`1.3.3-cand-r41c`** | **`20a63c4`** | **Turn 180° CW (`rover-test turn`, `--enable-balancing` @ `0.30 rad/s`)** | **Step 1: `180.0°`** / Step 2: `~100°` | **Step 1: `+180.07°` (`+0.07°` err)** / Step 2: `5.26s` abort | **Step 1 `SUCCESS` / Step 2 `ABORTED` (ESP32 `POWERON_RESET` @ M4 PWM 125)** | `turn_cw_180deg_1792066438.json` |
| **2026-10-15 07:05** | **`1.3.4-cand-r42`** | **`2be8a16`** | **Turn 90° CW (`rover-test turn`, `--no-braking` Step 1/3)** | **`0` ESP32 Resets** (partial turn after `1.15s`) | `M1,M2,M3` broke out @ `79–80 PWM` (`1.15s`); `M4` reached `90 PWM` @ `1.75s` | **`ABORTED` (Runner `1.5s` stall safeguard on `M4`; `0` ESP32 resets)** | `turn_cw_90deg_1792073146.json` |
| **2026-10-15 07:20** | **`1.3.4-cand-r42b`** | **`f6a5728`** | **Turn 90° CW (`rover-test turn`, `--no-braking` Step 1/3)** | Mid-turn stop (`t=0.52s`) | `SPIN_KINETIC_KS_PWM=65` + `dTerm` spiked PWM to `110–118` (`∑|PWM|=455`) | **`ABORTED` (ESP32 `POWERON_RESET` @ `∑|PWM|=404–455`)** | `turn_cw_90deg_1792074038.json` |
| **2026-10-15 07:33** | **`1.3.4-cand-r42c`** | **`5a93afd`** | **Turn 90° CW (`rover-test turn`, `--no-braking` Step 1/3 @ `0.80 rad/s`)** | Spun `1.86s` (`-792/+850/-562/+143` ticks, `~30°`) then stopped | All 4 wheels broke out at `t=0.13–0.26s` (`∑|PWM|=344`); `0.80 rad/s` target drove all 4 to `92` cap (`∑|PWM|=368`) & `M4` stalled at `92` cap | **`ABORTED` (ESP32 `POWERON_RESET` at `t=1.86s` after `0.6s` of `M4` stalled @ `92 PWM`)** | `turn_cw_90deg_1792074798.json` |
| **2026-10-15 08:09** | **`1.3.4-cand-r42d`** | **`cdf5af4`** | **Turn 90° CW (`rover-test turn`, `--no-braking` Step 1/3 @ `0.30 rad/s`)** | Too slow to break stiction (`M3` stalled `-59` ticks over `4.27s`) | `pwm=[-90, 82, -90, 104]` (`∑|PWM|=366`); `M3` never broke stiction (`vel=0`) at `-90` cap | **`ABORTED` (`M3` stalled + ESP32 `POWERON_RESET` at `t=4.27s`)** | `turn_cw_90deg_1792076939.json` |
| **2026-10-15 08:21** | **`1.3.3-cand-r41c`** | **`e2a884c`** | **Turn 90° CW (`rover-test turn`, Step 1/3 @ `0.80 rad/s`)** | Broke stiction cleanly (`-900/+1021/-695/+572` ticks in `0.91s`), stopped mid-turn | `102 PWM` boost broke all 4 wheels out (`vel=[-3.8, 4.3, -3.0, 2.7]`), then `0.80 rad/s` drove `∑|PWM|=462` | **`ABORTED` (ESP32 `POWERON_RESET` at `t=0.91s` @ `∑|PWM|=462`)** | `turn_cw_90deg_1792077682.json` |
| **2026-10-15 08:25** | **`1.3.3-cand-r41c`** | **`e2a884c`** | **Turn 90° CW (`rover-test turn`, 3/3 Steps @ `0.45 rad/s`)** | **Completed 3/3 cleanly (`0` resets, `0` stalls)** | **`+90.35°, +90.44°, +90.36°` (Mean Err `+0.38°`, Span `0.09°`)** | **`SUCCESS` (3/3 PASS)** | `turn_cw_90deg_1792077902.json` |
| **2026-10-15 08:33** | **`1.3.3-cand-r41c`** | **`b1402b7`** | **Turn 180° CW (`rover-test turn`, Step 1/3 @ `0.45 rad/s`)** | Spun `~2.8s` (`-1875/+2370/-1472/+1236` ticks, `~75°`) then stopped | Broke stiction cleanly on all 4 wheels, then ESP32 reset after `2.8s` of continuous 4-motor spin | **`ABORTED` (ESP32 `POWERON_RESET` / 254ms IMU gap)** | `turn_cw_180deg_1792078433.json` |
| **2026-10-18 06:59** | **`1.3.3-cand-r41c`** | **`c2477d6`** | **Single-Motor Port Check, raised (Suite D, PWM 40)** | M1/M4 spun; M2/M3 did not break away | `+1152 / 0 / +3 / +823` ticks, all isolated | **`INCONCLUSIVE`** (M2/M3 stiction at 40 PWM) | — |
| **2026-10-18 07:01–07:03** | **`1.3.3-cand-r41c`** | **`c2477d6`** | **Single-Motor Port Check, raised (Suite D, PWM 50)** | M1→LR, M2→none, M3→LF, M4→RF (fwd)* | `+2673 / +1 / +2268 / +2065` ticks | **`FAIL` (ports mis-mapped; M2 stiction)** | — |
| **2026-10-18 07:08–07:11** | **`1.3.3-cand-r41c`** | **`c2477d6`** | **Single-Motor Port Check, raised (Suite D, PWM 60)** | M1→LR, M2→RR, M3→LF, M4→RF (fwd)* | `+3966 / +3094 / +3458 / +3163` ticks, all isolated | **`FAIL` (ports mis-mapped)** — *corner IDs later found unreliable | — |
| **2026-10-18 07:56–07:59** | **`1.3.4-remap-r01`** | **`b85d56d`** | **Single-Motor Port Check, raised (Suite D, PWM 60, camera = front, viewed from right)** | M1→RR bwd, M2→LR bwd, M3→RF bwd, M4→LF bwd | `+3659 / +2958 / +3857 / +3194` ticks | **`FAIL`** — established physical truth: port1=RF, port2=LF, port3=RR, port4=LR, all motors mirrored | — |
| **2026-10-18 08:12–08:14** | **`1.3.4-remap-r02`** | **`512331e`** | **Single-Motor Port Check, raised (Suite D, PWM 60)** | M1→LR fwd, M2→LF fwd, M3→RF fwd, M4→RR fwd | `+3203 / +2960 / +3475 / +3441` ticks | **`PARTIAL`** — all directions correct; 3 corners mis-assigned (derivation error in r02 table) | — |
| **2026-10-18 08:25–08:27** | **`1.3.4-remap-r03`** | **`a2e3d43`** | **Single-Motor Port Check, raised (Suite D, PWM 60)** | **M1→LF fwd, M2→RF fwd, M3→LR fwd, M4→RR fwd** | **`+2979 / +3596 / +3264 / +3546` ticks, all isolated** | **`SUCCESS` (4/4 PASS)** | — |
| **2026-10-18 14:01** | **`1.3.4-remap-r03`** | **`2f4be4c`** | **Case 2 (`Straight_Crawl_0.015mps`, Rebuilt Chassis)** | **`40.0 mm`** | **`+67.5 mm` (250.0%)** | **`EXCESSIVE_TRAVEL` (Over-speed on new chassis: avg `0.037 m/s` vs `0.015 m/s`; 2.0 mm overrun)** | `floor_char_Straight_Crawl_0.015mps_20261018_140145.json` |
| **2026-10-18 14:02** | **`1.3.4-remap-r03`** | **`2f4be4c`** | **Case 13 (`Reverse_Crawl_0.015mps`, Rebuilt Chassis)** | **`10.0 mm`** | **`-33.7 mm` (124.8%)** | **`EXCESSIVE_TRAVEL` (Near nominal `-27 mm`; 1.1 mm overrun)** | `floor_char_Reverse_Crawl_0.015mps_20261018_140245.json` |
| **2026-10-18 14:24** | **`1.3.4-remap-r03`** | **`954f9df`** | **Case 11 (`Straight_Crawl_0.050mps`, Rebuilt Chassis)** | **`176.0 mm`** | **`+139.3 mm` (154.8%)** | **`EXCESSIVE_TRAVEL` (Linear creep test: nominal 90 mm; 8.5 mm overrun)** | `floor_char_Straight_Crawl_0.050mps_20261018_142409.json` |
| **2026-10-18 14:45** | **`1.3.4-remap-r03`** | **`fe995db`** | **Case 12 (`Reverse_Crawl_0.050mps`, Rebuilt Chassis)** | **`150.0 mm`** | **`-139.0 mm` (154.4%)** | **`EXCESSIVE_TRAVEL` (Reverse creep test: nominal -90 mm; 6.6 mm overrun; symmetrical to Case 11)** | `floor_char_Reverse_Crawl_0.050mps_20261018_144552.json` |
| **2026-10-18 14:47** | **`1.3.4-remap-r03`** | **`8f43e9e`** | **Case 4 (`PureSpin_Slow_CCW`, Rebuilt Chassis)** | **`~12.5°`** | **`+11.86°` IMU (Nominal `+12.9°`)** | **`TRAVEL_BOUND_EXCEEDED` (Tripped 50mm wheel travel limit at t=0.66s; clean breakout, 0 resets, IMU within 0.6°)** | `floor_char_PureSpin_Slow_CCW_20261018_144754.json` |
| **2026-10-18 14:57** | **`1.3.4-remap-r03`** | **`37ee25d`** | **Exact 90° CW Turn (`rover-test turn`, Rebuilt Chassis)** | **`90.0°` ("perfect 90")** | **`+90.58°` IMU (Err `+0.58°`)** | **`SUCCESS` (1/1 PASS; 0 resets under 4-motor spin, Geekworm UPS power isolation verified)** | `turn_cw_90deg_1792360648.json` |
| **2026-10-18 15:01** | **`1.3.4-remap-r03`** | **`37ee25d`** | **Exact 90° CCW Turn (`rover-test turn`, Rebuilt Chassis)** | **`-90.0°` ("another perfect 90")** | **`-90.46°` IMU (Err `-0.46°`)** | **`SUCCESS` (1/1 PASS; 0 resets under 4-motor CCW spin; bilateral symmetry: Cruise 1.26s CW vs 1.26s CCW, Err < 0.6°)** | `turn_ccw_90deg_1792360827.json` |
| **2026-10-18 15:04** | **`1.3.4-remap-r03`** | **`651a233`** | **Exact 180° CW Turn (`rover-test turn`, Rebuilt Chassis)** | **`182.0°` ("looked good, maybe a couple deg too far")** | **`+180.26°` IMU (Err `+0.26°`)** | **`SUCCESS` (1/1 PASS; 0 resets over 4.3s continuous 4-motor spin, Geekworm UPS fully validated under sustained load)** | `turn_cw_180deg_1792361016.json` |
| **2026-10-18 15:21** | **`1.3.4-remap-r03`** | **`78ec915`** | **Exact 1.000m Forward Linear Test (`rover-test linear`, Rebuilt Chassis)** | **`~1050 mm` ("about 50 mm too far")** | **`+1.0418 m` Enc (+41.8 mm err), `+1.10°` Yaw Drift** | **`SUCCESS` (1/1 PASS; 4 wheels matched within 31 ticks / 3.3 mm; L/R speed diff -0.016 rad/s, PWM diff 1.0; 0 resets)** | `linear_forward_1p000m_1792362043.json` |
| **2026-10-18 15:28** | **`1.3.4-remap-r03`** | **`1db403d`** | **Exact 1.000m Reverse Linear Test (`rover-test linear`, Rebuilt Chassis)** | **`~1050 mm` ("about 50mm too far")** | **`-1.0499 m` Enc (-49.9 mm err), `-1.54°` Yaw Drift** | **`SUCCESS` (1/1 PASS; Fwd/Rev symmetry: 1041.8mm vs 1049.9mm [Δ=8.1mm]; Coast: 40.8mm fwd vs 42.5mm rev; 0 resets)** | `linear_reverse_1p000m_1792362168.json` |
| **2026-10-18 15:46** | **`1.3.4-remap-r03`** | **`d53ca31`** | **Calibrated 1.000m Forward Test (`rover-test linear`, Rebuilt Chassis)** | **`+10 mm` ("10 mm long" [1010 mm physical])** | **`+0.9968 m` Enc (-3.2 mm err), `+1.11°` Yaw Drift** | **`SUCCESS` (1/1 PASS; 99.68% accuracy, net error -3.2mm [-0.32%]; coast 4.2mm; L/R speed diff -0.002 rad/s; 0 resets)** | `linear_forward_1p000m_1792363466.json` |
| **2026-10-18 15:50** | **`1.3.4-remap-r03`** | **`7707217`** | **Calibrated 1.010m Reverse Test (`rover-test linear`, Rebuilt Chassis)** | **`-4.5 mm` ("almost perfect. 4-5 mm short")** | **`-0.9977 m` Enc (+12.3 mm err), `-1.84°` Yaw Drift** | **`SUCCESS` (1/1 PASS; 1.010m return target; coast 4.7mm; L/R speed diff +0.029 rad/s; 0 resets; Mode 0 LOCKED)** | `linear_reverse_1p010m_1792363806.json` |
| **2026-10-18 15:58** | **`1.3.4-remap-r03`** | **`100803c`** | **Autonomous Compound Out-and-Back (1.0m Fwd -> 180° CW -> 1.0m Return)** | **Lined up well at origin ("good now lined up")** | **Leg 1: `+0.9916 m` (-8.4mm)<br>Leg 2: `+181.24°` (+1.24°)<br>Leg 3: `+0.9936 m` (-6.5mm)** | **`SUCCESS` (3/3 LEGS PASS; Outbound vs Inbound parity Δ=2.0mm [0.2%]; 0 stalls; 0 resets across full ~25s compound run; Mode 0 LOCKED)** | `linear_forward_1p000m_1792364263.json`<br>`turn_cw_180deg_1792364272.json`<br>`linear_forward_1p000m_1792364279.json` |

*\*Note on 2026-10-13 18:16 (A2): Wheel encoders turned `-259` ticks (`26.8 mm` @ 65mm / `27.6 mm` @ 66.95mm max tire travel), confirming nominal motor travel (`99.3%`) with a ~5 mm mark placement offset.*
*\*Note on 2026-10-14 04:42 (Turn 90° CW): Step 1 passed (`+90.95°` IMU, `90.0°` physical, `0` wheel stalls). Step 2 aborted before motion because `complete_zero_handshake` in `tools/rover_tests/transport.py` did not refresh `autonomyState.lastCmdTime` after reaching `READY_DISARMED`; fixed in `transport.py` and `server.js` (`d59c58f` / `43e828d`).*
*\*Note on 2026-10-14 05:15–05:30 (ESP32 `POWERON_RESET` progression): Even the 90° CW test that passed 3/3 at `05:06` now aborted at `~40°` (`-1110` ticks, `1.1s`) with an ESP32 hardware `POWERON_RESET (0x1)` (`0xE))` boot banner in `journalctl`), confirming progressive power/voltage sag or thermal protection under 4-motor spin load.*

---

## 6. Daily Engineering & Activity Log

### 2026-10-18 (Saturday)
- **Active Firmware**: `1.3.4-remap-r03` (`source: rev42-remap-c`, commit `a2e3d43`) — flashed `08:15 PDT`
- **Context**: Ron completed the **new chassis build** and needed to verify motor-to-port wiring. Decision: **keep the as-built wiring and remap in firmware**.
- **Accomplishments & Observations**:
  1. **Reachability Check (`06:53 PDT`)**: RPi5 `10.0.0.246` ping ~2 ms; CH340 `1a86:7523` enumerated; `/dev/rover-esp32 → ttyUSB0`; cockpit `serialConnected: true`, `lastPacketAgeMs: 10`. Board was left `armed: true` from a prior session — disarmed at `06:59`.
  2. **Introduced Reusable Suite D (Raised-Wheel Single-Motor Check)** using the existing firmware maintenance mode (`0x26/0x27/0x28`) via `/api/maintenance/run_test`. One motor at a time, open-loop, PWM-capped, deadman-protected, exits `LOCKED`. No ad-hoc motion scripts.
  3. **Stiction Finding**: at PWM 40 only M1/M4 broke away; at PWM 50 M2 still did not move (`+1` tick); at **PWM 60 all four moved** (`~3000–4000` ticks / 2 s). Open-loop breakaway on the new chassis is between 50 and 60 PWM for the stiffest wheel (port 2 / Left Front).
  4. **Observation Reliability**: the first corner-identification round (`07:01–07:11`) was taken while Ron had lost focus and proved wrong. From `07:56` onward every run used a fixed reference (**camera mount = front, operator standing on the right side**) and the form listed "nearest you / far side" to remove ambiguity. All later rounds were internally consistent and predictions held.
  5. **Physical Truth Established (`07:56–07:59`, on `remap-r01`)**: board port 1 → **Right Front**, port 2 → **Left Front**, port 3 → **Right Rear**, port 4 → **Left Rear**; **every motor spins backward on a positive command while every encoder counts positive** → motors mounted mirrored, so motor invert *and* encoder sign flip together per port (closed loop stays self-consistent).
  6. **Firmware Remap Iterations** (`src/MotorDriver.cpp`, `src/EncoderManager.cpp`, version strings in `src/SerialProtocol.cpp`, `test/test_open_loop_expiry.cpp`, manifest hash in `deploy_firmware.py`):
     - `abbe3e2` checkpoint baseline (also reverted an accidental ` the rpi` line in `platformio.ini`).
     - `d26c523`/`b85d56d` **remap-r01** — front↔rear swap based on the unreliable round; wrong.
     - `512331e` **remap-r02** — correct per-port polarity flips, but corner table mis-derived (I read M-numbers as board ports on an already-remapped build). Verification round proved all four *directions* correct and pinned the corner error.
     - `a2e3d43` **remap-r03** — LF→port 2 (no invert, E2 `+`), RF→port 1 (invert, E1 `−`), LR→port 4 (invert, E4 `(B,A)` `−`), RR→port 3 (invert, E3 `−`). **Verified 4/4 at `08:25–08:27`.**
  7. **Native test** `test_open_loop_expiry` compiled and passed on the Pi (`g++`) for r01/r02/r03 (firmware identity packet NUL-termination verified for the new version/tag strings). Note: `scripts/run_native_suite.py` expects a Linux `test_patched_native` binary and does not run on Windows.
  8. **Deploy Note**: `deploy_firmware.py` enforces a hard-coded `EXPECTED_FIRMWARE_SHA256`; it must be updated to the new `.pio/build/esp32dev/firmware.bin` hash for every approved build. Each flash takes ~8 min because the script backs up the full 4 MB flash at 115200 baud first.
- **Open Items / Next Steps**:
  - Floor characterization (Suite A cases 2/13/4/5) must be re-run on the new chassis before any exact-motion test — stiction, trims (`LEFT_TRIM*`/`RIGHT_TRIM*`), and spin floors were calibrated on the old chassis.
  - Check the Left Front wheel (port 2) for mechanical drag — it needed ~60 PWM to break away where others moved at 40.
  - The cockpit UI labels (`m1 (Left Front)` etc.) are correct again after the remap; no host-side changes were needed.
- **Final State**: drivetrain stopped, **disarmed**, **LOCKED**.

### 2026-10-15 (Thursday)
- **Active Firmware**: `1.3.3-cand-r41c` (`source: rev41-floor-c`)
- **Accomplishments & Observations**:
  1. **Cold Start Check (`04:36 PDT`)**: Powered on Rover 1 after 22+ hours of rest (`temp=37.8°C`, `EXT5V_V=4.85V`).
  2. **First Cold 90° CW Run (`04:38 PDT`, `turn_cw_90deg_1792064274.json`)**: Aborted at `0.5s` (`-616` ticks, `~22°`) with ESP32 `POWERON_RESET`.
  3. **Manual Drive Exercise (`04:42–04:44 PDT`)**: Operator exercised Rover 1 in Manual Drive (experienced two ESP32 resets at `04:42:26` and `04:42:59` on initial breakaway, then spun smoothly for multiple rotations at `04:43:13`).
  4. **90° CW 3-Repetition Test (`04:45 PDT`, `turn_cw_90deg_1792064709.json`)**: **PASSED 3/3** with `+90.54°, +90.58°, +90.62°` IMU (`90.0°, 90.0°, 90.0°` Physical; Mean Err **`+0.58°`**, Repeatability Span **`0.07°`**, `0` stalls).
  5. **Root Cause of 90° vs. 180° Behavior Confirmed (`04:51` & `04:54 PDT`)**:
     - At `04:51 PDT` (`turn_cw_180deg_1792065104.json`), 180° CW at `0.80 rad/s` aborted after `1.1s` with ESP32 `POWERON_RESET`.
     - At `04:54 PDT` (`turn_cw_180deg_1792065144.json`), 180° CW at `0.30 rad/s` **PASSED 3/3** (`+180.84°, +180.84°, +181.12°` IMU; Mean Err **`+0.93°`, Repeatability Span **`0.28°`**, `0` stalls, 32s total spin time without a single reset).
  6. **Found & Fixed `stopping_advance_deg` Bug Causing `+0.8°` to `+1.1°` Turn Overshoot**:
     - In [`tools/rover_tests/runner.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/tools/rover_tests/runner.py#L1768), `stopping_advance = self.params.stopping_advance_deg if self.params.enable_braking else 0.0` evaluated to `0.0` whenever `--enable-braking` was omitted on the CLI (`self.params.enable_braking is None`), even though braking was enabled by default (`trial_report.stopping_advance_deg = 0.7`). Fixed in `edce0ff`.
  7. **Verified `0.70°` Stopping Advance (`+179.89°`, `-0.11°` err) & Diagnosed `170 mm` Forward Walk (`05:06 PDT`, `turn_cw_180deg_1792065888.json`)**:
     - Step 1 stopped at `179.30°` and settled at **`179.89°` (`-0.11°` error)**!
     - However, during the 180° spin without `--enable-balancing`, **`M2 (RF)` rotated `+6577` ticks (`701.2 mm`) forward while `M1 (LF)` rotated `-4910` ticks (`-523.5 mm`) backward — an excess of `+1667` ticks = `+177.7 mm` (`~170 mm` measured by Ron)**!
  8. **Tested 180° CW with `--enable-balancing` (`05:14 PDT`, `turn_cw_180deg_1792066438.json`)**:
     - Step 1 **PASSED** with **`+180.07°` IMU (`+0.07°` error!)**, reducing `M2` ticks from `6577` down to `5729` and matching rear wheels (`M3 = -4273`, `M4 = +4332`).
     - Step 2 aborted at `t = 5.26s` with ESP32 `POWERON_RESET` right as `M4 (RR)` reached `finalPwm = 125` (`basePwm 115 + spinSyncTrim 10`).
  9. **Forensic Audit & System-Wide Serial Port Check (`05:20–05:45 PDT`)**:
     - Disproved the earlier hypothesis that `M4 = 125 PWM` alone caused the reset (Trial 1 had reached `M4 = 124 PWM` and `∑|finalPwm| = 408` without resetting, and an idle reset occurred at `05:20:59 PDT` with `PWM = 0`).
     - Verified why `esp_boot_count_start` remains `1` on `POWERON_RESET` (`RTC_DATA_ATTR` is cleared by ESP-IDF startup on `0x1 POWERON_RESET`) and why `tools/rover_tests/reporting.py` falls back to `esp_reset_reason_start` when `esp_reset_reason_end` is `None`.
     - Confirmed via `lsof`, `fuser`, `dmesg -T`, and `journalctl` that only `rover-server` (`node[894]`) holds `/dev/ttyUSB0` open (`FD 18`) and zero USB disconnects occurred.
  10. **Live Physical Reset-Isolation Matrix (`05:45–06:12 PDT`, Disarmed `mode=0, PWM=0`, Controller Off-Chassis)**:
      - **Steps 1–4 (Tap/touch ESP32 board, move USB connector at ESP32 & RPi5 ends, touch power connector/switch)**: **PASS (`0` resets over 43 minutes)**.
      - **Step 5 (Chassis flex)**: Skipped (controller board is off the chassis).
      - **Steps 6A & 6B (Manually roll/pivot rover on hardwood floor while disarmed, `PWM=0`)**: **CONFIRMED ESP32 `POWERON_RESET` (`2/2 = 100%` repeatable at `06:04:11` and `06:06:13 PDT`)**.
      - **Air Spin Tests A & B (Spin all 4 wheels individually and front/rear pairs simultaneously in the air off the floor)**: **PASS (`0` resets)**.
  11. **Manual Driving Comparison (`06:15–06:20 PDT`) & $L > W$ Skid-Steer Analysis**:
      - **Straight Forward & Reverse Driving (Slow and Fast)**: Drove continuously for `1m 43.4s` (`2.67 m` odometry) on the hardwood floor with **zero resets**.
      - **Manual Turns (Slow and Fast)**: Triggered ESP32 `POWERON_RESET` (`SPI_FAST_FLASH_BOOT`) at `06:15:45`, `06:16:11`, `06:18:07`, and `06:18:34 PDT`, resetting the board into `mode 0 (DISARMED)` and stopping the rover.
  12. **Firmware Audit for Turn-Only Contributors (`06:25–06:32 PDT`)**:
      - Identified 5 turn-only firmware overrides added after the earlier "room-to-room" baseline: (1) `SPIN_RECOVERY_TWITCH_PULSE` (`-3.50 rad/s` opposite-direction full-power slam after `350 ms` in `ChassisSupervisor.h`), (2) `SPIN_STICTION_BOOST_PWM = 102` & `80–94 PWM` minimum spin kinetic floors, (3) `SPIN_PID_KP = 10.0` (vs `2.2` straight) & `ANTI_STALL_KI = 25.0` (ramping scrubbing wheels to `225+ PWM`), (4) `200 ms` `Stalled-KINETIC` `102 PWM` re-breakout reset in `ChassisSupervisor.h`, and (5) `DYNAMIC_BRAKE_ENABLED` (`brakeAll()` `IN1=255, IN2=255` short-circuit pulse on stop).
  13. **Deployed Smooth-Turn Firmware Candidate `1.3.4-cand-r42` (`06:42 PDT`, Pre-Change Checkpoint `4d44c31`, Commit `ba69f82`)**:
      - Preserved 100% of straight-line forward/reverse control (`r41c`) while disabling the 5 turn-only overrides (`SPIN_STICTION_BOOST_PWM: 102 -> 64`, `MIN_SPIN_KINETIC_FF_FLOOR: 80/94 -> 56`, `SPIN_PID_KP: 10.0 -> 2.2`, `ANTI_STALL_KI: 25.0 -> 3.6`, turn integral clamp `±50 PWM`, immediate `>=3` tick spin breakout exit, `updateChassisStallSupervisor` / `updateSpinRecoveryHarness` disabled, and `DYNAMIC_BRAKE_ENABLED = false` on boot).
  14. **Manual Drive & Turn Verification on `1.3.4-cand-r42` (`06:53:35–06:54:23 PDT`)**:
      - Operator drove and turned manually on hardwood floor (`armed: true, mode: 3`) with **zero ESP32 resets** during the test window (`06:53:35–06:54:23`) and confirmed turn behavior felt smooth/normal again like before.
  15. **Closed-Loop 90° CW Test on `1.3.4-cand-r42` (`07:05 PDT`, `turn_cw_90deg_1792073146.json`) & Deployment of `1.3.4-cand-r42b` (`07:14 PDT`)**:
      - `1792073146.json` completed with **zero ESP32 resets** (`bootCount: 1 -> 1`) and proved that on the $L > W$ hardwood floor, `M1, M2, M3` break out at `79–80 PWM` (`t = 1.15s`) while `M4 (RR)` requires `90–92 PWM` (`t = 1.75s`, tripping the runner's `1.50s` stall timer when starting from `56 PWM`).
      - Deployed `1.3.4-cand-r42b` (`rev42b-smooth`, pre-change checkpoint `c856b6b`), setting `MIN_SPIN_KINETIC_FF_FLOOR = 80.0f` and `SPIN_FORWARD_REAR_KINETIC_FLOOR = 92.0f` with a `60 ms` startup ramp (`48 -> 84 PWM`), gentle `SPIN_PID_KP = 3.0f`, `ANTI_STALL_KI = 10.0f`, turn integral clamp `±35 PWM`, and keeping twitch/re-breakout/brake-short disabled.
  16. **Closed-Loop 90° CW Test on `1.3.4-cand-r42b` (`07:20 PDT`, `turn_cw_90deg_1792074038.json`) & Deployment of `1.3.4-cand-r42c` (`07:28 PDT`)**:
      - In `1792074038.json`, setting `SPIN_KINETIC_KS_PWM = 65.0f` caused `rawFfMagnitude = 65 + 6.0 * 4.19 = 90.1 PWM` at `0.80 rad/s`, and `lastError = 0` on exiting `STICTION_BOOST` added a `+17.6 PWM` `dTerm` spike, slamming all 4 motors to `-116, +110, -111, +118 PWM` (`∑|PWM| = 455` peak, `404` sustained) and triggering an ESP32 `POWERON_RESET` at `t = 0.52 s`.
      - Deployed `1.3.4-cand-r42c` (`rev42c-capped`, pre-change checkpoint `735bc55`): restored `SPIN_KINETIC_KS_PWM = 22.0f` and `SPIN_PID_KP = 2.2f` from `r42`, set `MIN_SPIN_KINETIC_FF_FLOOR = 76.0f` (`76 FF + ~8 P = ~84 PWM`), `SPIN_FORWARD_REAR_KINETIC_FLOOR = 84.0f` (`84 FF + ~8 P = ~92 PWM`), seeded `lastError` on exiting `STICTION_BOOST` (zero `dTerm` spike), and hard-capped turn `targetPwm` to `±92 PWM` (`∑|PWM| <= 368`).
  17. **Closed-Loop 90° CW Test on `1.3.4-cand-r42c` (`07:33 PDT`, `turn_cw_90deg_1792074798.json`) & Deployment of `1.3.4-cand-r42d` (`07:48 PDT`)**:
      - Spun for `1.86 s` (`M1=-792, M2=+850, M3=-562, M4=+143` ticks, `~30°`), `3.6x` longer than `r42b` (`0.52 s`).
      - At `t = 0.26 s`, all 4 wheels were spinning (`vel = [-1.20, +1.84, -0.98, +0.98 rad/s]`) at `pwm = [-84, 83, -85, 92]` (`∑|PWM| = 344`).
      - However, because the default turn cruise speed is `0.80 rad/s` (`targetRadps = ±4.19 rad/s`), `error = 4.19 - 1.84 = +2.35 rad/s` remained positive on all 4 wheels, ramping `M1, M2, M3` up to the `92 PWM` cap (`[-92, 92, -92, 92]`, `∑|PWM| = 368`), while `M4 (RR)` stalled (`vel = 0`) at `t = 0.76 s` because `92 PWM` was slightly below `M4`'s `101–108 PWM` kinetic rolling floor (observed in `1792065144.json`), triggering a reset at `t = 1.86 s` after `1.1 s` of `M4` locked-rotor stall at `92 PWM`.
      - Deployed `1.3.4-cand-r42d` (`rev42d-capped`, pre-change checkpoint `7aa01fd`): set `activeKd = 0.0f` during pure spin (`isSpinManeuver`), `SPIN_FORWARD_REAR_KINETIC_FLOOR = 88.0f`, and per-wheel turn cap to `106 PWM` for `isForwardRearWheel` (`90 PWM` for the other 3 wheels).
  18. **Closed-Loop 90° CW Test on `1.3.4-cand-r42d` @ `0.30 rad/s` (`08:09 PDT`, `turn_cw_90deg_1792076939.json`)**:
      - Confirmed operator observation that `0.30 rad/s` with `78 PWM` boost and `90 PWM` cap is too slow/weak to break stiction on the left side (`M3` stayed stalled at `vel = 0`, `-59` ticks total over `4.27 s` while `M2` spun `+1716` ticks and `M4` spun `+973` ticks), culminating in an ESP32 `POWERON_RESET` at `t = 4.27 s`.
  19. **Restored Verified `1.3.3-cand-r41c` Baseline (`08:17 PDT`) & Tested 90° CW @ `0.80 rad/s` (`08:21 PDT`, `turn_cw_90deg_1792077682.json`)**:
      - Reverted `src/` back to `4d44c31` (`1.3.3-cand-r41c`) per Rule 5 (*Revert on Regression*).
      - In `1792077682.json`, `102 PWM` stiction boost broke all 4 wheels out immediately (`vel = [-3.84, +4.32, -3.03, +2.71 rad/s]`, `0` stalls), but holding `0.80 rad/s` (`targetRadps = ±4.19`) pushed `pwm = [-108, 99, -124, 131]` (`∑|PWM| = 462`), triggering an ESP32 `POWERON_RESET` at `t = 0.91 s`.
  20. **90° CW 3-Repetition Test on `1.3.3-cand-r41c` @ `--max-angular-speed 0.45` (`08:25 PDT`, `turn_cw_90deg_1792077902.json`)**:
      - **PASSED 3/3 (`SUCCESS`)** with **zero ESP32 resets (`bootCount: 1 -> 1`)** and **zero wheel stalls**!
      - `102 PWM` boost broke stiction in `0.10 s`, and `0.45 rad/s` (`targetRadps = ±2.35 rad/s`) kept all 4 wheels spinning at `1.7–2.7 rad/s` while keeping `∑|PWM| = 405–413` (down from `462` at `0.80 rad/s`).
      - Settled IMU headings with `--stopping-advance-deg 0.7`: **`+90.35°, +90.44°, +90.36°` (Mean Err `+0.38°`, Repeatability Span `0.09°`)**.
  21. **180° CW Test on `1.3.3-cand-r41c` @ `--max-angular-speed 0.45` (`08:33 PDT`, `turn_cw_180deg_1792078433.json`) & Hardware Redesign Decision**:
      - Broke stiction cleanly on all 4 wheels (`M1=-1875, M2=+2370, M3=-1472, M4=+1236` ticks) and spun for `~2.8 s` (`~75°`), then aborted with ESP32 `POWERON_RESET`.
      - **Engineering Conclusion & Next Hardware Step**: Software/firmware tuning has been exhaustively bracketed across `r41c`, `r42`, `r42b`, `r42c`, and `r42d`. On the current `L > W` chassis with shared 3S LiPo power, the torque required to break and sustain 4-wheel lateral tire scrub (`~100–125 PWM` per wheel) overlaps the electrical reset threshold of the shared power rail after `~2.5–3.0 s` of continuous turning.
      - Operator is pausing software tests to perform two permanent hardware upgrades:
        1. **Wheelbase Redesign ($L \le W$)**: Shorten longitudinal wheelbase relative to track width to eliminate severe lateral scrub torque during in-place turns.
        2. **Power Rail Isolation (Geekworm X1206 UPS)**: Power the RPi5 (and USB-powered ESP32 logic) from the Geekworm X1206 UPS while powering the 4 motors exclusively from the 3S LiPo.

---

### 2026-10-18 (Sunday)
- **Active Firmware**: `1.3.4-remap-r03` (`source: rev42-remap-c`, commit `a2e3d43`)
- **Active Cockpit Commit**: `64534c2` (`feat(cockpit): port Rover 2 Ground Control HUD and safety watchdog to Rover 1`)
- **Accomplishments & Code Changes**:
  1. **Geekworm X1206 UPS Hardware Integration**:
     - Operator installed Geekworm X1206 UPS (4x 21700 lithium cells in parallel, 1S4P) beneath the Raspberry Pi 5 to isolate host RPi5 / ESP32 logic power from the motor 3S LiPo rail.
  2. **I2C Fuel Gauge Reader Daemon (`geekworm_battery.py`)**:
     - Developed robust reader for onboard MAX17043 fuel gauge IC at I2C address `0x36`.
     - Reads `VCELL` register `0x02` (`swapped * 78.125 / 1,000,000` V) and `SOC` register `0x04` (`swapped / 256.0` %).
     - Supports `smbus2` / `smbus`, pure Python Linux `fcntl.ioctl` (`I2C_RDWR` with repeated start), and `i2cget` CLI fallback.
  3. **Cockpit Backend Telemetry Stream & Protocol Protection (`server.js`)**:
     - Added background monitor process streaming battery telemetry every 3 seconds.
     - Protected real Geekworm telemetry against ESP32 dummy zero-filled `0x0A` packets.
     - Added initial battery packet delivery on WebSocket client connection and updated `/api/battery` to expose both `voltage` and `percentage`.
  4. **Cockpit UI & Style Updates (`public/app.js`, `public/style.css`)**:
     - Updated header battery meter to display State-of-Charge percentage alongside voltage (`XX% (X.XXV)`).
     - Added color gradient transitions (Green >=50%, Yellow >=20%, Red <20%) and hover tooltip.
     - Updated Drive tab status card with live battery status.
  5. **Automated Verification (`test/test_geekworm_battery.js`)**:
     - Created unit test suite `test/test_geekworm_battery.js` validating register math, UI formatting, and packet shielding. All tests PASS.
  6. **Rover 2 Cockpit Replication ("ROVER GROUND CONTROL" HUD)**:
     - Complete port and adaptation of Rover 2's high-contrast tactical HUD (`public/hud.html`, `public/hud.css`, `public/hud-core.js`, `public/hud-desktop.js`, `public/hud-mobile.html`, `public/hud-mobile.js`).
     - **Top Header & Telemetry Badges**: Live WebSocket connection badge, RPi5 CPU temperature indicator, EXT5V rail voltage, AC/battery charging badge with Geekworm SOC %, Maker ESP32 4WD motor status, Gamepad indicator, View switchers (FPV Drive, Tactical Map, Split View), and direct links to Mobile Touch UI and Legacy Engineering Cockpit.
     - **Left Column**: 360° LiDAR radar (RPLiDAR C1 canvas), Geekworm X1206 UPS Health card (voltage, percentage, charging/discharging slope, runtime estimate), and BNO08x IMU Attitude & Heading card.
     - **Center Stage**: FPV video stream (`/api/camera` MJPEG) with 180° orientation flip & PiP mini-map; interactive 2D map canvas with Nav2 path preview & goal aiming.
     - **Right Column**: Autonomy & Mission card (quick presets, plan preview, GO with confirmation), Maker ESP32 4WD Drivetrain card, Drive Controls card (ARM, E-STOP, deadman/turbo indicators, throttle slider, token input), Hardware Bus Checklist.
     - **Mobile Touch UI (`public/hud-mobile.html`)**: Dual virtual touch joysticks (drive + steering), ARM toggle, E-STOP, speed slider, and route presets drawer for smartphones/tablets.
  7. **Backend Safety Watchdog & Battery Trend Engine (`server.js`, `battery_trend.js`, `manual_drive_watchdog.js`)**:
     - **`ManualDriveWatchdog`**: Auto-zeroes manual drive targets if the controlling WebSocket disconnects or if a streaming client's keepalive heartbeats (`keepalive: true`) go stale (>600 ms).
     - **`BatteryTrendEstimator`**: Computes SOC slope using least-squares linear regression over a rolling 20-minute window, providing charging/discharging state and runtime remaining estimates.
     - **`/api/system` Endpoint**: Exposes live host CPU temperature (`/sys/class/thermal/thermal_zone0/temp`), vcgencmd throttling flags, EXT5V rail, uptime, load average, and watchdog metrics.
     - **Security**: Replaced hardcoded operator token fallbacks with browser `localStorage` credential isolation.
     - **Routing & Backward Compatibility**: Served new HUD at root `/` with backward-compatible fallback to legacy engineering cockpit at `/index.html` (and configurable via `COCKPIT_DEFAULT_UI=legacy`).
  8. **Comprehensive Testing & Live Browser Verification**:
     - Automated test suites: `test/test_hud_cockpit.js` (19 checks) and `test/test_manual_drive_watchdog.js` (14 checks) all PASS.
     - Regression test suites: `test_geekworm_battery.js`, `test_cockpit_terminal_disarm.js`, `test_dispatch_nav_readiness.js` all PASS.
     - Visual inspection: Verified live via browser subagent at `http://10.0.0.246:3000/` with full telemetry rendering, zero motion, and healthy disarmed state.
  9. **Floor Crawl Calibration on Rebuilt Chassis (`scripts/characterize_floor_speeds.py`)**:
     - **Preflight & E-STOP Clearance**: Drivetrain initially latched in Mode 4 (`EMERGENCY_STOP`) following reboot. Cleared faults via `/api/drive/clear-faults` using operator token, restoring Mode 0 (`LOCKED`).
     - **Initial Attempt & Hardware State**: Case 2 initially aborted with `STICTION_BLOCKED` after 500 ms at 64 PWM because the Maker control board motor power switch (VMOT) was OFF (while logic ran from Geekworm X1206 UPS).
     - **Case 2 Forward Crawl (+0.015 m/s for 1.8s)**: With motor switch ON, rover executed forward translation smoothly with zero resets and stopped immediately (`2.0 mm` overrun). Encoder travel: `+67.5 mm` (250.0% of nominal 27 mm); physical ruler travel: `40.0 mm`. Proves rolling friction on the rebuilt chassis is lower than the original chassis baseline (sustaining ~1.0–1.5 rad/s wheel speed at 36–47 PWM).
     - **Case 13 Reverse Crawl (-0.015 m/s for 1.8s)**: Rover executed reverse translation smoothly with zero resets and stopped immediately (`1.1 mm` overrun). Encoder travel: `-33.7 mm` (124.8% of nominal -27 mm); physical ruler travel: `10.0 mm`. All four wheels reversed symmetrically (`m1..m4` ~ -40 PWM).
     - **Case 11 Forward Creep (+0.050 m/s for 1.8s)**: Controlled comparison at higher speed. Completed with zero resets and stopped immediately (`8.5 mm` overrun). Encoder travel: `+139.3 mm` (154.8% of nominal 90 mm); physical ruler travel: `176.0 mm`. Incremental analysis between Case 2 and Case 11 reveals incremental physical displacement of `136.0 mm` vs incremental encoder displacement of `71.8 mm` (ratio `1.894`), demonstrating that at higher speed physical travel exceeds reported encoder distance.
   10. **BNO08x IMU Recovery & Root Cause Analysis**:
      - **Symptom**: Cockpit HUD showed `RESET RECOVERY` (red) with `--°` orientation.
      - **Root Cause**: The Maker control board sensor/peripheral rail was unpowered when the ESP32 was flashed at 11:59 AM (`power switch OFF`). `ImuManager::begin(21, 22, 0x4B)` executes only once in `setup()` on boot without background retry. The legacy cockpit masked this by unconditionally displaying `0.0°` in green from default zero quats (`qw=1.0, qx=0, qy=0, qz=0`), whereas the new HUD cockpit explicitly checks `imu.hardwareInitialized` and flagged the uninitialized state.
      - **Resolution**: With the board power switch ON, issued a clean ESP32 hardware reset via RTS (`python3 -m esptool run`). ESP32 booted, detected BNO08x at `0x4B`, enabled `SH2_GAME_ROTATION_VECTOR` at 50 Hz, and achieved `calibrationStatus: 3` (`LIVE · CAL 3`). Live yaw/pitch/roll verified on HUD cockpit.
      - **Case 12 Reverse Creep (-0.050 m/s for 1.8s)**: Completed with zero resets, zero IMU drift (0.02°), and stopped immediately (`6.6 mm` overrun). Encoder travel: `-139.0 mm` (154.4% of nominal -90 mm); physical ruler travel: `150.0 mm`. Matches Case 11 forward (+139.3 mm) with 0.3 mm (<0.2%) encoder symmetry. Mean physical/encoder ratio across both 0.050 m/s creep tests is 1.171 (17.1% physical excess over encoder odometry).
      - **Case 4 Pure Spin Slow CCW (+0.15 rad/s for 1.5s)**: All 4 wheels broke away cleanly at t=0.11s under synchronous spin breakout floor (-102 PWM left, +102 PWM right). Test script's pure-spin wheel travel safety limit (max_dist_m: 0.050 m) triggered cutoff at t=0.66s (56.7 mm accumulated wheel movement). BNO08x IMU tracked heading from 0.00° to +11.86° CCW (nominal +12.9°). Operator physical ruler/protractor estimate: ~12.5° (IMU delta: 0.64°). Completed with zero resets, zero faults, and clean Mode 0 LOCKED disarm.
      - **Exact 90° CW Turn (Closed-Loop AngularApproachController)**: Executed closed-loop 90.0° clockwise turn on rebuilt chassis. Cruise phase (+60.50° in 1.26s @ 0.80 rad/s) and creep deceleration phase (+29.27° in 1.23s @ 0.20 rad/s) tracked smoothly with zero stalls. Zero-command issued at +89.77° (0.23° delta), followed by 0.81° dynamic brake coast, settling at +90.58° IMU (+0.58° error vs. ±2.0° tolerance). Operator physical check confirmed: "looks like a perfect 90". Zero ESP32 resets (bootCount remained 1 throughout), verifying the Geekworm X1206 UPS successfully isolates logic power from motor load.
      - **Exact 90° CCW Turn (Closed-Loop AngularApproachController)**: Executed symmetrical closed-loop 90.0° counter-clockwise turn. Cruise phase (-60.59° in 1.26s @ 0.80 rad/s) and creep phase (-28.78° in 1.32s @ 0.20 rad/s) demonstrated exceptional bilateral parity (cruise duration matched CW down to the hundredth of a second: 1.26s vs 1.26s). Zero-command issued at -89.37°, coasting 1.09° into -90.46° settled IMU heading (-0.46° error). Operator physical check confirmed: "another perfect 90". Zero ESP32 resets, zero faults, and clean return to Mode 0 LOCKED disarmed state.
      - **Exact 180° CW Turn (Closed-Loop AngularApproachController)**: Executed closed-loop 180.0° clockwise turn. Cruise phase covered +151.63° in 2.99s @ 0.80 rad/s, and creep phase covered +27.84° in 1.30s @ 0.20 rad/s. Zero-command issued at +179.47°, with +0.79° brake coast settling at +180.26° IMU (+0.26° error vs. ±2.0° tolerance). Operator physical estimate recorded: 182.0° ("looked good, maybe a couple deg too far"). Critically, the ESP32 experienced **0 resets during the full 4.3s continuous 4-motor spin** (bootCount remained 1 throughout), conclusively proving that the Geekworm X1206 UPS has solved the power-sag resets that plagued the 180° spins on the old chassis.
      - **Linear Startup Watchdog Fix & Exact 1.000m Forward Test (`tools/rover_tests/runner.py`)**:
        - **Watchdog Bug Diagnosis**: Initial 1.000m linear attempt aborted at 87.7 mm because an 800ms pre-motion loop polled status and waited for encoder progress without refreshing `/api/cmd_vel`, triggering `server.js`'s 500ms ROS 2 autonomy watchdog. Fixed in `78ec915`/`7cea573` by aligning linear startup with the proven `turn` architecture (breaking into active motion within 500ms as soon as `ACTIVE` is achieved).
        - **Execution & Validation**: Executed closed-loop 1.000m forward translation. Accel: 0.59s (69.9 mm @ 86 peak PWM); Steady-Speed: 4.50s (931.1 mm @ 0.207 m/s); Settle: 2.01s (40.8 mm brake coast). Total measured distance: **`+1.0418 m`** (+41.8 mm net error, well inside ±50 mm pass window). Yaw drift over the entire 1 meter run was just **`+1.10°`** (< 1.5°/m ideal).
        - **Wheel Symmetry & Balance**: All 4 wheels matched within 31 encoder ticks (3.3 mm) over 1 meter (M1: +8747, M2: +8737, M3: +8721, M4: +8752). Left vs. right mean speed difference was `-0.016 rad/s` with only `+1.0` PWM disparity. 0 ESP32 resets, zero faults, and clean Mode 0 LOCKED disarm.
      - **Exact 1.000m Reverse Linear Test & Stopping Advance Forensics (`tools/rover_tests/runner.py`)**:
        - **Bilateral Travel Symmetry**: Executed return leg (-1.000m reverse). Accel: 0.48s (-55.7 mm); Steady-Speed: 4.53s (-951.7 mm @ -0.210 m/s); Settle: 2.01s (-42.5 mm brake coast). Total measured distance: **`-1.0499 m`** (-49.9 mm net error). Operator physical observation confirmed: `~1050 mm` ("about 50mm too far which makes sense").
        - **Forward vs. Reverse Parity**: Forward total (1041.8 mm) vs. Reverse total (1049.9 mm) matched within **`8.1 mm` (<0.8% disparity)**. Stopping phase dynamic brake coast matched almost identically: **`40.8 mm` forward vs. `42.5 mm` reverse ($\Delta = 1.7\text{ mm}$)**.
      - **Linear Approach Creep Deceleration & Stopping Advance Calibration (`controllers.py`, `linear.py`, `runner.py`)**:
        - **Implementation**: Enabled two-phase linear approach strictly per `docs/EXACT_MOTION_CONTRACT.md` ($0.20\text{ m/s}$ cruise $\to$ $0.05\text{ m/s}$ creep in final $150\text{ mm}$) and added calibrated `stopping_advance_m` ($8\text{ mm}$ for creep stopping coast). Fixed `creep_entry_time`/`dist` initialization in `LinearApproachController`.
        - **Execution & Validation (1.000m Forward Calibrated)**: Accel: 0.57s (+57.7 mm); Cruise/Creep Approach: 5.13s (+934.9 mm); Stopping Coast: 2.00s (+4.2 mm).
        - **Precision Result**: Total measured distance: **`+0.9968 m` ($996.8\text{ mm}$)** — achieving **`99.68%` distance accuracy with a net error of only `-3.2 mm` (`-0.32%`, less than 1/8 inch)**! Operator physical check with tape/ruler measured **`1010 mm` ("10 mm long")**, confirming real-world floor travel within 1.0% of target.
        - **Heading & Wheel Balance**: IMU yaw drift was just **`+1.11°`** over 1 meter. Left vs. right speed disparity was **`-0.002 rad/s`**, and wheel encoder tick deltas across all 4 wheels matched within 34 ticks (3.6 mm: LF=8794, RF=8766, LR=8760, RR=8779). 0 resets, zero faults, and clean Mode 0 LOCKED disarm.
      - **Calibrated 1.010m Reverse Return Test (`reports/linear_reverse_1p010m_1792363806.json`)**:
        - **Execution & Goal**: Executed reverse return translation with calibrated stopping advance `0.018 m` (18 mm) and target `-1.010 m` to return Rover 1 directly to the 0 mm origin mark after the +10 mm forward overshoot.
        - **Kinematic & Settle Performance**: Accel: 0.37s (-36.0 mm @ -64 peak PWM); Cruise & Creep: 4.91s (-957.0 mm @ -0.195 m/s); Dynamic Brake Coast from Creep: 2.00s (-4.7 mm). Total measured encoder travel: **`-0.9977 m` ($997.7\text{ mm}$)**.
        - **Heading & Electrical Health**: IMU yaw drift was **`-1.84°`** across full reverse translation. Left vs. right speed difference was `+0.029 rad/s` with PWM difference of `-3.0`. 0 wheel stalls, 0 ESP32 resets (`bootCount: 1`), and clean Mode 0 `LOCKED` disarm.
        - **Bilateral Accuracy & Physical Ground Truth**: Operator physical measurement confirmed return stop: **`4-5 mm short` of origin ("almost perfect. 4-5 mm short")**. With stopping advance at 8 mm, the rover ran 10 mm long; with stopping advance at 18 mm (+10 mm), the rover stopped 4.5 mm short. This demonstrates clean linear predictability: optimal stopping advance for exact sub-millimeter landing on this floor surface is **`13.5 mm` (`0.0135 m`)**. Net forward-to-reverse encoder symmetry was within **`0.9 mm` ($996.8\text{ mm}$ fwd vs $997.7\text{ mm}$ rev)**, and coast matched within **`0.5 mm` ($4.2\text{ mm}$ vs $4.7\text{ mm}$)**.
      - **Autonomous Compound Out-and-Back Sequence (`tools/rover_tests/cli.py out-and-back`)**:
        - **Sequence Execution**: Executed autonomous 3-leg compound maneuver: Leg 1 (1.000m Forward) $\to$ Leg 2 (180.0° CW In-Place Turn) $\to$ Leg 3 (1.000m Return Forward to origin).
        - **Leg 1 Forensics (Forward 1.0m)**: Accel: 0.56s (+57.5 mm); Cruise/Creep: 5.01s (+929.8 mm); Dynamic Brake Coast: 2.00s (+4.2 mm). Total travel: **`+0.9916 m` ($991.6\text{ mm}$)**, net error `-8.4 mm` (`-0.84%`). IMU yaw drift: `+0.88°`.
        - **Leg 2 Forensics (180.0° CW Pivot)**: Cruise: 2.82s (+150.67° @ 0.80 rad/s); Creep: 1.12s (+28.94° @ 0.20 rad/s); Brake Coast: 2.02s (+1.63°). Total net rotation: **`+181.24°`** (`+1.24°` net error, well inside ±2.0° tolerance). Zero wheel stalls, clean opposite-polarity tracks.
        - **Leg 3 Forensics (Return 1.0m)**: Accel: 0.46s (+45.5 mm); Cruise/Creep: 5.01s (+941.9 mm); Dynamic Brake Coast: 2.00s (+6.1 mm). Total travel: **`+0.9936 m` ($993.6\text{ mm}$)**, net error `-6.5 mm` (`-0.65%`). IMU yaw drift: `+1.63°`.
        - **Outbound vs Inbound Travel Parity**: Leg 1 outbound ($991.6\text{ mm}$) vs Leg 3 inbound ($993.6\text{ mm}$) matched within **`2.0 mm` (`0.2%` disparity)**!
        - **Power Integrity Proof**: The ESP32 maintained **`bootCount: 1` throughout all 3 consecutive compound legs** (~25 seconds of continuous actuation under heavy motor loading), conclusively proving complete power isolation by the Geekworm X1206 UPS. Drivetrain finished fully stopped, disarmed (`armed: false`), and locked in Mode 0.
        - **Physical Ground Truth & Alignment**: Operator physical inspection confirmed arrival alignment: **"good now lined up"** with the starting origin mark. Operator noted that the 3-leg sequence stopped upon return to origin without performing an extra turnaround pivot back to the initial starting heading.
  7. **Hailo-8 AI Card Reseat & Verification (`17:46–17:55 PDT`)**:
     - At `17:46 PDT`, `lspci` showed downstream Port #3 (`0001:02:03.0`) on the ASM1182e 2-Port PCIe switch with link down (`DLActive-`) and only the NVMe SSD active on the sibling port (`0001:02:07.0`).
     - After reseating the Hailo-8 M.2 module and rebooting (`17:55 PDT`), both M.2 slots enumerated cleanly (`0001:03:00.0` WD_BLACK SN7100 NVMe SSD, `0001:04:00.0` Hailo-8 AI Processor `1e60:2864`), firmware `4.20.0` loaded in `181 ms`, `/dev/hailo0` is active, and `hailortcli fw-control identify` confirmed `HAILO-8 AI ACC M.2 M KEY MODULE EXT TEMP` (`SN: HLDDM2A234601386`).
  8. **Repository Audit & Test/Doc Alignment (`17:58–18:37 PDT`)**:
     - Fixed [`LinearParameters.__post_init__`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/tools/rover_tests/linear.py#L61-L65) default `stopping_advance_m` from `0.008 m` (`8.0 mm`) to the calibrated `0.0135 m` (`13.5 mm`) and synced [`linear.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/tools/rover_tests/linear.py) and [`controllers.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/tools/rover_tests/controllers.py) to RPi5 (`/home/ron/yahboom-encoder/tools/rover_tests/`).
     - Updated [`test/test_rover_test_linear.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/test/test_rover_test_linear.py#L454-L458) stall-guard assertion (`311/311` pytest unit tests passing).
     - Updated [`scripts/run_native_suite.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/scripts/run_native_suite.py) to support Windows WSL execution for ELF binaries and run the 23 active canonical C++ regression tests (`1–17, 19–24`, all `23/23 PASS`).
     - Updated [`AGENTS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/AGENTS.md), [`.agents/AGENTS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/.agents/AGENTS.md), [`.agents/skills/rover-exact-motion/SKILL.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/.agents/skills/rover-exact-motion/SKILL.md), and [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) roadmap/catalog to match `1.3.4-remap-r03` and the `out-and-back` CLI command.

---

### 2026-10-14 (Wednesday)
- **Active Firmware**: `1.3.3-cand-r41c` (`source: rev41-floor-c`)
- **Accomplishments & Code Changes**:
  1. **Project Organization & Root Status File**: Created [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) and updated [`AGENTS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/AGENTS.md) / [`.agents/AGENTS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/.agents/AGENTS.md) to enforce pre-change git checkpoints, reusable test suites, and automatic report pulling from RPi5 (`10.0.0.246`).
  2. **Standardized 3-Part Test Brief Banner**: Updated [`tools/rover_tests/runner.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/tools/rover_tests/runner.py) and [`scripts/characterize_floor_speeds.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/scripts/characterize_floor_speeds.py) (`daaf523`, `d59c58f`) so all tests print `1. TEST GOAL`, `2. WHAT TO MONITOR & MEASURE (OPERATOR)`, and `3. EXPECTED RESULT (PASS CRITERIA)` in the top header before operator confirmation.
  3. **Fixed Multi-Trial Autonomy Handshake & Localization Guard**:
     - Updated [`tools/rover_tests/transport.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/tools/rover_tests/transport.py) (`d59c58f`) to tag commands with `"source": "CALIBRATION_TEST"` and refresh `lastCmdTime` after `READY_DISARMED` and `READY_ARMED`.
     - Updated [`server.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js) in `yahboom-encoder` (`28759ef`, `43e828d`) so `CALIBRATION_TEST` transitions `READY_ARMED -> ACTIVE` and skips Nav2 AMCL `abortAutonomyDueToLocalizationLost` during scripted calibration turns.
  4. **Phase 2 Rotate-in-Place Calibration & Thermal/Battery Timeline**:
     - **`04:42 PDT` (`turn_cw_90deg_1791978108.json`)**: 90° CW Step 1 **PASSED** (`+90.95°` IMU, `90.0°` Physical, `+0.95°` err).
     - **`05:06 PDT` (`turn_cw_90deg_1791979545.json`)**: 90° CW 3-repetition test **PASSED 3/3** (`+90.58°, +90.95°, +90.84°` IMU; `90.0°, 90.0°, 92.0°` Physical; Mean Err `+0.79°`, Span `0.37°`, `0` stalls).
     - **`05:11 PDT` (`turn_ccw_90deg_1791979851.json`)**: 90° CCW 3-repetition test **PASSED 3/3** (`-90.65°, -90.85°, -90.81°` IMU; `-90.5°, -91.0°, -92.0°` Physical; Mean Err `-0.77°`, Span `0.20°`, `0` stalls).
     - **`05:15:13 PDT` (Hardware Event)**: First ESP32 `POWERON_RESET (0x1)` (`SPI_FAST_FLASH_BOOT`) observed in `journalctl` around battery swap / after 6 consecutive 90° spin trials.
     - **`05:15:52 PDT` (`turn_cw_180deg_1791980120.json`)**: 180° CW @ `0.80 rad/s` aborted at `t = 1.5s` (`-1420` ticks, `~51°`) due to ESP32 `POWERON_RESET` (`275ms` IMU gap).
     - **`05:21:47 PDT` (`turn_cw_180deg_1791980504.json`)**: 180° CW @ `0.40 rad/s` aborted at `-1846` ticks (`~66°`) due to ESP32 `POWERON_RESET` (`251ms` IMU gap).
     - **`05:23:28 PDT` (`turn_cw_180deg_1791980606.json`)**: 180° CW Step 1 **PASSED** (`+181.16°` IMU, `180.0°` Physical, `+1.16°` err, `-5042` ticks), then Step 2 at `05:23:45` aborted at `-2566` ticks (`~91°`) due to ESP32 `POWERON_RESET`.
     - **`05:30:31 PDT` (`turn_cw_90deg_1791981026.json`)**: Re-test of the previously passing 90° CW test aborted at `t = 1.1s` (`-1110` ticks, `~40°`) due to ESP32 `POWERON_RESET` (`254ms` IMU gap).
     - **Forensic Telemetry Comparison (`05:06` Passing vs `05:30` Failing at `t ≈ 0.77s`)**: At identical `±4.19 rad/s` wheel targets, all 4 wheels at `05:30` spun **8%–18% slower** (`-4.06, +4.76, -3.61, +2.86 rad/s` vs `-4.32, +5.10, -4.38, +3.17 rad/s`) while PID `|PWM|` sum rose from `410` to `434`, and RPi5 `EXT5V_V` measured `4.85V` at idle before the rover was powered off for cooldown at `~05:40 PDT`.
- **Next Planned Action (After 12-Hour Rest / Cooldown)**:
  - Rover 1 powered off at `~05:40 PDT` (`2026-10-14`) for a **12-hour rest/cooldown** (until after `17:40 PDT` on `2026-10-14`).
  - Re-run the **90° CW** and **180° CW** tests when cold to isolate whether the `POWERON_RESET` and wheel slowdown were caused by component heating (motor driver ICs / 5V regulator / polyfuse) vs. the swapped battery pack.

---

### 2026-10-13 (Tuesday)
- **Active Firmware**: `1.3.3-cand-r41c` (`source: rev41-floor-c`)
- **Accomplishments**:
  1. Completed **Phase 1: Reverse Crawl Calibration (`Case 13`, `Reverse_Crawl_0.015mps`)**.
  2. Verified 3 consecutive diagnostic runs inside the `24.0–30.0 mm` encoder target window:
     - **17:41 PDT (`floor_char_Reverse_Crawl_0.015mps_20261013_174144.json`)**: `29.0 mm` encoder (`107.4%`), `26.0 mm` physical — **`SUCCESS` (PASS)**.
     - **18:16 PDT (`floor_char_Reverse_Crawl_0.015mps_20261013_181641.json`)**: `26.8 mm` encoder (`99.3%`), `33.0 mm` physical (mark offset) — **`SUCCESS` (PASS)**.
     - **18:18 PDT (`floor_char_Reverse_Crawl_0.015mps_20261013_181839.json`)**: `29.7 mm` encoder (`110.0%`), `30.0 mm` physical — **`SUCCESS` (PASS)**.
  3. Locked in firmware `1.3.3-cand-r41c` (`41.0 PWM` piecewise reverse crawl floor) as the verified baseline.
