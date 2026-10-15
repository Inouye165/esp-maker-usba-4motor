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
| **Active Firmware Build** | **`1.3.4-cand-r42d`** (`source: rev42d-capped`) | Smooth-turn candidate D: `76/88 PWM` spin floor + `Kd=0` on spin + `90/106 PWM` per-wheel turn cap |
| **Effective Wheel Diameter** | `0.06695 m` (radius `0.033475 m`) | Firmware nominal `0.065 m`; `1974.1667` ticks/rev |
| **Effective Track Width** | `0.3408575433 m` | Calibrated differential track width |
| **IMU Sensor & Mode** | BNO08x — `SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC` (`0x08`) | Non-magnetic yaw only (magnetic orientation forbidden indoors) |

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
  - Forward crawl (`0.015 m/s` to `0.050 m/s`) and reverse crawl (`-0.015 m/s`, Case 13) calibrated and verified under `1.3.3-cand-r41c`.
- [ ] **Phase 2: Rotate-in-Place (In-Place Spin) Calibration & Verification** (`CURRENT FOCUS`)
  - **Step 2A (Low-Speed & Nominal Spin Floor Check)**: Run reusable floor characterization Cases **4, 5, 6, 7** (`PureSpin_Slow_CCW`, `PureSpin_Slow_CW`, `PureSpin_Nominal_CCW`, `PureSpin_Nominal_CW`) to verify smooth breakaway, wheel symmetry, and no stall/overspeed during pure rotation.
  - **Step 2B (Closed-Loop Exact Angle Turns)**: Run `rover-test turn` for **90° CW/CCW** and **180° CW/CCW** using `AngularApproachController` ($0.80\text{ rad/s} \to 0.20\text{ rad/s}$ creep at $30^\circ$) and verify physical angle matches IMU within $\le 2.0^\circ$.
- [ ] **Phase 3: Spin-to-Arc & Curved Transition Verification**
  - Run Case **8** (`Transition_PureSpin_To_Arc`) and Case **1** (`FailedArc_Baseline`) to confirm smooth handoff when Nav2 transitions from in-place rotation (`RotationShimController`) to DWB path following.
- [ ] **Phase 4: Exact Linear Approach Verification**
  - Run `rover-test linear` forward and reverse ($0.5\text{ m}$ and $1.0\text{ m}$) using `LinearApproachController` ($0.20\text{ m/s} \to 0.05\text{ m/s}$ creep at $0.15\text{ m}$).
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
| **Exact Linear Forward (0.5 m)** | `python -m tools.rover_tests.cli linear --distance 0.5 --direction forward --trials 1 --inter-trial-approval` | `LinearApproachController` | Settled error $\le 0.05\text{ m}$ ($50\text{ mm}$) |
| **Exact Linear Reverse (0.5 m)** | `python -m tools.rover_tests.cli linear --distance 0.5 --direction reverse --trials 1 --inter-trial-approval` | `LinearApproachController` | Settled error $\le 0.05\text{ m}$ ($50\text{ mm}$) |

---

### C. End-to-End Autonomous Navigation Suite (`tools/gold_standard_home_test`)
*Run on Raspberry Pi (`docs/GOLD_STANDARD_HOME_TEST.md`):*

| Test Name | Reusable Command | Purpose / Pass Criterion |
| :--- | :--- | :--- |
| **Autonomous Dry-Run** | `./tools/run_gold_standard_home_test.sh --dry-run` | Verifies AMCL localization, HOME pose load, and safety gates without moving |
| **Safety Watchdog Self-Test** | `./tools/run_gold_standard_home_test.sh --test-safety` | Verifies stale-telemetry and reboot fail-safe aborts |
| **Gold Standard HOME Mission** | `./tools/run_gold_standard_home_test.sh --execute` | 2.0 ft outbound $\to$ 180° CW turn $\to$ return HOME within $\le 40\text{ mm}$ and $\le 3.0^\circ$ |

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

*\*Note on 2026-10-13 18:16 (A2): Wheel encoders turned `-259` ticks (`26.8 mm` @ 65mm / `27.6 mm` @ 66.95mm max tire travel), confirming nominal motor travel (`99.3%`) with a ~5 mm mark placement offset.*
*\*Note on 2026-10-14 04:42 (Turn 90° CW): Step 1 passed (`+90.95°` IMU, `90.0°` physical, `0` wheel stalls). Step 2 aborted before motion because `complete_zero_handshake` in `tools/rover_tests/transport.py` did not refresh `autonomyState.lastCmdTime` after reaching `READY_DISARMED`; fixed in `transport.py` and `server.js` (`d59c58f` / `43e828d`).*
*\*Note on 2026-10-14 05:15–05:30 (ESP32 `POWERON_RESET` progression): Even the 90° CW test that passed 3/3 at `05:06` now aborted at `~40°` (`-1110` ticks, `1.1s`) with an ESP32 hardware `POWERON_RESET (0x1)` (`0xE))` boot banner in `journalctl`), confirming progressive power/voltage sag or thermal protection under 4-motor spin load.*

---

## 6. Daily Engineering & Activity Log

### 2026-10-15 (Thursday)
- **Active Firmware**: `1.3.4-cand-r42d` (`source: rev42d-capped`)
- **Accomplishments & Observations**:
  1. **Cold Start Check (`04:36 PDT`)**: Powered on Rover 1 after 22+ hours of rest (`temp=37.8°C`, `EXT5V_V=4.85V`).
  2. **First Cold 90° CW Run (`04:38 PDT`, `turn_cw_90deg_1792064274.json`)**: Aborted at `0.5s` (`-616` ticks, `~22°`) with ESP32 `POWERON_RESET`.
  3. **Manual Drive Exercise (`04:42–04:44 PDT`)**: Operator exercised Rover 1 in Manual Drive (experienced two ESP32 resets at `04:42:26` and `04:42:59` on initial breakaway, then spun smoothly for multiple rotations at `04:43:13`).
  4. **90° CW 3-Repetition Test (`04:45 PDT`, `turn_cw_90deg_1792064709.json`)**: **PASSED 3/3** with `+90.54°, +90.58°, +90.62°` IMU (`90.0°, 90.0°, 90.0°` Physical; Mean Err **`+0.58°`**, Repeatability Span **`0.07°`**, `0` stalls).
  5. **Root Cause of 90° vs. 180° Behavior Confirmed (`04:51` & `04:54 PDT`)**:
     - At `04:51 PDT` (`turn_cw_180deg_1792065104.json`), 180° CW at `0.80 rad/s` aborted after `1.1s` with ESP32 `POWERON_RESET`.
     - At `04:54 PDT` (`turn_cw_180deg_1792065144.json`), 180° CW at `0.30 rad/s` **PASSED 3/3** (`+180.84°, +180.84°, +181.12°` IMU; Mean Err **`+0.93°`**, Repeatability Span **`0.28°`**, `0` stalls, 32s total spin time without a single reset).
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
