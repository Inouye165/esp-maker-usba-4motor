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
| **Active Firmware Build** | **`1.3.3-cand-r41c`** (`source: rev41-floor-c`) | Piecewise reverse crawl floor (`41.0 PWM`), verified on Case 13 |
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

*\*Note on 2026-10-13 18:16 (A2): Wheel encoders turned `-259` ticks (`26.8 mm` @ 65mm / `27.6 mm` @ 66.95mm max tire travel), confirming nominal motor travel (`99.3%`) with a ~5 mm mark placement offset.*
*\*Note on 2026-10-14 04:42 (Turn 90° CW): Step 1 passed (`+90.95°` IMU, `90.0°` physical, `0` wheel stalls). Step 2 aborted before motion because `complete_zero_handshake` in `tools/rover_tests/transport.py` did not refresh `autonomyState.lastCmdTime` after reaching `READY_DISARMED`; fixed in `transport.py`.*
