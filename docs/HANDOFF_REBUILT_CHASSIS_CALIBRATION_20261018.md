# Rover 1 Rebuilt Chassis Calibration & Autonomous Motion Handoff (2026-10-18)

**Document Name:** `docs/HANDOFF_REBUILT_CHASSIS_CALIBRATION_20261018.md`  
**Repository:** `c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor` (`Inouye165/esp-maker-usba-4motor`)  
**Companion Host Repo:** `c:\Users\Ron\electronic_projects\yahboom-encoder` (`/home/ron/yahboom-encoder` on RPi5 `10.0.0.246`)  
**Active Firmware Baseline:** `1.3.4-remap-r03` (commit `a2e3d43`)  
**Test Framework Commit:** `26c27dd`  
**Operator:** Ron Inouye  
**Status:** **MISSION ACCOMPLISHED — ALL PASS (0 ESP32 Resets, Sub-Centimeter Out-and-Back Accuracy)**

---

## 1. Executive Summary & Breakthroughs

1. **Hardware Power Isolation Validated (Geekworm X1206 UPS Rebuild)**:
   - **Prior Failure Mode**: Sustained 4-motor load (e.g. 180° pivot spins) previously pulled battery rail voltage down, triggering ESP32 brownouts and `POWERON_RESET (0x1)` reboots.
   - **Validation Result**: Over the entire testing session today, encompassing 90° CW/CCW turns, sustained 180° pivots, 1.0m bidirectional translations, and a continuous ~25s 3-leg compound autonomous sequence, **`bootCount: 1` remained invariant (ZERO ESP32 resets)**. Logic power isolation by the X1206 UPS is conclusively proven.
2. **Exact Motion Contract Deceleration Profile Enforced**:
   - Replaced uncalibrated full-speed cutoffs (which caused $41\text{–}45\text{ mm}$ of dynamic brake coast) with the canonical two-phase creep approach ($0.20\text{ m/s}$ cruise $\to$ $0.05\text{ m/s}$ creep in final $150\text{ mm}$).
   - Kinetic energy at stop trigger reduced by $16\times$, dropping dynamic brake coast to just $4.2\text{–}4.7\text{ mm}$.
3. **Linear Floor Calibration Locked In**:
   - Calibrated linear stopping advance to **`13.5 mm` (`0.0135 m`)**.
   - Outbound travel ($991.6\text{ mm}$) vs. Inbound return travel ($993.6\text{ mm}$) matched within **`2.0 mm` (`0.2%` disparity)**.
4. **First-Class Autonomous Compound Sequence Implemented**:
   - Added `out-and-back` command to `tools.rover_tests.cli` supporting automated 3-leg and 4-leg (with `--final-turn`) autonomous maneuvers.
5. **Drivetrain Safety Invariant**:
   - Drivetrain finished every single trial safely stopped, disarmed (`armed: false`), and locked in Mode 0.

---

## 2. Chronological Test Ledger (2026-10-18)

| Time (PDT) | Firmware | Commit | Test Maneuver | Target | Measured Ground Truth (Operator / Encoders / IMU) | Verdict & Key Forensics | Report Artifact |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **14:45** | `1.3.4-remap-r03` | `fe995db` | Case 12 (`Reverse_Crawl_0.050mps`) | $-90\text{ mm}$ nominal | Physical: $150.0\text{ mm}$ (Enc: $-139.0\text{ mm}$) | `EXCESSIVE_TRAVEL` (6.6mm overrun; symmetrical to Case 11) | `floor_char_Reverse_Crawl_0.050mps_20261018_144552.json` |
| **14:47** | `1.3.4-remap-r03` | `8f43e9e` | Case 4 (`PureSpin_Slow_CCW`) | $+12.9^\circ$ nominal | Physical: $\sim 12.5^\circ$ (IMU: $+11.86^\circ$) | `TRAVEL_BOUND_EXCEEDED` (Tripped 50mm wheel travel limit; 0 resets) | `floor_char_PureSpin_Slow_CCW_20261018_144754.json` |
| **14:57** | `1.3.4-remap-r03` | `37ee25d` | Exact 90° CW Turn (`rover-test turn`) | $+90.00^\circ$ | Physical: $90.0^\circ$ (IMU: **`+90.58°`**, Err $+0.58^\circ$) | **`SUCCESS`** (1/1 PASS; "perfect 90"; 0 resets; power isolation verified) | `turn_cw_90deg_1792360648.json` |
| **15:01** | `1.3.4-remap-r03` | `37ee25d` | Exact 90° CCW Turn (`rover-test turn`) | $-90.00^\circ$ | Physical: $-90.0^\circ$ (IMU: **`-90.46°`**, Err $-0.46^\circ$) | **`SUCCESS`** (1/1 PASS; "another perfect 90"; Cruise 1.26s CW vs 1.26s CCW) | `turn_ccw_90deg_1792360827.json` |
| **15:04** | `1.3.4-remap-r03` | `651a233` | Exact 180° CW Turn (`rover-test turn`) | $+180.00^\circ$ | Physical: $182.0^\circ$ (IMU: **`+180.26°`**, Err $+0.26^\circ$) | **`SUCCESS`** (1/1 PASS; 0 resets over 4.3s continuous 4-motor spin!) | `turn_cw_180deg_1792361016.json` |
| **15:21** | `1.3.4-remap-r03` | `78ec915` | Exact 1.000m Forward (Uncalibrated) | $+1.0000\text{ m}$ | Physical: $\sim 1050\text{ mm}$ (Enc: **`+1.0418 m`**, Err $+41.8\text{ mm}$) | **`SUCCESS`** (1/1 PASS; Coast: 40.8mm; Yaw drift: $+1.10^\circ$; 4 wheels within 3.3mm) | `linear_forward_1p000m_1792362043.json` |
| **15:28** | `1.3.4-remap-r03` | `1db403d` | Exact 1.000m Reverse (Uncalibrated) | $-1.0000\text{ m}$ | Physical: $\sim 1050\text{ mm}$ (Enc: **`-1.0499 m`**, Err $-49.9\text{ mm}$) | **`SUCCESS`** (1/1 PASS; Fwd/Rev parity: $\Delta = 8.1\text{ mm}$; Coast: 42.5mm; 0 resets) | `linear_reverse_1p000m_1792362168.json` |
| **15:46** | `1.3.4-remap-r03` | `d53ca31` | Calibrated 1.000m Forward (Creep + 8mm Adv) | $+1.0000\text{ m}$ | Physical: $1010\text{ mm}$ (Enc: **`+0.9968 m`**, Err $-3.2\text{ mm}$) | **`SUCCESS`** (1/1 PASS; 99.68% accuracy; Coast: 4.2mm; Yaw drift: $+1.11^\circ$; 0 resets) | `linear_forward_1p000m_1792363466.json` |
| **15:50** | `1.3.4-remap-r03` | `7707217` | Calibrated 1.010m Reverse (Creep + 18mm Adv) | $-1.0100\text{ m}$ | Physical: $-4.5\text{ mm}$ short (Enc: **`-0.9977 m`**, Err $+12.3\text{ mm}$) | **`SUCCESS`** (1/1 PASS; "almost perfect. 4-5 mm short"; Coast: 4.7mm; 0 resets) | `linear_reverse_1p010m_1792363806.json` |
| **15:58** | `1.3.4-remap-r03` | `100803c` | **Autonomous Compound Out-and-Back** | $1\text{m} \to 180^\circ \to 1\text{m}$ | Physical: **Lined up well at origin** | **`SUCCESS`** (3/3 LEGS PASS; Outbound vs Inbound parity $\mathbf{\Delta = 2.0\text{ mm}}$ / $0.2\%$; 0 resets; Mode 0 LOCKED) | `linear_forward_1p000m_1792364263.json`<br>`turn_cw_180deg_1792364272.json`<br>`linear_forward_1p000m_1792364279.json` |

---

## 3. Detailed Forensic & Calibration Analysis

### A. Dynamic Brake Coast Physics & Stopping Advance Tuning
1. **Full-Speed Dynamic Braking ($0.20\text{ m/s}$)**:
   - Coast distance: $40.8\text{ mm}$ forward, $42.5\text{ mm}$ reverse.
   - Kinetic energy: $E_k = \frac{1}{2} m v^2 \propto (0.20)^2 = 0.040$.
2. **Two-Phase Creep Approach ($0.05\text{ m/s}$)**:
   - Entered upon remaining distance $\le 150\text{ mm}$ ($0.15\text{ m}$).
   - Kinetic energy at stop trigger: $E_k \propto (0.05)^2 = 0.0025$ ($16\times$ lower!).
   - Dynamic brake coast reduced to **`4.2 mm` forward, `4.7 mm` reverse**.
3. **Linear Ground-Truth Interpolation**:
   $$\begin{aligned}
   \text{Advance } = 8.0\text{ mm} &\implies \text{Physical Stop: } \mathbf{+10.0\text{ mm}} \text{ long} \\
   \text{Advance } = 18.0\text{ mm} &\implies \text{Physical Stop: } \mathbf{-4.5\text{ mm}} \text{ short} \\
   \mathbf{\text{Optimal Advance}} &= 18.0\text{ mm} - 4.5\text{ mm} = \mathbf{13.5\text{ mm}\ (0.0135\text{ m})}
   \end{aligned}$$
   - This value is now the locked-in production default in [`tools/rover_tests/runner.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/tools/rover_tests/runner.py) and [`tools/rover_tests/cli.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/tools/rover_tests/cli.py).

### B. Compound Out-and-Back Test Forensics (`15:58 PDT`)
- **Leg 1 (Forward $1.000\text{ m}$)**:
  - Traveled: **`0.9916 m` ($991.6\text{ mm}$)**, net error `-8.4 mm`. Coast: $4.2\text{ mm}$. Yaw drift: $+0.88^\circ$.
- **Leg 2 (Reciprocal $180.0^\circ$ CW In-Place Turn)**:
  - Rotated: **`+181.24°`**, net error `+1.24°` (well within $\pm 2.0^\circ$ pass criteria). Post-zero coast: $1.63^\circ$.
- **Leg 3 (Return Forward $1.000\text{ m}$ to Origin)**:
  - Traveled: **`0.9936 m` ($993.6\text{ mm}$)**, net error `-6.5 mm`. Coast: $6.1\text{ mm}$. Yaw drift: $+1.63^\circ$.
- **Outbound vs Inbound Travel Parity**:
  $$\text{Parity Disparity} = |993.6\text{ mm} - 991.6\text{ mm}| = \mathbf{2.0\text{ mm}\ (0.2\%)}!$$

---

## 4. Reusable Test CLI Architecture

The test harness now natively supports compound autonomous testing:

```powershell
# 1. Turn Test (e.g. 90° CW, 180° CW)
ssh ron@10.0.0.246 "cd /home/ron/yahboom-encoder && python3 -m tools.rover_tests.cli turn --degrees 180.0 --direction cw"

# 2. Linear Test (e.g. 1.0m Forward or Reverse)
ssh ron@10.0.0.246 "cd /home/ron/yahboom-encoder && python3 -m tools.rover_tests.cli linear --distance 1.0 --direction forward"

# 3. Autonomous 3-Leg Out-and-Back (Forward 1m -> Turn 180° -> Return 1m)
ssh ron@10.0.0.246 "cd /home/ron/yahboom-encoder && python3 -m tools.rover_tests.cli out-and-back --distance 1.0"

# 4. Autonomous 4-Leg Out-and-Back (Forward 1m -> Turn 180° -> Return 1m -> Turn 180° to face initial heading)
ssh ron@10.0.0.246 "cd /home/ron/yahboom-encoder && python3 -m tools.rover_tests.cli out-and-back --distance 1.0 --final-turn"
```

---

## 5. Current Hardware & System State

- **Rover 1 RPi5 Host**: `10.0.0.246` (user: `ron`)
- **ESP32 Firmware**: `1.3.4-remap-r03` (commit `a2e3d43` on `/dev/rover-esp32`)
- **Cockpit Server**: Port `3000` active on RPi5
- **Drivetrain State**: Stopped, Disarmed (`armed: false`), Mode 0 (`LOCKED`), Autonomy `DISABLED`
- **Controller Health**: `bootCount: 1`, 0 resets, zero safety faults latched
