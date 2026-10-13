# Rover 1 Reverse Ultra-Low-Speed Crawl Candidate Review Package (r41c)

## Firmware Identity
- **Candidate Version**: `1.3.3-cand-r41c`
- **Candidate Source Tag**: `rev41-floor-c`
- **Hardware Target**: Maker ESP32 (Rover 1)
- **Status**: REVIEW ONLY. Flashing, deployment, and physical motion remain strictly HELD pending review.

## Evidence Baseline & Rationale

### 1. Reverse Floor Piecewise Correction
Physical evidence established:
- $-40\text{ PWM}$ eventually stalls.
- $-41\text{ PWM}$ sustained all 4 wheels moving for the complete 1.404-second delivered hold.
- Late speed passed through target speed region but was still decelerating.
- **Classification**: $-41\text{ PWM}$ is classified as `SUSTAINED_TRANSIENT_NEAR_TARGET`, **not** steady-state converged.

Piecewise Reverse Feedforward Floor:
1. `targetVel == 0`: Zero output through the existing zero-command path.
2. For $0.01 \le |targetVel| \le 0.461538\text{ rad/s}$:
   Reverse feedforward floor is held flat at **$41.0\text{ PWM}$**.
3. For $0.461538 < |targetVel| < 0.80\text{ rad/s}$:
   Continuously linearly interpolate from $41.0\text{ PWM}$ at $0.461538\text{ rad/s}$ to $56.0\text{ PWM}$ at $0.80\text{ rad/s}$:
   $$\text{floor} = 41.0 + \frac{|targetVel| - 0.461538}{0.80 - 0.461538} \times (56.0 - 41.0)$$
4. For $|targetVel| \ge 0.80\text{ rad/s}$:
   Preserve existing behavior exactly (`STRAIGHT_KINETIC_FF_FLOOR = 56.0f`).

### 2. Forward Case 2 Invariance
- Forward crawl formula is bit-for-bit unchanged:
  $$\text{floor} = 37.0 + \frac{targetVel}{0.80} \times (56.0 - 37.0)$$
  Delivering $47.96\text{ PWM} \to 48.0\text{ PWM}$ at $+0.461538\text{ rad/s}$ ($v_x = +0.015\text{ m/s}$).
- Cleaned up redundant constants: defined `ULTRA_LOW_SPEED_MIN_FF_FLOOR_FWD = 37.0f`, `ULTRA_LOW_SPEED_REVERSE_CRAWL_FLOOR_PWM = 41.0f`, `ULTRA_LOW_SPEED_REVERSE_ANCHOR_RADPS = 0.461538f`, and aliased `ULTRA_LOW_SPEED_MIN_FF_FLOOR = ULTRA_LOW_SPEED_MIN_FF_FLOOR_FWD` for documented backward compatibility.

### 3. Closed-Loop Regulation
This is a feedforward baseline, not an output clamp. PID feedback (`Kp = 2.2`, `Straight Crawl Ki = 9.0`) remains fully capable of:
- Reducing output magnitude below $41\text{ PWM}$ during reverse overspeed.
- Increasing output magnitude above $41\text{ PWM}$ during reverse underspeed.

### 4. Firmware Identity NUL Termination
The candidate source tag `rev41-floor-c` is 13 characters, leaving 3 trailing NUL bytes in the fixed 16-byte protocol field (bytes 56-71). This guarantees that reading the source tag as a standard C-string terminates cleanly at byte 69 with zero chance of concatenating with build date data beginning at byte 72.

### 5. Telemetry.cpp and Telemetry.h Placeholder Status
`Telemetry.cpp` and `Telemetry.h` both contain `// Obsolete web status telemetry code removed in Phase 1\n` (58 bytes). They are intentional obsolete-code placeholders preserved from earlier refactorings. Their identical hash is intentional and expected.

## Critical Evidence Limitations & Disclaimers
1. **Sustained Transient Near Target**: $-41\text{ PWM}$ was observed to sustain 4-wheel motion over 1.404 seconds while decelerating through the target region. It is **not** an empirically proven steady-state equilibrium.
2. **Synthetic Plant Model Disclaimer**: The native C++ synthetic plant simulation (`runCrawlCandidateSimulation`) is regression evidence only to verify algorithmic trends and error unwinding. It is **not** physically predictive. For example, it calculates approximately $53.5\text{ mm}$ for forward Case 2 where physical floor testing measured $25-28\text{ mm}$. Synthetic Case 13 distance figures must **never** be cited as proof that the physical rover will pass.
3. **Physical Next Gate**: Upon review approval and flashing, run the normal closed-loop Case 13 physical trial once. Do not perform additional open-loop PWM trials unless telemetry reveals an unresolved physical defect.

## Test Runner Execution
The test runner script inside the package has executable permissions preserved (`0755`). It may be run directly or using:
```bash
bash tests/run_all_native_tests.sh
```
or
```bash
cd tests && ./run_all_native_tests.sh
```

## Package Structure (100% Self-Contained)
- `production_source/`: Complete production source tree including all headers and translation units (`DifferentialDrive.{h,cpp}`, `CommandManager.{h,cpp}`, `MotionLimiter.{h,cpp}`, `RoverConfig.{h,cpp}`, `WheelController.{h,cpp}`, `SerialProtocol.{h,cpp}`, etc.).
- `changed_source/`: The 3 candidate modified files specifically (`RoverConfig.h`, `WheelController.cpp`, `SerialProtocol.cpp`).
- `tests/`: All four test suites and `mock_include/`.
- `tests/run_all_native_tests.sh`: Fully self-contained test runner that resolves headers strictly within the extracted package.
- `evidence/firmware_test_output.txt`: Complete test execution transcript proving 100% pass across all suites.
- `diff/reverse_crawl_candidate_concise.diff`: Baseline-to-candidate concise diff.
- `manifest_sha256.txt`: SHA-256 checksum manifest of every file.
