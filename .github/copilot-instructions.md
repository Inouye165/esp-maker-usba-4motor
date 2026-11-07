# GitHub Copilot Instructions - Rover 1 (`esp-maker-usba-4motor`)

Authoritative rules and hardware reference are maintained in:
- [`AGENTS.md`](../AGENTS.md) — Hardware pinout, port remap (`1.3.4-remap-r03`), ESP32-S3 ToF bench (`1.2.4-tof-mux`), branch discipline, and build/test commands.
- [`PROJECT_STATUS.md`](../PROJECT_STATUS.md) — Roadmap, reusable test catalog, test ledger, and daily engineering log.
- [`HANDOFF.md`](../HANDOFF.md) & [`TODO.md`](../TODO.md) — Current session state and prioritized tasks.
- [`docs/EXACT_MOTION_CONTRACT.md`](../docs/EXACT_MOTION_CONTRACT.md) — Mandatory closed-loop motion contract.

## Exact Motion Contract (Mandatory)
All physical rover movement, distance translation, and turning MUST adhere to [`docs/EXACT_MOTION_CONTRACT.md`](../docs/EXACT_MOTION_CONTRACT.md) and use [`tools/rover_tests/cli.py`](../tools/rover_tests/cli.py):

* **Linear Distance (`python -m tools.rover_tests.cli linear` / `out-and-back`)**:
  * Use `LinearApproachController` (cruise $0.20\text{ m/s}$, creep $0.05\text{ m/s}$ in final $0.15\text{ m}$ zone, calibrated stopping advance **`0.0135 m` / `13.5 mm`**).
  * Use calibrated effective wheel diameter **`0.06695 m`** (`1974.1667` ticks/rev).
  * Never command constant cruise speed until target without deceleration.
* **In-Place Rotation (`python -m tools.rover_tests.cli turn` / `out-and-back`)**:
  * Use `AngularApproachController` (cruise $0.80\text{ rad/s}$, creep $0.20\text{ rad/s}$ in final $30.0^\circ$ zone, calibrated stopping advance **`0.7^\circ`**).
  * Use **`SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC`** (`0x08`) for relative yaw. Never use magnetic `SH2_ROTATION_VECTOR`.
* **Universal Safety Rules**:
  * Never confuse the **Maker-ESP32-Pro** motor controller (`/dev/rover-esp32` → `/dev/ttyUSB0`, `1a86:7523`) with the **Lonely Binary ESP32-S3** ToF/LED controller (`/dev/ttyAMA0` or `/dev/ttyACM0`, `303a:1001`).
  * Obtain Ron's explicit approval before any physical motion or firmware flash.
  * Monitor logical wheels `m1 (LF), m2 (RF), m3 (LR), m4 (RR)` continuously.
  * Always finish stopped, disarmed (`armed: false`), and locked (`Mode 0`).
