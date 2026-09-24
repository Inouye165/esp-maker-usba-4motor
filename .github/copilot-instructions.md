# GitHub Copilot Instructions - Rover Project

## Exact Motion Contract (Mandatory)
All physical rover movement, distance translation, and turning MUST adhere to [`docs/EXACT_MOTION_CONTRACT.md`](../docs/EXACT_MOTION_CONTRACT.md).

* **Linear Distance**:
  * Use `LinearApproachController` (cruise $0.20\text{ m/s}$, creep $0.05\text{ m/s}$ in $0.15\text{ m}$ zone).
  * Use calibrated effective wheel diameter `0.06695 m`.
  * Never command constant cruise speed until target without deceleration.
* **In-Place Rotation**:
  * Use `AngularApproachController` (cruise $0.80\text{ rad/s}$, creep $0.20\text{ rad/s}$ in $30.0^\circ$ zone).
  * Use `SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC` for relative yaw.
  * Never use magnetic `SH2_ROTATION_VECTOR`.
* **Universal Rules**:
  * Obtain Ron's explicit approval before physical motion.
  * Monitor M1–M4 response continuously.
  * Always finish stopped, disarmed, and locked.
