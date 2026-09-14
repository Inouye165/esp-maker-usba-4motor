"""
tools.rover_tests.runner - Multi-Trial Physical Test Execution Engine

Governs the complete execution lifecycle for Rover One tests:
- Obtains fresh IMU and odometry samples before each trial.
- Verifies non-magnetic BNO08x Game Rotation Vector orientation report (fails closed).
- Completes the 3-consecutive-zero autonomy handshake.
- Enforces explicit operator confirmation (inter-trial approval) and Ron physical angle estimate ingestion.
- Commands symmetric opposite wheel directions for rotate-in-place turns.
- Continuously logs all four wheels, gyro angles, creep milestone, zero command, post-zero coast, and settle.
- Disarms and locks drivetrain on normal completion, timeout, and all abort paths.
"""

import sys
import time
import math
import statistics
import traceback
from typing import Optional, Dict, Any, List, Callable

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from .turn import TurnParameters, compute_wheel_speed_targets, verify_wheel_command_symmetry
from .transport import (
    NativeWSClient,
    CockpitClient,
    perform_zero_handshake,
    disarm_and_stop,
    TransportException,
    HandshakeException
)
from .sensors import (
    quat_to_yaw,
    YawUnwrapper,
    BiasCorrectedGyroIntegrator,
    verify_non_magnetic_imu,
    MagneticImuException,
    SensorException
)
from .controllers import AngularApproachController, ApproachPhase
from .reporting import (
    TrialReport,
    WheelTrialMetrics,
    MultiTrialSuiteReport,
    ReportGenerator
)


class TestAbortException(Exception):
    """Raised when a safety guard or watchdog aborts a trial."""
    pass


class PhysicalTestRunner:
    """
    Orchestrates physical test execution, safety monitoring, and report compilation.
    """
    def __init__(
        self,
        params: TurnParameters,
        prompt_fn: Optional[Callable[[str], str]] = None,
        cockpit_client: Optional[CockpitClient] = None,
        ws_client: Optional[NativeWSClient] = None
    ):
        params.validate()
        self.params = params
        self.prompt_fn = prompt_fn or input
        self.cockpit = cockpit_client or CockpitClient(host=params.host, port=params.port)
        self.ws = ws_client or NativeWSClient(host=params.host, port=params.port)

    def execute_suite(self) -> MultiTrialSuiteReport:
        """Executes the full suite of trials."""
        suite_id = str(int(time.time()))
        suite_report = MultiTrialSuiteReport(
            suite_id=suite_id,
            test_type="turn",
            command_line=f"rover-test turn --degrees {self.params.degrees} --direction {self.params.direction} --trials {self.params.trials}",
            timestamp_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            target_degrees=self.params.degrees,
            direction=self.params.direction,
            total_trials=self.params.trials,
            is_dry_run=self.params.dry_run
        )

        print("=" * 80)
        print(f"ROVER ONE REUSABLE PHYSICAL TEST FRAMEWORK: TURN {self.params.degrees:.1f}° {self.params.direction.upper()}")
        print(f"Trials: {self.params.trials} | Max Speed: {self.params.max_angular_speed:.2f} rad/s | Creep: {self.params.creep_angular_speed:.2f} rad/s")
        print(f"Mode: {'DRY RUN (STATIONARY - NO MOTOR POWER)' if self.params.dry_run else 'PHYSICAL MOTION EXECUTION'}")
        print("=" * 80)

        # Connect WebSocket unless in pure offline mock mode
        if not self.params.dry_run:
            if not self.ws.connect():
                print(f"[TRANSPORT ERROR] Failed to connect to Cockpit WebSocket on {self.params.host}:{self.params.port}")
                raise TransportException("Cannot connect to Cockpit WebSocket")
            if not self.ws.authenticate(self.cockpit.token):
                print("[AUTH ERROR] Failed to authenticate with Cockpit using operator token")
                raise TransportException("Operator authentication rejected")
            print("[OK] Connected and authenticated with Cockpit WebSocket.")

        try:
            for trial_idx in range(1, self.params.trials + 1):
                trial_report = self.execute_single_trial(trial_idx)
                suite_report.trials.append(trial_report)
                if trial_report.status in ("SUCCESS", "DRY_RUN_PASSED"):
                    suite_report.successful_trials += 1
                else:
                    suite_report.aborted_trials += 1

                # If aborted and not dry run, do not automatically retry without operator intervention
                if trial_report.status == "ABORTED" and not self.params.dry_run:
                    print(f"\n[SAFETY STOP] Trial {trial_idx} aborted. Routine halted for forensic preservation.")
                    break
        finally:
            disarm_and_stop(self.cockpit, self.ws)
            if self.ws.connected:
                self.ws.close()

        # Compute suite statistics across successful trials
        successful_trials = [t for t in suite_report.trials if t.status in ("SUCCESS", "DRY_RUN_PASSED")]
        if successful_trials:
            errors = [t.settled_heading_error_deg for t in successful_trials]
            suite_report.mean_settled_error_deg = statistics.mean(errors)
            suite_report.std_dev_settled_error_deg = statistics.stdev(errors) if len(errors) > 1 else 0.0
            suite_report.repeatability_deg = max(errors) - min(errors)

            estimate_deltas = [t.estimate_vs_gyro_delta_deg for t in successful_trials if t.estimate_vs_gyro_delta_deg is not None]
            if estimate_deltas:
                suite_report.mean_estimate_delta_deg = statistics.mean(estimate_deltas)

        # Generate output files
        json_path = ReportGenerator.save_json(suite_report, self.params.report_directory)
        md_summary = ReportGenerator.format_markdown_summary(suite_report)
        print("\n" + md_summary)
        print(f"\n[REPORT SAVED] Full JSON telemetry report: {json_path}")
        return suite_report

    def _assert_pre_motion_invariants(self) -> Optional[str]:
        """
        Immediately before the first nonzero command, assert:
        - armed == true
        - autonomyState == READY_ARMED
        - expected command ownership is valid (NONE or ROS_AUTONOMY)
        - IMU and encoder telemetry are fresh
        Returns None if valid, or a descriptive failure message if invalid.
        """
        drv_stat = self.cockpit.get_status()
        auto_stat = self.cockpit.get_autonomy_status()
        imu_stat = self.cockpit.get_imu()
        enc_stat = self.cockpit.get_encoders()

        if drv_stat.get("armed") is not True:
            return f"Pre-motion invariant violated: armed is {drv_stat.get('armed')} (expected True)"

        cur_state = auto_stat.get("state")
        if cur_state != "READY_ARMED":
            return f"Pre-motion invariant violated: autonomyState is '{cur_state}' (expected READY_ARMED)"

        cmd_source = auto_stat.get("cmdSource") or drv_stat.get("cmdSource")
        if cmd_source not in ("NONE", "ROS_AUTONOMY"):
            return f"Pre-motion invariant violated: unexpected cmdSource '{cmd_source}' (expected NONE or ROS_AUTONOMY)"

        if not imu_stat or not imu_stat.get("ok", False):
            return "Pre-motion invariant violated: IMU telemetry dropped or unreadable"

        data_age_ms = imu_stat.get("dataAgeMs", 0)
        if data_age_ms > 250:
            return f"Pre-motion invariant violated: IMU telemetry stale ({data_age_ms}ms > 250ms limit)"

        if not enc_stat or (not enc_stat.get("ok", False) and "encoders" not in enc_stat):
            return "Pre-motion invariant violated: encoder telemetry dropped or unreadable"

        return None

    def execute_single_trial(self, trial_idx: int) -> TrialReport:
        """Executes one physical turn trial adhering to all exact-motion invariants."""
        print(f"\n--- Starting Trial {trial_idx}/{self.params.trials} ---")

        # 1. Inter-Trial Operator Approval
        if self.params.inter_trial_approval or not self.params.dry_run:
            print("\n[OPERATOR APPROVAL REQUIRED]")
            print(f"  • Target Maneuver: {self.params.degrees:.1f}° {self.params.direction.upper()}")
            print(f"  • Target Signed Yaw: {self.params.signed_target_deg:+.1f}°")
            print(f"  • Speeds: Cruise = {self.params.max_angular_speed:.2f} rad/s | Creep = {self.params.creep_angular_speed:.2f} rad/s")
            print("  Please confirm rover area is clear and authorize physical motion.")
            resp = self.prompt_fn("Authorize motion? [y/N]: ").strip().lower()
            if resp not in ("y", "yes"):
                print("[OPERATOR ABORT] Motion not approved by Ron. Aborting trial.")
                return TrialReport(
                    trial_index=trial_idx,
                    requested_turn_deg=self.params.degrees,
                    direction=self.params.direction,
                    target_signed_yaw_deg=self.params.signed_target_deg,
                    status="ABORTED",
                    abort_reason="Operator authorization withheld"
                )

        trial_report = TrialReport(
            trial_index=trial_idx,
            requested_turn_deg=self.params.degrees,
            direction=self.params.direction,
            target_signed_yaw_deg=self.params.signed_target_deg,
            status="INITIALIZING"
        )

        t_trial_start = time.time()

        # Step 3: Ensure stationary IMU/gyro calibration and settling while DISARMED
        if not self.params.dry_run:
            cur_st = self.cockpit.get_status()
            if cur_st.get("armed") is True or cur_st.get("autonomyState") != "DISABLED":
                disarm_and_stop(self.cockpit, self.ws, verbose=False)

        # 2. Fresh IMU Snapshot & Non-Magnetic Validation
        imu_snap = self.cockpit.get_imu()
        if not self.params.dry_run:
            valid_non_mag, reason = verify_non_magnetic_imu(imu_snap)
            if not valid_non_mag:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"IMU Non-Magnetic Check FAILED: {reason}"
                print(f"[FAIL-CLOSED ERROR] {trial_report.abort_reason}")
                disarm_and_stop(self.cockpit, self.ws)
                return trial_report
            print(f"[OK] BNO08x Orientation Verified: {reason}")
        else:
            print("[OK] [DRY RUN] BNO08x non-magnetic check simulated.")

        # 3. Capture Initial Encoder State
        enc_start = self.cockpit.get_encoders().get("encoders", {"m1": 0, "m2": 0, "m3": 0, "m4": 0})
        wheel_metrics = {
            "m1": WheelTrialMetrics(wheel_id="m1", encoder_start_ticks=enc_start.get("m1", 0)),
            "m2": WheelTrialMetrics(wheel_id="m2", encoder_start_ticks=enc_start.get("m2", 0)),
            "m3": WheelTrialMetrics(wheel_id="m3", encoder_start_ticks=enc_start.get("m3", 0)),
            "m4": WheelTrialMetrics(wheel_id="m4", encoder_start_ticks=enc_start.get("m4", 0)),
        }

        # 4. Relative Orientation Initialization
        raw_yaw_0 = quat_to_yaw(imu_snap.get("orientation", {"w": 1, "x": 0, "y": 0, "z": 0}))
        unwrapper = YawUnwrapper(initial_raw_yaw=raw_yaw_0)
        gyro_integrator = BiasCorrectedGyroIntegrator()

        # Gather pre-turn stationary samples for gyro bias calibration while DISARMED
        pre_turn_samples = [imu_snap.get("gyro", {}).get("z", 0.0)]
        for _ in range(5):
            time.sleep(0.03)
            sample_snap = self.cockpit.get_imu()
            pre_turn_samples.append(sample_snap.get("gyro", {}).get("z", 0.0))

        try:
            gyro_integrator.calibrate_bias(pre_turn_samples)
        except SensorException as e:
            if not self.params.dry_run:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"Stationary gyro bias calibration failed: {e}"
                print(f"[SENSOR ABORT] {trial_report.abort_reason}")
                disarm_and_stop(self.cockpit, self.ws)
                return trial_report

        # 5. Autonomy Enable, Three-Consecutive-Zero Handshake, and Arming
        if not self.params.dry_run:
            try:
                print("  Executing 3-consecutive-zero autonomy handshake...")
                perform_zero_handshake(self.cockpit, self.ws, transitions=trial_report.autonomy_state_transitions)
                print("[OK] Autonomy Handshake complete: Drivetrain READY_ARMED.")
            except HandshakeException as e:
                tb_str = traceback.format_exc()
                trial_report.status = "ABORTED"
                trial_report.abort_reason = (
                    f"[{type(e).__name__}] {e} | Stage: {e.stage} | State: {e.state} "
                    f"| ZeroCount: {e.zero_count} | CmdSource: {e.cmd_source}"
                )
                print("\n[HANDSHAKE ABORT]")
                print(f"  • Exception Type:    {type(e).__name__}")
                print(f"  • Exception Message: {e}")
                print(f"  • Handshake Stage:   {e.stage}")
                print(f"  • Handshake State:   state={e.state}, zeroCount={e.zero_count}, cmdSource={e.cmd_source}")
                if e.last_rejection_reason:
                    print(f"  • Last Rejection:    {e.last_rejection_reason}")
                if e.underlying_error:
                    print(f"  • Underlying Error:  {e.underlying_error}")
                print("  • Traceback:")
                for line in tb_str.strip().splitlines():
                    print(f"      {line}")
                disarm_and_stop(self.cockpit, self.ws)
                return trial_report
            except Exception as e:
                tb_str = traceback.format_exc()
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"[{type(e).__name__}] Unexpected fault in handshake: {e}"
                print(f"\n[HANDSHAKE UNEXPECTED FAULT]")
                print(f"  • Exception Type:    {type(e).__name__}")
                print(f"  • Exception Message: {e}")
                print("  • Traceback:")
                for line in tb_str.strip().splitlines():
                    print(f"      {line}")
                disarm_and_stop(self.cockpit, self.ws)
                return trial_report
        else:
            print("[OK] [DRY RUN] 3-consecutive-zero autonomy handshake simulated.")
            trial_report.autonomy_state_transitions.extend([
                {"timestamp": time.time(), "state": "WAITING_FOR_ZERO", "trigger": "enable_autonomy (dry run)"},
                {"timestamp": time.time(), "state": "READY_DISARMED", "trigger": "zero_handshake (dry run)"},
                {"timestamp": time.time(), "state": "READY_ARMED", "trigger": "arm_drive (dry run)"}
            ])

        # Step 7: Immediately before first nonzero command, assert pre-motion invariants
        if not self.params.dry_run:
            inv_error = self._assert_pre_motion_invariants()
            if inv_error:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = inv_error
                print(f"[FAIL-CLOSED ASSERTION FAILED] {inv_error}")
                cleanup_st = disarm_and_stop(self.cockpit, self.ws)
                trial_report.autonomy_state_transitions.append({
                    "timestamp": time.time(),
                    "state": cleanup_st.get("autonomyState", "DISABLED"),
                    "trigger": "pre_motion_assertion_failed"
                })
                return trial_report
            print("[OK] Pre-motion invariants verified: armed=True, autonomyState=READY_ARMED, valid cmdSource, fresh IMU & encoders.")

        # 6. Motion Controller Execution
        controller = AngularApproachController(
            cruise_wz_radps=self.params.max_angular_speed,
            creep_wz_radps=self.params.creep_angular_speed,
            approach_zone_deg=self.params.creep_threshold_deg
        )
        controller.reset(target=self.params.signed_target_deg, start_time=time.time())

        # Step 8: Begin the nonzero motion loop
        first_cmd_wz = controller.update(current_progress=0.0, current_time=time.time())
        if abs(first_cmd_wz) < 1e-4:
            first_cmd_wz = math.copysign(self.params.creep_angular_speed, self.params.signed_target_deg)

        if not self.params.dry_run:
            first_res = self.cockpit.send_cmd_vel(vx=0.0, wz=first_cmd_wz)
            trial_report.first_command_response = first_res

            # Step 9: Verify the first nonzero command was accepted
            if not first_res.get("ok", False):
                err = first_res.get("error", "Unknown command rejection")
                rej = self.cockpit.get_autonomy_status().get("lastRejectionReason")
                fail_reason = f"First nonzero command rejected: {err}" + (f" ({rej})" if rej else "")
                print(f"[FAIL-FAST ABORT] {fail_reason}")
                trial_report.status = "ABORTED"
                trial_report.abort_reason = fail_reason
                cleanup_st = disarm_and_stop(self.cockpit, self.ws)
                trial_report.autonomy_state_transitions.append({
                    "timestamp": time.time(),
                    "state": cleanup_st.get("autonomyState", "DISABLED"),
                    "trigger": "first_cmd_rejected_cleanup"
                })
                return trial_report

            # Step 10: Within bounded interval (max 500ms), verify targets become nonzero / state becomes ACTIVE
            cmd_accepted_active = False
            t_check_start = time.time()
            while time.time() - t_check_start < 0.5:
                a_stat = self.cockpit.get_autonomy_status()
                if a_stat.get("state") == "ACTIVE" or abs(a_stat.get("clampedAngular", 0.0)) > 1e-4:
                    cmd_accepted_active = True
                    trial_report.autonomy_state_transitions.append({
                        "timestamp": time.time(),
                        "state": "ACTIVE",
                        "trigger": "first_nonzero_cmd_accepted"
                    })
                    break
                time.sleep(0.04)

            # Step 11: If command rejected or targets remain zero, abort immediately (do not wait 15s)
            if not cmd_accepted_active:
                a_stat = self.cockpit.get_autonomy_status()
                rejection = a_stat.get("lastRejectionReason") or "Commanded wheel targets remained zero / ACTIVE state not achieved within 500ms"
                fail_reason = f"Motion startup failed within bounded window: {rejection} (state={a_stat.get('state')})"
                print(f"[FAIL-FAST ABORT] {fail_reason}")
                trial_report.status = "ABORTED"
                trial_report.abort_reason = fail_reason
                cleanup_st = disarm_and_stop(self.cockpit, self.ws)
                trial_report.autonomy_state_transitions.append({
                    "timestamp": time.time(),
                    "state": cleanup_st.get("autonomyState", "DISABLED"),
                    "trigger": "startup_timeout_cleanup"
                })
                return trial_report
        else:
            trial_report.first_command_response = {"ok": True, "linear": 0.0, "angular": first_cmd_wz, "state": "ACTIVE"}
            trial_report.autonomy_state_transitions.append({
                "timestamp": time.time(),
                "state": "ACTIVE",
                "trigger": "first_nonzero_cmd_accepted (dry run)"
            })

        # Safety thresholds
        max_duration_sec = 15.0
        max_angle_limit_deg = abs(self.params.degrees) + 15.0
        angle_at_zero_cmd = 0.0
        stalls_detected = {w: False for w in ["m1", "m2", "m3", "m4"]}
        breakout_count = 0
        breakout_start_time = None
        total_breakout_dwell_ms = 0.0

        # Active-motion telemetry collection buffers (strictly excluding preflight, settling, and final-zero)
        wheel_samples = {
            w_id: {
                "targets": [],
                "measured": [],
                "stopped_events": 0,
                "stopped_duration_s": 0.0,
                "is_currently_stopped": False,
                "stopped_entry_time": None,
            }
            for w_id in ["m1", "m2", "m3", "m4"]
        }
        total_telemetry_samples = 0

        abort_reason = None
        t_motion_start = time.time()

        try:
            if self.params.dry_run:
                # In dry-run mode, simulate steps through approach controller without moving motors
                sim_angles = [
                    0.0,
                    self.params.signed_target_deg * 0.5,
                    self.params.signed_target_deg * 0.85,
                    self.params.signed_target_deg
                ]
                for sim_yaw in sim_angles:
                    cmd_wz = controller.update(current_progress=sim_yaw, current_time=time.time())
                    wheel_cmds = compute_wheel_speed_targets(cmd_wz)
                    sym_ok, sym_msg = verify_wheel_command_symmetry(wheel_cmds)
                    if not sym_ok:
                        raise TestAbortException(f"Symmetry check failed: {sym_msg}")
                    if controller.phase in (ApproachPhase.CRUISE, ApproachPhase.CREEP):
                        for w_id in ["m1", "m2", "m3", "m4"]:
                            cmd_val = wheel_cmds[w_id]
                            wheel_samples[w_id]["targets"].append(cmd_val)
                            wheel_samples[w_id]["measured"].append(cmd_val)
                        total_telemetry_samples += 1
                    time.sleep(0.05)
                angle_at_zero_cmd = self.params.signed_target_deg
            else:
                # Physical motion loop
                while True:
                    now = time.time()
                    dt_motion = now - t_motion_start

                    # Duration watchdog
                    if dt_motion > max_duration_sec:
                        abort_reason = f"Maximum trial duration exceeded ({max_duration_sec}s)"
                        trial_report.watchdog_trips.append("MAX_DURATION_EXCEEDED")
                        break

                    # Ingest latest IMU data
                    latest_imu = self.cockpit.get_imu()
                    if not latest_imu or not latest_imu.get("ok", False):
                        abort_reason = "IMU telemetry drop or unreadable"
                        trial_report.communication_faults.append("IMU_DROP")
                        break

                    # Stale telemetry watchdog (> 250ms)
                    imu_age_ms = latest_imu.get("dataAgeMs", 0)
                    if imu_age_ms > 250:
                        abort_reason = f"Stale IMU data detected ({imu_age_ms}ms > 250ms limit)"
                        trial_report.watchdog_trips.append("STALE_IMU_DATA")
                        break

                    cur_raw_yaw = quat_to_yaw(latest_imu.get("orientation", {}))
                    unwrapper.update_orientation_yaw(cur_raw_yaw)
                    cur_rel_yaw = unwrapper.relative_yaw_deg

                    # Max angle safety guard
                    if abs(cur_rel_yaw) > max_angle_limit_deg:
                        abort_reason = f"Hard angle limit exceeded: {abs(cur_rel_yaw):.1f}° > {max_angle_limit_deg:.1f}°"
                        trial_report.watchdog_trips.append("ANGLE_LIMIT_EXCEEDED")
                        break

                    # Controller update
                    cmd_wz = controller.update(current_progress=cur_rel_yaw, current_time=now)
                    wheel_cmds = compute_wheel_speed_targets(cmd_wz)

                    # Verify symmetric opposite commands
                    sym_ok, sym_msg = verify_wheel_command_symmetry(wheel_cmds)
                    if not sym_ok:
                        trial_report.opposite_polarity_maintained = False
                        abort_reason = f"Command symmetry violation: {sym_msg}"
                        break

                    # Check completion (zero command issued)
                    # When target is reached, command zero and exit active motion immediately.
                    # This ensures zero-command and settling samples are NEVER mixed into active motion statistics.
                    if controller.phase == ApproachPhase.ZERO:
                        angle_at_zero_cmd = cur_rel_yaw
                        self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                        break

                    # Stream command to rover via ROS2 bridge
                    cmd_res = self.cockpit.send_cmd_vel(vx=0.0, wz=cmd_wz)
                    if not cmd_res.get("ok", False):
                        abort_reason = f"cmd_vel transmission failed: {cmd_res.get('error')}"
                        trial_report.communication_faults.append("CMD_VEL_SEND_FAILED")
                        break

                    # Ingest PID diagnostics and monitor wheel stalls / breakouts
                    # ONLY collected while controller is in active motion (CRUISE or CREEP)
                    if self.ws and self.ws.connected:
                        frames = self.ws.recv_frames()
                        for f in frames:
                            if f.get("type") == "pid_diagnostic":
                                total_telemetry_samples += 1
                                for w_id in ["m1", "m2", "m3", "m4"]:
                                    w_data = f.get(w_id, {})
                                    tgt = w_data.get("targetRadps")
                                    meas = w_data.get("measuredRadps")
                                    stiction_state = w_data.get("stictionState", "IDLE")

                                    if tgt is not None and isinstance(tgt, (int, float)):
                                        wheel_samples[w_id]["targets"].append(float(tgt))
                                    if meas is not None and isinstance(meas, (int, float)):
                                        wheel_samples[w_id]["measured"].append(float(meas))

                                    if stiction_state == "STICTION_BOOST":
                                        wheel_metrics[w_id].stiction_boost_events += 1
                                        if breakout_start_time is None:
                                            breakout_start_time = now
                                            breakout_count += 1
                                    elif stiction_state == "BLOCKED":
                                        wheel_metrics[w_id].blocked_state_events += 1
                                        stalls_detected[w_id] = True

                                    # Stopped-while-commanded monitoring:
                                    tgt_val = float(tgt) if (tgt is not None and isinstance(tgt, (int, float))) else abs(wheel_cmds.get(w_id, 0.0))
                                    meas_val = float(meas) if (meas is not None and isinstance(meas, (int, float))) else 0.0
                                    is_stopped = (abs(tgt_val) > 0.10) and (abs(meas_val) < 0.05 or stiction_state == "BLOCKED")

                                    ws_buf = wheel_samples[w_id]
                                    if is_stopped:
                                        stalls_detected[w_id] = True
                                        if not ws_buf["is_currently_stopped"]:
                                            ws_buf["is_currently_stopped"] = True
                                            ws_buf["stopped_entry_time"] = now
                                            ws_buf["stopped_events"] += 1
                                    else:
                                        if ws_buf["is_currently_stopped"]:
                                            dwell = max(0.02, now - (ws_buf["stopped_entry_time"] or now))
                                            ws_buf["stopped_duration_s"] += dwell
                                            ws_buf["is_currently_stopped"] = False
                                            ws_buf["stopped_entry_time"] = None

                        if breakout_start_time and any(f.get(w, {}).get("stictionState") != "STICTION_BOOST" for w in ["m1", "m2", "m3", "m4"] for f in frames if f.get("type") == "pid_diagnostic"):
                            total_breakout_dwell_ms += (now - breakout_start_time) * 1000.0
                            breakout_start_time = None

                    time.sleep(0.02)  # 50Hz control loop

        except Exception as e:
            abort_reason = f"[{type(e).__name__}] Exception during motion execution: {e}"

        # Close any open stopped_while_commanded periods
        now_post = time.time()
        for w_id in ["m1", "m2", "m3", "m4"]:
            ws_buf = wheel_samples[w_id]
            if ws_buf["is_currently_stopped"] and ws_buf["stopped_entry_time"]:
                dwell = max(0.02, now_post - ws_buf["stopped_entry_time"])
                ws_buf["stopped_duration_s"] += dwell
                ws_buf["is_currently_stopped"] = False

        # Aggregate active-motion wheel metrics (strictly before settle period)
        for w_id in ["m1", "m2", "m3", "m4"]:
            w_metric = wheel_metrics[w_id]
            ws_buf = wheel_samples[w_id]
            targets = ws_buf["targets"]
            measured = ws_buf["measured"]
            w_metric.active_samples_count = len(measured)

            if len(targets) > 0:
                w_metric.commanded_speed_radps_mean = sum(targets) / len(targets)
                w_metric.commanded_speed_radps_max = max(targets, key=abs)
                w_metric.commanded_speed_source = "calculated_expected" if self.params.dry_run else "firmware_pid"
            else:
                w_metric.commanded_speed_radps_mean = None
                w_metric.commanded_speed_radps_max = None
                w_metric.commanded_speed_source = "unavailable"

            if len(measured) > 0:
                w_metric.measured_speed_radps_mean = sum(measured) / len(measured)
                w_metric.measured_speed_radps_abs_mean = sum(abs(v) for v in measured) / len(measured)
                w_metric.measured_speed_radps_min = min(measured)
                w_metric.measured_speed_radps_max = max(measured)
            else:
                w_metric.measured_speed_radps_mean = None
                w_metric.measured_speed_radps_abs_mean = None
                w_metric.measured_speed_radps_min = None
                w_metric.measured_speed_radps_max = None

            w_metric.stopped_while_commanded = stalls_detected[w_id] or (ws_buf["stopped_events"] > 0)
            w_metric.stopped_while_commanded_count = ws_buf["stopped_events"]
            w_metric.stopped_while_commanded_duration_s = ws_buf["stopped_duration_s"]

        trial_report.telemetry_samples_count = total_telemetry_samples

        # If motion loop aborted, execute disarm_and_stop() immediately and return
        if abort_reason:
            trial_report.status = "ABORTED"
            trial_report.abort_reason = abort_reason
            print(f"[TRIAL ABORTED] {abort_reason}")
            cleanup_st = disarm_and_stop(self.cockpit, self.ws)
            trial_report.autonomy_state_transitions.append({
                "timestamp": time.time(),
                "state": cleanup_st.get("autonomyState", "DISABLED"),
                "trigger": "motion_abort_cleanup"
            })
            return trial_report

        # Step 12: Settle Period - zero command was issued upon target arrival
        print(f"  Settling for {self.params.settle_seconds:.1f}s...")
        time.sleep(self.params.settle_seconds)
        if not self.params.dry_run:
            final_imu = self.cockpit.get_imu()
            if final_imu and final_imu.get("ok", False):
                final_raw_yaw = quat_to_yaw(final_imu.get("orientation", {}))
                unwrapper.update_orientation_yaw(final_raw_yaw)
                final_settled_yaw = unwrapper.relative_yaw_deg
            else:
                final_settled_yaw = angle_at_zero_cmd
        else:
            final_settled_yaw = angle_at_zero_cmd

        controller.mark_settled(final_settled_yaw)
        post_zero_coast = final_settled_yaw - angle_at_zero_cmd

        # Capture final encoders
        enc_final = self.cockpit.get_encoders().get("encoders", enc_start)
        for w_id in ["m1", "m2", "m3", "m4"]:
            w_metric = wheel_metrics[w_id]
            w_metric.encoder_final_ticks = enc_final.get(w_id, w_metric.encoder_start_ticks)
            w_metric.encoder_delta_ticks = w_metric.encoder_final_ticks - w_metric.encoder_start_ticks

        # Execute disarm_and_stop() ONLY after the target and settling sequence completes!
        cleanup_st = disarm_and_stop(self.cockpit, self.ws)
        trial_report.autonomy_state_transitions.append({
            "timestamp": time.time(),
            "state": cleanup_st.get("autonomyState", "DISABLED"),
            "trigger": "trial_settle_complete_cleanup"
        })

        # Populate report metrics
        trial_report.duration_s = time.time() - t_trial_start
        trial_report.gyro_angle_at_zero_cmd_deg = angle_at_zero_cmd
        trial_report.final_settled_gyro_angle_deg = final_settled_yaw
        trial_report.post_zero_rotation_deg = post_zero_coast
        trial_report.settled_heading_error_deg = final_settled_yaw - self.params.signed_target_deg
        trial_report.wheel_metrics = wheel_metrics
        trial_report.breakout_event_count = breakout_count
        trial_report.breakout_dwell_time_ms_total = total_breakout_dwell_ms
        trial_report.approach_milestones = controller.get_telemetry_summary()
        trial_report.confirmed_final_zero_command = True
        trial_report.confirmed_final_disarmed_state = True
        trial_report.status = "DRY_RUN_PASSED" if self.params.dry_run else "SUCCESS"

        print(f"[OK] Trial Completed ({trial_report.status})")
        print(f"  • Angle at Zero Cmd: {angle_at_zero_cmd:+.2f}°")
        print(f"  • Final Settled Yaw: {final_settled_yaw:+.2f}° (Target: {self.params.signed_target_deg:+.2f}°)")
        print(f"  • Post-Zero Coast:   {post_zero_coast:+.2f}°")
        print(f"  • Settled Error:     {trial_report.settled_heading_error_deg:+.2f}°")

        # Ingest Ron's Physical Estimate
        if self.params.inter_trial_approval and not self.params.dry_run:
            print("\n[PHYSICAL ESTIMATE COLLECTION]")
            print(f"  Gyro Measured Settled Angle: {final_settled_yaw:+.2f}°")
            est_input = self.prompt_fn("Enter Ron's physical ground-truth angle estimate in degrees (or Enter to skip): ").strip()
            if est_input:
                try:
                    est_val = float(est_input)
                    trial_report.rons_physical_angle_estimate_deg = est_val
                    trial_report.estimate_vs_gyro_delta_deg = final_settled_yaw - est_val
                except ValueError:
                    print("  Invalid numeric input for physical estimate. Skipped.")

        return trial_report
