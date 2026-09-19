"""
tools.rover_tests.reporting - Forensic Trial Reporting & Telemetry Serialization

Formats and serializes comprehensive multi-trial test reports adhering to Requirement 9:
- Requested turn and sign.
- Gyro angle at zero command, final settled gyro angle, post-zero coast rotation.
- Ron's physical angle estimate and deviation vs measured angle.
- Per-wheel commanded and measured speeds and encoder tick deltas.
- Wheel stall detection (whether any wheel stopped while commanded to move).
- Opposite-side polarity verification and intentional asymmetry reporting.
- Breakout-assistance events (stiction boost occurrences and durations).
- Safety faults, watchdog trips, and communication anomalies.
- Confirmed final zero/disarmed state.
"""

import json
import os
import time
import math
from dataclasses import dataclass, field, asdict
from typing import Dict, Any, List, Optional


@dataclass
class WheelTrialMetrics:
    wheel_id: str  # "m1", "m2", "m3", "m4"
    slot_mapping: str = ""    # "Slot 1" .. "Slot 4"
    corner: str = ""          # "LF", "RF", "LR", "RR"
    validity: str = "VALID"   # "VALID", "INVALID"

    # Commanded speed metrics (None if telemetry unavailable or invalid, never false 0.00)
    commanded_speed_radps_mean: Optional[float] = None
    commanded_speed_radps_max: Optional[float] = None
    commanded_speed_source: str = "firmware_pid"  # "firmware_pid", "calculated_expected", "unavailable", "INVALID_ZERO_TELEMETRY"

    # Measured speed metrics (None if telemetry unavailable or invalid, never false 0.00)
    measured_speed_radps_mean: Optional[float] = None
    measured_speed_radps_abs_mean: Optional[float] = None
    measured_speed_radps_min: Optional[float] = None
    measured_speed_radps_max: Optional[float] = None

    encoder_start_ticks: int = 0
    encoder_final_ticks: int = 0
    encoder_delta_ticks: int = 0

    stopped_while_commanded: Optional[bool] = None  # None = Unknown, True = Stalled while commanded, False = Commanded and moving
    stopped_while_commanded_count: int = 0
    stopped_while_commanded_duration_s: float = 0.0

    stiction_boost_events: int = 0
    blocked_state_events: int = 0
    active_samples_count: int = 0

    def __post_init__(self):
        canonical_slots = {
            "m1": ("Slot 1", "LF"),
            "m2": ("Slot 2", "RF"),
            "m3": ("Slot 3", "LR"),
            "m4": ("Slot 4", "RR"),
        }
        info = canonical_slots.get(self.wheel_id.lower(), ("Unknown Slot", "Unknown Corner"))
        if not self.slot_mapping:
            self.slot_mapping = info[0]
        if not self.corner:
            self.corner = info[1]


@dataclass
class TrialReport:
    trial_index: int
    requested_turn_deg: float
    direction: str
    target_signed_yaw_deg: float
    status: str  # "SUCCESS", "ABORTED", "DRY_RUN_PASSED"
    abort_reason: Optional[str] = None
    duration_s: float = 0.0

    # Gyro & Orientation Metrics
    gyro_angle_at_zero_cmd_deg: float = 0.0
    final_settled_gyro_angle_deg: Optional[float] = None
    post_zero_rotation_deg: Optional[float] = None
    settled_heading_error_deg: Optional[float] = None
    final_measurement_valid: bool = False
    rons_physical_angle_estimate_deg: Optional[float] = None
    estimate_vs_gyro_delta_deg: Optional[float] = None

    # Wheel & Actuation Metrics
    wheel_metrics: Dict[str, WheelTrialMetrics] = field(default_factory=dict)
    wheel_forensics_valid: bool = True
    opposite_polarity_maintained: bool = True
    intentional_asymmetry_reported: bool = False
    breakout_assistance_active: bool = False
    breakout_event_count: int = 0
    breakout_dwell_time_ms_total: float = 0.0

    # Safety & State Confirmation
    watchdog_trips: List[str] = field(default_factory=list)
    communication_faults: List[str] = field(default_factory=list)
    confirmed_final_zero_command: bool = False
    confirmed_final_disarmed_state: bool = False
    telemetry_samples_count: int = 0
    autonomy_state_transitions: List[Dict[str, Any]] = field(default_factory=list)
    first_command_response: Dict[str, Any] = field(default_factory=dict)

    # Zero-Command & Settling Instrumentation
    zero_command_send_time_monotonic: Optional[float] = None
    zero_command_response_time_monotonic: Optional[float] = None
    zero_command_latency_ms: Optional[float] = None
    zero_command_response: Dict[str, Any] = field(default_factory=dict)
    settle_duration_s: float = 0.0
    settle_achieved_imu_rate_hz: float = 0.0
    settle_achieved_pid_rate_hz: float = 0.0
    settle_imu_poll_count: int = 0
    settle_imu_valid_advancing_count: int = 0
    settle_imu_poll_rate_hz: float = 0.0
    settle_imu_valid_advancing_rate_hz: float = 0.0
    settle_pid_packets_count: int = 0
    settle_pid_post_zero_packets_count: int = 0
    settle_pid_backlog_packets_count: int = 0
    settle_pid_unknown_packets_count: int = 0
    settle_pid_packet_rate_hz: float = 0.0
    settle_pid_post_zero_packet_rate_hz: float = 0.0
    settle_imu_samples: List[Dict[str, Any]] = field(default_factory=list)
    settle_pid_packets: List[Dict[str, Any]] = field(default_factory=list)
    active_pid_packets: List[Dict[str, Any]] = field(default_factory=list)

    # Balancing & Dynamic Braking Instrumentation
    wheel_balancing_enabled: bool = False
    dynamic_braking_enabled: bool = False
    stopping_advance_deg: float = 0.0
    actuation_states_observed: List[str] = field(default_factory=list)
    brake_active_duration_ms: Optional[float] = None

    # Raw telemetry frames (optional / truncated in summary)
    approach_milestones: Dict[str, Any] = field(default_factory=dict)


@dataclass
class MultiTrialSuiteReport:
    suite_id: str
    test_type: str = "turn"
    command_line: str = ""
    timestamp_utc: str = ""
    target_degrees: float = 0.0
    direction: str = "cw"
    total_trials: int = 0
    successful_trials: int = 0
    aborted_trials: int = 0
    is_dry_run: bool = False
    wheel_balancing_enabled: bool = False
    dynamic_braking_enabled: bool = False
    stopping_advance_deg: float = 0.0
    trials: List[TrialReport] = field(default_factory=list)
    mean_settled_error_deg: Optional[float] = None
    std_dev_settled_error_deg: Optional[float] = None
    repeatability_deg: Optional[float] = None
    mean_estimate_delta_deg: Optional[float] = None


class ReportGenerator:
    """Serializes reports to JSON and formats readable Markdown summaries."""

    @staticmethod
    def save_json(report: MultiTrialSuiteReport, output_dir: str) -> str:
        os.makedirs(output_dir, exist_ok=True)
        filename = f"turn_{report.direction}_{int(report.target_degrees)}deg_{report.suite_id}.json"
        filepath = os.path.join(output_dir, filename)

        # Custom serializer for dataclasses
        data = asdict(report)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        return filepath

    @staticmethod
    def format_markdown_summary(report: MultiTrialSuiteReport) -> str:
        md = []
        md.append(f"# Physical Turn Test Report: {report.target_degrees:.1f}° {report.direction.upper()}")
        md.append(f"**Suite ID:** `{report.suite_id}` | **Mode:** `{'DRY RUN (STATIONARY)' if report.is_dry_run else 'PHYSICAL EXECUTION'}`")
        md.append(f"**Command:** `{report.command_line}`")
        md.append(f"**Result:** {report.successful_trials}/{report.total_trials} Trials Succeeded\n")

        # Summary Table
        md.append("## Multi-Trial Summary")
        md.append("| Metric | Value |")
        md.append("| :--- | :--- |")
        md.append(f"| Requested Turn | {report.target_degrees:.1f}° ({report.direction.upper()}) |")
        md.append(f"| Total Trials Executed | {report.total_trials} |")
        md.append(f"| Successful Trials | {report.successful_trials} |")
        md.append(f"| Aborted Trials | {report.aborted_trials} |")
        mean_err_str = f"{report.mean_settled_error_deg:+.2f}°" if report.mean_settled_error_deg is not None else "Unavailable"
        md.append(f"| Mean Settled Error | {mean_err_str} |")
        std_err_str = f"{report.std_dev_settled_error_deg:.2f}°" if report.std_dev_settled_error_deg is not None else "Unavailable"
        md.append(f"| Error Standard Deviation | {std_err_str} |")
        rep_str = f"{report.repeatability_deg:.2f}°" if report.repeatability_deg is not None else "Unavailable"
        md.append(f"| Repeatability Span | {rep_str} |")
        if report.mean_estimate_delta_deg is not None:
            md.append(f"| Mean Delta vs Ron's Estimate | {report.mean_estimate_delta_deg:+.2f}° |")
        md.append("")

        # Per-Trial Breakdown
        md.append("## Per-Trial Forensics")
        for t in report.trials:
            status_icon = "[PASS]" if t.status in ("SUCCESS", "DRY_RUN_PASSED") else "[FAIL]"
            md.append(f"### Trial {t.trial_index}: {status_icon} {t.status}")
            if t.abort_reason:
                md.append(f"> [!WARNING]\n> Abort Reason: {t.abort_reason}\n")

            md.append("| Measurement | Value | Description |")
            md.append("| :--- | :--- | :--- |")
            md.append(f"| Target Signed Angle | `{t.target_signed_yaw_deg:+.2f}°` | Intended continuous rotation |")
            md.append(f"| Angle at Zero Command | `{t.gyro_angle_at_zero_cmd_deg:+.2f}°` | IMU reading when $\\omega_z=0$ issued |")
            settled_str = f"`{t.final_settled_gyro_angle_deg:+.2f}°`" if t.final_settled_gyro_angle_deg is not None else "`Unavailable`"
            md.append(f"| Final Settled Angle | {settled_str} | IMU reading after settle period |")
            coast_str = f"`{t.post_zero_rotation_deg:+.2f}°`" if t.post_zero_rotation_deg is not None else "`Unavailable`"
            md.append(f"| Post-Zero Coast | {coast_str} | Rotation between zero cmd and standstill |")
            error_str = f"`{t.settled_heading_error_deg:+.2f}°`" if t.settled_heading_error_deg is not None else "`Unavailable`"
            md.append(f"| Settled Error vs Target | {error_str} | Final error against target |")

            if t.rons_physical_angle_estimate_deg is not None:
                md.append(f"| Ron's Physical Estimate | `{t.rons_physical_angle_estimate_deg:+.2f}°` | Ground truth protractor/laser observation |")
                delta_str = f"{t.estimate_vs_gyro_delta_deg:+.2f}°" if t.estimate_vs_gyro_delta_deg is not None else "N/A"
                md.append(f"| Estimate vs Gyro Delta | `{delta_str}` | Discrepancy (Gyro - Ron) |")
            else:
                md.append(f"| Ron's Physical Estimate | `Not Provided` | Ground truth manual observation |")

            md.append(f"| Opposing Polarity Verified | `{'YES' if t.opposite_polarity_maintained else 'NO (VIOLATION)'}` | Left & Right sides opposed |")
            md.append(f"| Breakout Events | `{t.breakout_event_count}` | Stiction boost activations |")
            md.append(f"| Active Telemetry Samples | `{t.telemetry_samples_count}` | Filtered PID diagnostic packets |")
            if t.zero_command_latency_ms is not None:
                md.append(f"| Zero Command Latency | `{t.zero_command_latency_ms:.1f} ms` | HTTP cmd_vel(0,0) turn-around |")
            if t.settle_duration_s > 0:
                md.append(
                    f"| Settle Telemetry Achieved | `IMU: {t.settle_imu_valid_advancing_count}/{t.settle_imu_poll_count} valid ({t.settle_imu_valid_advancing_rate_hz:.1f} Hz, poll {t.settle_imu_poll_rate_hz:.1f} Hz), "
                    f"PID: {t.settle_pid_post_zero_packets_count} post-zero ({t.settle_pid_post_zero_packet_rate_hz:.1f} Hz, {t.settle_pid_backlog_packets_count} backlog, {t.settle_pid_unknown_packets_count} unk)` | Duration: {t.settle_duration_s:.2f}s |"
                )
            md.append(f"| Wheel Balancing Active | `{'ENABLED' if t.wheel_balancing_enabled else 'DISABLED'}` | Low-speed synchronization trim |")
            md.append(f"| Dynamic Braking Active | `{'ENABLED' if t.dynamic_braking_enabled else 'DISABLED'}` | Shared low-speed brake pulse |")
            if t.stopping_advance_deg > 0.0:
                md.append(f"| Stopping Advance | `{t.stopping_advance_deg:.2f}°` | Pre-target stop trigger angle |")
            if t.actuation_states_observed:
                md.append(f"| Actuation States | `{' -> '.join(t.actuation_states_observed)}` | Firmware drivetrain states |")
            if t.brake_active_duration_ms is not None:
                md.append(f"| Brake Pulse Duration | `{t.brake_active_duration_ms:.1f} ms` | Active dynamic brake period |")
            md.append(f"| Final Zero & Disarmed Confirmed | `{'YES' if (t.confirmed_final_zero_command and t.confirmed_final_disarmed_state) else 'NO (FAIL-SAFE ERROR)'}` | Drivetrain locked & safe |")
            md.append("")

            # Wheel Details
            md.append("#### Wheel Actuation & Encoder Performance")
            md.append("| Wheel (Slot / Corner) | Commanded Target (rad/s) | Measured Mean (rad/s) | Mean Abs Speed (rad/s) | Speed Range [Min, Max] (rad/s) | Encoder Delta (ticks) | Stopped While Commanded | Stiction Boosts |")
            md.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |")
            for w_id in ["m1", "m2", "m3", "m4"]:
                w = t.wheel_metrics.get(w_id, WheelTrialMetrics(wheel_id=w_id))
                wheel_label = f"**{w_id.upper()}** ({w.slot_mapping} / {w.corner})"

                # Commanded target formatting
                if w.validity == "INVALID" or w.commanded_speed_source == "INVALID_ZERO_TELEMETRY":
                    cmd_str = "*INVALID*"
                elif w.commanded_speed_radps_mean is not None:
                    if w.commanded_speed_source == "calculated_expected":
                        cmd_str = f"`{w.commanded_speed_radps_mean:+.2f}` *(calc)*"
                    else:
                        cmd_str = f"`{w.commanded_speed_radps_mean:+.2f}`"
                else:
                    cmd_str = "*Unavailable*"

                # Measured speed formatting
                if w.validity == "INVALID":
                    meas_str = "*INVALID*"
                    abs_str = "*INVALID*"
                    range_str = "*INVALID*"
                else:
                    if w.measured_speed_radps_mean is not None:
                        meas_str = f"`{w.measured_speed_radps_mean:+.2f}`"
                    else:
                        meas_str = "*Unavailable*"

                    if w.measured_speed_radps_abs_mean is not None:
                        abs_str = f"`{w.measured_speed_radps_abs_mean:.2f}`"
                    else:
                        abs_str = "*Unavailable*"

                    if w.measured_speed_radps_min is not None and w.measured_speed_radps_max is not None:
                        range_str = f"`[{w.measured_speed_radps_min:+.2f}, {w.measured_speed_radps_max:+.2f}]`"
                    else:
                        range_str = "*Unavailable*"

                # Stopped while commanded formatting
                if w.stopped_while_commanded is True:
                    stopped_str = f"**YES ({w.stopped_while_commanded_count} ev / {w.stopped_while_commanded_duration_s:.2f}s)**"
                elif w.stopped_while_commanded is False:
                    stopped_str = "No (0s)"
                else:
                    stopped_str = "Unknown"

                md.append(f"| {wheel_label} | {cmd_str} | {meas_str} | {abs_str} | {range_str} | {w.encoder_delta_ticks:+d} | {stopped_str} | {w.stiction_boost_events} |")
            md.append("")

        return "\n".join(md)
