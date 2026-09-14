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

    # Commanded speed metrics (None if telemetry unavailable, never false 0.00)
    commanded_speed_radps_mean: Optional[float] = None
    commanded_speed_radps_max: Optional[float] = None
    commanded_speed_source: str = "firmware_pid"  # "firmware_pid", "calculated_expected", "unavailable"

    # Measured speed metrics (None if telemetry unavailable, never false 0.00)
    measured_speed_radps_mean: Optional[float] = None
    measured_speed_radps_abs_mean: Optional[float] = None
    measured_speed_radps_min: Optional[float] = None
    measured_speed_radps_max: Optional[float] = None

    encoder_start_ticks: int = 0
    encoder_final_ticks: int = 0
    encoder_delta_ticks: int = 0

    stopped_while_commanded: bool = False
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
    final_settled_gyro_angle_deg: float = 0.0
    post_zero_rotation_deg: float = 0.0
    settled_heading_error_deg: float = 0.0
    rons_physical_angle_estimate_deg: Optional[float] = None
    estimate_vs_gyro_delta_deg: Optional[float] = None

    # Wheel & Actuation Metrics
    wheel_metrics: Dict[str, WheelTrialMetrics] = field(default_factory=dict)
    opposite_polarity_maintained: bool = True
    intentional_asymmetry_reported: bool = False
    breakout_assistance_active: bool = False
    breakout_event_count: int = 0
    breakout_dwell_time_ms_total: float = 0.0

    # Safety & State Confirmation
    watchdog_trips: List[str] = field(default_factory=list)
    communication_faults: List[str] = field(default_factory=list)
    confirmed_final_zero_command: bool = True
    confirmed_final_disarmed_state: bool = True
    telemetry_samples_count: int = 0
    autonomy_state_transitions: List[Dict[str, Any]] = field(default_factory=list)
    first_command_response: Dict[str, Any] = field(default_factory=dict)

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
    trials: List[TrialReport] = field(default_factory=list)
    mean_settled_error_deg: float = 0.0
    std_dev_settled_error_deg: float = 0.0
    repeatability_deg: float = 0.0
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
        md.append(f"| Mean Settled Error | {report.mean_settled_error_deg:+.2f}° |")
        md.append(f"| Error Standard Deviation | {report.std_dev_settled_error_deg:.2f}° |")
        md.append(f"| Repeatability Span | {report.repeatability_deg:.2f}° |")
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
            md.append(f"| Final Settled Angle | `{t.final_settled_gyro_angle_deg:+.2f}°` | IMU reading after settle period |")
            md.append(f"| Post-Zero Coast | `{t.post_zero_rotation_deg:+.2f}°` | Rotation between zero cmd and standstill |")
            md.append(f"| Settled Error vs Target | `{t.settled_heading_error_deg:+.2f}°` | Final error against target |")

            if t.rons_physical_angle_estimate_deg is not None:
                md.append(f"| Ron's Physical Estimate | `{t.rons_physical_angle_estimate_deg:+.2f}°` | Ground truth protractor/laser observation |")
                delta_str = f"{t.estimate_vs_gyro_delta_deg:+.2f}°" if t.estimate_vs_gyro_delta_deg is not None else "N/A"
                md.append(f"| Estimate vs Gyro Delta | `{delta_str}` | Discrepancy (Gyro - Ron) |")
            else:
                md.append(f"| Ron's Physical Estimate | `Not Provided` | Ground truth manual observation |")

            md.append(f"| Opposing Polarity Verified | `{'YES' if t.opposite_polarity_maintained else 'NO (VIOLATION)'}` | Left & Right sides opposed |")
            md.append(f"| Breakout Events | `{t.breakout_event_count}` | Stiction boost activations |")
            md.append(f"| Active Telemetry Samples | `{t.telemetry_samples_count}` | Filtered PID diagnostic packets |")
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
                if w.commanded_speed_radps_mean is not None:
                    if w.commanded_speed_source == "calculated_expected":
                        cmd_str = f"`{w.commanded_speed_radps_mean:+.2f}` *(calc)*"
                    else:
                        cmd_str = f"`{w.commanded_speed_radps_mean:+.2f}`"
                else:
                    cmd_str = "*Unavailable*"

                # Measured speed formatting
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
                if w.stopped_while_commanded:
                    stopped_str = f"**YES ({w.stopped_while_commanded_count} ev / {w.stopped_while_commanded_duration_s:.2f}s)**"
                else:
                    stopped_str = "No (0s)"

                md.append(f"| {wheel_label} | {cmd_str} | {meas_str} | {abs_str} | {range_str} | {w.encoder_delta_ticks:+d} | {stopped_str} | {w.stiction_boost_events} |")
            md.append("")

        return "\n".join(md)
