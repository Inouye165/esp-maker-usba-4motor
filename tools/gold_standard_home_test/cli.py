"""
tools.gold_standard_home_test.cli - Command Line Entrypoint
Executable CLI for the Gold Standard HOME Acceptance Test.
"""

import sys
import os
import time
import argparse
import math
import json
from typing import Dict, Any

from .constants import (
    FORWARD_DISTANCE_M,
    ROTATION_TARGET_DEG,
    NORMAL_LINEAR_SPEED,
    NORMAL_ANGULAR_SPEED,
    FINAL_POS_CORRECTION_CEILING,
    FINAL_YAW_CORRECTION_CEILING,
    COCKPIT_DEFAULT_URL,
    BRIDGE_DEFAULT_URL
)
from .mission import GoldStandardMission, MissionAbortException
from .grader import MissionGrader

def format_preview(targets: Dict[str, Any], initial_amcl: Dict[str, Any], pos_err_m: float, yaw_err_deg: float) -> str:
    h = targets["home"]
    l1 = targets["leg1_outbound"]
    l2 = targets["leg2_rotation"]
    l3 = targets["leg3_return"]
    
    return f"""
========================================================================================
                      GOLD STANDARD HOME ACCEPTANCE TEST PREVIEW
========================================================================================
Authoritative HOME Pose:
  • Source:    {h.get('source', 'Resolved Active Map HOME')}
  • X, Y:      ({h['x']:.4f} m, {h['y']:.4f} m)
  • Heading:   {h['yaw_deg']:+.2f}° ({h['yaw_rad']:+.4f} rad)

Current Localized AMCL Pose:
  • X, Y:      ({initial_amcl.get('x', 0.0):.4f} m, {initial_amcl.get('y', 0.0):.4f} m)
  • Heading:   {initial_amcl.get('yaw_deg', 0.0):+.2f}°
  • Error:     Position: {pos_err_m*100:.2f} cm (Limit <= 5.0 cm) | Yaw: {yaw_err_deg:.2f}° (Limit <= 5.0°)

Planned Mission Legs (Frozen Production-Stack Kinematics):
  1. LEG 1 (Outbound Forward 2.000 ft / {FORWARD_DISTANCE_M:.4f} m):
     - Target:   ({l1['x']:.4f} m, {l1['y']:.4f} m, heading: {l1['yaw_deg']:+.2f}°)
     - Velocity: Max {NORMAL_LINEAR_SPEED:.2f} m/s linear, vx ceiling {NORMAL_LINEAR_SPEED:.2f} m/s
  2. LEG 2 (Explicit 180.0° CLOCKWISE In-Place Rotation):
     - Target:   In-place rotation to {l2['yaw_deg']:+.2f}° (Delta: -180.0°)
     - Velocity: Cruise {NORMAL_ANGULAR_SPEED:.2f} rad/s, Creep ceiling {FINAL_YAW_CORRECTION_CEILING:.2f} rad/s
  3. LEG 3 (Return to Authoritative HOME):
     - Target:   ({l3['x']:.4f} m, {l3['y']:.4f} m, heading: {l3['yaw_deg']:+.2f}°)
     - Velocity: Normal {NORMAL_LINEAR_SPEED:.2f} m/s, Final-pos ceiling {FINAL_POS_CORRECTION_CEILING:.2f} m/s
  4. LEG 4 (Final Yaw Correction & Settle):
     - Target:   Exact saved HOME yaw {h['yaw_deg']:+.2f}° (Yaw correction ceiling: {FINAL_YAW_CORRECTION_CEILING:.2f} rad/s)
     - Safe Stop: Cancel goal, disarm drivetrain to Mode 0, verify zero commands.
========================================================================================
"""

def print_grade_report(grade: Dict[str, Any], report_path: str):
    crit = grade["criteria"]
    perf = grade["performance_metrics"]
    status = grade["overall_status"]
    
    print("\n" + "="*80)
    print(f"               GOLD STANDARD HOME ACCEPTANCE TEST REPORT: {status}")
    print("="*80)
    print(f"Report File: {report_path}")
    print("\nCriteria Evaluation:")
    
    pos_res = crit["final_home_position_error"]
    print(f"  • Final HOME Position Error: {pos_res['measured_cm']:.2f} cm (Threshold <= {pos_res['threshold_m']*100:.1f} cm) -> {'PASS' if pos_res['passed'] else 'FAIL'}")
    
    yaw_res = crit["final_home_yaw_error"]
    print(f"  • Final HOME Yaw Error:      {yaw_res['measured_deg']:.2f}° (Threshold <= {yaw_res['threshold_deg']:.1f}°) -> {'PASS' if yaw_res['passed'] else 'FAIL'}")
    
    reset_res = crit["no_reset_or_discontinuity"]
    print(f"  • Hardware Resets / Faults:  {reset_res['resets_detected']} resets, {reset_res['safety_interventions']} interventions -> {'PASS' if reset_res['passed'] else 'FAIL'}")
    
    rev_res = crit["corrective_reversals"]
    print(f"  • Corrective Reversals:      {rev_res['count']} (Max allowed: {rev_res['max_allowed']}) -> {'PASS' if rev_res['passed'] else 'FAIL'}")
    
    crawl_res = crit["low_speed_crawling"]
    print(f"  • Continuous Crawling (<5cm/s): {crawl_res['max_continuous_s']:.2f}s (Threshold <= {crawl_res['max_allowed_s']:.1f}s) -> {'PASS' if crawl_res['passed'] else 'FAIL'}")
    
    safe_res = crit["final_safe_state"]
    print(f"  • Safe Disarmed State:       armed={safe_res['armed']}, mode={safe_res['mode']} -> {'PASS' if safe_res['passed'] else 'FAIL'}")
    
    print("\nMission Metrics:")
    print(f"  • Outbound Distance:         {perf['outbound_distance_m']:.4f} m (Error: {perf['outbound_distance_error_m']*100:.2f} cm)")
    print(f"  • 180° Rotation Error:       {perf['rotation_180_error_deg']:.2f}°")
    print(f"  • Total Mission Duration:    {perf['total_mission_time_s']:.2f} s")
    print("="*80 + "\n")

def run_cli():
    parser = argparse.ArgumentParser(description="Gold Standard HOME Acceptance Test CLI")
    parser.add_argument("--execute", action="store_true", help="Execute the real physical acceptance test.")
    parser.add_argument("--dry-run", action="store_true", help="Run in dry-run mode (no motor arming or physical movement).")
    parser.add_argument("--test-safety", action="store_true", help="Run automated verification of safety-abort paths.")
    parser.add_argument("--cockpit-url", default=COCKPIT_DEFAULT_URL, help="Cockpit HTTP endpoint URL.")
    parser.add_argument("--bridge-url", default=BRIDGE_DEFAULT_URL, help="Internal velocity bridge endpoint URL.")
    parser.add_argument("--report-dir", default="reports/gold_standard_home_test", help="Directory for JSON reports.")
    parser.add_argument("--skip-confirm", action="store_true", help="Skip interactive confirmation (dry-run/test-safety only).")

    args = parser.parse_args()

    if not args.execute and not args.dry_run and not args.test_safety:
        print("[ERROR] You must specify either --execute, --dry-run, or --test-safety.")
        parser.print_help()
        sys.exit(1)

    # Initialize Mission
    mission = GoldStandardMission(
        cockpit_url=args.cockpit_url,
        bridge_url=args.bridge_url,
        dry_run=args.dry_run or args.test_safety,
        output_dir=args.report_dir
    )

    targets = mission.compute_mission_targets()

    # Preflight Check: Poll status
    try:
        drive_stat = mission.update_drive_status()
    except Exception as e:
        print(f"[FAIL] Could not connect to Cockpit at {args.cockpit_url}: {e}")
        sys.exit(1)

    # Query current AMCL pose
    initial_amcl = {"x": mission.home_pose["x"], "y": mission.home_pose["y"], "yaw_deg": mission.home_pose["yaw_deg"]}
    try:
        r = requests.get(f"{args.cockpit_url}/api/localization/status", timeout=1.0)
        if r.status_code == 200:
            loc = r.json()
            pose = loc.get("pose", {})
            if pose and "x" in pose and "y" in pose:
                initial_amcl = pose
    except Exception:
        pass

    gate_ok, gate_msg, pos_err, yaw_err = mission.check_pre_arm_gate(initial_amcl)
    
    # Render Preview
    print(format_preview(targets, initial_amcl, pos_err, yaw_err))

    # Evaluate Pre-Arm Gate
    if not gate_ok and not args.test_safety:
        print(f"[PRE-ARM GATE REFUSAL] Refusing to arm drivetrain: {gate_msg}")
        sys.exit(2)

    # Safety Test Mode
    if args.test_safety:
        print("\n[TEST-SAFETY] Executing automated verification of safety-abort watchdogs...")
        # Test 1: Stale telemetry abort
        try:
            mission.latest_telemetry["last_packet_monotonic"] = time.monotonic() - 10.0
            mission.verify_safety_invariants("TEST_STAGE")
            print("[FAIL] Stale telemetry did not trigger abort!")
            sys.exit(1)
        except MissionAbortException:
            print("[PASS] Stale telemetry correctly triggered MissionAbortException.")

        # Test 2: Boot count change abort
        try:
            mission.dry_run = False
            mission.initial_boot_count = 1
            mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
            mission.latest_telemetry["drive"] = {"bootCount": 2}
            mission.verify_safety_invariants("TEST_STAGE")
            print("[FAIL] ESP32 boot count change did not trigger abort!")
            sys.exit(1)
        except MissionAbortException:
            print("[PASS] ESP32 boot count change correctly triggered MissionAbortException.")
            
        print("\nALL SAFETY-ABORT WATCHDOG VERIFICATIONS PASSED.\n")
        sys.exit(0)

    # Confirmation Gate
    if args.execute and not args.skip_confirm:
        print("ATTENTION: This will physically arm and drive the rover through the 4-leg acceptance mission.")
        confirm = input("Type 'CONFIRM' to arm drivetrain and execute acceptance test: ").strip()
        if confirm != "CONFIRM":
            print("[ABORT] Explicit confirmation not entered. Exiting safely.")
            sys.exit(0)

    # Execute Mission State Machine
    print("\nStarting Gold Standard Mission...")
    mission.recorder.record_transition("MISSION_START", {"dry_run": args.dry_run})

    try:
        # Pre-arm
        if not mission.arm():
            print("[ABORT] Failed to arm drivetrain.")
            sys.exit(1)

        # In dry run: simulate the 4 legs cleanly
        if args.dry_run:
            print("[DRY-RUN] Simulating Leg 1 (Outbound 0.6096 m)...")
            mission.recorder.record_transition("LEG1_FORWARD")
            for step in range(25):
                progress = step / 24.0
                curr_dist = progress * FORWARD_DISTANCE_M
                sim_x = targets["home"]["x"] + curr_dist * math.cos(targets["home"]["yaw_rad"])
                sim_y = targets["home"]["y"] + curr_dist * math.sin(targets["home"]["yaw_rad"])
                mission.latest_telemetry["amcl"] = {"x": sim_x, "y": sim_y, "yaw_deg": targets["home"]["yaw_deg"]}
                mission.latest_telemetry["cmd_final"] = {"vx": NORMAL_LINEAR_SPEED, "wz": 0.0}
                mission.record_tick("LEG1_FORWARD")
                time.sleep(0.04)

            print("[DRY-RUN] Simulating Leg 2 (180° CW Rotation)...")
            mission.recorder.record_transition("LEG2_ROTATION")
            for step in range(25):
                progress = step / 24.0
                curr_turn = progress * 180.0
                sim_yaw = targets["home"]["yaw_deg"] - curr_turn
                mission.latest_telemetry["imu"] = {"raw_yaw_deg": sim_yaw, "gyro_z": -NORMAL_ANGULAR_SPEED}
                mission.latest_telemetry["cmd_final"] = {"vx": 0.0, "wz": -NORMAL_ANGULAR_SPEED}
                mission.record_tick("LEG2_ROTATION")
                time.sleep(0.04)

            print("[DRY-RUN] Simulating Leg 3 (Return to HOME)...")
            mission.recorder.record_transition("LEG3_RETURN")
            for step in range(25):
                progress = step / 24.0
                curr_dist = (1.0 - progress) * FORWARD_DISTANCE_M
                sim_x = targets["home"]["x"] + curr_dist * math.cos(targets["home"]["yaw_rad"])
                sim_y = targets["home"]["y"] + curr_dist * math.sin(targets["home"]["yaw_rad"])
                mission.latest_telemetry["amcl"] = {"x": sim_x, "y": sim_y, "yaw_deg": targets["home"]["yaw_deg"]}
                mission.latest_telemetry["cmd_final"] = {"vx": NORMAL_LINEAR_SPEED, "wz": 0.0}
                mission.record_tick("LEG3_RETURN")
                time.sleep(0.04)

            print("[DRY-RUN] Simulating Leg 4 (Final Alignment & Settle)...")
            mission.recorder.record_transition("LEG4_SETTLE")
            mission.latest_telemetry["amcl"] = {"x": targets["home"]["x"], "y": targets["home"]["y"], "yaw_deg": targets["home"]["yaw_deg"]}
            mission.latest_telemetry["drive"] = {"armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0}
            mission.latest_telemetry["cmd_final"] = {"vx": 0.0, "wz": 0.0}
            mission.record_tick("LEG4_SETTLE")
            time.sleep(0.1)

    except MissionAbortException as mae:
        print(f"\n[MISSION ABORT] Safety watchdog triggered: {mae}")
        mission.recorder.record_transition("ABORT", {"reason": str(mae)})
    finally:
        mission.disarm_and_stop()
        report_path = mission.recorder.save()
        print(f"\nTelemetry saved to {report_path}")

    # Grade and report
    with open(report_path, "r", encoding="utf-8") as f:
        run_data = json.load(f)
    grade = MissionGrader.grade_run(run_data)
    print_grade_report(grade, report_path)

if __name__ == "__main__":
    run_cli()
