"""
tools.rover_tests.cli - Canonical CLI Entry Point for Rover One Tests

Usage:
    rover-test turn --degrees 180 --direction cw --trials 3 --inter-trial-approval
    python -m tools.rover_tests.cli turn --degrees 90 --direction ccw --dry-run
"""

import os
import sys
import json
import time
import argparse
import traceback
from typing import List, Optional

from .turn import TurnParameters, TurnConfigurationException
from .linear import LinearParameters, LinearConfigurationException
from .runner import PhysicalTestRunner
from .transport import TransportException, HandshakeException


def get_default_host() -> str:
    # If running directly on the rover Pi, default to 127.0.0.1
    if os.path.exists("/dev/rover-esp32") or os.path.exists("/home/ron/yahboom-encoder"):
        return "127.0.0.1"
    return "10.0.0.246"


def prompt_target_degrees(default: float = 180.0, prompt_fn=input) -> float:
    while True:
        try:
            val_str = prompt_fn(f"Enter target turn degrees (e.g. 90, 180, 350) [{default:.0f}]: ").strip()
            if not val_str:
                return default
            val = float(val_str)
            if val <= 0:
                print("Degrees must be positive. Please try again.")
                continue
            return val
        except ValueError:
            print("Invalid input. Please enter a valid number for degrees (e.g. 90, 180, 350).")
        except (EOFError, KeyboardInterrupt):
            print("\nOperation cancelled by operator.")
            sys.exit(130)


def prompt_target_distance(default: float = 1.0, prompt_fn=input) -> float:
    while True:
        try:
            val_str = prompt_fn(f"Enter target distance in meters (e.g. 0.5, 1.0, 2.0) [{default:.2f}]: ").strip()
            if not val_str:
                return default
            val = float(val_str)
            if val <= 0:
                print("Distance must be positive. Please try again.")
                continue
            return val
        except ValueError:
            print("Invalid input. Please enter a valid positive number for distance in meters.")
        except (EOFError, KeyboardInterrupt):
            print("\nOperation cancelled by operator.")
            sys.exit(130)


def prompt_linear_direction(default: str = "forward", prompt_fn=input) -> str:
    while True:
        try:
            val_str = prompt_fn(f"Enter direction (forward / reverse) [{default}]: ").strip().lower()
            if not val_str:
                return default
            if val_str in ("forward", "fwd", "f"):
                return "forward"
            if val_str in ("reverse", "rev", "r", "back", "backward"):
                return "reverse"
            print("Invalid direction. Please enter 'forward' or 'reverse'.")
        except (EOFError, KeyboardInterrupt):
            print("\nOperation cancelled by operator.")
            sys.exit(130)


def prompt_repetitions_count(default: int = 1, prompt_fn=input) -> int:
    while True:
        try:
            val_str = prompt_fn(f"Enter number of repetitions (1-8) [{default}]: ").strip()
            if not val_str:
                return default
            val = int(val_str)
            if 1 <= val <= 8:
                return val
            print("Repetitions must be between 1 and 8. Please try again.")
        except ValueError:
            print("Invalid input. Please enter an integer between 1 and 8.")
        except (EOFError, KeyboardInterrupt):
            print("\nOperation cancelled by operator.")
            sys.exit(130)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rover-test",
        description="Rover One Reusable Physical-Test Framework (Exact Motion Contract)"
    )
    subparsers = parser.add_subparsers(dest="subcommand", required=True, help="Test subcommand to execute")

    # turn subcommand
    turn_parser = subparsers.add_parser("turn", help="Execute in-place rotation test")
    turn_parser.add_argument("--degrees", type=float, default=None, help="Target turn angle magnitude in degrees (e.g. 90, 180, 360)")
    turn_parser.add_argument("--direction", type=str, choices=["cw", "ccw"], default="cw", help="Turn direction: cw (clockwise) or ccw (counter-clockwise)")
    turn_parser.add_argument("--trials", type=int, default=1, help="Number of consecutive trials to perform (default: 1)")
    turn_parser.add_argument("--repetitions", "--reps", "-r", type=int, default=None, help="Number of repetitions / steps to perform (1-8, overrides --trials)")
    turn_parser.add_argument("--interactive", "-i", action="store_true", help="Prompt operator interactively for degrees and repetitions")
    turn_parser.add_argument("--max-angular-speed", type=float, default=0.80, help="Cruise angular velocity in rad/s (default: 0.80 per contract)")
    turn_parser.add_argument("--creep-angular-speed", type=float, default=0.20, help="Creep approach angular velocity in rad/s (default: 0.20 per contract)")
    turn_parser.add_argument("--creep-threshold-deg", type=float, default=30.0, help="Approach zone threshold in degrees where creep begins (default: 30.0)")
    turn_parser.add_argument("--settle-seconds", type=float, default=2.0, help="Post-motion standstill settling duration in seconds (default: 2.0)")
    turn_parser.add_argument("--angle-tolerance-deg", type=float, default=2.0, help="Pass/fail angular error tolerance threshold in degrees (default: 2.0)")
    turn_parser.add_argument("--inter-trial-approval", action="store_true", help="Prompt operator for approval before every trial and ask for Ron's physical angle estimate after settling")
    turn_parser.add_argument("--dry-run", action="store_true", help="Simulate execution without sending motor power or arming (safe stationary verification)")
    turn_parser.add_argument("--enable-balancing", action=argparse.BooleanOptionalAction, default=None, help="Enable improved wheel-balancing controls and sync trim")
    turn_parser.add_argument("--enable-braking", "--braking", action=argparse.BooleanOptionalAction, default=None, help="Enable shared low-speed dynamic braking pulse")
    turn_parser.add_argument("--clear-faults", action="store_true", help="Clear latched firmware safety faults if rover is confirmed stationary, zero-commanded, and disarmed (operator-authorized)")
    turn_parser.add_argument("--stopping-advance-deg", type=float, default=None, help="Adjustable stopping advance angle in degrees (default: 0.7 when braking is enabled, 0.0 otherwise)")
    turn_parser.add_argument("--report-directory", type=str, default="reports", help="Directory where JSON and Markdown test reports are saved (default: reports)")
    turn_parser.add_argument("--host", type=str, default=get_default_host(), help="Rover Raspberry Pi 5 IP or hostname (default: auto-detected)")
    turn_parser.add_argument("--port", type=int, default=3000, help="Cockpit server port (default: 3000)")

    # linear subcommand
    linear_parser = subparsers.add_parser("linear", help="Execute straight-line translation test (forward / reverse)")
    linear_parser.add_argument("--distance", type=float, default=None, help="Target distance magnitude in meters (e.g. 0.5, 1.0, 2.0)")
    linear_parser.add_argument("--direction", type=str, choices=["forward", "reverse"], default="forward", help="Direction of travel: forward or reverse (default: forward)")
    linear_parser.add_argument("--trials", type=int, default=1, help="Number of consecutive trials to perform (default: 1)")
    linear_parser.add_argument("--repetitions", "--reps", "-r", type=int, default=None, help="Number of repetitions / steps to perform (1-8, overrides --trials)")
    linear_parser.add_argument("--interactive", "-i", action="store_true", help="Prompt operator interactively for distance, direction, and repetitions")
    linear_parser.add_argument("--max-linear-speed", "--speed-mps", "--speed", type=float, default=0.20, help="Cruise linear velocity in m/s (default: 0.20 per contract)")
    linear_parser.add_argument("--creep-linear-speed", type=float, default=0.05, help="Creep approach linear velocity in m/s (default: 0.05 per contract)")
    linear_parser.add_argument("--creep-threshold-m", type=float, default=0.15, help="Approach zone threshold in meters where creep begins (default: 0.15 per contract)")
    linear_parser.add_argument("--settle-seconds", type=float, default=2.0, help="Post-motion standstill settling duration in seconds (default: 2.0)")
    linear_parser.add_argument("--distance-tolerance-m", type=float, default=0.05, help="Pass/fail distance error tolerance threshold in meters (default: 0.05)")
    linear_parser.add_argument("--inter-trial-approval", action="store_true", help="Prompt operator for approval before every trial and ask for Ron's physical distance estimate after settling")
    linear_parser.add_argument("--dry-run", action="store_true", help="Simulate execution without sending motor power or arming (safe stationary verification)")
    linear_parser.add_argument("--enable-balancing", action=argparse.BooleanOptionalAction, default=None, help="Enable improved wheel-balancing controls and sync trim")
    linear_parser.add_argument("--enable-braking", "--braking", action=argparse.BooleanOptionalAction, default=None, help="Enable shared low-speed dynamic braking pulse")
    linear_parser.add_argument("--stopping-advance-m", type=float, default=None, help="Pre-target stopping distance advance in meters (default: 0.0135m calibrated two-phase creep stopping advance)")
    linear_parser.add_argument("--clear-faults", action="store_true", help="Clear latched firmware safety faults if rover is confirmed stationary, zero-commanded, and disarmed (operator-authorized)")
    linear_parser.add_argument("--report-directory", type=str, default="reports", help="Directory where JSON and Markdown test reports are saved (default: reports)")
    linear_parser.add_argument("--host", type=str, default=get_default_host(), help="Rover Raspberry Pi 5 IP or hostname (default: auto-detected)")
    linear_parser.add_argument("--port", type=int, default=3000, help="Cockpit server port (default: 3000)")

    # out-and-back subcommand
    oab_parser = subparsers.add_parser("out-and-back", help="Execute compound out-and-back test: forward -> 180° turn -> forward return")
    oab_parser.add_argument("--distance", type=float, default=1.0, help="Outbound / return target distance in meters (default: 1.0)")
    oab_parser.add_argument("--turn-direction", type=str, choices=["cw", "ccw"], default="cw", help="Turn direction for reciprocal heading (default: cw)")
    oab_parser.add_argument("--turn-degrees", type=float, default=180.0, help="Turn angle in degrees (default: 180.0)")
    oab_parser.add_argument("--stopping-advance-m", type=float, default=0.0135, help="Calibrated linear stopping advance in meters (default: 0.0135)")
    oab_parser.add_argument("--stopping-advance-deg", type=float, default=0.7, help="Calibrated angular stopping advance in degrees (default: 0.7)")
    oab_parser.add_argument("--settle-seconds", type=float, default=2.0, help="Settling duration between legs in seconds (default: 2.0)")
    oab_parser.add_argument("--dry-run", action="store_true", help="Simulate execution without sending motor power or arming")
    oab_parser.add_argument("--clear-faults", action="store_true", help="Clear latched firmware safety faults if rover is disarmed")
    oab_parser.add_argument("--final-turn", action="store_true", help="Perform a final 180° turnaround at the origin to face initial outbound heading (4-leg complete cycle)")
    oab_parser.add_argument("--report-directory", type=str, default="reports", help="Directory where JSON and Markdown test reports are saved (default: reports)")
    oab_parser.add_argument("--host", type=str, default=get_default_host(), help="Rover Raspberry Pi 5 IP or hostname (default: auto-detected)")
    oab_parser.add_argument("--port", type=int, default=3000, help="Cockpit server port (default: 3000)")

    return parser


def main(argv: Optional[List[str]] = None, prompt_fn=input, is_interactive: Optional[bool] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.subcommand == "turn":
        if is_interactive is None:
            is_interactive = sys.stdin.isatty() or getattr(args, "interactive", False)

        degrees = args.degrees
        reps = args.repetitions

        # Prompt for target degrees if missing or interactive requested
        if getattr(args, "interactive", False) or (degrees is None and is_interactive):
            default_deg = degrees if degrees is not None else 180.0
            degrees = prompt_target_degrees(default=default_deg, prompt_fn=prompt_fn)
        elif degrees is None:
            print("[CONFIGURATION ERROR] Target degrees must be specified (e.g. --degrees 180) in non-interactive mode.", file=sys.stderr)
            return 2

        # Prompt for repetitions if missing or interactive requested
        if getattr(args, "interactive", False) or (reps is None and is_interactive):
            default_reps = reps if reps is not None else (args.trials if 1 <= args.trials <= 8 else 1)
            reps = prompt_repetitions_count(default=default_reps, prompt_fn=prompt_fn)
        elif reps is None:
            reps = args.trials

        params = TurnParameters(
            degrees=degrees,
            direction=args.direction,
            trials=reps,
            repetitions=reps,
            max_angular_speed=args.max_angular_speed,
            creep_angular_speed=args.creep_angular_speed,
            creep_threshold_deg=args.creep_threshold_deg,
            settle_seconds=args.settle_seconds,
            angle_tolerance_deg=args.angle_tolerance_deg,
            inter_trial_approval=args.inter_trial_approval,
            dry_run=args.dry_run,
            enable_balancing=args.enable_balancing,
            enable_braking=args.enable_braking,
            clear_faults=args.clear_faults,
            stopping_advance_deg=args.stopping_advance_deg,
            report_directory=args.report_directory,
            host=args.host,
            port=args.port
        )

        try:
            params.validate()
        except TurnConfigurationException as e:
            print(f"[CONFIGURATION ERROR] {e}", file=sys.stderr)
            return 2

        runner = PhysicalTestRunner(params)
        try:
            suite_report = runner.execute_suite()
            if suite_report.aborted_trials > 0:
                return 1
            return 0
        except TransportException as e:
            tb_str = traceback.format_exc()
            print(f"\n[TRANSPORT ERROR] [{type(e).__name__}] {e}", file=sys.stderr)
            print("Traceback:", file=sys.stderr)
            for line in tb_str.strip().splitlines():
                print(f"  {line}", file=sys.stderr)
            return 3
        except KeyboardInterrupt:
            print("\n[INTERRUPTED] Operation interrupted by operator. Drivetrain disarmed.", file=sys.stderr)
            return 130
        except Exception as e:
            tb_str = traceback.format_exc()
            print(f"\n[EXECUTION ERROR] Unexpected fault: [{type(e).__name__}] {e}", file=sys.stderr)
            print("Traceback:", file=sys.stderr)
            for line in tb_str.strip().splitlines():
                print(f"  {line}", file=sys.stderr)
            return 4

    elif args.subcommand == "linear":
        if is_interactive is None:
            is_interactive = sys.stdin.isatty() or getattr(args, "interactive", False)

        distance = args.distance
        direction = args.direction
        reps = args.repetitions

        # Prompt for target distance if missing or interactive requested
        if getattr(args, "interactive", False) or (distance is None and is_interactive):
            default_dist = distance if distance is not None else 1.0
            distance = prompt_target_distance(default=default_dist, prompt_fn=prompt_fn)
        elif distance is None:
            print("[CONFIGURATION ERROR] Target distance must be specified (e.g. --distance 1.0) in non-interactive mode.", file=sys.stderr)
            return 2

        # Prompt for direction if interactive requested
        if getattr(args, "interactive", False) and is_interactive:
            direction = prompt_linear_direction(default=direction, prompt_fn=prompt_fn)

        # Prompt for repetitions if missing or interactive requested
        if getattr(args, "interactive", False) or (reps is None and is_interactive):
            default_reps = reps if reps is not None else (args.trials if 1 <= args.trials <= 8 else 1)
            reps = prompt_repetitions_count(default=default_reps, prompt_fn=prompt_fn)
        elif reps is None:
            reps = args.trials

        linear_params = LinearParameters(
            distance_m=distance,
            direction=direction,
            trials=reps,
            repetitions=reps,
            max_linear_speed=args.max_linear_speed,
            creep_linear_speed=args.creep_linear_speed,
            creep_threshold_m=args.creep_threshold_m,
            settle_seconds=args.settle_seconds,
            distance_tolerance_m=args.distance_tolerance_m,
            inter_trial_approval=args.inter_trial_approval,
            dry_run=args.dry_run,
            enable_balancing=args.enable_balancing,
            enable_braking=args.enable_braking,
            clear_faults=args.clear_faults,
            stopping_advance_m=args.stopping_advance_m,
            report_directory=args.report_directory,
            host=args.host,
            port=args.port
        )

        try:
            linear_params.validate()
        except LinearConfigurationException as e:
            print(f"[CONFIGURATION ERROR] {e}", file=sys.stderr)
            return 2

        runner = PhysicalTestRunner(linear_params)
        try:
            suite_report = runner.execute_suite()
            if suite_report.aborted_trials > 0:
                return 1
            return 0
        except TransportException as e:
            tb_str = traceback.format_exc()
            print(f"\n[TRANSPORT ERROR] [{type(e).__name__}] {e}", file=sys.stderr)
            print("Traceback:", file=sys.stderr)
            for line in tb_str.strip().splitlines():
                print(f"  {line}", file=sys.stderr)
            return 3
        except KeyboardInterrupt:
            print("\n[INTERRUPTED] Operation interrupted by operator. Drivetrain disarmed.", file=sys.stderr)
            return 130
        except Exception as e:
            tb_str = traceback.format_exc()
            print(f"\n[EXECUTION ERROR] Unexpected fault: [{type(e).__name__}] {e}", file=sys.stderr)
            print("Traceback:", file=sys.stderr)
            for line in tb_str.strip().splitlines():
                print(f"  {line}", file=sys.stderr)
            return 4

    elif args.subcommand == "out-and-back":
        print("=" * 80)
        print("ROVER ONE REUSABLE PHYSICAL TEST FRAMEWORK: COMPOUND OUT-AND-BACK")
        print(f"Leg 1: {args.distance:.3f}m Forward | Leg 2: {args.turn_degrees:.1f}° {args.turn_direction.upper()} Turn | Leg 3: {args.distance:.3f}m Return Forward")
        print(f"Stopping Advance: Linear {args.stopping_advance_m*1000:.1f}mm | Angular {args.stopping_advance_deg:.2f}°")
        print(f"Mode: {'DRY RUN (STATIONARY - NO MOTOR POWER)' if args.dry_run else 'PHYSICAL MOTION EXECUTION'}")
        print("-" * 80)
        print("1. TEST GOAL:")
        print("   Validate continuous compound out-and-back autonomous motion:")
        print("   forward distance, in-place 180° rotation, and return translation to origin.")
        print("2. WHAT TO MONITOR & MEASURE (OPERATOR):")
        print("   • Ensure >= 1.5 m clear floor space in front of Rover 1.")
        print("   • Watch Rover 1 drive forward 1.0m, pivot 180° in place at the 1m mark,")
        print("     and drive forward 1.0m back to the starting line.")
        print("   • Measure final return landing displacement relative to origin reference mark.")
        print("3. EXPECTED RESULT (PASS CRITERIA):")
        print("   • Final return displacement <= ±50 mm from origin line.")
        print("   • Zero ESP32 power-sag resets (bootCount invariant).")
        print("   • Finished safely stopped, disarmed, and locked in Mode 0.")
        print("=" * 80)

        # Step 1: Forward Translation
        print("\n" + "=" * 80)
        print(">>> COMPOUND STEP 1/3: FORWARD TRANSLATION <<<")
        print("=" * 80)
        leg1_params = LinearParameters(
            distance_m=args.distance,
            direction="forward",
            trials=1,
            repetitions=1,
            stopping_advance_m=args.stopping_advance_m,
            settle_seconds=args.settle_seconds,
            dry_run=args.dry_run,
            clear_faults=args.clear_faults,
            report_directory=args.report_directory,
            host=args.host,
            port=args.port
        )
        try:
            leg1_params.validate()
        except LinearConfigurationException as e:
            print(f"[CONFIGURATION ERROR] {e}", file=sys.stderr)
            return 2

        runner1 = PhysicalTestRunner(leg1_params)
        report1 = runner1.execute_suite()
        if report1.aborted_trials > 0:
            print("\n[COMPOUND ABORTED] Leg 1 (Forward) failed or was aborted.", file=sys.stderr)
            return 1

        # Pause between legs for mechanical and electrical settling
        time.sleep(1.0)

        # Step 2: Reciprocal 180° Turn
        print("\n" + "=" * 80)
        print(f">>> COMPOUND STEP 2/3: RECIPROCAL {args.turn_degrees:.1f}° {args.turn_direction.upper()} IN-PLACE TURN <<<")
        print("=" * 80)
        leg2_params = TurnParameters(
            degrees=args.turn_degrees,
            direction=args.turn_direction,
            trials=1,
            repetitions=1,
            stopping_advance_deg=args.stopping_advance_deg,
            settle_seconds=args.settle_seconds,
            dry_run=args.dry_run,
            report_directory=args.report_directory,
            host=args.host,
            port=args.port
        )
        try:
            leg2_params.validate()
        except TurnConfigurationException as e:
            print(f"[CONFIGURATION ERROR] {e}", file=sys.stderr)
            return 2

        runner2 = PhysicalTestRunner(leg2_params)
        report2 = runner2.execute_suite()
        if report2.aborted_trials > 0:
            print("\n[COMPOUND ABORTED] Leg 2 (Turn) failed or was aborted.", file=sys.stderr)
            return 1

        time.sleep(1.0)

        # Step 3: Return Forward Translation
        print("\n" + "=" * 80)
        print(">>> COMPOUND STEP 3/3: RETURN TRANSLATION <<<")
        print("=" * 80)
        leg3_params = LinearParameters(
            distance_m=args.distance,
            direction="forward",
            trials=1,
            repetitions=1,
            stopping_advance_m=args.stopping_advance_m,
            settle_seconds=args.settle_seconds,
            dry_run=args.dry_run,
            report_directory=args.report_directory,
            host=args.host,
            port=args.port
        )
        try:
            leg3_params.validate()
        except LinearConfigurationException as e:
            print(f"[CONFIGURATION ERROR] {e}", file=sys.stderr)
            return 2

        runner3 = PhysicalTestRunner(leg3_params)
        report3 = runner3.execute_suite()
        if report3.aborted_trials > 0:
            print("\n[COMPOUND ABORTED] Leg 3 (Return) failed or was aborted.", file=sys.stderr)
            return 1

        report4 = None
        if args.final_turn:
            time.sleep(1.0)
            print("\n" + "=" * 80)
            print(f">>> COMPOUND STEP 4/4: FINAL RECIPROCAL {args.turn_degrees:.1f}° {args.turn_direction.upper()} IN-PLACE TURN (FACE ORIGIN) <<<")
            print("=" * 80)
            leg4_params = TurnParameters(
                degrees=args.turn_degrees,
                direction=args.turn_direction,
                trials=1,
                repetitions=1,
                stopping_advance_deg=args.stopping_advance_deg,
                settle_seconds=args.settle_seconds,
                dry_run=args.dry_run,
                report_directory=args.report_directory,
                host=args.host,
                port=args.port
            )
            try:
                leg4_params.validate()
            except TurnConfigurationException as e:
                print(f"[CONFIGURATION ERROR] {e}", file=sys.stderr)
                return 2

            runner4 = PhysicalTestRunner(leg4_params)
            report4 = runner4.execute_suite()
            if report4.aborted_trials > 0:
                print("\n[COMPOUND ABORTED] Leg 4 (Final Turn) failed or was aborted.", file=sys.stderr)
                return 1

        print("\n" + "=" * 80)
        print(">>> COMPOUND OUT-AND-BACK SEQUENCE COMPLETED SUCCESSFULLY <<<")
        turn_t = report2.trials[0]
        turn_ang = turn_t.final_settled_gyro_angle_deg if turn_t.final_settled_gyro_angle_deg is not None else 0.0
        turn_err = turn_t.settled_heading_error_deg if turn_t.settled_heading_error_deg is not None else 0.0
        print(f"Leg 1 (Forward):   {report1.trials[0].measured_distance_m:.4f}m (Err: {report1.trials[0].settled_distance_error_m*1000:+.1f}mm)")
        print(f"Leg 2 (Turn):      {turn_ang:.2f}° (Err: {turn_err:+.2f}°)")
        print(f"Leg 3 (Return):    {report3.trials[0].measured_distance_m:.4f}m (Err: {report3.trials[0].settled_distance_error_m*1000:+.1f}mm)")
        if report4 is not None:
            t4 = report4.trials[0]
            a4 = t4.final_settled_gyro_angle_deg if t4.final_settled_gyro_angle_deg is not None else 0.0
            e4 = t4.settled_heading_error_deg if t4.settled_heading_error_deg is not None else 0.0
            print(f"Leg 4 (Face Fwd):  {a4:.2f}° (Err: {e4:+.2f}°)")
        print("=" * 80)
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
