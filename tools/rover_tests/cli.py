"""
tools.rover_tests.cli - Canonical CLI Entry Point for Rover One Tests

Usage:
    rover-test turn --degrees 180 --direction cw --trials 3 --inter-trial-approval
    python -m tools.rover_tests.cli turn --degrees 90 --direction ccw --dry-run
"""

import os
import sys
import argparse
import traceback
from typing import List, Optional

from .turn import TurnParameters, TurnConfigurationException
from .runner import PhysicalTestRunner
from .transport import TransportException, HandshakeException


def get_default_host() -> str:
    # If running directly on the rover Pi, default to 127.0.0.1
    if os.path.exists("/dev/rover-esp32") or os.path.exists("/home/ron/yahboom-encoder"):
        return "127.0.0.1"
    return "10.0.0.246"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rover-test",
        description="Rover One Reusable Physical-Test Framework (Exact Motion Contract)"
    )
    subparsers = parser.add_subparsers(dest="subcommand", required=True, help="Test subcommand to execute")

    # turn subcommand
    turn_parser = subparsers.add_parser("turn", help="Execute in-place rotation test")
    turn_parser.add_argument("--degrees", type=float, required=True, help="Target turn angle magnitude in degrees (e.g. 90, 180, 360)")
    turn_parser.add_argument("--direction", type=str, choices=["cw", "ccw"], default="cw", help="Turn direction: cw (clockwise) or ccw (counter-clockwise)")
    turn_parser.add_argument("--trials", type=int, default=1, help="Number of consecutive trials to perform (default: 1)")
    turn_parser.add_argument("--repetitions", "--reps", "-r", type=int, default=None, help="Number of repetitions / steps to perform (overrides --trials)")
    turn_parser.add_argument("--max-angular-speed", type=float, default=0.80, help="Cruise angular velocity in rad/s (default: 0.80 per contract)")
    turn_parser.add_argument("--creep-angular-speed", type=float, default=0.20, help="Creep approach angular velocity in rad/s (default: 0.20 per contract)")
    turn_parser.add_argument("--creep-threshold-deg", type=float, default=30.0, help="Approach zone threshold in degrees where creep begins (default: 30.0)")
    turn_parser.add_argument("--settle-seconds", type=float, default=2.0, help="Post-motion standstill settling duration in seconds (default: 2.0)")
    turn_parser.add_argument("--angle-tolerance-deg", type=float, default=2.0, help="Pass/fail angular error tolerance threshold in degrees (default: 2.0)")
    turn_parser.add_argument("--inter-trial-approval", action="store_true", help="Prompt operator for approval before every trial and ask for Ron's physical angle estimate after settling")
    turn_parser.add_argument("--dry-run", action="store_true", help="Simulate execution without sending motor power or arming (safe stationary verification)")
    turn_parser.add_argument("--enable-balancing", action="store_true", help="Enable improved wheel-balancing controls and sync trim")
    turn_parser.add_argument("--enable-braking", "--braking", action="store_true", help="Enable shared low-speed dynamic braking pulse")
    turn_parser.add_argument("--clear-faults", action="store_true", help="Clear latched firmware safety faults if rover is confirmed stationary, zero-commanded, and disarmed (operator-authorized)")
    turn_parser.add_argument("--stopping-advance-deg", type=float, default=None, help="Adjustable stopping advance angle in degrees (default: 0.5 when braking is enabled, 0.0 otherwise)")
    turn_parser.add_argument("--report-directory", type=str, default="reports", help="Directory where JSON and Markdown test reports are saved (default: reports)")
    turn_parser.add_argument("--host", type=str, default=get_default_host(), help="Rover Raspberry Pi 5 IP or hostname (default: auto-detected)")
    turn_parser.add_argument("--port", type=int, default=3000, help="Cockpit server port (default: 3000)")

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.subcommand == "turn":
        reps = args.repetitions if getattr(args, "repetitions", None) is not None else args.trials
        params = TurnParameters(
            degrees=args.degrees,
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

    return 0


if __name__ == "__main__":
    sys.exit(main())
