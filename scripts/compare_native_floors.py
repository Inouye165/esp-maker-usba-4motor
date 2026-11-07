#!/usr/bin/env python3
"""
scripts/compare_native_floors.py
Automated comparison runner for native C++ regression tests:
Compares drivetrain controller behavior under minFfFloor = 22 PWM vs minFfFloor = 38 PWM.
Evaluates all 19 test cases against the original tracking assertions and friction scenarios.
"""

import os
import sys
import subprocess
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WHEEL_CONTROLLER_PATH = os.path.join(REPO_ROOT, "src", "WheelController.cpp")

def set_min_ff_floor(floor_value):
    with open(WHEEL_CONTROLLER_PATH, "r", encoding="utf-8") as f:
        content = f.read()
    pattern = r"(float minFfFloor;\s+if \(isSpinManeuver\) \{.*?\} else \{\s+minFfFloor = )([0-9\.]+)f;"
    replacement = r"\g<1>" + f"{floor_value:.1f}f;"
    new_content, count = re.subn(pattern, replacement, content, flags=re.DOTALL)
    if count == 0:
        raise RuntimeError(f"Failed to locate minFfFloor in {WHEEL_CONTROLLER_PATH}")
    with open(WHEEL_CONTROLLER_PATH, "w", encoding="utf-8") as f:
        f.write(new_content)
    print(f"[SET] Configured minFfFloor = {floor_value} PWM in src/WheelController.cpp")

def run_wsl_command(cmd):
    full_cmd = f"wsl bash -c \"cd /mnt/c/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor && {cmd}\""
    res = subprocess.run(full_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return res.returncode, res.stdout, res.stderr

def compile_test_runner():
    cmd = "g++ -O2 -std=c++17 -Iinclude -Isrc -Itest -Itest/mock_include test/test_wheel_controller_native.cpp -DTEST_PATCHED -o test_patched_native"
    rc, out, err = run_wsl_command(cmd)
    if rc != 0:
        print("[COMPILE ERROR]:", err)
        return False
    return True

def run_test_case(test_num):
    cmd = f"./test_patched_native {test_num}"
    rc, out, err = run_wsl_command(cmd)
    return rc, out, err

def main():
    print("=" * 70)
    print("NATIVE REGRESSION SUITE COMPARISON: 22 PWM vs 38 PWM FEEDFORWARD FLOOR")
    print("Preserving original tracking assertions and friction scenarios")
    print("=" * 70)

    results = {22: {}, 38: {}}

    for floor in [22.0, 38.0]:
        print(f"\n--- EVALUATING minFfFloor = {floor} PWM ---")
        set_min_ff_floor(floor)
        if not compile_test_runner():
            sys.exit(1)

        for test_idx in range(1, 20):
            rc, out, err = run_test_case(test_idx)
            passed = (rc == 0)
            results[int(floor)][test_idx] = {
                "passed": passed,
                "stdout": out,
                "stderr": err,
                "returncode": rc
            }
            status_str = "PASS" if passed else "FAIL"
            print(f"  Test {test_idx:2d}: {status_str}")

    # Restore 38 PWM
    set_min_ff_floor(38.0)

    print("\n" + "=" * 70)
    print("DETAILED COMPARISON TABLE: 22 PWM vs 38 PWM")
    print("=" * 70)
    print(f"{'Test #':<8} | {'Description':<35} | {'22 PWM':<10} | {'38 PWM':<10} | {'Discrepancy / Root Cause'}")
    print("-" * 105)

    descriptions = {
        1: "Tight Arc Breakout & Tracking",
        2: "Zero-Cycle & Reversing Arc Bypass",
        3: "Pure Spin",
        4: "One Blocked Wheel",
        5: "Repeated Transitions Blocked",
        6: "Zero Command Disarm & Watchdog",
        7: "Spin Recovery Authoritative State",
        8: "MotionLimiter Stopping Symmetry",
        9: "Recovery Intent & Latched Shutdown",
        10: "Integral Continuity & Anti-Windup",
        11: "Rolling Arc Stall Reproduction",
        12: "Rolling Arc Stall Correction",
        13: "Fwd/Rev Arc Breakout & Tracking",
        14: "FF & Integrator Crossing Immunity",
        15: "Sustained Blockage & Disarm Invariants",
        16: "KINETIC Renewed Stall Recovery",
        17: "Asymmetric Stall & Integration",
        18: "Kinetic FF Floor Invariant (38 PWM)",
        19: "Gradual Nonzero Slowdown & Stop"
    }

    for t in range(1, 20):
        p22 = results[22][t]["passed"]
        p38 = results[38][t]["passed"]
        desc = descriptions.get(t, "Unknown")
        cause = ""

        if not p22 and not p38:
            cause = "Both failed"
        elif p22 and not p38:
            if t == 10:
                cause = "Sub-test 10A: pwm2=61 >= 60 (base FF increased 31.5 -> 38.0)"
            elif t == 12:
                # check 12A vs 12B
                cause = "Sub-test 12A/12B: final PWM unwinding / 75 PWM scrub threshold"
            else:
                cause = "Failed under 38 PWM"
        elif not p22 and p38:
            if t == 18:
                cause = "Test 18 explicitly asserts feedforward == 38.0 PWM"
            else:
                cause = "Passed only under 38 PWM"
        else:
            cause = "Identical pass"

        s22 = "PASS" if p22 else "FAIL"
        s38 = "PASS" if p38 else "FAIL"
        print(f"Test {t:<3d} | {desc:<35} | {s22:<10} | {s38:<10} | {cause}")

    print("=" * 105)

if __name__ == "__main__":
    main()
