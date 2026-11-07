import subprocess
import sys
import os

repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
bin_path = os.path.join(repo_dir, "test_patched_native")

test_names = [
    "Tight Arc Breakout & Tracking",
    "Zero-Cycle & Reversing Arc Bypass",
    "Pure Spin",
    "One Blocked Wheel",
    "Repeated Transitions Blocked",
    "Zero Command Disarm & Watchdog",
    "Spin Recovery Authoritative State",
    "MotionLimiter Stopping Symmetry",
    "Recovery Intent & Latched Shutdown",
    "Integral Continuity & Anti-Windup",
    "Rolling Arc Stall Reproduction",
    "Rolling Arc Stall Correction",
    "Fwd/Rev Arc Breakout & Tracking",
    "FF & Integrator Crossing Immunity",
    "Sustained Blockage & Disarm Invariants",
    "KINETIC Renewed Stall Recovery",
    "Asymmetric Stall & Integration",
    "Kinetic FF Floor Invariant (38 PWM - ARCHIVED)",
    "Gradual Nonzero Slowdown & Stop",
    "Straight Startup Boost (64 PWM) & Invariants",
    "Straight Kinetic FF Floor (56 PWM) & Handoff",
    "Straight Crawl Kinetic Integral Gain",
    "Separate Recovery Assist Candidate",
    "Ultra-Low Speed Crawl Candidates"
]

print("=" * 70)
print("RUNNING COMPLETE NATIVE REGRESSION SUITE (Canonical Tests 1-17, 19-24)")
print("=" * 70)

use_wsl = False
if sys.platform == "win32" and os.path.exists(bin_path):
    with open(bin_path, "rb") as f:
        if f.read(4) == b"\x7fELF":
            use_wsl = True

all_passed = True
for i in range(1, 25):
    if i == 18:
        continue  # Test 18 is archived (superseded by Test 21-24)
    cmd = ["wsl", "./test_patched_native", str(i)] if use_wsl else [bin_path, str(i)]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, cwd=repo_dir)
    status = "PASS" if res.returncode == 0 else "FAIL"
    desc = test_names[i-1] if i-1 < len(test_names) else ""
    print(f"Test {i:2d} | {desc:<44} | {status}")
    if res.returncode != 0:
        all_passed = False
        print(f"       -> Details: {res.stderr.strip() or res.stdout.strip()[-200:]}")

print("=" * 70)
print("Result: " + ("ALL PASSED" if all_passed else "FAILURES ENCOUNTERED"))
print("=" * 70)
sys.exit(0 if all_passed else 1)
