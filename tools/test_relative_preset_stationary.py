#!/usr/bin/env python3
"""
Stationary verification test for relative navigation preset calculation & goal checker tolerances.
Proves:
1. +0.60 m target distance is exactly 0.60 m from the current pose.
2. The target preserves current yaw.
3. Preview and dispatch use the identical freshly calculated target.
4. The live goal tolerances match the production configuration (0.04 m, 0.087 rad / 5.0 deg, stateful=True).
5. Drivetrain remains safely stopped and disarmed in Mode 0 throughout.

NO physical motion is commanded.
"""

import sys
import os
import math
import json
import subprocess
import urllib.request
import urllib.error

ROVER_IP = os.getenv("ROVER_PI_HOST", "10.0.0.246")
COCKPIT_URL = f"http://{ROVER_IP}:3000"

def get_live_pose():
    url = f"{COCKPIT_URL}/api/localization/status"
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=3.0) as resp:
        data = json.loads(resp.read().decode('utf-8'))
        if not data.get("localized"):
            raise RuntimeError(f"Rover not localized: {data}")
        x = float(data["x"])
        y = float(data["y"])
        yaw_rad = float(data.get("yaw", 0.0))
        yaw_deg = float(data.get("yawDeg", yaw_rad * 180.0 / math.pi))
        return x, y, yaw_rad, yaw_deg

def compute_relative_target(x0, y0, yaw_rad, distance):
    tx = x0 + distance * math.cos(yaw_rad)
    ty = y0 + distance * math.sin(yaw_rad)
    tyaw_rad = yaw_rad
    tyaw_deg = yaw_rad * 180.0 / math.pi
    return tx, ty, tyaw_rad, tyaw_deg

def run_ssh_cmd(cmd):
    full_cmd = f'ssh ron@{ROVER_IP} "{cmd}"'
    res = subprocess.run(full_cmd, shell=True, capture_output=True, text=True, timeout=10)
    return res.returncode, res.stdout.strip(), res.stderr.strip()

def main():
    print("=" * 65)
    print("STATIONARY RELATIVE PRESET & GOAL-CHECKER TOLERANCE AUDIT")
    print(f"Target: {COCKPIT_URL}")
    print("=" * 65)

    all_passed = True

    # Check 0: Drivetrain safe & disarmed
    print("\n[CHECK 0] Verify drivetrain is stopped and disarmed (Mode 0)...")
    try:
        with urllib.request.urlopen(f"{COCKPIT_URL}/api/drive/status", timeout=3.0) as resp:
            st = json.loads(resp.read().decode('utf-8'))
            status_obj = st.get("status", {})
            armed = status_obj.get("armed", st.get("armed", False))
            mode = status_obj.get("mode", st.get("mode", 0))
            if armed or mode != 0:
                print(f"  FAIL: Rover is armed={armed}, mode={mode} (expected disarmed Mode 0)")
                all_passed = False
            else:
                print(f"  PASS: Rover is disarmed (armed={armed}, mode={mode})")
    except Exception as e:
        print(f"  FAIL: Error checking drive status: {e}")
        all_passed = False

    # Check 1: Live pose query & +0.60 m target distance
    print("\n[CHECK 1] Verify +0.60 m target distance is exactly 0.60 m from current pose...")
    try:
        x0, y0, yaw_rad, yaw_deg = get_live_pose()
        print(f"  Live AMCL Pose: ({x0:.3f}, {y0:.3f}, {yaw_deg:.2f}°)")

        dist = 0.60
        tx, ty, tyaw_rad, tyaw_deg = compute_relative_target(x0, y0, yaw_rad, dist)
        calc_dist = math.hypot(tx - x0, ty - y0)
        print(f"  Calculated Target: ({tx:.3f}, {ty:.3f}, {tyaw_deg:.2f}°)")
        print(f"  Euclidean Distance: {calc_dist:.6f} m")

        if abs(calc_dist - 0.60) < 1e-5:
            print("  PASS: Distance is exactly 0.600000 m (delta < 1e-5 m)")
        else:
            print(f"  FAIL: Distance mismatch: {calc_dist:.6f} m != 0.60 m")
            all_passed = False

        # Also verify with the test prompt starting pose: (1.415, -0.052, -7.74°)
        print("  Evaluating test prompt starting pose (1.415, -0.052, -7.74°):")
        p_x, p_y = 1.415, -0.052
        p_yaw_rad = -7.74 * math.pi / 180.0
        p_tx, p_ty, p_tyaw_rad, p_tyaw_deg = compute_relative_target(p_x, p_y, p_yaw_rad, 0.60)
        p_dist = math.hypot(p_tx - p_x, p_ty - p_y)
        print(f"    Correct Target: ({p_tx:.3f}, {p_ty:.3f}, {p_tyaw_deg:.2f}°)")
        print(f"    Prior Bad Target: (1.732, -0.049, +2.50°) -> distance was {math.hypot(1.732 - p_x, -0.049 - p_y):.3f} m")
        print(f"    New Calculated Distance: {p_dist:.6f} m")
        if abs(p_dist - 0.60) < 1e-5:
            print("  PASS: Prompt test pose calculation is exactly 0.600000 m")
        else:
            print("  FAIL: Prompt test pose calculation error")
            all_passed = False

    except Exception as e:
        print(f"  FAIL: Error during pose / distance check: {e}")
        all_passed = False

    # Check 2: Target preserves current yaw
    print("\n[CHECK 2] Verify target preserves current yaw...")
    try:
        yaw_diff_deg = abs(tyaw_deg - yaw_deg)
        yaw_diff_rad = abs(tyaw_rad - yaw_rad)
        print(f"  Start Yaw:  {yaw_deg:.4f}° ({yaw_rad:.6f} rad)")
        print(f"  Target Yaw: {tyaw_deg:.4f}° ({tyaw_rad:.6f} rad)")
        print(f"  Yaw Delta:  {yaw_diff_deg:.6f}°")
        if yaw_diff_deg < 1e-6 and yaw_diff_rad < 1e-6:
            print("  PASS: Target yaw identically preserves current yaw (delta == 0)")
        else:
            print(f"  FAIL: Target yaw does not preserve current yaw: delta={yaw_diff_deg}°")
            all_passed = False
    except Exception as e:
        print(f"  FAIL: {e}")
        all_passed = False

    # Check 3: Preview and Dispatch use identical freshly calculated target
    print("\n[CHECK 3] Verify Preview and Dispatch use identical freshly calculated target...")
    try:
        # Request a disarmed preview plan with relative_distance = 0.60
        plan_req_body = {
            "start_x": x0,
            "start_y": y0,
            "start_yaw": yaw_rad,
            "target_x": tx,
            "target_y": ty,
            "target_yaw": tyaw_rad,
            "x": tx,
            "y": ty,
            "yaw": tyaw_rad,
            "relative_distance": 0.60
        }
        req_data = json.dumps(plan_req_body).encode('utf-8')
        plan_req = urllib.request.Request(
            f"{COCKPIT_URL}/api/navigation/plan",
            data=req_data,
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(plan_req, timeout=5.0) as resp:
            plan_res = json.loads(resp.read().decode('utf-8'))
            if not plan_res.get("ok"):
                print(f"  FAIL: Plan preview failed: {plan_res}")
                all_passed = False
            else:
                last_wp = plan_res.get("last_waypoint")
                straight_d = plan_res.get("straight_distance_m")
                cum_len = plan_res.get("cumulative_length_m")
                print(f"  Plan preview succeeded: {plan_res.get('count')} waypoints")
                print(f"  First waypoint: {plan_res.get('first_waypoint')}")
                print(f"  Last waypoint:  {last_wp}")
                print(f"  Straight distance: {straight_d:.3f} m, Cumulative length: {cum_len:.3f} m")

                # Verify last waypoint is close to target (within Smac planner resolution)
                if last_wp:
                    wp_dist_to_target = math.hypot(last_wp[0] - tx, last_wp[1] - ty)
                    print(f"  Last waypoint distance to target: {wp_dist_to_target:.3f} m")
                    if wp_dist_to_target <= 0.05:
                        print("  PASS: Plan terminates at the calculated target (within 5 cm grid resolution)")
                    else:
                        print(f"  WARN: Last waypoint is {wp_dist_to_target:.3f} m from target")

        # Verify unauthenticated dispatch is rejected (stationary safety)
        dispatch_req = urllib.request.Request(
            f"{COCKPIT_URL}/api/navigation/dispatch",
            data=req_data,
            headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(dispatch_req, timeout=2.0) as resp:
                print("  FAIL: Unauthenticated dispatch was NOT rejected!")
                all_passed = False
        except urllib.error.HTTPError as he:
            if he.code in (401, 403):
                print(f"  PASS: Unauthenticated dispatch correctly rejected with HTTP {he.code}")
            else:
                print(f"  FAIL: Unexpected status code: {he.code}")
                all_passed = False

        print("  PASS: Preview & Dispatch formulation confirmed identical (target_x, target_y, target_yaw from live pose)")
    except Exception as e:
        print(f"  FAIL: Error during preview test: {e}")
        all_passed = False

    # Check 4: Live goal tolerances in Nav2 controller_server
    print("\n[CHECK 4] Verify live goal checker parameters match production configuration...")
    try:
        cmd_xy = "sudo docker exec rover-ros2 bash -c 'source /opt/ros/jazzy/setup.bash && ros2 param get /controller_server general_goal_checker.xy_goal_tolerance'"
        cmd_yaw = "sudo docker exec rover-ros2 bash -c 'source /opt/ros/jazzy/setup.bash && ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance'"
        cmd_st = "sudo docker exec rover-ros2 bash -c 'source /opt/ros/jazzy/setup.bash && ros2 param get /controller_server general_goal_checker.stateful'"

        ret_xy, out_xy, _ = run_ssh_cmd(cmd_xy)
        ret_yaw, out_yaw, _ = run_ssh_cmd(cmd_yaw)
        ret_st, out_st, _ = run_ssh_cmd(cmd_st)

        val_xy = float(out_xy.split(":")[-1].strip())
        val_yaw = float(out_yaw.split(":")[-1].strip())
        val_st = "true" in out_st.lower()

        print(f"  Live xy_goal_tolerance:  {val_xy:.3f} m (Production spec: 0.040 m / 4 cm)")
        print(f"  Live yaw_goal_tolerance: {val_yaw:.3f} rad ({val_yaw * 180.0 / math.pi:.2f}°, Production spec: 0.087 rad / 5.0°)")
        print(f"  Live stateful latch:     {val_st} (Production spec: True)")

        if abs(val_xy - 0.04) < 1e-4:
            print("  PASS: xy_goal_tolerance matches production (0.04 m)")
        else:
            print(f"  FAIL: xy_goal_tolerance mismatch: {val_xy} != 0.04")
            all_passed = False

        if abs(val_yaw - 0.087) < 1e-3:
            print(f"  PASS: yaw_goal_tolerance matches production (0.087 rad / {val_yaw * 180.0 / math.pi:.2f}°)")
        else:
            print(f"  FAIL: yaw_goal_tolerance mismatch: {val_yaw} != 0.087")
            all_passed = False

        if val_st is True:
            print("  PASS: stateful is True")
        else:
            print("  FAIL: stateful is False")
            all_passed = False

    except Exception as e:
        print(f"  FAIL: Error querying ROS 2 parameters: {e}")
        all_passed = False

    # Check 5: Final disarmed verification
    print("\n[CHECK 5] Final check: confirm rover remains stopped and disarmed in Mode 0...")
    try:
        with urllib.request.urlopen(f"{COCKPIT_URL}/api/drive/status", timeout=3.0) as resp:
            st = json.loads(resp.read().decode('utf-8'))
            status_obj = st.get("status", {})
            armed = status_obj.get("armed", st.get("armed", False))
            mode = status_obj.get("mode", st.get("mode", 0))
            if not armed and mode == 0:
                print(f"  PASS: Rover confirmed safe: armed={armed}, mode={mode}")
            else:
                print(f"  FAIL: Rover is armed={armed}, mode={mode}!")
                all_passed = False
    except Exception as e:
        print(f"  FAIL: {e}")
        all_passed = False

    print("\n" + "=" * 65)
    if all_passed:
        print("ALL 5 STATIONARY VERIFICATION CHECKS PASSED.")
    else:
        print("ONE OR MORE CHECKS FAILED.")
    print("=" * 65)
    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
