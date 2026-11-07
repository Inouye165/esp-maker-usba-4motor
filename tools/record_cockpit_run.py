#!/usr/bin/env python3
"""
record_cockpit_run.py
High-resolution (20 Hz) flight recorder for the physical Cockpit UI navigation run.
Logs:
- Cockpit navigation status (status, target, distance_remaining_m, path length)
- Cockpit autonomy status (state, limitedLinear, limitedAngular, rawLinear, rawAngular, rejectedCount)
- Cockpit drive status (armed, mode, espClearanceMask, espClearanceAgeMs, reqLinear, limLinear, odom)
- Localization status (x, y, yawDeg, sigmaX, sigmaY, ageMs, localized)
- ROS 2 collision monitor state & cmd_vel (via SSH / direct query)
"""

import os
import sys
import time
import json
import urllib.request
import urllib.error
import threading

COCKPIT_BASE = "http://10.0.0.246:3000"
RUN_ID = int(time.time())
REPORT_PATH = os.path.abspath(f"reports/cockpit_ui_nav_run_telemetry_{RUN_ID}.json")
LATEST_REPORT_PATH = os.path.abspath("reports/cockpit_ui_nav_run_telemetry.json")

os.makedirs("reports", exist_ok=True)

samples = []
running = True
stop_event = threading.Event()

def get_json(endpoint, timeout=0.15):
    try:
        req = urllib.request.Request(f"{COCKPIT_BASE}{endpoint}")
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.loads(response.read().decode('utf-8'))
    except Exception as e:
        return {"error": str(e)}

def poll_loop():
    global samples, running
    start_time = time.time()
    print(f"Flight telemetry recorder started at {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    while not stop_event.is_set():
        t0 = time.time()
        elapsed = t0 - start_time
        
        # Parallel or fast sequential fetch
        loc = get_json("/api/localization/status")
        drv = get_json("/api/drive/status")
        aut = get_json("/api/autonomy/status")
        nav = get_json("/api/navigation/status")
        
        sample = {
            "elapsed_s": round(elapsed, 4),
            "timestamp": round(t0, 4),
            "loc": loc,
            "drv": drv,
            "aut": aut,
            "nav": nav
        }
        samples.append(sample)
        
        # Target 20 Hz (50 ms cycle)
        dt = time.time() - t0
        rem = 0.050 - dt
        if rem > 0:
            time.sleep(rem)

    print(f"Flight telemetry recorder stopped. Collected {len(samples)} samples.")
    # Save output
    report_obj = {
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "total_samples": len(samples),
        "samples": samples
    }
    with open(REPORT_PATH, "w") as f:
        json.dump(report_obj, f, indent=2)
    with open(LATEST_REPORT_PATH, "w") as f:
        json.dump(report_obj, f, indent=2)
    print(f"Saved telemetry to {REPORT_PATH} and {LATEST_REPORT_PATH}")

if __name__ == "__main__":
    t = threading.Thread(target=poll_loop, daemon=True)
    t.start()
    try:
        # Wait until stdin or interrupt or max duration (180s for browser UI interaction)
        max_t = time.time() + 180
        seen_executing = False
        while time.time() < max_t:
            time.sleep(0.5)
            # Print periodic heartbeat
            if samples:
                s = samples[-1]
                nav_st = s.get("nav", {}).get("status", "UNKNOWN")
                drv_armed = s.get("drv", {}).get("status", {}).get("armed", False)
                drv_mode = s.get("drv", {}).get("status", {}).get("mode", 0)
                loc_x = s.get("loc", {}).get("x", 0.0)
                loc_y = s.get("loc", {}).get("y", 0.0)
                rem_dist = s.get("nav", {}).get("distance_remaining_m", None)
                print(f"[{s['elapsed_s']:.2f}s] Nav={nav_st} | Armed={drv_armed} Mode={drv_mode} | Pose=({loc_x:.3f}, {loc_y:.3f}) | Rem={rem_dist}", flush=True)
                if nav_st == "EXECUTING" or drv_armed:
                    seen_executing = True
                if seen_executing and nav_st in ["SUCCEEDED", "FAILED", "CANCELLED"] and not drv_armed:
                    # Completed - allow 2.5s to capture settled post-arrival telemetry
                    time.sleep(2.5)
                    break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        t.join(timeout=2.0)
