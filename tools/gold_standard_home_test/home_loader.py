"""
tools.gold_standard_home_test.home_loader - Dynamic Authoritative HOME Pose Loader
Loads the active map's saved HOME pose from disk or Cockpit API.
Coordinates and yaws are never hardcoded.
"""

import os
import json
import math
from typing import Dict, Any, Optional

DEFAULT_MAP_DIRS = [
    "/ros2_ws/maps",
    "/home/ron/yahboom-encoder/ros2/volumes/maps",
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "ros2", "volumes", "maps"),
    os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
]

def wrap_angle_rad(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad

def wrap_angle_deg(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg

def find_active_map_file() -> Optional[str]:
    """Finds active_map_path.txt across search paths."""
    for d in DEFAULT_MAP_DIRS:
        p = os.path.join(d, "active_map_path.txt")
        if os.path.isfile(p):
            try:
                with open(p, "r", encoding="utf-8") as f:
                    content = f.read().strip()
                    if content:
                        return content
            except Exception:
                pass
    return None

def load_home_from_file(json_path: str) -> Optional[Dict[str, Any]]:
    """Loads and validates HOME pose from a JSON file."""
    if not os.path.isfile(json_path):
        return None
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            pose = data.get("pose", {})
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            
            x = pos.get("x")
            y = pos.get("y")
            z = pos.get("z", 0.0)
            
            if x is None or y is None:
                return None
                
            yaw_deg = pose.get("yaw_deg")
            yaw_rad = pose.get("yaw_rad")
            
            if yaw_deg is None and yaw_rad is not None:
                yaw_deg = math.degrees(yaw_rad)
            elif yaw_rad is None and yaw_deg is not None:
                yaw_rad = math.radians(yaw_deg)
            elif yaw_rad is None and yaw_deg is None:
                # Compute from quaternion
                qz = ori.get("z", 0.0)
                qw = ori.get("w", 1.0)
                yaw_rad = 2.0 * math.atan2(qz, qw)
                yaw_deg = math.degrees(yaw_rad)
                
            return {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw_rad": wrap_angle_rad(float(yaw_rad)),
                "yaw_deg": wrap_angle_deg(float(yaw_deg)),
                "qz": float(ori.get("z", 0.0)),
                "qw": float(ori.get("w", 1.0)),
                "frame_id": data.get("frame_id", "map"),
                "source_file": json_path,
                "description": data.get("description", "Authoritative HOME pose")
            }
    except Exception:
        return None

def load_authoritative_home(cockpit_url: Optional[str] = None) -> Dict[str, Any]:
    """
    Authoritatively resolves the active map's HOME pose.
    Precedence:
    1. active_map_path.txt -> matching home_pose_slam_<map>.json or home_pose_slam.json in same dir.
    2. Candidate home_pose_slam files in standard directories.
    3. Live Cockpit /api/navigation/home endpoint if reachable.
    Raises ValueError if no valid HOME pose file can be resolved.
    """
    active_map = find_active_map_file()
    search_dirs = []
    
    if active_map:
        active_map_name = os.path.splitext(os.path.basename(active_map))[0]
        active_dir = os.path.dirname(active_map)
        search_dirs.append(active_dir)
        
        # Check specific candidates
        candidates = [
            os.path.join(active_dir, f"home_pose_slam_{active_map_name}.json"),
            os.path.join(active_dir, "home_pose_slam.json")
        ]
        for c in candidates:
            res = load_home_from_file(c)
            if res:
                return res
                
    search_dirs.extend(DEFAULT_MAP_DIRS)
    
    for d in search_dirs:
        if not os.path.isdir(d):
            continue
        try:
            for fname in os.listdir(d):
                if fname.startswith("home_pose_slam") and fname.endswith(".json"):
                    res = load_home_from_file(os.path.join(d, fname))
                    if res:
                        return res
        except Exception:
            pass

    # Try Cockpit API fallback
    if cockpit_url:
        try:
            import urllib.request
            req = urllib.request.Request(f"{cockpit_url}/api/navigation/home", headers={"Accept": "application/json"})
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                if data.get("ok") and "home" in data:
                    h = data["home"]
                    return {
                        "x": float(h["x"]),
                        "y": float(h["y"]),
                        "z": 0.0,
                        "yaw_rad": float(h.get("yaw_rad", math.radians(h.get("yaw_deg", 0.0)))),
                        "yaw_deg": float(h.get("yaw_deg", math.degrees(h.get("yaw_rad", 0.0)))),
                        "qz": 0.0,
                        "qw": 1.0,
                        "frame_id": "map",
                        "source_file": "Cockpit /api/navigation/home API",
                        "description": h.get("description", "Cockpit live HOME")
                    }
        except Exception:
            pass

    raise ValueError("Could not resolve authoritative saved HOME pose from active map or Cockpit.")
