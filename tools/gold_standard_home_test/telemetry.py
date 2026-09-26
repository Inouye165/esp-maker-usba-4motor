"""
tools.gold_standard_home_test.telemetry - Synchronized 20+ Hz Telemetry Recorder
Records complete mission data across all legs into timestamped JSON reports.
"""

import os
import time
import json
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, asdict, field

@dataclass
class TelemetryFrame:
    t_rel_s: float
    timestamp_epoch: float
    mission_stage: str
    
    # Velocity Commands
    raw_cmd: Dict[str, float]       # vx, wz
    smoothed_cmd: Dict[str, float]  # vx, wz
    final_cmd: Dict[str, float]     # vx, wz
    cmd_source: str                 # "ROS_AUTONOMY", "EXACT_MOTION", "NONE", etc.
    
    # AMCL Localization
    amcl: Dict[str, Any]            # x, y, yaw_deg, cov_x, cov_y, cov_yaw, localized
    
    # Odometry
    odom: Dict[str, Any]            # x, y, yaw_deg, vx, wz, left_dist, right_dist
    
    # IMU
    imu: Dict[str, Any]             # raw_yaw_deg, rel_yaw_deg, gyro_z, non_magnetic_valid
    
    # Motor & Drive Hardware
    drive: Dict[str, Any]           # armed, mode, reqLinear, reqAngular, limLinear, limAngular, bootCount, resetReason, rtcResetReason, lockStatus
    
    # Safety / Collision Monitor
    collision_monitor: Dict[str, Any] # action, polygon
    
    # Distance / Heading to HOME
    to_home: Dict[str, float]       # pos_err_m, pos_err_cm, yaw_err_deg


class TelemetryRecorder:
    def __init__(self, output_dir: str = "reports/gold_standard_home_test"):
        self.output_dir = output_dir
        self.t0 = time.monotonic()
        self.start_epoch = time.time()
        self.frames: List[Dict[str, Any]] = []
        self.transitions: List[Dict[str, Any]] = []
        self.metadata: Dict[str, Any] = {}
        
    def record_transition(self, stage_name: str, details: Optional[Dict[str, Any]] = None):
        entry = {
            "t_rel_s": round(time.monotonic() - self.t0, 4),
            "timestamp_epoch": round(time.time(), 4),
            "stage": stage_name,
            "details": details or {}
        }
        self.transitions.append(entry)

    def record_frame(self, frame: TelemetryFrame):
        self.frames.append(asdict(frame))

    def save(self, run_id: Optional[str] = None) -> str:
        os.makedirs(self.output_dir, exist_ok=True)
        if not run_id:
            run_id = time.strftime("%Y%m%d_%H%M%S")
        filename = f"gold_standard_run_{run_id}.json"
        filepath = os.path.join(self.output_dir, filename)
        
        payload = {
            "metadata": {
                "run_id": run_id,
                "start_epoch": self.start_epoch,
                "duration_s": round(time.monotonic() - self.t0, 3),
                "total_frames": len(self.frames),
                "sample_rate_hz": round(len(self.frames) / max(0.001, (time.monotonic() - self.t0)), 2),
                **self.metadata
            },
            "transitions": self.transitions,
            "samples": self.frames
        }
        
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
            
        return filepath
