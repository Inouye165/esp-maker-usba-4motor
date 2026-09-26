"""
tools.gold_standard_home_test - Gold Standard HOME Acceptance Test Package
"""

from .constants import (
    FORWARD_DISTANCE_M,
    ROTATION_TARGET_DEG,
    NORMAL_LINEAR_SPEED,
    NORMAL_ANGULAR_SPEED,
    FINAL_POS_CORRECTION_CEILING,
    FINAL_YAW_CORRECTION_CEILING,
    PRE_ARM_MAX_POS_ERR_M,
    PRE_ARM_MAX_YAW_ERR_DEG,
    PASS_FINAL_POS_ERR_M,
    PASS_FINAL_YAW_ERR_DEG,
    PASS_MAX_CRAWL_WINDOW_SEC,
    PASS_MAX_CORRECTIVE_REVERSALS
)
from .home_loader import load_authoritative_home
from .mission import GoldStandardMission, MissionAbortException
from .grader import MissionGrader
from .telemetry import TelemetryRecorder, TelemetryFrame

__all__ = [
    "FORWARD_DISTANCE_M",
    "ROTATION_TARGET_DEG",
    "NORMAL_LINEAR_SPEED",
    "NORMAL_ANGULAR_SPEED",
    "FINAL_POS_CORRECTION_CEILING",
    "FINAL_YAW_CORRECTION_CEILING",
    "PRE_ARM_MAX_POS_ERR_M",
    "PRE_ARM_MAX_YAW_ERR_DEG",
    "PASS_FINAL_POS_ERR_M",
    "PASS_FINAL_YAW_ERR_DEG",
    "PASS_MAX_CRAWL_WINDOW_SEC",
    "PASS_MAX_CORRECTIVE_REVERSALS",
    "load_authoritative_home",
    "GoldStandardMission",
    "MissionAbortException",
    "MissionGrader",
    "TelemetryRecorder",
    "TelemetryFrame"
]
