# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""Realtime servo bridge for UFACTORY xArm and 850 arms, behind explicit safety gates."""
from .bridge import (CONNECTED, DISCONNECTED, ENABLED, FALLBACK_JOINT_LIMITS_RAD, INITIAL_SPEED_SCALE_CEILING,
                     JOINT_LIMIT_MARGIN_RAD, MAX_JOINT_STEP_RAD, MONITORING, N_JOINTS, SLEW_STALL_TICKS, URDF_JOINT_LIMITS_RAD,
                     WATCHDOG_MAX_S, MockArm, MotionNotAllowed, RobotBridge, make_arm, metres_to_xarm_pos,
                     xarm_pos_to_metres)
from .recovery import NEEDS_OPERATOR, RECOVER, RECOVER_HOME, Decision, RecoveryPolicy, error_title
from .safety import (SafetyConfig, SafetyError, SafetyReport, compare, park, read_controller_safety,
                     set_collision_sensitivity, verify_safety_config)

__version__ = "0.1.0"
__all__ = [
    "CONNECTED", "DISCONNECTED", "ENABLED", "MONITORING", "N_JOINTS",
    "FALLBACK_JOINT_LIMITS_RAD", "URDF_JOINT_LIMITS_RAD", "JOINT_LIMIT_MARGIN_RAD",
    "INITIAL_SPEED_SCALE_CEILING", "MAX_JOINT_STEP_RAD", "SLEW_STALL_TICKS", "WATCHDOG_MAX_S",
    "MockArm", "MotionNotAllowed", "RobotBridge", "make_arm", "metres_to_xarm_pos", "xarm_pos_to_metres",
    "NEEDS_OPERATOR", "RECOVER", "RECOVER_HOME", "Decision", "RecoveryPolicy", "error_title",
    "SafetyConfig", "SafetyError", "SafetyReport", "compare", "park", "read_controller_safety",
    "set_collision_sensitivity", "verify_safety_config",
]
