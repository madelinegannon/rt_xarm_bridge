# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""The motion gates: mock default, allow_motion, safety precondition, speed ceilings and ramp,
watchdog, no unattended autonomy, controller limits in degrees, e-stop reaching the SDK."""
import math
import time

import _common as T
from _common import check, close, wait_for

from rt_xarm_bridge import (CONNECTED, DISCONNECTED, ENABLED, INITIAL_SPEED_SCALE_CEILING, JOINT_LIMIT_MARGIN_RAD,
                            MAX_JOINT_STEP_RAD, MONITORING, SLEW_STALL_TICKS, URDF_JOINT_LIMITS_RAD, WATCHDOG_MAX_S, MockArm,
                            MotionNotAllowed, RecoveryPolicy, RobotBridge, SafetyConfig, SafetyError, make_arm)

print("== gate 1: mock is the default, a real arm needs an IP ==")
check("make_arm defaults to nothing: kind is explicit", isinstance(make_arm("mock"), MockArm))
try:
    make_arm("xarm")
    check("xarm without an IP refused", False)
except ValueError as e:
    check("xarm without an IP refused", "IP" in str(e), str(e))
try:
    make_arm("bogus")
    check("unknown kind refused", False)
except ValueError:
    check("unknown kind refused", True)

print("== gate 2: without allow_motion the bridge only monitors ==")
arm = MockArm(initial_q=[0.2] * 6)
b = RobotBridge(arm, rate_hz=200.0)
b.soft_connect()
check("MONITORING reachable", b.state == MONITORING, b.state)
check("joints are read", wait_for(lambda: b.latest_real_q() is not None) and close(b.latest_real_q()[0], [0.2] * 6))
for name, fn in (("arm_robot", b.arm_robot), ("enable", b.enable), ("connect", b.connect)):
    try:
        fn()
        check(f"{name} refused", False)
    except MotionNotAllowed:
        check(f"{name} refused", True)
check("still MONITORING, servos untouched", b.state == MONITORING and arm.mode == 0 and not arm.motion_enabled and not arm.servo_cmds)
check("allow_motion reads False", b.allow_motion is False)
try:
    b.auto_arm = True
    check("auto_arm cannot be switched on without allow_motion", False)
except MotionNotAllowed:
    check("auto_arm cannot be switched on without allow_motion", True)
b.shutdown()
try:
    RobotBridge(MockArm(), auto_arm=True, recovery=RecoveryPolicy())
    check("auto_arm at construction without allow_motion refused", False)
except MotionNotAllowed:
    check("auto_arm at construction without allow_motion refused", True)
try:
    RobotBridge(MockArm(), allow_motion=True)
    check("allow_motion without a SafetyConfig refused", False)
except SafetyError:
    check("allow_motion without a SafetyConfig refused", True)
try:
    RobotBridge(MockArm(), allow_motion=True, safety=SafetyConfig(collision_sensitivity=1))
    check("incomplete SafetyConfig refused at construction", False)
except SafetyError as e:
    check("incomplete SafetyConfig refused at construction", "reduced_mode" in str(e), str(e))
try:
    RobotBridge(MockArm(), allow_motion=True, safety=MockArm().safety_config(), auto_arm=True)
    check("auto_arm without a RecoveryPolicy refused", False)
except ValueError:
    check("auto_arm without a RecoveryPolicy refused", True)

print("== gate 3: the controller must match the declared safety config ==")
arm = MockArm(collision_sensitivity=3)
cfg = arm.safety_config()
cfg.collision_sensitivity = 1
events = []
b = RobotBridge(arm, allow_motion=True, safety=cfg, rate_hz=200.0, on_event=events.append)
b.soft_connect()
ok = b.arm_robot()
check("arm_robot returns False on mismatch", ok is False)
check("stays MONITORING", b.state == MONITORING, b.state)
check("halt names the field", b.halt_reason is not None and "collision_sensitivity" in b.halt_reason, str(b.halt_reason))
check("servos disarmed again", not arm.motion_enabled and arm.state == 4, f"enabled={arm.motion_enabled} state={arm.state}")
check("the bridge never writes controller config", arm.set_sensitivity_calls == 0 and arm.save_conf_calls == 0)
check("safety_refused event", any(e.get("event") == "safety_refused" for e in events), str([e.get("event") for e in events]))
check("report kept", b.safety_report is not None and not b.safety_report.ok)
try:
    b.enable(sync_q=[0.0] * 6)
    check("enable from MONITORING fails", b.state != ENABLED)
except MotionNotAllowed:
    check("enable from MONITORING fails", False, "should not be MotionNotAllowed with allow_motion")
check("halted: arm_robot refused until recover", b.arm_robot() is False and "halted" in (b.last_error or ""), str(b.last_error))
arm.collision_sensitivity = 1          # the operator fixes the controller
b.recover()
check("after recover + fix: CONNECTED", b.arm_robot() is True and b.state == CONNECTED, b.state)
check("report now ok", b.safety_report.ok)
b.shutdown()
arm = MockArm(tcp_boundary_mm=[900.0, -900.0, 700.0, -700.0, 1200.0, 0.0])
cfg = arm.safety_config()
cfg.tcp_boundary_mm = [700.0, -700.0, 700.0, -700.0, 1200.0, 0.0]
b = RobotBridge(arm, allow_motion=True, safety=cfg, rate_hz=200.0)
check("boundary wider than declared refused", b.connect() is False and "x_max" in b.halt_reason, str(b.halt_reason))
b.shutdown()

print("== gate 4: speed starts low and ramps; config cannot exceed the ceilings ==")
arm = MockArm()
b = RobotBridge(arm, allow_motion=True, safety=arm.safety_config(), rate_hz=200.0, initial_speed_scale=0.9,
                max_joint_step=0.5, watchdog_s=5.0)
check("initial speed clamped to the ceiling", b.speed_scale == INITIAL_SPEED_SCALE_CEILING, str(b.speed_scale))
check("per-tick step clamped to the ceiling", b._max_joint_step == MAX_JOINT_STEP_RAD, str(b._max_joint_step))
check("watchdog clamped", b._watchdog_s == WATCHDOG_MAX_S, str(b._watchdog_s))
b.speed_scale = 0.9
check("an operator can raise it at runtime", b.speed_scale == 0.9)
b.connect()
b.speed_scale = 1.0
b.enable(sync_q=[0.0] * 6)
check("effective speed resets to zero on enable", b.speed_effective == 0.0, str(b.speed_effective))
b.publish_desired_q([1.0] * 6)
time.sleep(0.06)
steps = [max(abs(x) for x in q) for _, q in arm.servo_cmds]
check("first packets barely move (ramp from standstill)", steps and steps[0] < 0.002, str(steps[:3]))
time.sleep(0.5)
check("ramp reached the cap", b.speed_effective > 0.15, str(b.speed_effective))
b.shutdown()

print("== gate 4a: the ramp is wall-clock, not tick-count ==")


class SlowArm(MockArm):
    """A host whose servo call runs long: the thread ticks at half its nominal rate."""

    def set_servo_angle_j(self, angles, is_radian=True, **kwargs):
        time.sleep(0.04)
        return super().set_servo_angle_j(angles, is_radian=is_radian, **kwargs)


arm = SlowArm()
b = RobotBridge(arm, allow_motion=True, safety=arm.safety_config(), rate_hz=50.0, watchdog_s=5.0)
b.connect()
b.enable(sync_q=[0.0] * 6)
b.publish_desired_q([1.0] * 6)
time.sleep(0.8)
n_ticks = len(arm.servo_cmds)
check("the thread really ticked slow (test premise)", n_ticks < 32, f"{n_ticks} ticks in 0.8 s at a nominal 50 Hz")
check("the cap is still reached on time", b.speed_effective >= INITIAL_SPEED_SCALE_CEILING - 1e-9, str(b.speed_effective))
b.shutdown()
b = RobotBridge(MockArm(), rate_hz=100.0)
b.speed_scale = 1.0
b._slew_t = 0.0
b._slew_speed(0.3)                                       # a 0.3 s stall on a 10 ms tick
check("a stalled tick credits at most SLEW_STALL_TICKS ticks", abs(b.speed_effective - 0.33 * SLEW_STALL_TICKS * 0.01) < 1e-9, str(b.speed_effective))
b.shutdown()

print("== gate 4b: a stale target stops the stream ==")
arm = MockArm()
b = RobotBridge(arm, allow_motion=True, safety=arm.safety_config(), rate_hz=200.0, watchdog_s=0.2, speed_slew_per_s=10.0)
b.connect()
b.enable(sync_q=[0.0] * 6)
b.publish_desired_q([0.05] * 6)
time.sleep(0.1)
n1 = len(arm.servo_cmds)
time.sleep(0.3)
n2 = len(arm.servo_cmds)
time.sleep(0.2)
n3 = len(arm.servo_cmds)
check("streamed while fresh", n1 > 0, str(n1))
check("stopped once stale", n3 == n2, f"{n2} -> {n3}")
b.shutdown()

print("== gate 5: no unattended autonomy by default ==")
arm = MockArm()
b = RobotBridge(arm, allow_motion=True, safety=arm.safety_config(), rate_hz=200.0)
b.connect()
b.enable(sync_q=[0.0] * 6)
arm.inject_error(24)
check("fault demotes to MONITORING", wait_for(lambda: b.state == MONITORING, 1.0), b.state)
n = len(arm.servo_cmds)
time.sleep(0.4)
check("auto-recover off: stays stopped", b.state == MONITORING and b.recoveries_total == 0 and len(arm.servo_cmds) == n)
check("recovery hint stays empty", b.pop_recovery_hint() is None)
arm2 = MockArm()
b2 = RobotBridge(arm2, rate_hz=200.0, reconnect_backoff_s=(0.05, 0.05))
b2.soft_connect()
arm2.inject_disconnect()
check("drop -> DISCONNECTED", wait_for(lambda: b2.state == DISCONNECTED, 1.0), b2.state)
time.sleep(0.4)
check("reconnect off: stays DISCONNECTED", b2.state == DISCONNECTED and arm2.connect_calls == 1, f"{b2.state} calls={arm2.connect_calls}")
b.shutdown()
b2.shutdown()

print("== controller joint limits: degrees converted, intersected with the URDF table ==")
arm = MockArm(reduced_mode=True)              # answers in degrees, like the controller
b = RobotBridge(arm, rate_hz=200.0)
b.soft_connect()
j1 = b.joint_limits[0]
check("controller limits converted to radians", b.controller_limits is not None and close(b.controller_limits[0], (math.radians(-170), math.radians(170))), str(b.controller_limits))
check("effective J1 is the tighter controller value", close(j1, (math.radians(-170), math.radians(170))), str(j1))
check("effective J3 upper keeps the URDF value where it is tighter", abs(b.joint_limits[2][1] - min(URDF_JOINT_LIMITS_RAD[2][1], math.radians(3.5))) < 1e-9, str(b.joint_limits[2]))
clamped = b._inside_limits([3.0, 0, 0, 0, 0, 0])
check("clamp uses the effective limits", abs(clamped[0] - (math.radians(170) - JOINT_LIMIT_MARGIN_RAD)) < 1e-9, str(clamped[0]))
b.shutdown()
arm = MockArm(reduced_mode=False)             # +-360 on every joint: no controller limits
b = RobotBridge(arm, rate_hz=200.0)
b.soft_connect()
check("reduced mode off: URDF table only", b.controller_limits is None and b.joint_limits == list(URDF_JOINT_LIMITS_RAD))
b.shutdown()
arm = MockArm(reduced_mode=True, limits_in_degrees=False)
b = RobotBridge(arm, rate_hz=200.0)
b.soft_connect()
check("radians accepted as-is", close(b.joint_limits[0], (math.radians(-170), math.radians(170))), str(b.joint_limits[0]))
b.shutdown()

print("== emergency_stop reaches the SDK ==")
arm = MockArm()
b = RobotBridge(arm, allow_motion=True, safety=arm.safety_config(), rate_hz=200.0)
b.connect()
b.enable(sync_q=[0.0] * 6)
b.emergency_stop()
check("SDK emergency_stop called", arm.estop_count == 1, str(arm.estop_count))
check("servos off, controller stopped, bridge MONITORING", not arm.motion_enabled and arm.state == 4 and b.state == MONITORING)
b.shutdown()

T.finish("test_gates")
