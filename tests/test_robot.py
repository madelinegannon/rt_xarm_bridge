# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""RobotBridge + MockArm: state machine, streaming, watchdog, stop/recover/e-stop, speed cap and
ramp, joint-limit net. No hardware."""
import time

import _common as T
from _common import check, close

from rt_xarm_bridge import (CONNECTED, DISCONNECTED, ENABLED, FALLBACK_JOINT_LIMITS_RAD, JOINT_LIMIT_MARGIN_RAD,
                            MONITORING, MockArm, RobotBridge, metres_to_xarm_pos, xarm_pos_to_metres)


def motion_bridge(arm, **kw):
    return RobotBridge(arm, allow_motion=True, safety=arm.safety_config(), **kw)


print("== unit conversion: mm <-> metres ==")
m = xarm_pos_to_metres([1500.0, -200.0, 850.0])
check("mm to metres", close(m, [1.5, -0.2, 0.85]), str(m))
mm = metres_to_xarm_pos([0.3, -0.1, 1.01])
check("metres to mm", close(mm, [300.0, -100.0, 1010.0]), str(mm))
rt = xarm_pos_to_metres(metres_to_xarm_pos([0.5, 0.25, 1.0]))
check("round-trip", close(rt, [0.5, 0.25, 1.0]), str(rt))

print("== soft_connect -> MONITORING ==")
arm = MockArm(initial_q=[0.0] * 6)
bridge = motion_bridge(arm, rate_hz=200.0, watchdog_s=0.2, max_joint_step=0)
bridge.soft_connect()
check("state MONITORING", bridge.state == MONITORING, bridge.state)
check("servo mode NOT set", arm.mode == 0, f"mode={arm.mode}")
check("motion NOT enabled", not arm.motion_enabled, str(arm.motion_enabled))
time.sleep(0.1)
rq = bridge.latest_real_q()
check("real_q mirrored in MONITORING", rq is not None and close(rq[0], [0.0] * 6), str(rq))
check("joint_limits populated", bridge.joint_limits is not None and len(bridge.joint_limits) == 6, str(bridge.joint_limits))
check("joint_limits are the URDF table while reduced mode is off", bridge.joint_limits == list(FALLBACK_JOINT_LIMITS_RAD))
check("arm_position_m available", bridge.arm_position_m is not None)

print("== arm_robot -> CONNECTED ==")
check("arm_robot returns True", bridge.arm_robot() is True)
check("state CONNECTED", bridge.state == CONNECTED, bridge.state)
check("servo mode set", arm.mode == 1 and arm.motion_enabled)
check("safety report ok", bridge.safety_report is not None and bridge.safety_report.ok)

print("== enable + stream -> arm tracks desired ==")
bridge.enable(sync_q=[0.0] * 6)
check("state ENABLED", bridge.state == ENABLED)
bridge.publish_desired_q([0.1, 0.1, -0.1, 0.1, 0.1, 0.1])
time.sleep(0.15)
c1 = len(arm.servo_cmds)
check("servo commands streamed", c1 > 0, f"c1={c1}")
_, q = arm.get_servo_angle()
check("arm tracked desired", close(q, [0.1, 0.1, -0.1, 0.1, 0.1, 0.1]), str(q))

print("== watchdog halts streaming on stale desired ==")
time.sleep(0.35)
c2 = len(arm.servo_cmds)
time.sleep(0.2)
c3 = len(arm.servo_cmds)
check("streamed during fresh window", c2 > c1, f"c1={c1} c2={c2}")
check("watchdog stopped streaming", c3 == c2, f"c2={c2} c3={c3}")

print("== STOP -> MONITORING + set_state(4) ==")
bridge.stop()
check("state MONITORING after stop", bridge.state == MONITORING, bridge.state)
check("arm stopped (state 4)", arm.state == 4, str(arm.state))

print("== recover -> errors cleared, still MONITORING ==")
bridge.recover()
check("state MONITORING after recover", bridge.state == MONITORING, bridge.state)
check("error cleared", arm.error_code == 0)
check("warn cleared", arm.warn_code == 0)
check("last_error cleared", bridge.last_error is None)
bridge.shutdown()

print("== emergency_stop -> MONITORING ==")
arm2 = MockArm(initial_q=[0.0] * 6)
b2 = motion_bridge(arm2, rate_hz=200.0, max_joint_step=0)
b2.connect()
b2.enable(sync_q=[0.0] * 6)
b2.emergency_stop()
check("state MONITORING after estop", b2.state == MONITORING, b2.state)
check("motion disabled after estop", not arm2.motion_enabled, str(arm2.motion_enabled))
check("SDK emergency_stop reached", arm2.estop_count == 1, str(arm2.estop_count))
check("last_error set", b2.last_error is not None and "Emergency" in b2.last_error, str(b2.last_error))
b2.recover()
check("motion re-enabled after recover", arm2.motion_enabled)
check("state still MONITORING after recover", b2.state == MONITORING, b2.state)
check("last_error cleared after recover", b2.last_error is None)
b2.shutdown()

print("== disconnect -> DISCONNECTED ==")
arm3 = MockArm(initial_q=[0.0] * 6)
b3 = RobotBridge(arm3, rate_hz=200.0, max_joint_step=0)
b3.soft_connect()
b3.disconnect()
check("state DISCONNECTED", b3.state == DISCONNECTED, b3.state)
check("arm disconnected", not arm3.connected)
b3.shutdown()

print("== sync-on-enable: no jump ==")
arm5 = MockArm(initial_q=[0.5, 0.5, -0.5, 0.5, 0.5, 0.5])
b5 = motion_bridge(arm5, rate_hz=200.0, max_joint_step=0)
b5.connect()
b5.enable(sync_q=[0.5, 0.5, -0.5, 0.5, 0.5, 0.5])
time.sleep(0.1)
_, q5 = arm5.get_servo_angle()
check("first target == sync pose", close(q5, [0.5, 0.5, -0.5, 0.5, 0.5, 0.5]), str(q5))
b5.shutdown()

print("== enable guard: cannot enable from MONITORING ==")
arm6 = MockArm(initial_q=[0.0] * 6)
b6 = motion_bridge(arm6, rate_hz=200.0, max_joint_step=0)
b6.soft_connect()
check("enable returns False", b6.enable(sync_q=[0.0] * 6) is False)
check("enable blocked from MONITORING", b6.state == MONITORING, b6.state)
check("error captured on blocked enable", b6.last_error is not None and "CONNECTED" in b6.last_error, str(b6.last_error))
b6.shutdown()

print("== error capture ==")
b7 = RobotBridge(MockArm(initial_q=[0.0] * 6), rate_hz=200.0, max_joint_step=0)
b7._capture_error("test_context", RuntimeError("test error"))
check("last_error captured", b7.last_error == "test_context: test error", str(b7.last_error))
b7.clear_error()
check("clear_error works", b7.last_error is None)

print("== MockArm callback registration ==")
arm8 = MockArm()
arm8.register_error_warn_changed_callback(lambda d: None)
arm8.register_state_changed_callback(lambda d: None)
arm8.register_connect_changed_callback(lambda d: None)
check("callback registration stored", arm8._on_error_warn is not None)

print("== speed_scale: 0% holds position ==")
arm9 = MockArm(initial_q=[0.0] * 6)
b9 = motion_bridge(arm9, rate_hz=200.0, max_joint_step=0.05)
b9.connect()
b9.speed_scale = 0.0
b9.enable(sync_q=[0.0] * 6)
time.sleep(0.05)
n_before = len(arm9.servo_cmds)
b9.publish_desired_q([1.0] * 6)
time.sleep(0.08)
b9.stop()
time.sleep(0.05)
cmds = [q for _, q in arm9.servo_cmds[n_before:]]
check("speed_scale=0 holds last_sent", bool(cmds) and all(max(abs(v) for v in c) < 1e-9 for c in cmds), str(cmds[-1:] if cmds else "no cmds"))
b9.shutdown()

print("== speed_scale: 50% halves the step once the ramp is in ==")
arm10 = MockArm(initial_q=[0.0] * 6)
b10 = motion_bridge(arm10, rate_hz=200.0, max_joint_step=0.05, speed_slew_per_s=50.0)
b10.connect()
b10.speed_scale = 0.5
b10.enable(sync_q=[0.0] * 6)
time.sleep(0.1)               # 50/s: the ramp is done well inside this
n_before10 = len(arm10.servo_cmds)
b10.publish_desired_q([1.0] * 6)
time.sleep(0.08)
b10.stop()
time.sleep(0.05)
cmds10 = [q for _, q in arm10.servo_cmds[n_before10:]]
first_step = max(abs(v) for v in cmds10[0]) if cmds10 else None
check("speed_scale=0.5 halves the 0.05 clamp", first_step is not None and abs(first_step - 0.025) < 1e-6, f"step={first_step}")
b10.shutdown()

print("== speed_scale: property clamps ==")
b_clamp = RobotBridge(MockArm(), rate_hz=200.0, max_joint_step=0)
b_clamp.speed_scale = -0.5
check("speed_scale clamps low", b_clamp.speed_scale == 0.0, str(b_clamp.speed_scale))
b_clamp.speed_scale = 2.0
check("speed_scale clamps high", b_clamp.speed_scale == 1.0, str(b_clamp.speed_scale))
b_clamp.speed_scale = 0.42
check("speed_scale stores", abs(b_clamp.speed_scale - 0.42) < 1e-9, str(b_clamp.speed_scale))

print("== startup sync: the first packet must not teleport the arm ==")
REAL_Q = [0.0, 1.012291, -0.785398, 0.0, -0.226893, 0.0]     # where the arm was left
SIM_Q = [0.0, -1.05, -1.84, 0.0, -0.2, 0.0]                  # what a simulation stage starts at
arm_s = MockArm(initial_q=REAL_Q)
b_s = motion_bridge(arm_s, rate_hz=200.0, watchdog_s=5.0)
b_s.connect()
for _ in range(50):
    if b_s.latest_real_q() is not None:
        break
    time.sleep(0.01)
rq_s = b_s.latest_real_q()
check("bridge read the measured pose", rq_s is not None and close(rq_s[0], REAL_Q), str(rq_s))
b_s.enable(sync_q=list(rq_s[0]))
time.sleep(0.08)
first = arm_s.servo_cmds[0][1]
check("first packet is the measured pose", close(first, REAL_Q), str(first))
check("first packet is NOT the sim pose", not close(first, SIM_Q, atol=0.1), str(first))
b_s.stop()
b_s.shutdown()

print("== startup sync: enable() with no sync_q still clamps from the measured pose ==")
arm_n = MockArm(initial_q=REAL_Q)
b_n = motion_bridge(arm_n, rate_hz=200.0, watchdog_s=5.0, max_joint_step=0.05)
b_n.connect()
for _ in range(50):
    if b_n.latest_real_q() is not None:
        break
    time.sleep(0.01)
b_n.enable()
b_n.publish_desired_q(SIM_Q)
time.sleep(0.08)
check("some packet was sent", len(arm_n.servo_cmds) > 0, str(len(arm_n.servo_cmds)))
step_n = max(abs(a - b) for a, b in zip(arm_n.servo_cmds[0][1], REAL_Q))
check("first step clamped from measured", step_n <= 0.05 + 1e-6, f"step={step_n:.4f} rad")
b_n.stop()
b_n.shutdown()

print("== halt: refuse to drive, and say why ==")
arm_h = MockArm(initial_q=REAL_Q)
b_h = motion_bridge(arm_h, rate_hz=200.0, watchdog_s=5.0)
b_h.connect()
b_h.halt("startup pose check: J2 is 143 deg from home (limit 143 deg)")
time.sleep(0.05)
check("never ENABLED", b_h.state != ENABLED, b_h.state)
check("halt_reason set", "startup pose check" in (b_h.halt_reason or ""), str(b_h.halt_reason))
check("nothing streamed", len(arm_h.servo_cmds) == 0, str(len(arm_h.servo_cmds)))
b_h.shutdown()

print("== arm_robot waits for servo mode before CONNECTED ==")
arm_r = MockArm(initial_q=REAL_Q)
b_r = motion_bridge(arm_r, rate_hz=200.0, watchdog_s=5.0)
b_r.connect()
check("servo mode set", arm_r.mode == 1, f"mode={arm_r.mode}")
check("state ready (not 3/4)", arm_r.state not in (3, 4), f"state={arm_r.state}")
check("CONNECTED", b_r.state == CONNECTED, b_r.state)
b_r.shutdown()

print("== speed cap: a raise ramps in, a cut applies at once ==")
FAR = [2.0] * 6                 # where the twin is; the arm is at 0 and a long way behind
arm_sp = MockArm(initial_q=[0.0] * 6)
b_sp = motion_bridge(arm_sp, rate_hz=100.0, watchdog_s=5.0, max_joint_step=0.05, speed_slew_per_s=0.33)
b_sp.connect()
b_sp.enable(sync_q=[0.0] * 6)
b_sp.speed_scale = 0.1
time.sleep(0.4)                 # from zero at 0.33/s the cap of 0.1 is reached in 0.3 s
check("the cap is reached", abs(b_sp.speed_effective - 0.1) < 1e-9, f"eff={b_sp.speed_effective}")
b_sp.publish_desired_q(FAR)
time.sleep(0.3)                 # let it fall behind at the low cap
n_before_sp = len(arm_sp.servo_cmds)
b_sp.speed_scale = 0.95         # the slider jump that causes a lurch unramped
time.sleep(0.10)
after = [q for _, q in arm_sp.servo_cmds[n_before_sp:]]
steps = [max(abs(a - b) for a, b in zip(after[i], after[i - 1])) for i in range(1, len(after))]
check("commands kept flowing", len(steps) > 3, str(len(steps)))
if steps:
    worst = max(steps)
    check("no step at the full new cap", worst < 0.02, f"worst={worst:.5f} rad (unramped would be 0.0475)")
    check("the cap is still ramping, not arrived", b_sp.speed_effective < 0.95, f"eff={b_sp.speed_effective:.3f}")
    check("but it is above where it started", b_sp.speed_effective > 0.1, f"eff={b_sp.speed_effective:.3f}")
b_sp.speed_scale = 0.0          # the slider must stay an emergency brake
time.sleep(0.05)
check("a cut to zero is immediate", b_sp.speed_effective == 0.0, f"eff={b_sp.speed_effective}")
n_stop = len(arm_sp.servo_cmds)
time.sleep(0.08)
frozen = [q for _, q in arm_sp.servo_cmds[n_stop:]]
if len(frozen) > 1:
    check("and the arm holds", max(abs(a - b) for a, b in zip(frozen[-1], frozen[0])) < 1e-9)
b_sp.stop()
b_sp.shutdown()

print("== speed cap: reaches the target if left alone ==")
b_r = motion_bridge(MockArm(initial_q=[0.0] * 6), rate_hz=200.0, watchdog_s=5.0, speed_slew_per_s=10.0)
b_r.connect()
b_r.enable(sync_q=[0.0] * 6)
b_r.speed_scale = 0.0
time.sleep(0.05)
b_r.speed_scale = 1.0
time.sleep(0.4)
check("ramp completes", abs(b_r.speed_effective - 1.0) < 1e-9, f"eff={b_r.speed_effective:.4f}")
check("speed_scale still reports the request", b_r.speed_scale == 1.0, str(b_r.speed_scale))
b_r.stop()
b_r.shutdown()

print("== joint-limit net: streamed targets stay 2 deg inside the limits ==")
J5_LO = FALLBACK_JOINT_LIMITS_RAD[4][0]
arm_l = MockArm(initial_q=[0.0] * 6)
b_l = motion_bridge(arm_l, rate_hz=200.0, watchdog_s=5.0, speed_slew_per_s=10.0)
b_l.connect()
b_l.enable(sync_q=[0.0] * 6)
b_l.speed_scale = 1.0
time.sleep(0.15)
b_l.publish_desired_q([0.0, 0.0, -0.5, 0.0, J5_LO, 0.0])      # the C23 pose: J5 exactly on its limit
time.sleep(0.6)
j5 = [q[4] for _, q in arm_l.servo_cmds]
check("J5 never reaches the limit: held 2 deg inside", min(j5) >= J5_LO + JOINT_LIMIT_MARGIN_RAD - 1e-9, f"min J5 {min(j5):.4f} vs limit {J5_LO:.4f}")
check("and it did get close (the clamp stopped it, not the speed cap)", min(j5) < J5_LO + 0.05, f"min J5 {min(j5):.4f}")
b_l.stop()
b_l.shutdown()

print("== joint-limit net: an arm re-enabled outside the band eases back in, no jump ==")
q_out = [0.0, 0.0, -0.5, 0.0, J5_LO - 0.0001, 0.0]            # measured pose just past the limit
arm_o = MockArm(initial_q=list(q_out))
b_o = motion_bridge(arm_o, rate_hz=200.0, watchdog_s=5.0, speed_slew_per_s=10.0)
b_o.connect()
b_o.speed_scale = 0.3            # 0.015 rad per tick, so the 2 deg (0.035) takes three ticks
b_o.enable(sync_q=list(q_out))
time.sleep(0.3)
step_max = b_o._max_joint_step * 0.3 + 1e-9
first = arm_o.servo_cmds[0][1]
steps = [max(abs(x - y) for x, y in zip(b[1], a[1])) for a, b in zip(arm_o.servo_cmds, arm_o.servo_cmds[1:])]
check("first packet is at most one step from the measured pose (no jump)", max(abs(x - y) for x, y in zip(first, q_out)) <= step_max, str(first))
check("every step within the per-tick clamp", max(steps) <= step_max, f"max step {max(steps):.4f}")
check("ends inside the band", arm_o.servo_cmds[-1][1][4] >= J5_LO + JOINT_LIMIT_MARGIN_RAD - 1e-9, str(arm_o.servo_cmds[-1][1]))
b_o.stop()
b_o.shutdown()

T.finish("test_robot")
