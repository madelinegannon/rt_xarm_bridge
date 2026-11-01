# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""Supervised-automation tests: RecoveryPolicy (pure) + RobotBridge auto-recovery, reconnect and
halts, with auto_arm and reconnect switched on explicitly. MockArm only."""
import time

import _common as T
from _common import check, wait_for

from rt_xarm_bridge import (CONNECTED, DISCONNECTED, ENABLED, MONITORING,
                            MockArm, RobotBridge, RecoveryPolicy)
from rt_xarm_bridge.recovery import ESTOP, NEEDS_OPERATOR, NEVER, RECOVER, RECOVER_HOME, SERVO_LIMITED, TRAJECTORY, TRANSIENT


def make(auto_arm=True, policy=None, **kw):
    arm = MockArm(initial_q=[0.1] * 6)
    events = []
    b = RobotBridge(arm, allow_motion=True, safety=arm.safety_config(), rate_hz=200.0, watchdog_s=0.5,
                    max_joint_step=0, auto_arm=auto_arm, reconnect=True,
                    recovery=policy if policy is not None else RecoveryPolicy(
                        {"cooldown_s": {"31": 0.05, "22": 0.05, "23": 0.05, "default": 0.02}}),
                    reconnect_backoff_s=(0.05, 0.1, 0.2), on_event=events.append, **kw)
    return arm, b, events


print("== RecoveryPolicy classification ==")
p = RecoveryPolicy()
check("C2 is estop", p.classify(2) == ESTOP)
check("C31 transient", p.classify(31) == TRANSIENT)
check("C60 servo_j speed transient", p.classify(60) == TRANSIENT)
check("C12 servo limited", p.classify(12) == SERVO_LIMITED)
check("C22 trajectory", p.classify(22) == TRAJECTORY)
check("C39 never", p.classify(39) == NEVER)
check("C110 never (>=100)", p.classify(110) == NEVER)

print("== RecoveryPolicy decisions ==")
p = RecoveryPolicy()
d = p.decide(2, 0.0)
check("estop -> needs_operator", d.action == NEEDS_OPERATOR and "E-stop" in d.reason, d.reason)
d = p.decide(24, 0.0)
check("C24 -> recover w/ default cooldown", d.action == RECOVER and d.cooldown_s == 1.0, str(d))
d = p.decide(31, 0.0)
check("C31 -> recover_home 3s", d.action == RECOVER_HOME and d.cooldown_s == 3.0, str(d))
p.decide(31, 10.0)
d = p.decide(31, 20.0)
check("3rd collision in 5 min -> lockout", d.action == NEEDS_OPERATOR and "collisions" in d.reason, d.reason)
p = RecoveryPolicy()
for t in (0.0, 100.0, 200.0):
    p.record_recovery(t)
d = p.decide(24, 250.0)
check("4th recovery in 10 min -> needs_operator", d.action == NEEDS_OPERATOR, d.reason)
d = p.decide(24, 700.0)
check("window slides (3 recoveries aged out)", d.action == RECOVER, d.reason)
p = RecoveryPolicy()
d1 = p.decide(12, 0.0)
d2 = p.decide(13, 60.0)
check("servo: first attempt ok, repeat within 10 min -> operator", d1.action == RECOVER and d2.action == NEEDS_OPERATOR, f"{d1.action} {d2.action}")
p = RecoveryPolicy()
d1 = p.decide(22, 0.0)
d2 = p.decide(23, 30.0)
check("trajectory: once -> recover_home, twice -> operator", d1.action == RECOVER_HOME and d2.action == NEEDS_OPERATOR, f"{d1.action} {d2.action}")
p.reset()
check("reset clears windows", p.decide(22, 31.0).action == RECOVER_HOME)
check("config override merges", RecoveryPolicy({"max_per_window": 1}).config["window_s"] == 600.0)

print("== the safety config is re-verified on every arm, and never written ==")
arm, b, ev = make(auto_arm=False)
b.soft_connect()
check("arms when the controller matches", b.arm_robot() is True and b.state == CONNECTED, b.state)
check("safety_check event", any(e.get("event") == "safety_check" and e.get("ok") for e in ev))
check("no controller writes", arm.save_conf_calls == 0 and arm.set_sensitivity_calls == 0)
b.stop()
arm.collision_sensitivity = 5      # someone changed the controller behind our back
b.recover()
check("re-arm refused after the controller changed", b.arm_robot() is False and "collision_sensitivity" in (b.halt_reason or ""), str(b.halt_reason))
b.shutdown()

print("== transient error (C24) auto-recovers ==")
arm, b, ev = make()
b.soft_connect()
b.arm_robot()
b.enable(sync_q=[0.1] * 6)
b.publish_desired_q([0.2] * 6)
check("streaming", wait_for(lambda: len(arm.servo_cmds) > 0))
n0 = len(arm.servo_cmds)
arm.inject_error(24)
check("demoted to MONITORING", wait_for(lambda: b.state == MONITORING, 1.0), b.state)
check("back to ENABLED by itself", wait_for(lambda: b.state == ENABLED, 2.0), b.state)
check("error cleared on controller", arm.error_code == 0 and arm.state == 0, f"err={arm.error_code} st={arm.state}")
check("recovery counted", b.recoveries_total == 1 and b.halt_reason is None)
check("C24 -> 'resync' hint, no HOME", b.pop_recovery_hint() == "resync")
b.publish_desired_q([0.3] * 6)
check("streams again after recovery", wait_for(lambda: len(arm.servo_cmds) > n0))
names = [e["event"] for e in ev]
check("event trail", names.index("fault") < names.index("recovery_decision") < names.index("recovered"), str(names))
check("safety re-checked during recovery", names.count("safety_check") >= 2, str(names.count("safety_check")))
b.shutdown()

print("== collision (C31) recovers with HOME hint, 3x -> lockout ==")
arm, b, ev = make(policy=RecoveryPolicy({"cooldown_s": {"31": 0.05, "default": 0.02}, "max_per_window": 10}))
b.soft_connect()
b.arm_robot()
b.enable(sync_q=[0.1] * 6)
b.publish_desired_q([0.1] * 6)
for i in range(2):
    arm.inject_error(31)
    check(f"collision #{i+1} recovered", wait_for(lambda: b.state == ENABLED and arm.error_code == 0, 2.0), b.state)
    check(f"collision #{i+1} HOME hint", b.pop_recovery_hint() == "home")
    b.publish_desired_q([0.1] * 6)
    time.sleep(0.1)
arm.inject_error(31)
check("3rd collision -> halted", wait_for(lambda: b.halt_reason is not None, 2.0), str(b.halt_reason))
check("stays MONITORING, motion disabled", b.state == MONITORING and not arm.motion_enabled, f"{b.state} motion={arm.motion_enabled}")
time.sleep(0.3)
check("no further auto attempts while halted", b.state == MONITORING and b.recoveries_total == 2, f"{b.state} n={b.recoveries_total}")
b.recover()
check("operator recover lifts the halt", b.halt_reason is None and arm.error_code == 0 and arm.motion_enabled)
check("re-armed by the operator", b.arm_robot() is True and b.state == CONNECTED, b.state)
b.shutdown()

print("== physical E-stop (C2) is never auto-cleared ==")
arm, b, ev = make()
b.soft_connect()
b.arm_robot()
b.enable(sync_q=[0.1] * 6)
arm.inject_error(2, latched=True)
check("halted", wait_for(lambda: b.halt_reason is not None, 2.0), str(b.halt_reason))
check("reason mentions E-stop", "E-stop" in (b.halt_reason or ""))
time.sleep(0.3)
check("controller error untouched", arm.error_code == 2 and b.recoveries_total == 0)
b.recover()   # button still pressed: recover cannot clear it
check("recover while still pressed leaves error", arm.error_code == 2)
check("arm_robot refused while the controller is not ready", b.arm_robot() is False and "not ready" in (b.halt_reason or ""), str(b.halt_reason))
check("servos disarmed again", not arm.motion_enabled)
arm.release_estop()
b.recover()
check("after release + recover -> CONNECTED", b.arm_robot() is True and b.state == CONNECTED, b.state)
b.shutdown()

print("== hardware class (C39) -> operator; auto_arm off -> nothing happens ==")
arm, b, ev = make()
b.soft_connect()
b.arm_robot()
arm.inject_error(39)
check("C39 -> halted", wait_for(lambda: b.halt_reason is not None, 2.0), str(b.halt_reason))
b.shutdown()
arm, b, ev = make(auto_arm=False)
b.soft_connect()
b.arm_robot()
b.enable(sync_q=[0.1] * 6)
arm.inject_error(24)
check("auto_arm off: demoted", wait_for(lambda: b.state == MONITORING, 1.0), b.state)
time.sleep(0.3)
check("auto_arm off: stays MONITORING, no recovery", b.state == MONITORING and b.recoveries_total == 0)
b.shutdown()

print("== TCP reconnect with backoff + auto re-arm (opt-in) ==")
arm, b, ev = make()
b.soft_connect()
b.arm_robot()
b.enable(sync_q=[0.1] * 6)
arm.inject_disconnect(fail_next_connects=2)
check("DISCONNECTED on loss", wait_for(lambda: b.state == DISCONNECTED, 1.0), b.state)
check("reconnects and re-enables", wait_for(lambda: b.state == ENABLED, 3.0), b.state)
check("backoff: >=3 connect calls", arm.connect_calls >= 4, str(arm.connect_calls))
attempts = [e for e in ev if e["event"] == "reconnect_attempt"]
check("attempt events", len(attempts) >= 3, str(len(attempts)))
check("a reconnect re-enables at the measured pose -> 'resync' hint", b.pop_recovery_hint() == "resync")
b.publish_desired_q([0.4] * 6)
check("streams after reconnect", wait_for(lambda: any(abs(c[1][0] - 0.4) < 1e-9 for c in arm.servo_cmds), 1.0))
b.shutdown()

print("== operator disconnect does NOT auto-reconnect ==")
arm, b, ev = make()
b.soft_connect()
b.disconnect()
time.sleep(0.4)
check("stays DISCONNECTED", b.state == DISCONNECTED and arm.connect_calls == 1, f"{b.state} calls={arm.connect_calls}")
b.shutdown()

print("== soft_connect with the controller down retries until it answers (reconnect on) ==")
arm = MockArm()
arm.fail_connects = 2
b = RobotBridge(arm, rate_hz=200.0, reconnect=True, reconnect_backoff_s=(0.05, 0.05))
b.soft_connect()
check("initially DISCONNECTED", b.state == DISCONNECTED, b.state)
check("eventually MONITORING", wait_for(lambda: b.state == MONITORING, 2.0), b.state)
b.shutdown()

T.finish("test_recovery")
