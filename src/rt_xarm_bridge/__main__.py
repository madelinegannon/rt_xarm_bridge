# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""Command line for inspecting and managing an arm without writing code.

  rt-xarm-bridge check   [--arm mock|xarm --arm-ip IP] [--safety safety.yaml]   read-only comparison
  rt-xarm-bridge monitor [--arm ...] [--hz 2] [--seconds N]                     read-only stream
  rt-xarm-bridge stop    [--arm ...]                                             halt, servos off
  rt-xarm-bridge park    --arm ... --safety FILE --allow-motion --pose q1 .. q6  moves the arm
  rt-xarm-bridge set-collision --arm ... --level N                               writes controller config

Everything defaults to the mock arm. A real arm needs --arm xarm and --arm-ip; nothing here moves
it except `park`, which needs --allow-motion and a safety file that matches the controller.
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time

from .bridge import make_arm
from .safety import (SafetyConfig, SafetyError, compare, park, read_controller_safety,
                     set_collision_sensitivity)

log = logging.getLogger("rt_xarm_bridge")


def _connect(a):
    arm = make_arm(a.arm, ip=a.arm_ip)
    arm.connect()
    if a.arm == "xarm":
        time.sleep(0.3)      # the SDK's report stream needs a moment before properties are meaningful
    return arm


def cmd_check(a) -> int:
    arm = _connect(a)
    try:
        controller = read_controller_safety(arm)
        cfg = SafetyConfig.load(a.safety) if a.safety else SafetyConfig()
        report = compare(cfg, controller)
    finally:
        arm.disconnect()
    if a.json:
        print(json.dumps({"ok": report.ok, "controller": report.controller, "declared": report.declared,
                          "mismatches": report.mismatches}, indent=1, default=str))
    else:
        print(report.table())
        if not a.safety:
            print("no --safety file given: nothing is verified until you declare one (see safety.example.yaml)")
    return 0 if report.ok else 1


def cmd_monitor(a) -> int:
    arm = _connect(a)
    period = 1.0 / max(0.1, a.hz)
    t_end = None if a.seconds is None else time.monotonic() + a.seconds
    try:
        while t_end is None or time.monotonic() < t_end:
            code, q = arm.get_servo_angle(is_radian=True)
            qd = [round(math.degrees(v), 1) for v in q] if code == 0 and q is not None else None
            print(json.dumps({"t": time.strftime("%H:%M:%S"), "q_deg": qd,
                              "error_code": getattr(arm, "error_code", None), "warn_code": getattr(arm, "warn_code", None),
                              "state": getattr(arm, "state", None), "mode": getattr(arm, "mode", None),
                              "connected": getattr(arm, "connected", None)}), flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        arm.disconnect()
    return 0


def cmd_stop(a) -> int:
    arm = _connect(a)
    try:
        arm.set_state(4)
        arm.motion_enable(False)
        print(json.dumps({"stopped": True, "state": getattr(arm, "state", None),
                          "error_code": getattr(arm, "error_code", None)}))
    finally:
        arm.disconnect()
    return 0


def cmd_park(a) -> int:
    if not a.allow_motion:
        print("park moves the arm: refuse without --allow-motion", file=sys.stderr)
        return 2
    cfg = SafetyConfig.load(a.safety)
    cfg.require_complete()
    arm = _connect(a)
    try:
        report = compare(cfg, read_controller_safety(arm))
        if not report.ok:
            print(report.table())
            print("refusing to move: the controller does not match the declared safety config", file=sys.stderr)
            return 1
        q = [math.radians(v) for v in a.pose] if a.degrees else list(a.pose)
        ok = park(arm, q, speed_rad_s=a.speed, timeout_s=a.timeout)
        print(json.dumps({"parked": ok, "pose_rad": [round(v, 4) for v in q],
                          "error_code": getattr(arm, "error_code", None)}))
        return 0 if ok else 1
    finally:
        arm.disconnect()


def cmd_set_collision(a) -> int:
    arm = _connect(a)
    try:
        out = set_collision_sensitivity(arm, a.level)
    finally:
        arm.disconnect()
    print(json.dumps(out, indent=1, default=str))
    if out["ok"]:
        print("saved to the controller: power-cycle it and run `check` to prove the setting persisted")
    return 0 if out["ok"] else 1




def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--arm", choices=["mock", "xarm"], default="mock", help="mock (default) or a real controller")
    common.add_argument("--arm-ip", default="", help="controller IP; required with --arm xarm, no default")
    common.add_argument("-v", "--verbose", action="store_true")
    ap = argparse.ArgumentParser(prog="rt-xarm-bridge", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("check", parents=[common], help="read the controller's safety settings and compare with a safety file")
    p.add_argument("--safety", help="declared safety config (YAML); without it nothing is verified")
    p.add_argument("--json", action="store_true")
    p.set_defaults(fn=cmd_check)

    p = sub.add_parser("monitor", parents=[common], help="print joints, error/warn codes and state until Ctrl-C")
    p.add_argument("--hz", type=float, default=2.0)
    p.add_argument("--seconds", type=float, default=None)
    p.set_defaults(fn=cmd_monitor)

    p = sub.add_parser("stop", parents=[common], help="halt the controller and turn the servos off (brakes on)")
    p.set_defaults(fn=cmd_stop)

    p = sub.add_parser("park", parents=[common], help="slow position-mode move to a resting pose, then stop (moves the arm)")
    p.add_argument("--allow-motion", action="store_true", help="required: acknowledges that this moves the arm")
    p.add_argument("--safety", required=True, help="declared safety config; verified before any motion")
    p.add_argument("--pose", type=float, nargs=6, required=True, metavar="Q", help="six joint angles, radians")
    p.add_argument("--degrees", action="store_true", help="--pose is in degrees")
    p.add_argument("--speed", type=float, default=0.35, help="joint speed, rad/s (default 0.35)")
    p.add_argument("--timeout", type=float, default=25.0)
    p.set_defaults(fn=cmd_park)

    p = sub.add_parser("set-collision", parents=[common], help="set and persist the controller's collision sensitivity (0-5)")
    p.add_argument("--level", type=int, required=True, choices=range(0, 6))
    p.set_defaults(fn=cmd_set_collision)
    return ap


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    try:
        return a.fn(a)
    except SafetyError as e:
        print(f"safety config: {e}", file=sys.stderr)
        return 2
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
