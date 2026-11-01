# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""Controller safety configuration: what the operator declares, what the controller reports, and
the comparison that gates arming. Only `park` moves the arm; the CLI gates it."""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from pathlib import Path

import yaml

FIELDS = ("collision_sensitivity", "reduced_mode", "fence_mode", "tcp_boundary_mm",
          "max_tcp_speed_mm_s", "max_joint_speed_rad_s")
BOUNDARY_KEYS = ("x_max", "x_min", "y_max", "y_min", "z_max", "z_min")


class SafetyError(ValueError):
    """The declared safety configuration is missing, incomplete or malformed."""


@dataclass
class SafetyConfig:
    """What the operator declares the controller must be set to. Every field must be set: None means
    'not decided', and an undecided field refuses arming."""
    collision_sensitivity: int | None = None       # 0 disables collision detection; 1-5 enable it
    reduced_mode: bool | None = None               # controller reduced mode (speed and boundary limits)
    fence_mode: bool | None = None                 # controller TCP boundary ("fence") enforced
    tcp_boundary_mm: list[float] | None = None     # x_max, x_min, y_max, y_min, z_max, z_min (controller mm)
    max_tcp_speed_mm_s: float | None = None        # the controller's TCP speed limit must not exceed this
    max_joint_speed_rad_s: float | None = None     # the controller's joint speed limit must not exceed this

    @classmethod
    def load(cls, path) -> "SafetyConfig":
        path = Path(path)
        try:
            data = yaml.safe_load(path.read_text()) or {}
        except FileNotFoundError:
            raise SafetyError(f"{path}: not found") from None
        if not isinstance(data, dict):
            raise SafetyError(f"{path}: expected a mapping of safety fields")
        unknown = sorted(set(data) - set(FIELDS))
        if unknown:
            raise SafetyError(f"{path}: unknown keys {unknown}; known keys are {list(FIELDS)}")
        cfg = cls(**{k: data.get(k) for k in FIELDS})
        cfg.validate()
        return cfg

    def undeclared(self) -> list[str]:
        return [f for f in FIELDS if getattr(self, f) is None]

    def validate(self) -> None:
        """Type and range checks on the fields that are declared. Undeclared fields are reported by
        `undeclared()`, not here, so a partial file can still be inspected."""
        if self.collision_sensitivity is not None and (
                isinstance(self.collision_sensitivity, bool) or not isinstance(self.collision_sensitivity, int)
                or not 0 <= self.collision_sensitivity <= 5):
            raise SafetyError("collision_sensitivity must be an integer 0-5")
        for name in ("reduced_mode", "fence_mode"):
            v = getattr(self, name)
            if v is not None and not isinstance(v, bool):
                raise SafetyError(f"{name} must be true or false")
        if self.tcp_boundary_mm is not None:
            b = self.tcp_boundary_mm
            if not isinstance(b, (list, tuple)) or len(b) != 6 or not all(isinstance(v, (int, float)) for v in b):
                raise SafetyError("tcp_boundary_mm must be six numbers: x_max, x_min, y_max, y_min, z_max, z_min")
            if not (b[0] >= b[1] and b[2] >= b[3] and b[4] >= b[5]):
                raise SafetyError("tcp_boundary_mm: each max must be >= its min")
            self.tcp_boundary_mm = [float(v) for v in b]
        for name in ("max_tcp_speed_mm_s", "max_joint_speed_rad_s"):
            v = getattr(self, name)
            if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0):
                raise SafetyError(f"{name} must be a positive number")

    def require_complete(self) -> None:
        missing = self.undeclared()
        if missing:
            raise SafetyError("safety config leaves undecided: " + ", ".join(missing))


@dataclass
class SafetyReport:
    ok: bool
    controller: dict
    declared: dict
    mismatches: list[str]

    def table(self) -> str:
        rows = [("field", "declared", "controller", "result")]
        for f in FIELDS:
            d, c = self.declared.get(f), self.controller.get(f)
            bad = [m for m in self.mismatches if m.startswith(f + ":")]
            rows.append((f, _fmt(d), _fmt(c), bad[0].split(":", 1)[1].strip() if bad else "ok"))
        extra = [("firmware", "", _fmt(self.controller.get("version")), ""),
                 ("error_code / state", "", f"{_fmt(self.controller.get('error_code'))} / {_fmt(self.controller.get('state'))}", "")]
        rows += extra
        w = [max(len(str(r[i])) for r in rows) for i in range(4)]
        lines = ["  ".join(str(r[i]).ljust(w[i]) for i in range(4)) for r in rows]
        lines.insert(1, "  ".join("-" * w[i] for i in range(4)))
        lines.append("")
        lines.append("SAFETY OK: controller matches the declared configuration" if self.ok
                     else f"SAFETY MISMATCH ({len(self.mismatches)}): " + "; ".join(self.mismatches))
        return "\n".join(lines)


def _fmt(v) -> str:
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:g}"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    return str(v)


def _prop(arm, name):
    try:
        v = getattr(arm, name)
    except Exception:  # noqa: BLE001
        return None
    try:
        return v() if callable(v) else v
    except Exception:  # noqa: BLE001
        return None


def read_controller_safety(arm) -> dict:
    """Read the controller's safety-relevant settings. Read-only. Fields the SDK or firmware cannot
    report come back as None, which the comparison treats as a mismatch."""
    out = {f: None for f in FIELDS}
    out.update(version=None, joint_range_rad=None, collision_rebound=None, error_code=None, state=None)
    try:
        code, ver = arm.get_version()
        out["version"] = ver if code == 0 else None
    except Exception:  # noqa: BLE001
        pass
    out["collision_sensitivity"] = _prop(arm, "collision_sensitivity")
    out["error_code"] = _prop(arm, "error_code")
    out["state"] = _prop(arm, "state")
    states = None
    try:
        code, states = arm.get_reduced_states(is_radian=True)
        if code != 0:
            states = None
    except Exception:  # noqa: BLE001
        states = None
    if states:
        # [on, [x_max, x_min, y_max, y_min, z_max, z_min], tcp_speed, joint_speed, joint_range, fence_on, rebound_on]
        out["reduced_mode"] = bool(states[0])
        out["tcp_boundary_mm"] = [float(v) for v in states[1]]
        out["max_tcp_speed_mm_s"] = float(states[2])
        out["max_joint_speed_rad_s"] = float(states[3])
        out["joint_range_rad"] = states[4] if len(states) > 4 else None
        if len(states) > 5:
            out["fence_mode"] = bool(states[5])
        if len(states) > 6:
            out["collision_rebound"] = bool(states[6])
    if out["reduced_mode"] is None:
        out["reduced_mode"] = _as_bool(_prop(arm, "is_reduced_mode"))
    if out["fence_mode"] is None:
        out["fence_mode"] = _as_bool(_prop(arm, "is_fence_mode"))
    if out["tcp_boundary_mm"] is None:
        b = _prop(arm, "reduced_tcp_boundary")
        out["tcp_boundary_mm"] = [float(v) for v in b] if b is not None and len(b) >= 6 else None
    if out["collision_sensitivity"] is not None:
        out["collision_sensitivity"] = int(out["collision_sensitivity"])
    return out


def _as_bool(v):
    return None if v is None else bool(v)


def compare(declared: SafetyConfig, controller: dict) -> SafetyReport:
    """Exact match for levels and modes; the controller's speed limits must be at or below the declared
    ones; the controller's TCP box must lie inside the declared box."""
    mm: list[str] = []
    for f in FIELDS:
        d, c = getattr(declared, f), controller.get(f)
        if d is None:
            mm.append(f"{f}: not declared (null) in the safety config")
            continue
        if c is None:
            mm.append(f"{f}: controller did not report it")
            continue
        if f == "collision_sensitivity" and int(c) != int(d):
            mm.append(f"{f}: controller {c}, declared {d}")
        elif f in ("reduced_mode", "fence_mode") and bool(c) != bool(d):
            mm.append(f"{f}: controller {'on' if c else 'off'}, declared {'on' if d else 'off'}")
        elif f in ("max_tcp_speed_mm_s", "max_joint_speed_rad_s") and float(c) > float(d) + 1e-9:
            mm.append(f"{f}: controller allows {c:g}, declared at most {d:g}")
        elif f == "tcp_boundary_mm":
            outside = [k for i, k in enumerate(BOUNDARY_KEYS)
                       if ((c[i] > d[i] + 1e-6) if i % 2 == 0 else (c[i] < d[i] - 1e-6))]
            if outside:
                mm.append(f"{f}: controller box exceeds the declared box on {', '.join(outside)}")
    return SafetyReport(ok=not mm, controller=controller, declared=asdict(declared), mismatches=mm)


def verify_safety_config(arm, declared: SafetyConfig) -> SafetyReport:
    """Read the controller and compare. Read-only; the caller decides what a mismatch means."""
    return compare(declared, read_controller_safety(arm))


def set_collision_sensitivity(arm, level: int, settle_s: float = 0.5) -> dict:
    """Explicit operator action: set the level, persist it with save_conf (the controller forgets unsaved
    settings on reboot) and read it back. Nothing moves. Power-cycle the controller to prove persistence."""
    level = int(level)
    out = {"wanted": level, "before": _prop(arm, "collision_sensitivity")}
    if out["before"] is None or int(out["before"]) != level:
        out["set"] = arm.set_collision_sensitivity(level, wait=True)
    out["save_conf"] = arm.save_conf()
    time.sleep(settle_s)
    if hasattr(arm, "get_collision_sensitivity"):
        try:
            code, val = arm.get_collision_sensitivity()
            if code == 0 and val is not None:
                out["after"] = int(val)
        except Exception as e:  # noqa: BLE001
            out["read_error"] = str(e)
    if "after" not in out:
        out["after"] = _prop(arm, "collision_sensitivity")
    out["ok"] = out["after"] is not None and int(out["after"]) == level
    return out


def park(arm, q_rad, speed_rad_s: float = 0.35, timeout_s: float = 25.0) -> bool:
    """Slow position-mode joint move to a resting pose, then stop. Moves the arm: callers gate it."""
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    code = arm.set_servo_angle(angle=[float(v) for v in q_rad], speed=speed_rad_s, is_radian=True,
                               wait=True, timeout=timeout_s)
    arm.set_state(4)
    return code == 0 and int(_prop(arm, "error_code") or 0) == 0
