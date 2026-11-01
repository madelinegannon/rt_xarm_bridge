# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""Auto-recovery policy: which controller errors may be cleared without a human, how long to
wait, and when to give up and ask an operator. Pure Python, injectable clock.

Controller error codes (UFACTORY xArm/850 firmware table, xarm SDK 1.18):
  1/2/3   E-stop (controller button / external IO / three-state switch)  -> never auto
  10-17   servo motor (10 = any, 11..17 = joint N)                        -> limited
  18/19/28 force-torque / end-effector / end-module comms                 -> limited
  21 kinematic, 22 self-collision, 23 joint limit, 40 no-error-trajectory -> clear once, go HOME
  24 speed limit, 25 planning, 27 command reply, 30 feedback speed,
  31 collision current, 35 safety boundary, 36 delay-cmd overflow,
  60 servo_j linear speed                                                 -> transient
  26 Linux RT, 33 GPIO, 37 manual-mode motion, 38 abnormal joint angle,
  39 power board, 41/42 bad files, 110/111 board comms, >= 100            -> never auto
"""
from __future__ import annotations

from dataclasses import dataclass, field

ESTOP = "estop"
TRANSIENT = "transient"
SERVO_LIMITED = "servo_limited"
TRAJECTORY = "trajectory"
NEVER = "never"
UNKNOWN = "unknown"

RECOVER = "recover"
RECOVER_HOME = "recover_home"
NEEDS_OPERATOR = "needs_operator"

DEFAULT_CONFIG = {
    "estop": [1, 2, 3],
    "transient": [24, 25, 27, 30, 31, 35, 36, 60],
    "servo_limited": [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 28],
    "trajectory": [21, 22, 23, 40],
    "never": [26, 33, 37, 38, 39, 41, 42, 110, 111],
    "cooldown_s": {"31": 3.0, "22": 5.0, "23": 5.0, "default": 1.0},
    "max_per_window": 3,
    "window_s": 600.0,
    "collision_lockout_n": 3,
    "collision_lockout_window_s": 300.0,
    "servo_limited_interval_s": 600.0,
    "trajectory_repeat_window_s": 600.0,
    "post_recovery_home_s": 6.0,
}


@dataclass
class Decision:
    action: str            # RECOVER | RECOVER_HOME | NEEDS_OPERATOR
    cooldown_s: float = 0.0
    reason: str = ""
    error_class: str = UNKNOWN


@dataclass
class RecoveryPolicy:
    config: dict = field(default_factory=lambda: dict(DEFAULT_CONFIG))

    def __post_init__(self):
        merged = dict(DEFAULT_CONFIG)
        merged.update(self.config or {})
        self.config = merged
        self._recoveries: list[float] = []
        self._collisions: list[float] = []
        self._servo_last: float | None = None
        self._trajectory: list[float] = []

    def classify(self, code: int) -> str:
        c = self.config
        code = int(code)
        for name in (ESTOP, TRANSIENT, SERVO_LIMITED, TRAJECTORY, NEVER):
            if code in c[name]:
                return name
        if code >= 100:
            return NEVER
        return UNKNOWN

    def cooldown_for(self, code: int) -> float:
        cd = self.config["cooldown_s"]
        return float(cd.get(str(int(code)), cd.get("default", 1.0)))

    def decide(self, code: int, now: float) -> Decision:
        code = int(code)
        cls = self.classify(code)
        c = self.config
        self._prune(now)

        if cls == ESTOP:
            return Decision(NEEDS_OPERATOR, 0.0, f"E-stop C{code}: release it, then recover", cls)
        if cls in (NEVER, UNKNOWN):
            return Decision(NEEDS_OPERATOR, 0.0, f"C{code} is not auto-recoverable", cls)
        if len(self._recoveries) >= int(c["max_per_window"]):
            return Decision(NEEDS_OPERATOR, 0.0,
                            f"{len(self._recoveries)} recoveries in {int(c['window_s'])}s (C{code})", cls)

        if cls == TRANSIENT:
            if code == 31:
                self._collisions.append(now)
                if len(self._collisions) >= int(c["collision_lockout_n"]):
                    return Decision(NEEDS_OPERATOR, 0.0,
                                    f"{len(self._collisions)} collisions in {int(c['collision_lockout_window_s'])}s",
                                    cls)
                return Decision(RECOVER_HOME, self.cooldown_for(code), f"collision C31 #{len(self._collisions)}", cls)
            return Decision(RECOVER, self.cooldown_for(code), f"transient C{code}", cls)

        if cls == SERVO_LIMITED:
            if self._servo_last is not None and now - self._servo_last < float(c["servo_limited_interval_s"]):
                return Decision(NEEDS_OPERATOR, 0.0, f"repeat servo/comm error C{code} within interval", cls)
            self._servo_last = now
            return Decision(RECOVER, self.cooldown_for(code), f"servo/comm C{code} (one attempt)", cls)

        if cls == TRAJECTORY:
            self._trajectory.append(now)
            if len(self._trajectory) >= 2:
                return Decision(NEEDS_OPERATOR, 0.0, f"repeat trajectory fault C{code}", cls)
            return Decision(RECOVER_HOME, self.cooldown_for(code), f"trajectory C{code}: clear once, HOME", cls)

        return Decision(NEEDS_OPERATOR, 0.0, f"unhandled class {cls}", cls)

    def record_recovery(self, now: float):
        self._recoveries.append(now)

    def recoveries_in_window(self, now: float) -> int:
        self._prune(now)
        return len(self._recoveries)

    def reset(self):
        """Operator reset: clear every window so a fresh session starts clean."""
        self._recoveries.clear()
        self._collisions.clear()
        self._trajectory.clear()
        self._servo_last = None

    def _prune(self, now: float):
        c = self.config
        self._recoveries = [t for t in self._recoveries if now - t < float(c["window_s"])]
        self._collisions = [t for t in self._collisions if now - t < float(c["collision_lockout_window_s"])]
        self._trajectory = [t for t in self._trajectory if now - t < float(c["trajectory_repeat_window_s"])]


def error_title(code: int) -> str:
    """Human-readable title from the SDK table if available (for logs and UIs)."""
    try:
        from xarm.core.config.x_code import ControllerErrorCodeMap
        return ControllerErrorCodeMap[int(code)]["en"]["title"]
    except Exception:  # noqa: BLE001
        return f"controller error C{int(code)}"
