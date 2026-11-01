# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""Servo bridge for UFACTORY xArm and 850 arms.

Runs on its own thread: the caller hands off the latest desired joint vector (non-blocking) and the
thread streams it to the arm at a fixed cadence, publishing the measured joints back. `MockArm` has
the call surface of the `xarm.wrapper.XArmAPI` subset used here, so the bridge, its state machine
and every gate are exercised with no hardware.

State machine:
  DISCONNECTED -> MONITORING (read-only) -> CONNECTED (armed, servo mode) -> ENABLED (streaming)

Gates: leaving MONITORING needs `allow_motion=True` and a complete `SafetyConfig`; arming compares
that config with the controller and refuses on any mismatch; the speed cap starts at or below
INITIAL_SPEED_SCALE_CEILING and ramps from zero on every enable; auto-recovery and reconnection are
off unless asked for; a target older than the watchdog (at most WATCHDOG_MAX_S) is never streamed.

Units: joint angles are radians throughout (XArmAPI(is_radian=True), URDF). The xarm SDK reports
cartesian positions in millimetres; xarm_pos_to_metres / metres_to_xarm_pos convert.
"""
from __future__ import annotations

import logging
import math
import threading
import time

from .recovery import NEEDS_OPERATOR, RECOVER_HOME, RecoveryPolicy
from .safety import SafetyConfig, SafetyError, SafetyReport, verify_safety_config

log = logging.getLogger("rt_xarm_bridge")

DISCONNECTED = "DISCONNECTED"
MONITORING = "MONITORING"
CONNECTED = "CONNECTED"
ENABLED = "ENABLED"

N_JOINTS = 6

# xArm 850 URDF joint limits in radians; the controller's reduced limits are intersected with these.
URDF_JOINT_LIMITS_RAD = [
    (-3.1101767, 3.1101767),
    (-2.3038346, 2.3038346),
    (-3.1101767, 0.061087),
    (-3.1101767, 3.1101767),
    (-2.1642, 2.1642),
    (-3.1101767, 3.1101767),
]
FALLBACK_JOINT_LIMITS_RAD = URDF_JOINT_LIMITS_RAD

INITIAL_SPEED_SCALE_CEILING = 0.2   # a config may start the cap here at most; only an operator raises it at runtime
MAX_JOINT_STEP_RAD = 0.05           # per-tick clamp ceiling: 6.25 rad/s at 125 Hz, beyond the arm's own spec
WATCHDOG_MAX_S = 1.0                # a target older than this is never streamed, whatever the config says
SLEW_STALL_TICKS = 4                # a stalled tick credits the ramp at most this much time: a hiccup never releases the gap at once
JOINT_LIMIT_MARGIN_RAD = 0.035      # 2 deg inside the limits: the controller faults (C23) at the nominal limit
_TWO_PI = 2.0 * math.pi
_MAX_ERROR_HISTORY = 20


class MotionNotAllowed(PermissionError):
    """Built without allow_motion=True: the bridge monitors, it never arms or streams."""


def _field(data, name, default):
    if isinstance(data, dict):
        return data.get(name, default)
    return getattr(data, name, default)


def _clamp(v, lo, hi) -> float:
    return float(min(max(float(v), lo), hi))


def xarm_pos_to_metres(xarm_xyz):
    return [xarm_xyz[0] / 1000.0, xarm_xyz[1] / 1000.0, xarm_xyz[2] / 1000.0]


def metres_to_xarm_pos(metres_xyz):
    return [metres_xyz[0] * 1000.0, metres_xyz[1] * 1000.0, metres_xyz[2] * 1000.0]


class MockArm:
    """Stand-in for XArmAPI (the subset used here): perfect-tracking servo model, command recorder,
    fault injection, and a configurable safety state so every gate can be tested."""

    DEFAULT_BOUNDARY_MM = [700.0, -700.0, 700.0, -700.0, 1200.0, 0.0]
    DEFAULT_REDUCED_LIMITS_DEG = [(-170.0, 170.0), (-120.0, 120.0), (-170.0, 3.5),
                                  (-170.0, 170.0), (-110.0, 110.0), (-170.0, 170.0)]

    def __init__(self, initial_q=None, *, collision_sensitivity=3, reduced_mode=False, fence_mode=False,
                 tcp_boundary_mm=None, max_tcp_speed_mm_s=1000.0, max_joint_speed_rad_s=3.14,
                 limits_in_degrees=True, reduced_joint_limits_deg=None):
        self._lock = threading.Lock()
        self._q = list(initial_q) if initial_q is not None else [0.0] * N_JOINTS
        self.mode = 0
        self.state = 0
        self.error_code = 0
        self.warn_code = 0
        self.motion_enabled = False
        self.connected = False
        self.collision_sensitivity = collision_sensitivity   # a real controller ships with 3
        self.reduced_mode = bool(reduced_mode)
        self.fence_mode = bool(fence_mode)
        self.tcp_boundary_mm = list(tcp_boundary_mm) if tcp_boundary_mm is not None else list(self.DEFAULT_BOUNDARY_MM)
        self.max_tcp_speed_mm_s = float(max_tcp_speed_mm_s)
        self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        self.limits_in_degrees = bool(limits_in_degrees)     # the real controller answers in degrees
        self.reduced_joint_limits_deg = list(reduced_joint_limits_deg or self.DEFAULT_REDUCED_LIMITS_DEG)
        self.servo_cmds = []          # every streamed target (for assertions)
        self.position_cmds = []       # every position-mode move (park)
        self.estop_count = 0
        self.save_conf_calls = 0
        self.set_sensitivity_calls = 0
        self.fail_connects = 0        # next N connect() calls raise
        self.connect_calls = 0
        self._latched = False         # E-stop held: clean_error cannot clear
        self._on_error_warn = None
        self._on_state = None
        self._on_connect = None

    def safety_config(self) -> SafetyConfig:
        """A declared configuration that matches this mock's current settings."""
        return SafetyConfig(collision_sensitivity=int(self.collision_sensitivity), reduced_mode=self.reduced_mode,
                            fence_mode=self.fence_mode, tcp_boundary_mm=list(self.tcp_boundary_mm),
                            max_tcp_speed_mm_s=self.max_tcp_speed_mm_s, max_joint_speed_rad_s=self.max_joint_speed_rad_s)

    # --- connection / setup ---
    def connect(self):
        self.connect_calls += 1
        if self.fail_connects > 0:
            self.fail_connects -= 1
            raise ConnectionError("mock: controller unreachable")
        self.connected = True

    def disconnect(self):
        self.connected = False

    def motion_enable(self, enable=True):
        self.motion_enabled = bool(enable)
        return 0

    def clean_error(self):
        if not self._latched:
            self.error_code = 0
            if self.state == 4:
                self.state = 2   # after clean_error the arm sits in state 2 until set_state(0)
        return 0

    def clean_warn(self):
        self.warn_code = 0
        return 0

    def set_mode(self, mode):
        self.mode = mode
        return 0

    def set_state(self, state):
        if state == 0 and self.error_code != 0:
            return 1             # the controller refuses READY while an error is present
        self.state = state
        return 0

    def set_collision_sensitivity(self, level, wait=True):
        self.set_sensitivity_calls += 1
        self.collision_sensitivity = int(level)
        return 0

    def get_collision_sensitivity(self):
        return 0, int(self.collision_sensitivity)

    def save_conf(self):
        self.save_conf_calls += 1
        return 0

    def get_version(self):
        return 0, "mock-0.1"

    def get_reduced_states(self, is_radian=None):
        js = self.max_joint_speed_rad_s if is_radian else math.degrees(self.max_joint_speed_rad_s)
        jr = [[math.radians(lo), math.radians(hi)] if is_radian else [lo, hi] for lo, hi in self.reduced_joint_limits_deg]
        return 0, [self.reduced_mode, list(self.tcp_boundary_mm), self.max_tcp_speed_mm_s, js, jr,
                   self.fence_mode, False]

    @property
    def is_reduced_mode(self):
        return self.reduced_mode

    @property
    def is_fence_mode(self):
        return self.fence_mode

    @property
    def reduced_tcp_boundary(self):
        return list(self.tcp_boundary_mm)

    @property
    def reduced_joint_limits(self):
        """Like the controller: degrees, and +-360 on every joint while reduced mode is off."""
        if not self.reduced_mode:
            return [[-360.0, 360.0] for _ in range(N_JOINTS)]
        if self.limits_in_degrees:
            return [[lo, hi] for lo, hi in self.reduced_joint_limits_deg]
        return [[math.radians(lo), math.radians(hi)] for lo, hi in self.reduced_joint_limits_deg]

    # --- fault injection (tests only) ---
    def inject_error(self, code, hw_state=4, latched=False):
        self.error_code = int(code)
        self.state = hw_state
        self._latched = latched
        if self._on_error_warn:
            self._on_error_warn({"error_code": self.error_code, "warn_code": self.warn_code})
        if self._on_state:
            self._on_state({"state": self.state})

    def release_estop(self):
        self._latched = False

    def inject_disconnect(self, fail_next_connects=0):
        self.connected = False
        self.fail_connects = fail_next_connects
        if self._on_connect:
            self._on_connect({"connected": False})

    # --- runtime I/O ---
    def set_servo_angle_j(self, angles, is_radian=True, **kwargs):
        if not self.connected:
            raise ConnectionError("mock: not connected")
        q = [float(a) for a in angles]
        with self._lock:
            self._q = q
            self.servo_cmds.append((time.monotonic(), q))
        return 0

    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        is_radian=True, wait=False, timeout=None, **kwargs):
        if not self.connected:
            raise ConnectionError("mock: not connected")
        if self.mode != 0 or self.state != 0:
            return 9             # position moves need mode 0 and READY, like the controller
        q = [float(a) for a in angle]
        with self._lock:
            self._q = q
            self.position_cmds.append((time.monotonic(), q, speed))
        return 0

    def get_servo_angle(self, is_radian=True):
        if not self.connected:
            raise ConnectionError("mock: not connected")
        with self._lock:
            return 0, list(self._q)

    def get_position(self, is_radian=True):
        return 0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def emergency_stop(self):
        self.estop_count += 1
        self.state = 4
        return 0

    # --- callback registration (stored so inject_* can fire them like the SDK does) ---
    def register_error_warn_changed_callback(self, fn):
        self._on_error_warn = fn

    def register_state_changed_callback(self, fn):
        self._on_state = fn

    def register_connect_changed_callback(self, fn):
        self._on_connect = fn


def make_arm(kind: str, ip: str = "", initial_q=None, **mock_options):
    """kind='mock' -> MockArm (mock_options go to it); kind='xarm' -> XArmAPI, which needs an IP."""
    if kind == "mock":
        return MockArm(initial_q=initial_q, **mock_options)
    if kind == "xarm":
        if not ip:
            raise ValueError("a real arm needs its controller IP (--arm-ip); there is no default")
        from xarm.wrapper import XArmAPI  # lazy: only needed for real hardware
        return XArmAPI(ip, is_radian=True, do_not_open=True)
    raise ValueError(f"unknown arm kind {kind!r}: mock | xarm")


class RobotBridge:
    """Threaded bridge: streams desired_q to the arm, publishes measured real_q."""

    def __init__(self, arm, *, allow_motion=False, safety: SafetyConfig | None = None,
                 initial_speed_scale=INITIAL_SPEED_SCALE_CEILING, rate_hz=125.0, watchdog_s=0.5,
                 max_joint_step=MAX_JOINT_STEP_RAD, speed_slew_per_s=0.33,
                 auto_arm=False, recovery: RecoveryPolicy | None = None, reconnect=False,
                 reconnect_backoff_s=(1, 2, 4, 8, 15, 30), on_event=None, clock=time.monotonic):
        if allow_motion and safety is None:
            raise SafetyError("allow_motion=True needs a SafetyConfig: the controller is verified against it before arming")
        if safety is not None:
            safety.validate()
            safety.require_complete()
        if auto_arm and not allow_motion:
            raise MotionNotAllowed("auto_arm needs allow_motion=True")
        if auto_arm and recovery is None:
            raise ValueError("auto_arm needs a RecoveryPolicy")
        self._arm = arm
        self._allow_motion = bool(allow_motion)
        self._safety = safety
        self._safety_report: SafetyReport | None = None
        self._dt = 1.0 / float(rate_hz)
        self._watchdog_s = _clamp(watchdog_s, 0.0, WATCHDOG_MAX_S)
        self._max_joint_step = _clamp(max_joint_step, 0.0, MAX_JOINT_STEP_RAD)
        self._speed_scale = _clamp(initial_speed_scale, 0.0, INITIAL_SPEED_SCALE_CEILING)
        self._speed_eff = 0.0                   # what the clamp uses; ramps up toward _speed_scale
        self._speed_slew_per_s = float(speed_slew_per_s)
        self._slew_t = None                     # wall-clock of the last ramp step

        self._auto_arm = bool(auto_arm)         # after reconnect / auto-recovery, re-arm + ENABLE by itself
        self._recovery = recovery
        self._reconnect = bool(reconnect)       # retry TCP after an unexpected drop
        self._backoff = tuple(reconnect_backoff_s)
        self._on_event = on_event               # callable(dict) for logs; never raises into us
        self._clock = clock
        self._want_connected = False
        self._connect_attempt = 0
        self._next_connect_t = 0.0
        self._pending_error = None              # (code, t_seen) waiting for a recovery decision
        self._recover_at = None                 # (code, action, t_due)
        self._halt_reason = None                # str -> stays MONITORING until recover()
        self._recovery_hint = None              # "home" | "resync" for the app after a re-enable
        self._recoveries_total = 0

        self._lock = threading.Lock()
        self._state = DISCONNECTED
        self._desired_q = None
        self._desired_stamp = 0.0
        self._real_q = None
        self._real_stamp = 0.0
        self._last_sent = None
        self._limit_clamp_log_t: dict[int, float] = {}

        self._arm_error_code = 0
        self._arm_warn_code = 0
        self._arm_hw_state = 0
        self._arm_mode = 0
        self._arm_connected = False
        self._arm_position_m = None

        self._last_error = None
        self._error_history = []

        self._controller_limits = None          # what the controller reported, radians, or None
        self._joint_limits = None               # effective: controller limits intersected with the URDF table

        self._running = False
        self._thread = None

    # --- error capture ---
    def _capture_error(self, context, exc):
        msg = f"{context}: {exc}"
        with self._lock:
            self._last_error = msg
            self._error_history.append((time.monotonic(), msg))
            if len(self._error_history) > _MAX_ERROR_HISTORY:
                self._error_history.pop(0)
        log.error(msg)

    # --- properties ---
    @property
    def allow_motion(self):
        return self._allow_motion

    @property
    def safety_report(self):
        return self._safety_report

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def last_error(self):
        with self._lock:
            return self._last_error

    @property
    def arm_error_code(self):
        with self._lock:
            return self._arm_error_code

    @property
    def arm_warn_code(self):
        with self._lock:
            return self._arm_warn_code

    @property
    def arm_hw_state(self):
        with self._lock:
            return self._arm_hw_state

    @property
    def arm_mode(self):
        with self._lock:
            return self._arm_mode

    @property
    def arm_connected(self):
        with self._lock:
            return self._arm_connected

    @property
    def arm_position_m(self):
        with self._lock:
            return list(self._arm_position_m) if self._arm_position_m is not None else None

    @property
    def joint_limits(self):
        with self._lock:
            return list(self._joint_limits) if self._joint_limits is not None else list(URDF_JOINT_LIMITS_RAD)

    @property
    def controller_limits(self):
        with self._lock:
            return None if self._controller_limits is None else list(self._controller_limits)

    @property
    def speed_scale(self):
        return self._speed_scale

    @speed_scale.setter
    def speed_scale(self, v):
        """Runtime operator control, 0..1. Only this path can raise the cap above the construction ceiling."""
        self._speed_scale = _clamp(v, 0.0, 1.0)

    @property
    def speed_effective(self):
        """What the clamp is using right now: below speed_scale while a raise is still ramping in."""
        return self._speed_eff

    def _slew_speed(self, now):
        # a raise is an acceleration, not a setting: the arm may be behind an open-loop twin, and releasing
        # that gap at the new cap at once is a lurch (and a collision fault). Cuts apply at once.
        # integrated over measured time, so a host that oversleeps still ramps at speed_slew_per_s
        dt = self._dt if self._slew_t is None else _clamp(now - self._slew_t, 0.0, SLEW_STALL_TICKS * self._dt)
        self._slew_t = now
        if self._speed_eff > self._speed_scale:
            self._speed_eff = self._speed_scale
        elif self._speed_eff < self._speed_scale:
            self._speed_eff = min(self._speed_scale, self._speed_eff + self._speed_slew_per_s * dt)

    @property
    def halt_reason(self):
        with self._lock:
            return self._halt_reason

    @property
    def recoveries_total(self):
        with self._lock:
            return self._recoveries_total

    @property
    def auto_arm(self):
        return self._auto_arm

    @auto_arm.setter
    def auto_arm(self, v):
        if v and not self._allow_motion:
            raise MotionNotAllowed("auto_arm needs allow_motion=True")
        if v and self._recovery is None:
            raise ValueError("auto_arm needs a RecoveryPolicy")
        self._auto_arm = bool(v)

    def _inside_limits(self, q):
        """Clamp a desired pose 2 deg inside the effective joint limits, before the per-tick step clamp, so
        an arm re-enabled outside the band eases back in at the normal step and never in one jump."""
        out = list(q)
        m = JOINT_LIMIT_MARGIN_RAD
        now = None
        for i, (lo, hi) in enumerate(self.joint_limits[:len(out)]):
            v = float(out[i])
            c = min(max(v, lo + m), hi - m)
            if c != v:
                out[i] = c
                now = time.monotonic() if now is None else now
                if now - self._limit_clamp_log_t.get(i, -1e9) >= 10.0:      # one report per joint per 10 s
                    self._limit_clamp_log_t[i] = now
                    self._emit(event="joint_limit_clamp", joint=i + 1, wanted_deg=round(math.degrees(v), 2),
                               sent_deg=round(math.degrees(c), 2))
        return out

    def pop_recovery_hint(self):
        """One-shot: 'home' after a recovery that should settle via HOME first, 'resync' after any other
        re-enable at the measured pose (the app re-seeds its model from it), else None."""
        with self._lock:
            h, self._recovery_hint = self._recovery_hint, None
            return h

    def real_q_age(self):
        with self._lock:
            return None if self._real_q is None else self._clock() - self._real_stamp

    def _emit(self, **ev):
        if self._on_event is None:
            return
        try:
            self._on_event(dict(t=self._clock(), **ev))
        except Exception:  # noqa: BLE001
            pass

    def clear_error(self):
        with self._lock:
            self._last_error = None

    def latest_real_q(self):
        with self._lock:
            if self._real_q is None:
                return None
            return list(self._real_q), time.monotonic() - self._real_stamp

    # --- lifecycle ---
    def _start_thread(self):
        if self._thread is None:
            self._running = True
            self._thread = threading.Thread(target=self._loop, name="RobotBridge", daemon=True)
            self._thread.start()

    def _read_joint_limits(self):
        """Controller limits, converted from the degrees the controller answers in, intersected with the
        URDF table. +-360 on every joint means reduced mode is off: no controller limits then."""
        ctrl = None
        try:
            raw = getattr(self._arm, "reduced_joint_limits", None)
            if raw is not None and len(raw) >= N_JOINTS:
                pairs = [(float(raw[i][0]), float(raw[i][1])) for i in range(N_JOINTS)]
                if any(abs(v) > _TWO_PI for lo, hi in pairs for v in (lo, hi)):
                    pairs = [(math.radians(lo), math.radians(hi)) for lo, hi in pairs]
                if all(lo <= -_TWO_PI + 1e-6 and hi >= _TWO_PI - 1e-6 for lo, hi in pairs):
                    pairs = None
                ctrl = pairs
        except Exception as e:  # noqa: BLE001
            self._capture_error("read_joint_limits", e)
        eff = []
        for i, (ulo, uhi) in enumerate(URDF_JOINT_LIMITS_RAD):
            lo, hi = (max(ulo, ctrl[i][0]), min(uhi, ctrl[i][1])) if ctrl else (ulo, uhi)
            if lo >= hi - 2 * JOINT_LIMIT_MARGIN_RAD:
                log.warning("joint %d: controller limits (%.3f, %.3f) leave no room; using the URDF table", i + 1, lo, hi)
                lo, hi = ulo, uhi
            eff.append((lo, hi))
        with self._lock:
            self._controller_limits = ctrl
            self._joint_limits = eff
        log.info("joint limits: %s", "controller ∩ URDF" if ctrl else "URDF table (controller reports none)")

    def soft_connect(self):
        """Open the TCP link and start polling telemetry. Does not arm (no motion_enable, no set_mode,
        no set_state). State -> MONITORING. Allowed without allow_motion."""
        self._want_connected = True
        self._try_connect()
        self._start_thread()

    def _try_connect(self):
        try:
            self._arm.connect()
        except Exception as e:  # noqa: BLE001
            self._capture_error("connect", e)
            return False
        if getattr(self._arm, "connected", True) is False:
            self._capture_error("connect", RuntimeError("SDK reports not connected"))
            return False
        self._read_joint_limits()
        for reg, fn in (("register_error_warn_changed_callback", self._on_error_warn),
                        ("register_state_changed_callback", self._on_state_changed),
                        ("register_connect_changed_callback", self._on_connect_changed)):
            if hasattr(self._arm, reg):
                try:
                    getattr(self._arm, reg)(fn)
                except Exception:  # noqa: BLE001
                    pass
        with self._lock:
            self._state = MONITORING
            self._arm_connected = True
            self._connect_attempt = 0
        self._emit(event="connected")
        return True

    def _controller_ready(self, tries=10, wait_s=0.2):
        # the controller takes a moment to be in servo mode: streaming before it is there gets every
        # packet rejected with code=1 ("the mode may be incorrect")
        for _ in range(tries):
            err = int(getattr(self._arm, "error_code", 0) or 0)
            st = int(getattr(self._arm, "state", 0) or 0)
            if err == 0 and st not in (3, 4):
                return True
            time.sleep(wait_s)
        return False

    def _arming_calls(self):
        for name, fn in (("clean_error", lambda: self._arm.clean_error()),
                         ("clean_warn", lambda: self._arm.clean_warn()),
                         ("motion_enable", lambda: self._arm.motion_enable(True)),
                         ("set_mode", lambda: self._arm.set_mode(1)),
                         ("set_state", lambda: self._arm.set_state(0))):
            try:
                fn()
            except Exception as e:  # noqa: BLE001
                self._capture_error(f"arm.{name}", e)

    def _disarm_calls(self):
        for name, fn in (("set_state", lambda: self._arm.set_state(4)),
                         ("motion_enable", lambda: self._arm.motion_enable(False))):
            try:
                fn()
            except Exception as e:  # noqa: BLE001
                self._capture_error(f"disarm.{name}", e)

    def _check_safety(self, context):
        """Read the controller and compare with the declared config. Caller handles a False."""
        try:
            report = verify_safety_config(self._arm, self._safety)
        except Exception as e:  # noqa: BLE001
            self._capture_error(f"{context}.safety_check", e)
            return False
        self._safety_report = report
        self._emit(event="safety_check", ok=report.ok, mismatches=list(report.mismatches))
        if not report.ok:
            self._emit(event="safety_refused", context=context, mismatches=list(report.mismatches))
        return report.ok

    def arm_robot(self):
        """Arm for servo mode: clean errors, enable motion, set mode/state, verify the safety config.
        Only from MONITORING and only with allow_motion. State -> CONNECTED. Returns True on success."""
        if not self._allow_motion:
            raise MotionNotAllowed("arm_robot: the bridge was started without allow_motion; it only monitors")
        with self._lock:
            cur, halted = self._state, self._halt_reason
        if cur != MONITORING:
            self._capture_error("arm_robot", RuntimeError(f"must be MONITORING, got {cur}"))
            return False
        if halted:
            self._capture_error("arm_robot", RuntimeError(f"halted: {halted}; call recover() first"))
            return False
        self._arming_calls()
        if not self._controller_ready():
            err, st = getattr(self._arm, "error_code", None), getattr(self._arm, "state", None)
            self._disarm_calls()
            self.halt(f"controller not ready after arming: error_code={err} state={st}")
            return False
        if not self._check_safety("arm_robot"):
            self._disarm_calls()
            mm = self._safety_report.mismatches if self._safety_report else ["safety read failed"]
            self.halt("safety config mismatch: " + "; ".join(mm))
            return False
        with self._lock:
            self._state = CONNECTED
        log.info("armed -> CONNECTED")
        self._emit(event="armed")
        return True

    def connect(self):
        """soft_connect + arm_robot. Needs allow_motion. Returns True when CONNECTED."""
        self.soft_connect()
        return self.arm_robot()

    def disconnect(self):
        """Operator-requested drop of the TCP link. State -> DISCONNECTED, no auto-reconnect."""
        self._want_connected = False
        with self._lock:
            self._state = DISCONNECTED
            self._desired_q = None
            self._pending_error = None
            self._recover_at = None
        try:
            self._arm.disconnect()
        except Exception as e:  # noqa: BLE001
            self._capture_error("disconnect", e)
        log.info("disconnected")

    def enable(self, sync_q=None):
        """Enter ENABLED from CONNECTED. Pass the current measured pose as sync_q so the first target is the
        present pose. The speed cap ramps up from zero on every enable."""
        if not self._allow_motion:
            raise MotionNotAllowed("enable: the bridge was started without allow_motion; it only monitors")
        with self._lock:
            if self._state != CONNECTED:
                cur = self._state
            else:
                cur = None
                if sync_q is not None:
                    self._desired_q = [float(v) for v in sync_q]
                    self._desired_stamp = time.monotonic()
                    self._last_sent = [float(v) for v in sync_q]
                self._speed_eff, self._slew_t = 0.0, None
                self._state = ENABLED
        if cur is not None:
            self._capture_error("enable", RuntimeError(f"must be CONNECTED, got {cur}"))
            return False
        self._emit(event="enabled")
        return True

    def halt(self, reason: str):
        """Refuse to run and say why: stay where we are, never ENABLE, until recover() clears it."""
        with self._lock:
            self._halt_reason = reason
            self._last_error = f"HALTED: {reason}"
            self._desired_q = None
        log.error("HALTED: %s", reason)
        self._emit(event="halt", reason=reason)

    def disable(self):
        with self._lock:
            if self._state == ENABLED:
                self._state = CONNECTED

    def stop(self):
        """Leave ENABLED/CONNECTED and tell the controller to halt. State -> MONITORING."""
        with self._lock:
            self._state = MONITORING
            self._desired_q = None
        try:
            self._arm.set_state(4)
        except Exception as e:  # noqa: BLE001
            self._capture_error("stop", e)
        log.info("STOP -> MONITORING")

    def emergency_stop(self):
        """Software e-stop: the SDK's own emergency stop first, then halt and disable the servos (brakes).
        State -> MONITORING. Not a substitute for the hardware e-stop circuit."""
        with self._lock:
            self._state = MONITORING
            self._desired_q = None
        if hasattr(self._arm, "emergency_stop"):
            try:
                self._arm.emergency_stop()
            except Exception as e:  # noqa: BLE001
                self._capture_error("emergency_stop.sdk", e)
        self._disarm_calls()
        with self._lock:
            self._last_error = "Emergency stop triggered by operator"
        log.error("EMERGENCY STOP -> MONITORING")
        self._emit(event="emergency_stop")

    def recover(self):
        """Operator reset: clear error/warn codes after a stop or e-stop and lift a halt. Stays in
        MONITORING; call arm_robot() to re-arm."""
        with self._lock:
            cur = self._state
            self._halt_reason = None
            self._pending_error = None
            self._recover_at = None
        if self._recovery is not None:
            self._recovery.reset()
        if cur != MONITORING:
            self._capture_error("recover", RuntimeError(f"must be MONITORING, got {cur}"))
            return
        for name, fn in (("clean_error", lambda: self._arm.clean_error()),
                         ("clean_warn", lambda: self._arm.clean_warn()),
                         ("motion_enable", lambda: self._arm.motion_enable(True))):
            try:
                fn()
            except Exception as e:  # noqa: BLE001
                self._capture_error(f"recover.{name}", e)
        self.clear_error()
        log.info("recovered (errors cleared, still MONITORING)")

    def shutdown(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            self._arm.disconnect()
        except Exception as e:  # noqa: BLE001
            self._capture_error("shutdown", e)

    def publish_desired_q(self, q):
        """Non-blocking handoff from the caller's loop."""
        with self._lock:
            self._desired_q = [float(v) for v in q]
            self._desired_stamp = time.monotonic()

    # --- xarm SDK callbacks (fire on the SDK's internal thread) ---
    def _on_error_warn(self, data):
        with self._lock:
            self._arm_error_code = _field(data, "error_code", getattr(self._arm, "error_code", 0))
            self._arm_warn_code = _field(data, "warn_code", getattr(self._arm, "warn_code", 0))
            if self._arm_error_code != 0:
                self._last_error = f"arm error_code={self._arm_error_code}"

    def _on_state_changed(self, data):
        demoted = False
        with self._lock:
            self._arm_hw_state = _field(data, "state", getattr(self._arm, "state", 0))
            if self._arm_hw_state in (3, 4) and self._state in (CONNECTED, ENABLED):
                self._state = MONITORING
                self._desired_q = None
                self._last_error = f"arm state changed to {self._arm_hw_state} (stopped/offline)"
                code = self._arm_error_code or getattr(self._arm, "error_code", 0) or 0
                self._note_fault_locked(code)
                demoted, hw = True, self._arm_hw_state
        if demoted:
            self._emit(event="fault", code=code, hw_state=hw)

    def _on_connect_changed(self, data):
        with self._lock:
            connected = _field(data, "connected", getattr(self._arm, "connected", False))
            self._arm_connected = connected
            if not connected and self._state != DISCONNECTED:
                self._state = DISCONNECTED
                self._desired_q = None
                self._last_error = "arm connection lost"
                self._connect_attempt = 0
                self._next_connect_t = self._clock() + self._backoff[0]
        if not connected:
            self._emit(event="connection_lost")

    # --- recovery (bridge thread) ---
    def _note_fault_locked(self, code):
        if self._pending_error is None and self._recover_at is None and self._halt_reason is None:
            self._pending_error = (int(code), self._clock())

    def _tick_fault_detection(self):
        """Demote CONNECTED/ENABLED to MONITORING when the controller reports an error, and queue the fault
        for the recovery policy."""
        with self._lock:
            err, hw = self._arm_error_code, self._arm_hw_state
            active = self._state in (CONNECTED, ENABLED)
            if active and (err != 0 or hw in (3, 4)):
                self._state = MONITORING
                self._desired_q = None
                self._last_error = f"arm error_code={err} hw_state={hw}"
                self._note_fault_locked(err)
                demoted = True
            else:
                demoted = False
        if demoted:
            self._emit(event="fault", code=err, hw_state=hw)

    def _tick_recovery(self, now):
        if not self._auto_arm or self._recovery is None:
            return
        with self._lock:
            if self._halt_reason is not None or self._state != MONITORING:
                return
            pending, due = self._pending_error, self._recover_at
        if pending is not None and due is None:
            code, _ = pending
            if code == 0:   # a fault seen while the error code was still 0 (state-only): re-read now
                code = int(getattr(self._arm, "error_code", 0) or 0)
            d = self._recovery.decide(code, now)
            self._emit(event="recovery_decision", code=code, action=d.action, reason=d.reason,
                       error_class=d.error_class, cooldown_s=d.cooldown_s)
            with self._lock:
                self._pending_error = None
                if d.action == NEEDS_OPERATOR:
                    self._halt_reason = d.reason
                    self._last_error = f"HALTED: {d.reason}"
                else:
                    self._recover_at = (code, d.action, now + d.cooldown_s)
            if d.action == NEEDS_OPERATOR:
                try:
                    self._arm.motion_enable(False)   # brakes hold; nothing moves until an operator resets
                except Exception as e:  # noqa: BLE001
                    self._capture_error("needs_operator.motion_enable", e)
                self._emit(event="halt", reason=d.reason, code=code)
            return
        if due is not None and now >= due[2]:
            code, action, _ = due
            ok = self._auto_recover(code)
            with self._lock:
                self._recover_at = None
                if ok:
                    self._recoveries_total += 1
                    self._recovery_hint = "home" if action == RECOVER_HOME else "resync"
            if ok:
                self._recovery.record_recovery(now)
            else:
                reason = f"auto-recovery of C{code} failed"
                with self._lock:
                    self._halt_reason = reason
                    self._last_error = f"HALTED: {reason}"
                self._emit(event="halt", reason=reason, code=code)

    def _auto_recover(self, code):
        """clean -> enable -> servo mode -> ready, verify the safety config, re-arm and ENABLE at the measured pose."""
        self._emit(event="recovery_start", code=code)
        self._arming_calls()
        if not self._controller_ready():
            self._emit(event="recovery_failed", code=code, reason="controller not ready",
                       error_code=getattr(self._arm, "error_code", None), hw_state=getattr(self._arm, "state", None))
            return False
        if not self._check_safety("auto_recover"):
            self._disarm_calls()
            self._emit(event="recovery_failed", code=code, reason="safety config mismatch")
            return False
        try:
            c, q = self._arm.get_servo_angle(is_radian=True)
            real = list(q)[:N_JOINTS] if c == 0 and q is not None else None
        except Exception as e:  # noqa: BLE001
            self._capture_error("auto_recover.get_servo_angle", e)
            real = None
        with self._lock:
            self._state = CONNECTED
            self._arm_error_code = 0
            self._arm_hw_state = 0
        if real is not None:
            with self._lock:
                self._real_q = list(real)
                self._real_stamp = time.monotonic()
            self.enable(sync_q=real)
        self._emit(event="recovered", code=code, enabled=real is not None)
        return True

    def _tick_reconnect(self, now):
        if not (self._reconnect and self._want_connected):
            return
        if now < self._next_connect_t:
            return
        attempt = self._connect_attempt
        self._connect_attempt = attempt + 1
        self._next_connect_t = now + self._backoff[min(attempt, len(self._backoff) - 1)]
        self._emit(event="reconnect_attempt", attempt=attempt + 1)
        if self._try_connect():
            with self._lock:
                self._pending_error = None
                self._recover_at = None
            if self._auto_arm and self._halt_reason is None and self.arm_robot():
                try:
                    c, q = self._arm.get_servo_angle(is_radian=True)
                    if c == 0 and q is not None:
                        self.enable(sync_q=list(q)[:N_JOINTS])
                        with self._lock:
                            self._recovery_hint = "resync"
                except Exception as e:  # noqa: BLE001
                    self._capture_error("reconnect.get_servo_angle", e)

    # --- thread loop ---
    def _loop(self):
        while self._running:
            t0 = time.monotonic()
            state = self.state
            if state == DISCONNECTED:
                self._tick_reconnect(self._clock())
                time.sleep(self._dt)
                continue
            if state in (MONITORING, CONNECTED, ENABLED):
                try:
                    code, q = self._arm.get_servo_angle(is_radian=True)
                    if code == 0 and q is not None:
                        with self._lock:
                            self._real_q = list(q)[:N_JOINTS]
                            self._real_stamp = time.monotonic()
                except Exception as e:  # noqa: BLE001
                    self._capture_error("loop.get_servo_angle", e)
                try:
                    code_p, pos = self._arm.get_position(is_radian=True)
                    if code_p == 0 and pos is not None and len(pos) >= 3:
                        with self._lock:
                            self._arm_position_m = xarm_pos_to_metres(pos[:3])
                except Exception as e:  # noqa: BLE001
                    self._capture_error("loop.get_position", e)
                with self._lock:   # SDK property reads are cached in the SDK: no network round-trip
                    self._arm_error_code = getattr(self._arm, "error_code", 0)
                    self._arm_warn_code = getattr(self._arm, "warn_code", 0)
                    self._arm_hw_state = getattr(self._arm, "state", 0)
                    self._arm_mode = getattr(self._arm, "mode", 0)
                    self._arm_connected = getattr(self._arm, "connected", False)
                self._tick_fault_detection()
                self._tick_recovery(self._clock())
                state = self.state

            if state == ENABLED:
                with self._lock:
                    desired = None if self._desired_q is None else list(self._desired_q)
                    age = time.monotonic() - self._desired_stamp
                    last = None if self._last_sent is None else list(self._last_sent)
                self._slew_speed(t0)
                if desired is not None and age <= self._watchdog_s:
                    desired = self._inside_limits(desired)
                    target = desired
                    effective_step = self._max_joint_step * self._speed_eff
                    if last is None:
                        with self._lock:      # never stream an unclamped first packet: clamp from where the arm is
                            last = None if self._real_q is None else list(self._real_q)
                    if last is not None and self._max_joint_step > 0:
                        if effective_step <= 1e-9:
                            target = list(last)
                        else:
                            d = [a - b for a, b in zip(desired, last)]
                            m = max((abs(v) for v in d), default=0.0)
                            if m > effective_step:
                                k = effective_step / m
                                target = [b + v * k for b, v in zip(last, d)]
                    try:
                        self._arm.set_servo_angle_j(target, is_radian=True)
                        with self._lock:
                            self._last_sent = list(target)
                    except Exception as e:  # noqa: BLE001
                        self._capture_error("loop.set_servo_angle_j", e)
                # else: watchdog -> hold (stop streaming)

            dt = time.monotonic() - t0
            time.sleep(max(0.0, self._dt - dt))
