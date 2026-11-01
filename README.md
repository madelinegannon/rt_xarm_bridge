# rt_xarm_bridge

A realtime servo bridge for UFACTORY xArm and 850 arms. Your control loop hands it a desired joint
vector whenever it has one; a dedicated thread streams that target to the controller at a fixed
cadence (125 Hz by default), clamps every step, and publishes the measured joints back. It ships
with a hardware-free `MockArm` that has the same call surface as the SDK subset it uses, so the
whole state machine and every safety gate are exercised by the test suite without a robot.

> **Read [SAFETY.md](SAFETY.md) before connecting a real arm.** This is not a safety-rated system.
> The gates below make accidental motion hard; they do not replace a risk assessment, a hardware
> e-stop circuit, guarding, or a correctly configured controller. Those are yours.

## What the gates do

The bridge can *monitor* a real arm out of the box. It can only *move* one after deliberate steps:

1. **Mock is the default.** Every entry point uses `MockArm` unless told `--arm xarm --arm-ip <controller-ip>`.
   There is no default IP anywhere.
2. **Monitor-only unless armed by flag.** Without `allow_motion=True` (CLI: `--allow-motion`) the bridge
   stops at the `MONITORING` state: it reads joints and telemetry, and `arm_robot()` / `enable()` raise
   `MotionNotAllowed`. The flag is never read from a config file.
3. **The controller's safety configuration is a precondition.** Before the arm is armed the bridge reads
   collision sensitivity, reduced mode, the TCP boundary and the speed limits from the controller and
   compares them with a `safety.yaml` you wrote (see `safety.example.yaml`; every value must be declared).
   Any difference refuses to arm and names the field. The bridge never writes controller settings for you.
4. **Speed starts low and ramps.** A config may start the speed cap at 0.2 at most; raising it is a runtime
   operator action. On every enable the effective speed ramps up from zero, and a cut applies at once.
   The per-tick step clamp and the stale-target watchdog have ceilings that config cannot exceed.
5. **No unattended autonomy.** Automatic recovery after a controller fault and automatic reconnection after
   a dropped link are off unless you opt in, and even then an E-stop is never cleared by software.

## Install

```bash
pip install -e .
```

Python 3.10 or newer. Dependencies: `xarm-python-sdk`, `pyyaml`.

## Quick start, no hardware

```bash
rt-xarm-bridge check --arm mock                      # the safety comparison table, nothing declared yet
rt-xarm-bridge monitor --arm mock --seconds 2        # the telemetry stream
python tests/run_all.py                              # the whole suite against MockArm
```

```python
import time
from rt_xarm_bridge import MockArm, RobotBridge

arm = MockArm(initial_q=[0.0] * 6)
bridge = RobotBridge(arm, allow_motion=True, safety=arm.safety_config())
bridge.connect()                       # soft_connect (MONITORING) + arm_robot (CONNECTED), safety verified
bridge.enable(sync_q=[0.0] * 6)        # ENABLED: the first target is the present pose, speed ramps from 0
bridge.publish_desired_q([0.1, 0.2, -0.3, 0.0, 0.1, 0.0])
time.sleep(0.5)
print(bridge.latest_real_q())          # (measured joints, age in seconds)
bridge.stop()                          # MONITORING again
bridge.shutdown()
```

## A real arm

1. Configure the controller in UFACTORY Studio: collision sensitivity, reduced mode, the TCP boundary
   and the reduced speed limits your risk assessment calls for. Wire and test the hardware e-stop.
2. Copy `safety.example.yaml` to `safety.yaml` and declare exactly those values.
3. Prove the controller matches, read-only:
   ```bash
   rt-xarm-bridge check --arm xarm --arm-ip <controller-ip> --safety safety.yaml
   ```
4. Watch it, read-only: `rt-xarm-bridge monitor --arm xarm --arm-ip <controller-ip>`.
5. In your application, build the bridge with `allow_motion=True` and `safety=SafetyConfig.load("safety.yaml")`,
   keep a hand on the e-stop, and raise `bridge.speed_scale` only once you have watched it move slowly.

The CLI also offers `stop` (halt, servos off), `park` (a slow position-mode move to a resting pose;
needs `--allow-motion` and a matching safety file) and `set-collision --level N` (an explicit write
of the controller's collision sensitivity, persisted with `save_conf`).

## API

- `make_arm(kind, ip="", initial_q=None)`: `"mock"` or `"xarm"` (lazy SDK import; an IP is required).
- `RobotBridge(arm, *, allow_motion=False, safety=None, initial_speed_scale=0.2, rate_hz=125.0,
  watchdog_s=0.5, max_joint_step=0.05, speed_slew_per_s=0.33, auto_arm=False, recovery=None,
  reconnect=False, on_event=None)`.
  - States: `DISCONNECTED -> MONITORING -> CONNECTED -> ENABLED`.
  - `soft_connect()` (to MONITORING), `arm_robot()` (to CONNECTED, verifies the safety config),
    `connect()` (both), `enable(sync_q)`, `publish_desired_q(q)`, `disable()`, `stop()`,
    `emergency_stop()`, `recover()`, `disconnect()`, `shutdown()`, `halt(reason)`.
  - Reads: `state`, `latest_real_q()`, `real_q_age()`, `joint_limits`, `controller_limits`,
    `speed_scale` (settable at runtime, 0..1), `speed_effective`, `safety_report`, `halt_reason`,
    `last_error`, `arm_error_code`, `arm_warn_code`, `arm_hw_state`, `arm_mode`, `arm_connected`,
    `arm_position_m`, `recoveries_total`, `pop_recovery_hint()`.
  - `on_event` receives dicts (`connected`, `armed`, `enabled`, `safety_check`, `safety_refused`,
    `halt`, `fault`, `recovery_*`, `reconnect_attempt`, `joint_limit_clamp`, `emergency_stop`).
- `SafetyConfig` / `SafetyConfig.load(path)` / `verify_safety_config(arm, cfg)` / `SafetyReport.table()`.
- `RecoveryPolicy` classifies controller error codes and decides between recover, recover-via-home and
  needs-operator; only used when `auto_arm=True`.

Units: joint angles are radians throughout; the SDK's cartesian positions are millimetres, and
`xarm_pos_to_metres` / `metres_to_xarm_pos` convert.

## Tests

```bash
python tests/run_all.py              # four suites, MockArm only
```

## License

Apache-2.0, Copyright 2024-2026 ATONATON, LLC. See LICENSE, and NOTICE for the trademark notice.
