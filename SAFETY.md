# Safety

This software streams joint targets to an industrial robot arm. It is **not a safety-rated system**
and has not been certified by anyone. The responsibility for a safe cell is entirely yours.

## What you own

- **A risk assessment** for your cell, before anything moves. For arms that share space with people,
  ISO 10218-2 and ISO/TS 15066 are the references; your jurisdiction may add requirements.
- **A hardware emergency-stop circuit** wired to the controller's safety inputs, tested before each
  session, within reach of whoever is near the arm. `emergency_stop()` in this package tells the
  controller to stop over the network; it is a convenience, not a substitute, and it does nothing
  if the network, the host or this process has failed.
- **Guarding and distance.** Decide what is inside the arm's reach and keep people out of it unless
  the controller's collision detection, reduced speed and boundary make contact tolerable for your
  risk assessment.
- **The controller's own safety configuration.** Collision sensitivity, reduced mode, the TCP boundary,
  the reduced speed limits and the joint ranges live on the controller and are enforced there, with
  or without this software. Set them in UFACTORY Studio, save them, power-cycle, and confirm they
  persisted with `rt-xarm-bridge check`.
- **Supervision.** Nothing here is designed to run without someone able to stop it.

## What the gates protect against, and what they do not

| Gate | Protects against | Does not protect against |
|---|---|---|
| Mock by default, no default IP | Running the real arm by mistake | Someone passing the real IP deliberately |
| `allow_motion` flag, never from config | Arming through a config file or a stale script | A caller that passes the flag |
| Safety config verified before arming | Running against a controller someone reconfigured | A wrong declaration in `safety.yaml` |
| Speed cap at 0.2 from config, ramp from zero | A fast first move, a lurch when the cap is raised | Fast motion once an operator raises the cap |
| Per-tick step clamp (ceiling 0.05 rad) | A single large jump between targets | Sustained motion at the clamp rate |
| Joint-limit net 2 degrees inside the limits | Controller joint-limit faults | Cartesian collisions |
| Watchdog (ceiling 1 s) | Streaming a dead caller's last target | The arm holding a bad pose: it stops, it does not retreat |
| Auto-recovery and reconnect off by default | Unattended restarts after faults | A supervised run that opts in and then walks away |
| E-stop codes never auto-cleared | Software overriding a physical stop | A controller whose E-stop input is not wired |

The bridge never writes safety settings to the controller on its own. `set-collision` is an explicit
command that does one thing and tells you to power-cycle and verify.

## Operating procedure

1. Before each session: e-stop test, cell clear, `rt-xarm-bridge check --arm xarm --arm-ip <ip> --safety safety.yaml`.
2. Start your application with the mock arm first. Then with the real arm and *without* `allow_motion`:
   the bridge should reach `MONITORING` and report sensible joints.
3. Only then start with `allow_motion`. Keep the speed cap at its starting value for the first moves.
4. Raise `speed_scale` in steps, watching the arm. Cuts apply at once; use them.
5. After any fault, read `halt_reason` and the controller's error code before calling `recover()`.
   An E-stop must be released at the hardware before anything else.
6. Stop the application with `stop()` or `shutdown()`; never leave an `ENABLED` bridge running unattended.

## Known limitations

- The watchdog stops streaming when targets go stale; the controller then holds the last commanded
  pose. If that pose is the problem, you need the e-stop.
- The joint-limit net uses the controller's reduced joint limits intersected with the xArm 850 URDF
  table. Other arm models need their own table.
- `MockArm` proves the logic, not the hardware: latency, firmware behaviour and the SDK's own
  threading are only exercised on a real controller.
- The package has been used with xArm SDK 1.13 to 1.18 and 850-series firmware; other combinations
  may report safety settings differently, and an unreported setting is treated as a mismatch.
