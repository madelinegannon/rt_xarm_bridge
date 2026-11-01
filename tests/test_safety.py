# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""SafetyConfig loading and validation, the comparison rules, the controller read, the explicit
operator actions (set-collision, park) and the CLI, all against MockArm."""
import json
import os
import subprocess
import sys
import tempfile

import _common as T
from _common import check

import yaml
from rt_xarm_bridge import (MockArm, SafetyConfig, SafetyError, compare, park, read_controller_safety,
                            set_collision_sensitivity, verify_safety_config)
from rt_xarm_bridge.safety import FIELDS

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
ENV = dict(os.environ, PYTHONPATH=SRC)
COMPLETE = {"collision_sensitivity": 1, "reduced_mode": True, "fence_mode": True,
            "tcp_boundary_mm": [700, -700, 700, -700, 1200, 0], "max_tcp_speed_mm_s": 500, "max_joint_speed_rad_s": 1.5}


def write_yaml(d):
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump(d, f)
    f.close()
    return f.name


print("== SafetyConfig: load / undeclared / validate ==")
cfg = SafetyConfig.load(write_yaml(COMPLETE))
check("complete file loads", cfg.undeclared() == [], str(cfg.undeclared()))
check("boundary floats", cfg.tcp_boundary_mm == [700.0, -700.0, 700.0, -700.0, 1200.0, 0.0], str(cfg.tcp_boundary_mm))
partial = SafetyConfig.load(write_yaml({"collision_sensitivity": 2}))
check("partial file: the rest undeclared", set(partial.undeclared()) == set(FIELDS) - {"collision_sensitivity"})
example = SafetyConfig.load(os.path.join(os.path.dirname(SRC), "safety.example.yaml"))
check("shipped example declares nothing", example.undeclared() == list(FIELDS), str(example.undeclared()))
try:
    partial.require_complete()
    check("require_complete raises", False)
except SafetyError as e:
    check("require_complete names the fields", "reduced_mode" in str(e) and "fence_mode" in str(e), str(e))
for bad, why in (({"bogus": 1}, "unknown key"), ({"collision_sensitivity": 9}, "range"),
                 ({"collision_sensitivity": True}, "bool is not int"), ({"reduced_mode": "yes"}, "not a bool"),
                 ({"tcp_boundary_mm": [1, 2, 3]}, "six numbers"), ({"tcp_boundary_mm": [0, 1, 0, 0, 0, 0]}, "max >= min"),
                 ({"max_tcp_speed_mm_s": -1}, "positive")):
    try:
        SafetyConfig.load(write_yaml(bad))
        check(f"rejects {why}", False)
    except SafetyError:
        check(f"rejects {why}", True)
try:
    SafetyConfig.load("/nonexistent/safety.yaml")
    check("missing file raises SafetyError", False)
except SafetyError:
    check("missing file raises SafetyError", True)

print("== compare: rules and messages ==")
declared = SafetyConfig(**COMPLETE)
declared.validate()
ctrl_ok = {"collision_sensitivity": 1, "reduced_mode": True, "fence_mode": True,
           "tcp_boundary_mm": [600.0, -600.0, 700.0, -700.0, 1100.0, 50.0], "max_tcp_speed_mm_s": 400.0,
           "max_joint_speed_rad_s": 1.5, "version": "x", "error_code": 0, "state": 0}
r = compare(declared, ctrl_ok)
check("tighter controller passes", r.ok, str(r.mismatches))
check("table says OK", "SAFETY OK" in r.table(), r.table())
r = compare(declared, dict(ctrl_ok, collision_sensitivity=3))
check("sensitivity mismatch named", not r.ok and r.mismatches[0].startswith("collision_sensitivity:"), str(r.mismatches))
r = compare(declared, dict(ctrl_ok, reduced_mode=False))
check("mode mismatch named", any(m.startswith("reduced_mode:") for m in r.mismatches), str(r.mismatches))
r = compare(declared, dict(ctrl_ok, max_tcp_speed_mm_s=501.0))
check("speed above declared refused", any("max_tcp_speed_mm_s" in m for m in r.mismatches), str(r.mismatches))
r = compare(declared, dict(ctrl_ok, max_tcp_speed_mm_s=500.0))
check("speed equal to declared allowed", r.ok, str(r.mismatches))
r = compare(declared, dict(ctrl_ok, tcp_boundary_mm=[800.0, -700.0, 700.0, -700.0, 1200.0, -10.0]))
check("box outside on x_max and z_min", any("x_max" in m and "z_min" in m for m in r.mismatches), str(r.mismatches))
r = compare(declared, dict(ctrl_ok, fence_mode=None))
check("unreported field is a mismatch", any("did not report" in m for m in r.mismatches), str(r.mismatches))
r = compare(SafetyConfig(), ctrl_ok)
check("undeclared config: one mismatch per field", len(r.mismatches) == len(FIELDS) and all("not declared" in m for m in r.mismatches))
check("table says MISMATCH", "SAFETY MISMATCH" in r.table())

print("== read_controller_safety: MockArm and degraded stubs ==")
arm = MockArm(collision_sensitivity=2, reduced_mode=True, fence_mode=True, max_tcp_speed_mm_s=300.0)
got = read_controller_safety(arm)
check("mock values read", got["collision_sensitivity"] == 2 and got["reduced_mode"] and got["fence_mode"]
      and got["max_tcp_speed_mm_s"] == 300.0 and got["version"] == "mock-0.1", str(got))
check("verify matches its own config", verify_safety_config(arm, arm.safety_config()).ok)


class PropsOnly:
    collision_sensitivity = 4
    is_reduced_mode = True
    is_fence_mode = False
    reduced_tcp_boundary = [1, 2, 3, 4, 5, 6]
    error_code = 0
    state = 0

    def get_version(self):
        return 0, "v"


got = read_controller_safety(PropsOnly())
check("property fallbacks used", got["reduced_mode"] is True and got["fence_mode"] is False
      and got["tcp_boundary_mm"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] and got["max_tcp_speed_mm_s"] is None, str(got))


class Nothing:
    pass


got = read_controller_safety(Nothing())
check("bare object: all None", all(got[f] is None for f in FIELDS), str(got))

print("== set_collision_sensitivity (explicit) ==")
arm = MockArm(collision_sensitivity=3)
out = set_collision_sensitivity(arm, 1, settle_s=0.0)
check("set and saved", out["ok"] and out["before"] == 3 and out["after"] == 1 and arm.save_conf_calls == 1, str(out))
out = set_collision_sensitivity(arm, 1, settle_s=0.0)
check("already right: still save_conf, no set", out["ok"] and arm.set_sensitivity_calls == 1 and arm.save_conf_calls == 2, str(out))

print("== park (position mode) ==")
arm = MockArm(initial_q=[0.3] * 6)
arm.connect()
ok = park(arm, [0.0, 0.5, -0.5, 0.0, 0.2, 0.0], speed_rad_s=0.3, timeout_s=5.0)
check("park returns True", ok)
check("one position move at the pose", len(arm.position_cmds) == 1 and T.close(arm.position_cmds[0][1], [0.0, 0.5, -0.5, 0.0, 0.2, 0.0]), str(arm.position_cmds))
check("stopped after the move", arm.state == 4 and arm.mode == 0, f"state={arm.state} mode={arm.mode}")

print("== CLI ==")
def cli(*args):
    p = subprocess.run([sys.executable, "-m", "rt_xarm_bridge", *args], env=ENV, capture_output=True, text=True, timeout=60)
    return p.returncode, p.stdout + p.stderr

rc, out = cli("check", "--arm", "mock")
check("check without a safety file fails and says so", rc == 1 and "not declared" in out and "nothing is verified" in out, out[-300:])
mock_cfg = {k: getattr(MockArm().safety_config(), k) for k in FIELDS}
rc, out = cli("check", "--arm", "mock", "--safety", write_yaml(mock_cfg))
check("check with a matching file passes", rc == 0 and "SAFETY OK" in out, out[-300:])
rc, out = cli("check", "--arm", "mock", "--safety", write_yaml(dict(mock_cfg, collision_sensitivity=1)), "--json")
j = json.loads(out)
check("check --json reports the mismatch", rc == 1 and not j["ok"] and "collision_sensitivity" in j["mismatches"][0], out[-300:])
rc, out = cli("check", "--arm", "xarm")
check("real arm without --arm-ip refused", rc == 2 and "arm-ip" in out, out[-200:])
rc, out = cli("park", "--arm", "mock", "--safety", write_yaml(mock_cfg), "--pose", "0", "0", "0", "0", "0", "0")
check("park without --allow-motion refused", rc == 2 and "allow-motion" in out, out[-200:])
rc, out = cli("park", "--arm", "mock", "--safety", write_yaml(dict(mock_cfg, fence_mode=True)), "--allow-motion",
              "--pose", "0", "0", "0", "0", "0", "0")
check("park with a mismatching file refused", rc == 1 and "refusing to move" in out, out[-300:])
rc, out = cli("park", "--arm", "mock", "--safety", write_yaml(mock_cfg), "--allow-motion", "--pose", "0", "10", "-10", "0", "0", "0", "--degrees")
check("park with the gates satisfied runs", rc == 0 and '"parked": true' in out, out[-300:])
rc, out = cli("stop", "--arm", "mock")
check("stop runs", rc == 0 and '"stopped": true' in out, out[-200:])
rc, out = cli("set-collision", "--arm", "mock", "--level", "1")
check("set-collision runs", rc == 0 and '"ok": true' in out, out[-200:])
rc, out = cli("monitor", "--arm", "mock", "--hz", "20", "--seconds", "0.2")
check("monitor prints json lines", rc == 0 and out.count('"q_deg"') >= 2, out[-300:])

T.finish("test_safety")
