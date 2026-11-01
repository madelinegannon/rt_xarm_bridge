# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""Shared bits for the hand-rolled suites: src on the path, a check() counter, small helpers."""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))

PASS = 0
FAIL = 0


def check(name, cond, info=""):
    global PASS, FAIL
    PASS, FAIL = (PASS + 1, FAIL) if cond else (PASS, FAIL + 1)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}  {info if not cond else ''}")


def close(a, b, atol=1e-6):
    a, b = list(a), list(b)
    return len(a) == len(b) and all(abs(float(x) - float(y)) <= atol for x, y in zip(a, b))


def wait_for(pred, timeout=3.0, dt=0.01):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(dt)
    return pred()


def finish(label):
    print(f"\n{label}: {PASS} passed, {FAIL} failed")
    sys.exit(1 if FAIL else 0)
