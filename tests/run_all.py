# SPDX-License-Identifier: Apache-2.0
# Copyright 2024-2026 ATONATON, LLC
"""Runs every suite as a subprocess and aggregates. MockArm only: no hardware is ever touched.

  .venv/bin/python tests/run_all.py
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SUITES = ["test_safety.py", "test_gates.py", "test_robot.py", "test_recovery.py"]

results = []
for s in SUITES:
    print(f"\n========== {s} ==========", flush=True)
    rc = subprocess.call([sys.executable, os.path.join(HERE, s)])
    results.append((s, rc))

print("\n========== SUMMARY ==========")
allok = True
for s, rc in results:
    allok = allok and rc == 0
    print(f"  {'PASS' if rc == 0 else 'FAIL'}  {s}")
sys.exit(0 if allok else 1)
