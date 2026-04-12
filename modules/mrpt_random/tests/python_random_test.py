#!/usr/bin/env python3
"""Smoke tests for mrpt.random Python bindings."""
import sys

try:
    from mrpt.random import CRandomGenerator, getRandomGenerator, Randomize
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.random bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.random import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CRandomGenerator")
rng = CRandomGenerator(42)

u = rng.drawUniform(0.0, 1.0)
check("uniform in [0,1]", 0.0 <= u <= 1.0, f"got {u}")

g = rng.drawGaussian1D(0.0, 1.0)
check("gaussian is float", isinstance(g, float))

arr = rng.drawGaussianArray(100, 0.0, 1.0)
check("array shape", arr.shape == (100,), f"got {arr.shape}")
check("array mean ~0", abs(float(arr.mean())) < 1.0)  # wide tolerance for 100 samples

print("global generator")
Randomize(0)
g2 = getRandomGenerator()
check("global generator exists", g2 is not None)
v = g2.drawUniform(0.0, 10.0)
check("global uniform in [0,10]", 0.0 <= v <= 10.0, f"got {v}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
