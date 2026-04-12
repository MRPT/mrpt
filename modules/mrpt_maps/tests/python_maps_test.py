#!/usr/bin/env python3
"""Smoke tests for mrpt.maps Python bindings."""
import sys

try:
    from mrpt.maps import CSimplePointsMap, COccupancyGridMap2D
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.maps bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.maps import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CSimplePointsMap")
m = CSimplePointsMap()
check("empty size", m.size() == 0)

pts = np.array([[1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0],
                [7.0, 8.0, 9.0]], dtype=np.float32)
m.setPointsFromNumpy(pts)
check("size after insert", m.size() == 3)

out = m.getPointsAsNumpy()
check("getPointsAsNumpy shape", out.shape == (3, 3), f"got {out.shape}")
check("first point x", abs(float(out[0, 0]) - 1.0) < 1e-5)

m.insertPoint(10.0, 11.0, 12.0)
check("insertPoint", m.size() == 4)

m.clear()
check("clear", m.size() == 0)

print("COccupancyGridMap2D")
grid = COccupancyGridMap2D(-5.0, 5.0, -5.0, 5.0, 0.05)
check("grid non-empty", grid.getSizeX() > 0 and grid.getSizeY() > 0)
check("resolution", abs(grid.getResolution() - 0.05) < 1e-6)

arr = grid.getAsNumpy()
check("getAsNumpy 2D", arr.ndim == 2, f"got ndim={arr.ndim}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
