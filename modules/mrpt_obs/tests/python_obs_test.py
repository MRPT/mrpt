#!/usr/bin/env python3
"""Smoke tests for mrpt.obs Python bindings."""
import sys, math

try:
    from mrpt.obs import (
        CObservation2DRangeScan, CObservationOdometry,
        CObservationIMU, CActionCollection,
        CActionRobotMovement2D, CSensoryFrame,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.obs bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.obs import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CObservation2DRangeScan")
scan = CObservation2DRangeScan()
scan.aperture = math.pi
scan.maxRange = 80.0
N = 100
ranges = np.linspace(1.0, 5.0, N).astype(np.float32)
scan.setRangesFromNumpy(ranges)
check("aperture",  abs(scan.aperture - math.pi) < 1e-9)
check("maxRange",  abs(scan.maxRange - 80.0) < 1e-9)
back = scan.getRangesAsNumpy()
check("ranges shape",  back.shape == (N,), f"got {back.shape}")
check("ranges values", abs(float(back[0]) - 1.0) < 1e-4 and abs(float(back[-1]) - 5.0) < 1e-4)

print("CObservationOdometry")
odo = CObservationOdometry()
check("odometry created", odo is not None)

print("CSensoryFrame")
sf = CSensoryFrame()
sf.push_back(scan)
check("SF size == 1", sf.size() == 1)

print("CActionCollection")
ac = CActionCollection()
check("action collection empty", ac.size() == 0)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
