#!/usr/bin/env python3
"""Smoke tests for mrpt.math Python bindings."""
import sys, math

try:
    from mrpt.math import (
        TPoint2D, TPoint3D, TPose2D, TPose3D,
        CMatrixDouble, CVectorDouble,
        TSegment2D, TLine2D, TBoundingBox,
        TTwist2D, CHistogram, wrapToPi,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.math bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.math import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("TPoint2D / TPoint3D")
p2 = TPoint2D(3.0, 4.0)
check("x", abs(p2.x - 3.0) < 1e-12)
check("y", abs(p2.y - 4.0) < 1e-12)
check("repr", "3" in repr(p2) and "4" in repr(p2))

p3 = TPoint3D(1.0, 2.0, 3.0)
check("z", abs(p3.z - 3.0) < 1e-12)

print("TPose2D")
pose = TPose2D(1.0, 2.0, math.pi / 2)
check("phi", abs(pose.phi - math.pi / 2) < 1e-12)

print("CMatrixDouble")
m = CMatrixDouble(3, 3)
check("CMatrixDouble created", m is not None)
arr = m.as_numpy()
check("as_numpy shape", arr.shape == (3, 3), f"got {arr.shape}")

print("CVectorDouble")
v = CVectorDouble()
check("CVectorDouble created", v is not None)

print("TSegment2D")
seg = TSegment2D(TPoint2D(0, 0), TPoint2D(3, 4))
check("length", abs(seg.length() - 5.0) < 1e-9)

print("TTwist2D")
tw = TTwist2D(1.0, 0.0, 0.5)
check("vx", abs(tw.vx - 1.0) < 1e-12)
check("omega", abs(tw.omega - 0.5) < 1e-12)

print("wrapToPi")
check("wrapToPi(3pi/2)", abs(wrapToPi(3 * math.pi / 2) - (-math.pi / 2)) < 1e-9)

print("CHistogram")
h = CHistogram(0.0, 1.0, 10)
h.add(0.5)
xs, hits = h.getHistogram()
check("histogram has bins", len(xs) == 10)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
