#!/usr/bin/env python3
"""
Unit test for the mrpt.poses Python bindings.

Run via CTest (mrpt_add_python_binding_test) or directly:
  python3 modules/mrpt_poses/tests/python_poses_test.py
"""
import math
import sys

# ---------------------------------------------------------------------------
# Import
# ---------------------------------------------------------------------------
try:
    from mrpt.poses import (
        CPose2D, CPose3D,
        CPose3DPDFGaussian,
        SE_average2, SE_average3,
        CPoint2D, CPoint3D,
    )
except ImportError as e:
    msg = str(e)
    # Soft-skip only when the compiled extension simply isn't present
    # (e.g. bindings disabled at build time).  Any other ImportError —
    # including circular imports caused by a bad PYTHONPATH setup — is
    # a real failure and must surface as a test failure.
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.poses bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.poses import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = 0
FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}")
        PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else ""))
        FAIL += 1

# ---------------------------------------------------------------------------
# CPose2D
# ---------------------------------------------------------------------------
print("CPose2D")
p = CPose2D(1.0, 2.0, math.pi / 4)
check("x attr",      abs(p.x - 1.0) < 1e-9)
check("y attr",      abs(p.y - 2.0) < 1e-9)
check("phi attr",    abs(p.phi - math.pi / 4) < 1e-9)

p2 = CPose2D(0.0, 0.0, 0.0)
inv = p + p.inverse()
check("p + inv ≈ 0 (x)", abs(inv.x) < 1e-6)
check("p + inv ≈ 0 (y)", abs(inv.y) < 1e-6)

p.x = 5.0
check("x setter", abs(p.x - 5.0) < 1e-9)

check("norm",  p.norm() > 0)

# composition
a = CPose2D(1.0, 0.0, 0.0)
b = CPose2D(1.0, 0.0, 0.0)
c = a + b
check("compose x", abs(c.x - 2.0) < 1e-9)

# ---------------------------------------------------------------------------
# CPose3D
# ---------------------------------------------------------------------------
print("CPose3D")
p3 = CPose3D.FromXYZYawPitchRoll(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
check("x",  abs(p3.x - 1.0) < 1e-9)
check("y",  abs(p3.y - 2.0) < 1e-9)
check("z",  abs(p3.z - 3.0) < 1e-9)

p3.z = 99.0
check("z setter", abs(p3.z - 99.0) < 1e-9)

rot = np.array(p3.getRotationMatrix())
check("rotation matrix shape", rot.shape == (3, 3),
      f"got {rot.shape}")

# identity rotation → diagonal 1
p_id = CPose3D(0, 0, 0, 0, 0, 0)
r_id = np.array(p_id.getRotationMatrix())
check("identity rotation matrix",
      abs(float(r_id[0, 0]) - 1.0) < 1e-6 and
      abs(float(r_id[1, 1]) - 1.0) < 1e-6 and
      abs(float(r_id[2, 2]) - 1.0) < 1e-6)

pt = p_id.inverseComposePoint(1.0, 2.0, 3.0)
check("inverseComposePoint returns TPoint3D", hasattr(pt, 'x') and hasattr(pt, 'y') and hasattr(pt, 'z'), str(type(pt)))
check("inverseComposePoint at identity", abs(pt.x - 1.0) < 1e-6)

# ---------------------------------------------------------------------------
# CPoint2D / CPoint3D
# ---------------------------------------------------------------------------
print("CPoint2D / CPoint3D")
pt2 = CPoint2D(3.0, 4.0)
check("CPoint2D x", abs(pt2.x - 3.0) < 1e-9)
check("CPoint2D y", abs(pt2.y - 4.0) < 1e-9)

pt3 = CPoint3D(1.0, 2.0, 3.0)
check("CPoint3D z", abs(pt3.z - 3.0) < 1e-9)

# ---------------------------------------------------------------------------
# CPose3DPDFGaussian
# ---------------------------------------------------------------------------
print("CPose3DPDFGaussian")
mean = CPose3D(1.0, 2.0, 3.0, 0, 0, 0)
pdf = CPose3DPDFGaussian(mean)
check("PDF mean x", abs(pdf.mean.x - 1.0) < 1e-9)

pdf.mean = CPose3D(10.0, 0.0, 0.0, 0, 0, 0)
check("PDF mean setter", abs(pdf.mean.x - 10.0) < 1e-9)

sample = pdf.drawSingleSample()
check("drawSingleSample returns CPose3D",
      hasattr(sample, "x") and hasattr(sample, "y"))

# ---------------------------------------------------------------------------
# SE_average2 / SE_average3
# ---------------------------------------------------------------------------
print("SE_average2")
avg2 = SE_average2()
avg2.append(CPose2D(2.0, 0.0, 0.0))
avg2.append(CPose2D(4.0, 0.0, 0.0))
r2 = avg2.get_average()
check("SE_average2 mean x ≈ 3", abs(r2.x - 3.0) < 0.1,
      f"got x={r2.x}")

print("SE_average3")
avg3 = SE_average3()
avg3.append(CPose3D(10.0,  0.0, 0.0, 0, 0, 0))
avg3.append(CPose3D(10.2,  0.0, 0.0, 0, 0, 0))
avg3.append(CPose3D( 9.8,  0.0, 0.0, 0, 0, 0))
r3 = avg3.get_average()
check("SE_average3 mean x ≈ 10", abs(r3.x - 10.0) < 0.2,
      f"got x={r3.x}")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print(f"\nResults: {PASS} passed, {FAIL} failed")
if FAIL:
    sys.exit(1)
