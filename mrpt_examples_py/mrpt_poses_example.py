#!/usr/bin/env python3
"""
mrpt_poses_example.py — SE(2) and SE(3) poses with mrpt.poses.

Demonstrates:
  - CPose2D: 2D pose composition, inverse, norm
  - CPose3D: 3D pose, static builders, rotation matrix, inverseComposePoint
  - CPose3DPDFGaussian: Gaussian PDF over SE(3)
  - SE_average2 / SE_average3: pose averaging on the Lie group
  - CPoint2D / CPoint3D: bare point types
"""

import math
import numpy as np
from mrpt.poses import (
    CPose2D, CPose3D,
    CPose3DPDFGaussian,
    SE_average2, SE_average3,
    CPoint2D, CPoint3D,
)

# ---------------------------------------------------------------------------
# CPose2D — 2D pose arithmetic
# ---------------------------------------------------------------------------
print("── CPose2D ─────────────────────────────────")
p1 = CPose2D(1.0, 2.0, math.radians(45.0))
print(f"p1 = {p1}")

p1.x += 0.5
p1.phi = math.radians(90.0)
print(f"p1 modified: {p1}  norm={p1.norm():.4f}")

p2 = CPose2D(2.0, 0.0, 0.0)
p3 = p1 + p2
print(f"p3 = p1 + p2: {p3}")

p3_inv = p3.inverse()
print(f"p3 inverse:   {p3_inv}")

# round-trip: p + (-p) ≈ identity
identity = p3 + p3_inv
print(f"p3 + p3_inv ≈ 0: x={identity.x:.6f} y={identity.y:.6f}")
assert abs(identity.x) < 1e-6 and abs(identity.y) < 1e-6
print("  composition round-trip ✓")

# ---------------------------------------------------------------------------
# CPose3D — 3D pose
# ---------------------------------------------------------------------------
print("\n── CPose3D ─────────────────────────────────")
p3d = CPose3D.FromXYZYawPitchRoll(1.0, 2.0, 3.0, math.radians(30), 0, 0)
print(f"p3d = {p3d}")
print(f"  z = {p3d.z:.4f}")

p3d.z = 5.0
print(f"  z after set: {p3d.z:.4f}")

rot = p3d.getRotationMatrix()
print(f"  rotation matrix (3x3 numpy):\n{rot}")
assert rot.shape == (3, 3)

# inverseComposePoint: transform a global point to robot frame
global_pt = p3d.inverseComposePoint(1.0, 2.0, 5.0)
print(f"  inverseComposePoint((1,2,5)): {global_pt}")

# ---------------------------------------------------------------------------
# CPoint2D / CPoint3D
# ---------------------------------------------------------------------------
pt2 = CPoint2D(3.0, 4.0)
pt3 = CPoint3D(1.0, 2.0, 3.0)
print(f"\nCPoint2D: {pt2}")
print(f"CPoint3D: {pt3}")

# ---------------------------------------------------------------------------
# CPose3DPDFGaussian — Gaussian distribution over SE(3)
# ---------------------------------------------------------------------------
print("\n── CPose3DPDFGaussian ──────────────────────")
mean_pose = CPose3D(0, 0, 0, 0, 0, 0)
pdf = CPose3DPDFGaussian(mean_pose)
print(f"PDF mean: {pdf.mean}")

pdf.mean = CPose3D(10, 0, 0, 0, 0, 0)
print(f"PDF mean after set: {pdf.mean}")

sample = pdf.drawSingleSample()
print(f"Random sample: {sample}")

# ---------------------------------------------------------------------------
# SE_average3 — Lie-group mean of SE(3) poses
# ---------------------------------------------------------------------------
print("\n── SE_average3 ─────────────────────────────")
avg = SE_average3()
avg.append(CPose3D(10.0,  0.0, 0.0, 0, 0, 0))
avg.append(CPose3D(10.1,  0.1, 0.0, 0, 0, 0))
avg.append(CPose3D( 9.9, -0.1, 0.0, 0, 0, 0))
result = avg.get_average()
print(f"Average of 3 poses near (10,0,0): {result}")
assert abs(result.x - 10.0) < 0.2
print("  average check ✓")

# SE_average2 — same for SE(2)
avg2 = SE_average2()
avg2.append(CPose2D(1.0, 0.0, 0.0))
avg2.append(CPose2D(1.2, 0.0, 0.0))
result2 = avg2.get_average()
print(f"\nAverage of 2D poses: {result2}")
