#!/usr/bin/env python3
"""
mrpt_math_example.py — geometry primitives with mrpt.math.

Demonstrates:
  - TPoint2D / TPoint3D: 2D and 3D points
  - TPose2D / TPose3D: 2D and 3D poses (position + orientation)
  - TSegment2D / TSegment3D: line segments
  - TLine2D / TLine3D: infinite lines
  - TPlane: 3D plane
  - TBoundingBox: axis-aligned bounding box
  - TTwist2D / TTwist3D: velocity twists
  - CPolygon: 2D polygon
  - CHistogram: 1D histogram
  - wrapToPi / wrapTo2Pi: angle normalisation
  - CMatrixDouble / CVectorDouble: dynamic matrices (bridge to numpy)
"""

import math, numpy as np
from mrpt.math import (
    TPoint2D, TPoint3D,
    TPose2D, TPose3D,
    TSegment2D, TSegment3D,
    TLine2D, TLine3D,
    TPlane,
    TBoundingBox,
    TTwist2D, TTwist3D,
    CPolygon,
    CHistogram,
    wrapToPi, wrapTo2Pi,
    CMatrixDouble, CVectorDouble,
)

# ---------------------------------------------------------------------------
# Points and poses
# ---------------------------------------------------------------------------
p2 = TPoint2D(1.0, 2.0)
p3 = TPoint3D(1.0, 2.0, 3.0)
print(f"TPoint2D: {p2}")
print(f"TPoint3D: {p3}")
assert p2.x == 1.0 and p2.y == 2.0
assert p3.z == 3.0

pose2 = TPose2D(1.0, 0.5, math.pi / 4)
pose3 = TPose3D(1.0, 2.0, 0.5, 0.1, 0.2, 0.3)
print(f"\nTPose2D: {pose2}")
print(f"TPose3D: {pose3}")

# ---------------------------------------------------------------------------
# Segment distance
# ---------------------------------------------------------------------------
seg = TSegment2D(TPoint2D(0, 0), TPoint2D(4, 0))
dist = seg.distance(TPoint2D(2, 3))
print(f"\nTSegment2D distance to (2,3): {dist:.4f}")   # should be 3
assert abs(dist - 3.0) < 1e-6

# ---------------------------------------------------------------------------
# Line from two points
# ---------------------------------------------------------------------------
line = TLine2D(TPoint2D(0, 0), TPoint2D(1, 1))
print(f"\nTLine2D: {line}")
d = line.distance(TPoint2D(1, 0))
print(f"  distance to (1,0): {d:.4f}")   # sqrt(2)/2 ≈ 0.7071
assert abs(d - math.sqrt(2) / 2) < 1e-6

# ---------------------------------------------------------------------------
# Plane
# ---------------------------------------------------------------------------
plane = TPlane(TPoint3D(0,0,0), TPoint3D(1,0,0), TPoint3D(0,1,0))  # XY plane
print(f"\nTPlane (XY plane): {plane}")
d3 = plane.distance(TPoint3D(0, 0, 5))
print(f"  distance to (0,0,5): {d3:.4f}")   # should be 5
assert abs(d3 - 5.0) < 1e-6

# ---------------------------------------------------------------------------
# BoundingBox
# ---------------------------------------------------------------------------
bb = TBoundingBox(TPoint3D(-1,-1,-1), TPoint3D(1,1,1))
print(f"\nTBoundingBox: {bb}")
ctr = bb.center()
print(f"  center: ({ctr.x},{ctr.y},{ctr.z})")
assert ctr.x == 0.0 and ctr.y == 0.0

# ---------------------------------------------------------------------------
# Twists
# ---------------------------------------------------------------------------
tw2 = TTwist2D(1.0, 0.0, 0.1)   # vx, vy, omega
tw3 = TTwist3D(1.0, 0.0, 0.0, 0.0, 0.0, 0.1)
print(f"\nTTwist2D: {tw2}")
print(f"TTwist3D: {tw3}")

# ---------------------------------------------------------------------------
# CPolygon
# ---------------------------------------------------------------------------
poly = CPolygon()
poly.AddVertex(0, 0)
poly.AddVertex(1, 0)
poly.AddVertex(1, 1)
poly.AddVertex(0, 1)
print(f"\nCPolygon: {poly.size()} vertices")
assert poly.size() == 4

# ---------------------------------------------------------------------------
# CHistogram
# ---------------------------------------------------------------------------
hist = CHistogram(0.0, 10.0, 10)   # [0,10], 10 bins
for v in [1.5, 2.5, 2.5, 7.0]:
    hist.add(v)
bins, counts = hist.getHistogramNormalized()
print(f"\nCHistogram ({hist.getBinCount()} bins): {counts}")
assert hist.getBinCount() == 10

# ---------------------------------------------------------------------------
# Angle wrapping
# ---------------------------------------------------------------------------
print(f"\nwrapToPi(3π/2)  = {wrapToPi(3*math.pi/2):.4f}  (expect ~-1.5708)")
print(f"wrapTo2Pi(-π/4) = {wrapTo2Pi(-math.pi/4):.4f} (expect ~4.7124)")
assert abs(wrapToPi(3*math.pi/2)  - (-math.pi/2)) < 1e-9
assert abs(wrapTo2Pi(-math.pi/4)  - (7*math.pi/4)) < 1e-9
print("  wrap checks ✓")

# ---------------------------------------------------------------------------
# CMatrixDouble — numpy bridge (minimal API)
# ---------------------------------------------------------------------------
m = CMatrixDouble(3, 3)
print(f"\nCMatrixDouble(3,3): {m}")
arr = np.array([[1.0, 2.0], [3.0, 4.0]])
m2 = CMatrixDouble(arr)     # construct from numpy
np_out = m2.as_numpy()
print(f"  CMatrixDouble from numpy, back to numpy: {np_out}")
np.testing.assert_allclose(np_out, arr)
print("  matrix numpy bridge ✓")
