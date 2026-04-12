#!/usr/bin/env python3
"""
mrpt_maps_example.py — metric map types with mrpt.maps.

Demonstrates:
  - CSimplePointsMap: point cloud creation, numpy import/export
  - COccupancyGridMap2D: create, set/get cells, numpy export
"""

import numpy as np
import tempfile, os
from mrpt.maps import CSimplePointsMap, COccupancyGridMap2D

# ---------------------------------------------------------------------------
# CSimplePointsMap — XYZ point cloud
# ---------------------------------------------------------------------------
pts = CSimplePointsMap()

# Insert individual points
for i in range(5):
    pts.insertPoint(float(i), float(i) * 0.1, 0.0)

print(f"CSimplePointsMap: {len(pts)} points")
x, y, z = pts.getPoint(2)
print(f"  point[2]: ({x:.2f}, {y:.2f}, {z:.2f})")

# NumPy round-trip
cloud = np.array([[1.0, 2.0, 3.0],
                  [4.0, 5.0, 6.0],
                  [7.0, 8.0, 9.0]], dtype=np.float32)
pts2 = CSimplePointsMap()
pts2.setPointsFromNumpy(cloud)
arr = pts2.getPointsAsNumpy()
print(f"\nNumPy round-trip: {arr.shape}")
np.testing.assert_allclose(arr, cloud, atol=1e-5)
print("  numpy round-trip ✓")

# Save/load text
with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as f:
    fname = f.name
try:
    pts2.save3D_to_text_file(fname)
    pts3 = CSimplePointsMap()
    pts3.load3D_from_text_file(fname)
    assert pts3.size() == 3
    print(f"  save/load text ✓ ({pts3.size()} points)")
finally:
    os.unlink(fname)

# ---------------------------------------------------------------------------
# COccupancyGridMap2D — probabilistic 2D occupancy grid
# ---------------------------------------------------------------------------
grid = COccupancyGridMap2D(-5.0, 5.0, -5.0, 5.0, 0.1)  # 10x10 m, 0.1 m/cell
print(f"\nCOccupancyGridMap2D: {grid.getSizeX()}x{grid.getSizeY()} cells, "
      f"res={grid.getResolution()} m")

# Mark an obstacle cell (low probability = occupied)
grid.setCell(50, 50, 0.1)
grid.setPos(1.0, 1.0, 0.1)

print(f"  cell(50,50) occupancy = {grid.getCell(50,50):.2f}")
print(f"  pos(1,1) occupancy    = {grid.getPos(1.0,1.0):.2f}")

# Index ↔ metric conversion
ix = grid.x2idx(1.0)
iy = grid.y2idx(1.0)
print(f"  pos(1,1) → cell index ({ix},{iy})")
print(f"  cell({ix},{iy}) → pos ({grid.idx2x(ix):.2f},{grid.idx2y(iy):.2f})")

# NumPy export
g_np = grid.getAsNumpy()
print(f"\nGrid as numpy: shape={g_np.shape}, dtype={g_np.dtype}")
assert g_np.shape == (grid.getSizeY(), grid.getSizeX())
print("  numpy export ✓")
