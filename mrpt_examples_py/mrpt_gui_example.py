#!/usr/bin/env python3
"""
mrpt_gui_example.py — 3D window with mrpt.gui.

Demonstrates:
  - CDisplayWindow3D: create, scene access, camera control
  - Adding objects (CGridPlaneXY, CPointCloudColoured) to the scene
  - Non-interactive usage: open, add objects, repaint, close immediately.

NOTE: This example requires a display (X11/Wayland).  In headless / CI
environments it will exit gracefully without rendering.
"""

import sys
try:
    from mrpt.gui import CDisplayWindow3D
    from mrpt.viz import (
        Scene,
        CGridPlaneXY,
        CPointCloudColoured,
        CAxis,
    )
    from mrpt.math import TPoint3D
except ImportError as e:
    print(f"Import failed (headless?): {e}")
    sys.exit(0)

# ---------------------------------------------------------------------------
# Create window
# ---------------------------------------------------------------------------
win = CDisplayWindow3D("mrpt_gui_example — 3D viewer", 800, 600)
print(f"Window created: {win}")
print(f"  isOpen = {win.isOpen()}")

# Camera setup
win.setCameraElevationDeg(30)
win.setCameraAzimuthDeg(45)
win.setCameraZoom(10)
win.setCameraPointingToPoint(0, 0, 0)

# ---------------------------------------------------------------------------
# Populate the scene
# ---------------------------------------------------------------------------
scene = win.get3DSceneAndLock()

# Floor grid
grid = CGridPlaneXY.Create()
scene.insert(grid)

# Coordinate axes
axes = CAxis.Create()
axes.setAxisLimits(-3, -3, -3, 3, 3, 3)
scene.insert(axes)

# A small coloured point cloud
pts = CPointCloudColoured.Create()
import math
for i in range(100):
    angle = 2 * math.pi * i / 100
    r = 2.0
    pts.push_back_point(
        r * math.cos(angle),
        r * math.sin(angle),
        math.sin(2 * angle),          # z varies
        abs(math.cos(angle)),         # R
        abs(math.sin(angle)),         # G
        0.5,                          # B
    )
pts.setPointSize(4.0)
scene.insert(pts)

win.unlockAccess3DScene()
win.forceRepaint()

print("Scene populated: grid + axes + 100-point colour cloud")
print(f"  Camera elevation={win.getCameraElevationDeg():.0f}°  "
      f"azimuth={win.getCameraAzimuthDeg():.0f}°  "
      f"zoom={win.getCameraZoom():.1f}")
print("  (window would stay open interactively; closing immediately in this demo)")
