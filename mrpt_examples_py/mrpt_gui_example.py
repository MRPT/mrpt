#!/usr/bin/env python3
"""
Opens a 3D window with mrpt.gui and populates its scene.

Demonstrates:
  - CDisplayWindowGUI: create, set its 3D scene, camera control
  - Adding objects (CGridPlaneXY, CPointCloudColoured, CAxis) to the scene
  - spinOnce(): processing window events and repainting from your own loop

Pass --interactive to keep the window open until it is closed; otherwise it
shows a few frames and quits.

NOTE: This example requires a display (X11/Wayland). Without one, it exits
gracefully without rendering.
"""

import argparse
import math
import os
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true",
                    help="Keep the window open until it is closed")
args = parser.parse_args()

if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    print("No display: skipping this example.")
    sys.exit(0)

from mrpt.gui import CDisplayWindowGUI
from mrpt.viz import Scene, CGridPlaneXY, CPointCloudColoured, CAxis

# ---------------------------------------------------------------------------
# Create window
# ---------------------------------------------------------------------------
win = CDisplayWindowGUI("mrpt_gui_example: 3D viewer", 800, 600)
print(f"Window created: {win}")
print(f"  isOpen = {win.isOpen()}")

# Camera setup (the mouse also moves it: drag to rotate, wheel to zoom)
cam = win.camera()
cam.setElevationDegrees(30)
cam.setAzimuthDegrees(45)
cam.setZoomDistance(10)
cam.setCameraPointing(0, 0, 0)

# ---------------------------------------------------------------------------
# Populate the scene
# ---------------------------------------------------------------------------
scene = Scene()

# Floor grid
scene.insert(CGridPlaneXY())

# Coordinate axes
axes = CAxis()
axes.setAxisLimits(-3, -3, -3, 3, 3, 3)
scene.insert(axes)

# A small coloured point cloud
pts = CPointCloudColoured()
for i in range(100):
    angle = 2 * math.pi * i / 100
    r = 2.0
    pts.push_back(
        r * math.cos(angle),
        r * math.sin(angle),
        math.sin(2 * angle),          # z varies
        abs(math.cos(angle)),         # R
        abs(math.sin(angle)),         # G
        0.5,                          # B
    )
pts.setPointSize(4.0)
scene.insert(pts)

win.background_scene = scene
print("Scene populated: grid + axes + 100-point colour cloud")
print(f"  Camera elevation={cam.elevation_deg:.0f} deg  "
      f"azimuth={cam.azimuth_deg:.0f} deg  zoom={cam.zoom:.1f}")

# ---------------------------------------------------------------------------
# GUI loop
# ---------------------------------------------------------------------------
frames = 0
while win.spinOnce():
    frames += 1
    if not args.interactive and frames >= 20:
        break
    time.sleep(0.02)
print(f"Rendered {frames} frames.")
