#!/usr/bin/env python3
"""
mrpt_viz_example.py — 3D scene building with mrpt.viz.

Demonstrates:
  - Scene + Viewport: top-level scene graph
  - CGridPlaneXY, CAxis: floor grid and coordinate axes
  - CBox, CSphere, CCylinder: primitive shapes
  - CPointCloud / CPointCloudColoured: point clouds from numpy
  - CSetOfLines / CSimpleLine: line primitives
  - CText / CText3D: text labels
  - CEllipsoid3D: uncertainty ellipsoid
  - CArrow: 3D arrows
  - stock_objects: pre-built sensor / robot models
  - CCamera: scene camera control

NOTE: This script builds and inspects the scene without opening a display
window (use mrpt_gui_example.py for interactive 3D).
"""

import math, numpy as np
from mrpt.viz import (
    Scene, Viewport,
    CGridPlaneXY, CAxis,
    CBox, CSphere, CCylinder, CArrow,
    CText, CText3D,
    CSetOfLines, CSimpleLine,
    CEllipsoid3D,
    CPointCloud, CPointCloudColoured,
    stock_objects,
)
from mrpt.math import TPoint3D, TSegment3D, CMatrixDouble33

# ---------------------------------------------------------------------------
# Scene and viewport
# ---------------------------------------------------------------------------
scene = Scene()
vp = scene.getViewport("main")
print(f"Scene created, viewport: {vp.name}")

# Camera setup
cam = vp.getCamera()
cam.setAzimuthDegrees(45)
cam.setElevationDegrees(30)
cam.setZoomDistance(10)

# ---------------------------------------------------------------------------
# Floor grid and axes
# ---------------------------------------------------------------------------
grid = CGridPlaneXY(-10, 10, -10, 10, 0, 1)
scene.insert(grid)

axes = CAxis(-3, -3, -3, 3, 3, 3, 1.0, 2.0, True)
scene.insert(axes)

# ---------------------------------------------------------------------------
# Primitive shapes
# ---------------------------------------------------------------------------
box = CBox(TPoint3D(-0.5, -0.5, 0), TPoint3D(0.5, 0.5, 1.0), False)
box.setColor(200, 100, 0)
scene.insert(box)

sphere = CSphere(0.5, 20)
sphere.pose.x = 2.0
sphere.setColor(0, 180, 255)
scene.insert(sphere)

cyl = CCylinder(0.2, 0.0, 1.5, 20)   # cone shape
cyl.pose.x = -2.0
cyl.setColor(50, 200, 50)
scene.insert(cyl)

# ---------------------------------------------------------------------------
# Arrow
# ---------------------------------------------------------------------------
arrow = CArrow()
arrow.setArrowEnds(0, 0, 1.5, 0, 0, 2.5)
arrow.setColor(255, 0, 0)
scene.insert(arrow)

# ---------------------------------------------------------------------------
# Point clouds
# ---------------------------------------------------------------------------
# Plain XYZ cloud
pc = CPointCloud()
for i in range(50):
    angle = 2 * math.pi * i / 50
    pc.insertPoint(3 * math.cos(angle), 3 * math.sin(angle), 0.0)
scene.insert(pc)

# Coloured cloud
pcc = CPointCloudColoured()
for i in range(100):
    angle = 2 * math.pi * i / 100
    r = 2.0
    pcc.push_back(r * math.cos(angle), r * math.sin(angle), math.sin(2*angle),
                  abs(math.cos(angle)), abs(math.sin(angle)), 0.5)
print(f"  CPointCloudColoured: {len(pcc)} points")
scene.insert(pcc)

# ---------------------------------------------------------------------------
# Lines
# ---------------------------------------------------------------------------
lines = CSetOfLines()
lines.appendLine(0, 0, 0, 1, 1, 1)
lines.appendLine(1, 1, 1, 2, 0, 0)
lines.appendLine(TSegment3D(TPoint3D(0,0,0), TPoint3D(-1,-1,1)))
print(f"  CSetOfLines: {len(lines)} segments")
scene.insert(lines)

simple_line = CSimpleLine()
simple_line.setLineCoords(-3, 0, 0, 3, 0, 2)
scene.insert(simple_line)

# ---------------------------------------------------------------------------
# Text labels
# ---------------------------------------------------------------------------
lbl = CText("Hello MRPT")
lbl.setFont("sans")
scene.insert(lbl)

lbl3 = CText3D("3D label", "sans", 0.3)
lbl3.pose.x = 1.0
lbl3.pose.y = 1.0
scene.insert(lbl3)

# ---------------------------------------------------------------------------
# Uncertainty ellipsoid
# ---------------------------------------------------------------------------
cov = CMatrixDouble33()
arr = np.diag([0.5, 0.2, 0.1])
cov2 = CMatrixDouble33(arr)
ell = CEllipsoid3D()
ell.setCovMatrix(cov2)
ell.setQuantiles(2.0)
ell.set3DsegmentsCount(20)
ell.pose.x = -2.0
ell.pose.y = 2.0
scene.insert(ell)

# ---------------------------------------------------------------------------
# Stock objects
# ---------------------------------------------------------------------------
corner = stock_objects.CornerXYZSimple(0.5, 2.0)
scene.insert(corner)

pioneer = stock_objects.RobotPioneer()
scene.insert(pioneer)

hokuyo = stock_objects.Hokuyo_URG()
scene.insert(hokuyo)

print(f"\nScene built successfully with {len([grid, axes, box, sphere, cyl, arrow, pc, pcc, lines, simple_line, lbl, lbl3, ell, corner, pioneer, hokuyo])} objects")
print("  (use mrpt_gui_example.py to display interactively)")
