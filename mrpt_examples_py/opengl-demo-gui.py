#!/usr/bin/env python3

# . install/setup.bash && python3 opengl-demo-gui.py

import time
import math

from mrpt.gui import CDisplayWindow3D
from mrpt.viz import CGridPlaneXY, CBox, CSetOfObjects, stock_objects
from mrpt.math import TPoint3D
from mrpt.img import TColor
from mrpt.system import CTicTac

# Create GUI:
win = CDisplayWindow3D('MRPT GUI demo', 800, 600)

# Lock 3D scene before modifying it (rendering runs on a separate thread):
scene = win.get3DSceneAndLock()

# A grid on the XY horizontal plane:
glGrid = CGridPlaneXY.Create(-3, 3, -3, 3, 0, 1)
scene.insert(glGrid)

# A couple of XYZ "corners":
glCorner = stock_objects.CornerXYZ(2.0)
scene.insert(glCorner)

glCorner2: CSetOfObjects = stock_objects.CornerXYZ(1.0)
glCorner2.setLocation(4.0, 0.0, 0.0)
scene.insert(glCorner2)

# NOTE: CEllipsoidInverseDepth3D not yet wrapped (pybind11_plan_v3.md §0.4).

# A floor "block":
glFloor = CBox()
glFloor.setBoxCorners(TPoint3D(-15, -15, 0), TPoint3D(15, 15, 0.1))
glFloor.setLocation(0, 0, -3.0)
glFloor.setColor(0x70/255.0, 0x70/255.0, 0x70/255.0)
scene.insert(glFloor)

# A mobile box to illustrate animations:
glBox = CBox()
glBox.setBoxCorners(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1))
# NOTE: setBoxBorderColor / castShadows not yet wrapped (pybind11_plan_v3.md §0.4).
scene.insert(glBox)

# Move camera:
win.setCameraAzimuthDeg(-40.0)
win.setCameraElevationDeg(30.0)

win.unlockAccess3DScene()

print('Close the window to quit the program')
timer = CTicTac()

while win.isOpen():
    y = 5.0 * math.sin(1.0 * timer.Tac())
    glBox.setLocation(4.0, y, 0.0)
    win.repaint()
    time.sleep(50e-3)
