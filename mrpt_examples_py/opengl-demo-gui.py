#!/usr/bin/env python3
"""
Animated 3D scene in a GUI window, with mrpt.gui and mrpt.viz.

Usage:

  . install/setup.bash && python3 opengl-demo-gui.py [--duration SECONDS]
"""

import argparse
import time
import math

from mrpt.gui import CDisplayWindowGUI
from mrpt.viz import Scene, CGridPlaneXY, CBox, CSetOfObjects, stock_objects
from mrpt.math import TPoint3D
from mrpt.img import TColor
from mrpt.system import CTicTac

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=float, default=0,
                    help="Close the window after this many seconds (default: 0 = never)")
args = parser.parse_args()

# Create GUI:
win = CDisplayWindowGUI('MRPT GUI demo', 800, 600)

# The 3D scene shown in the window:
scene = Scene()

# A grid on the XY horizontal plane:
glGrid = CGridPlaneXY(-3, 3, -3, 3, 0, 1)
scene.insert(glGrid)

# A couple of XYZ "corners":
glCorner = stock_objects.CornerXYZ(2.0)
scene.insert(glCorner)

glCorner2: CSetOfObjects = stock_objects.CornerXYZ(1.0)
glCorner2.setLocation(4.0, 0.0, 0.0)
scene.insert(glCorner2)

# A floor "block":
glFloor = CBox()
glFloor.setBoxCorners(TPoint3D(-15, -15, 0), TPoint3D(15, 15, 0.1))
glFloor.setLocation(0, 0, -3.0)
glFloor.setColor(0x70/255.0, 0x70/255.0, 0x70/255.0)
scene.insert(glFloor)

# A mobile box to illustrate animations:
glBox = CBox()
glBox.setBoxCorners(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1))
glBox.enableBoxBorder(True)
glBox.setBoxBorderColor(TColor(0, 0, 0))
glBox.castShadows = True
scene.insert(glBox)

win.background_scene = scene

# Move camera:
win.camera().setAzimuthDegrees(-40.0)
win.camera().setElevationDegrees(30.0)
win.camera().setZoomDistance(15.0)

print('Close the window to quit the program')
timer = CTicTac()

# spinOnce() processes the window events and repaints it. It returns False
# once the window is closed.
while win.spinOnce() and (args.duration <= 0 or timer.Tac() < args.duration):
    y = 5.0 * math.sin(1.0 * timer.Tac())
    glBox.setLocation(4.0, y, 0.0)
    time.sleep(20e-3)
