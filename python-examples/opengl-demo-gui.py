#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-pymrpt, ros-$ROS_DISTRO-python-mrpt,
# ros-$ROS_DISTRO-mrpt2, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

from mrpt.pymrpt import mrpt
import time
import math

# Create GUI:
win = mrpt.gui.CDisplayWindow3D('MRPT GUI demo', 800, 600)

# Get and lock 3D scene:
# Lock is required again each time the scene is modified, to prevent
# data race between the main and the rendering threads.
scene = win.get3DSceneAndLock()

# A grid on the XY horizontal plane:
# ctor args: xMin: float, xMax: float, yMin: float, yMax: float, z: float, frequency: float
glGrid = mrpt.opengl.CGridPlaneXY.Create(-3, 3, -3, 3, 0, 1)
scene.insert(glGrid)

# A couple of XYZ "corners":
glCorner = mrpt.opengl.stock_objects.CornerXYZ(2.0)
scene.insert(glCorner)

glCorner2: mrpt.opengl.CSetOfObjects = mrpt.opengl.stock_objects.CornerXYZ(1.0)
glCorner2.setLocation(4.0, 0.0, 0.0)
scene.insert(glCorner2)

# A 3D inverse-depth ellipsoid:
glEllip = mrpt.opengl.CEllipsoidInverseDepth3D()
cov = mrpt.math.CMatrixFixed_double_3UL_3UL_t.Zero()
cov[0, 0] = 0.01
cov[1, 1] = 0.001
cov[2, 2] = 0.002
mean = mrpt.math.CMatrixFixed_double_3UL_1UL_t()
mean[0, 0] = 0.2  # inv_range
mean[1, 0] = 0.5  # yaw
mean[2, 0] = -0.6  # pitch
glEllip.setCovMatrixAndMean(cov, mean)
scene.insert(glEllip)

# A floor "block":
glFloor = mrpt.opengl.CBox()
glFloor.setBoxCorners(mrpt.math.TPoint3D_double_t(-15, -15, 0),
                      mrpt.math.TPoint3D_double_t(15, 15, 0.1))
glFloor.setLocation(0, 0, -3.0)
glFloor.setColor_u8(mrpt.img.TColor(0x70, 0x70, 0x70))
scene.insert(glFloor)

# A mobile box to illustrate animations:
glBox = mrpt.opengl.CBox()
glBox.setBoxCorners(mrpt.math.TPoint3D_double_t(0, 0, 0),
                    mrpt.math.TPoint3D_double_t(1, 1, 1))
glBox.setBoxBorderColor(mrpt.img.TColor(0, 0, 0))
glBox.castShadows(True)
scene.insert(glBox)

# Shadows are disabled by default, enable them:
scene.getViewport().enableShadowCasting(True)
print('Shadow casting: ' + str(scene.getViewport().isShadowCastingEnabled()))

# Move camera:
win.setCameraAzimuthDeg(-40.0)
win.setCameraElevationDeg(30.0)

# end of scene lock:
win.unlockAccess3DScene()


print('Close the window to quit the program')
timer = mrpt.system.CTicTac()

while win.isOpen():
    y = 5.0*math.sin(1.0*timer.Tac())
    glBox.setLocation(4.0, y, 0.0)

    win.repaint()
    time.sleep(50e-3)
