#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-pymrpt, ros-$ROS_DISTRO-mrpt2, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

from mrpt.pymrpt import mrpt
import time

# Create GUI:
win = mrpt.gui.CDisplayWindow3D('MRPT GUI demo', 800, 600)

# Get and lock 3D scene:
# Lock is required again each time the scene is modified, to prevent
# data race between the main and the rendering threads.
scene = win.get3DSceneAndLock()

# ctor args: xMin: float, xMax: float, yMin: float, yMax: float, z: float, frequency: float
glGrid = mrpt.opengl.CGridPlaneXY.Create(-10, 10, -10, 10, 0, 1)
scene.insert(glGrid)

glCorner = mrpt.opengl.stock_objects.CornerXYZ(2.0)
scene.insert(glCorner)

glCorner2: mrpt.opengl.CSetOfObjects = mrpt.opengl.stock_objects.CornerXYZ(1.0)
glCorner2.setLocation(4.0, 0.0, 0.0)
scene.insert(glCorner2)

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


# end of scene lock:
win.unlockAccess3DScene()


print('Close the window to quit the program')

while win.isOpen():

    win.repaint()
    time.sleep(50e-3)
