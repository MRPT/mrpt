#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-pymrpt, ros-$ROS_DISTRO-python-mrpt,
# ros-$ROS_DISTRO-mrpt2, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

from mrpt.pymrpt import mrpt

# Aliases:
TPoint3D = mrpt.math.TPoint3D_double_t
TLine3D = mrpt.math.TLine3D

l1: TLine3D = TLine3D.FromTwoPoints(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0))
l2: TLine3D = TLine3D.FromTwoPoints(TPoint3D(1, 1, 1), TPoint3D(2, 2, 0))

print('l1             : ' + str(l1))
print('l2             : ' + str(l2))
print('dist(l1,l2)    : ' + str(mrpt.math.distance(l1, l2)))
print('l1.director    : ' + l1.getDirectorVector().asString())
