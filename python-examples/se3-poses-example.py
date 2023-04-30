#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-mrpt, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

from mrpt import pymrpt as m
from math import radians

p1 = m.mrpt.poses.CPose3D.FromXYZYawPitchRoll(
    1.0, 2.0, 0, radians(90), radians(0), radians(0))
p2 = m.mrpt.poses.CPose3D.FromXYZYawPitchRoll(
    3.0, 0.0, 0, radians(0.0), radians(0), radians(0))

p3 = p1 + p2
p4 = p3 - p1

print('p1             : ' + str(p1))
print('p2             : ' + str(p2))
print('p1(+)p2        : ' + str(p3))
print('(p1(+)p2)(-)p1 : ' + str(p4))
