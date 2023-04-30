#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-mrpt, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

from mrpt import pymrpt as m

# Aliases:
TPoint3D = m.mrpt.math.TPoint3D_double_t
TLine3D = m.mrpt.math.TLine3D

l1 = TLine3D.FromTwoPoints(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0))
l2 = TLine3D.FromTwoPoints(TPoint3D(1, 1, 1), TPoint3D(2, 2, 0))

print('l1             : ' + str(l1))
print('l2             : ' + str(l2))
print('dist(l1,l2)    : ' + str(m.mrpt.math.distance(l1, l2)))
