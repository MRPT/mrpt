#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-mrpt-*, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

from mrpt_math.pymrpt_math import mrpt as mrpt_math

# Aliases:
TPoint3D = mrpt_math.math.TPoint3D_double_t
TLine3D = mrpt_math.math.TLine3D

if __name__ == "__main__":
    l1 = TLine3D.FromTwoPoints(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0))
    l2 = TLine3D.FromTwoPoints(TPoint3D(1, 1, 1), TPoint3D(2, 2, 0))

    print('l1             : ' + str(l1))
    print('l2             : ' + str(l2))
    print('dist(l1,l2)    : ' + str(mrpt_math.math.distance(l1, l2)))
