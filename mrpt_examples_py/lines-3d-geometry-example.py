#!/usr/bin/env python3

from mrpt.math import TPoint3D, TLine3D

l1: TLine3D = TLine3D.FromTwoPoints(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0))
l2: TLine3D = TLine3D.FromTwoPoints(TPoint3D(1, 1, 1), TPoint3D(2, 2, 0))

print('l1             : ' + str(l1))
print('l2             : ' + str(l2))

# Distance from l1 to a point (line-to-line free function not yet wrapped,
# see pybind11_plan_v3.md §0.1 geometry.h free functions):
pt = TPoint3D(0, 1, 0)
print('dist(l1, pt)   : ' + str(l1.distance(pt)))

# Director vector is a plain list [dx, dy, dz]:
print('l1.director    : ' + str(l1.director))
