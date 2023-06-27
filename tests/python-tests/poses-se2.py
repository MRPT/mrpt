#!/usr/bin/env python3

from math import radians
from mrpt.pymrpt.mrpt.poses import CPose2D
import os

print('[Running python test ' + os.path.basename(__file__) + ']')

p1 = CPose2D(1.0, 2.0, radians(90.0))
p2 = CPose2D(3.0, 0.0, radians(0.0))

p3 = p1 + p2
p4 = p3 - p1

assert ((p4-p2).norm() < 1e-4)
assert (abs(p4.phi()-p2.phi()) < 1e-4)

print('[Successfully run ' + os.path.basename(__file__) + ']')
