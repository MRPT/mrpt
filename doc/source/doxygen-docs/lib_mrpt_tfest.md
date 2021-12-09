\defgroup mrpt_tfest_grp [mrpt-tfest]

Algorithms to find optimal transformations from sets of correspondences.

[TOC]

# Library mrpt-tfest


This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-tfest-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

<b>T</b>rans<b>f</b>ormation <b>est</b>imation (tfest): This module provides
functions in charge of solving the optimization problem of aligning a set of 2D
or 3D corresponding points, estimating the optimal transformation between the
two frames of reference.

Note that this does not include the related iterative ICP algorithm (see
mrpt::slam::CICP), included in the library \ref mrpt_slam_grp

See list of all functions: mrpt::tfest

# Library contents
