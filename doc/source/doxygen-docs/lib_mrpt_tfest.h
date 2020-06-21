/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_tfest_grp [mrpt-tfest]

Algorithms to find optimal transformations from sets of correspondences.



[TOC]

# Library `mrpt-tfest`
<small> [New in MRPT 2.0.0] </small>

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-tfest-dev

See: \ref mrpt_from_cmake

<b>T</b>rans<b>f</b>ormation <b>est</b>imation (tfest): This module provides
functions in charge of solving the optimization problem of aligning a set of 2D
or 3D corresponding points, estimating the optimal transformation between the
two frames of reference.

Note that this does not include the related iterative ICP algorithm (see
mrpt::slam::CICP), included in the library \ref mrpt_slam_grp

See list of all functions: mrpt::tfest

*/
