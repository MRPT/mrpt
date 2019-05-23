/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

// clang-format off

/** \defgroup mrpt_slam_grp [mrpt-slam]

SLAM and PF-localization algorithms

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

[TOC]

# Library `mrpt-slam`
<small> [New in MRPT 2.0.0] </small>

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-slam-dev

See: \ref mrpt_from_cmake

Interesting stuff in this library:

- mrpt::slam::CMetricMapBuilder: A virtual base for both ICP and RBPF-based SLAM.

- mrpt::slam::CMonteCarloLocalization2D: Particle filter-based (Monte Carlo) localization for a robot in a planar scenario.

- mrpt::maps::CMultiMetricMap: The most versatile kind of metric map, which contains an arbitrary number of any other maps.

- Kalman Filters-based Range-Bearing SLAM, in 2D and 3D: See mrpt::slam::CRangeBearingKFSLAM and mrpt::slam::CRangeBearingKFSLAM2D.

- Data association: The NN and the JCBB algorithms, as very generic templates. See data_association.h


See the full list of classes in mrpt::slam.
Note that there are many classes
in that namespace not in the library mrpt-slam, but in libraries mrpt-slam depends
on. However, in you set mrpt-slam as a dependence of your project, you can be safe
all mrpt::slam classes will be available to you.

See also: For Graph-SLAM, see the namespace mrpt::graphslam in the library mrpt-graphslam.


*/
