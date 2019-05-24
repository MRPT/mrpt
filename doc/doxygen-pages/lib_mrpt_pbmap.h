/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_pbmap_grp [mrpt-pbmap]

Plane-based maps

[TOC]

# Library `mrpt-pbmap`

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-pbmap-dev

See: \ref mrpt_from_cmake

This library implements the functionality to build Plane-based Maps (PbMaps)
from a set of point clouds plus their corresponding poses, which might be given
by e.g. the odometry of a robot.

A PbMap consists of a set of planar entities (patches) described by geometric
features (shape, relative position, etc.) and/or radiometric features (dominant
color). It is organized as an annotated, undirected graph, where nodes stand for
planar patches and edges connect neighbor planes when the distance between their
closest points is under a threshold.

A method for PbMap place recognition (useful for re-localization or loop
closure) is also implemented. This method relies on an interpretation tree which
applies geometric constraints to efficiently match sets of neighboring planes.

Refer to the [**PbMap Guide (PDF)**](pbmap-guide.pdf).

See:
- Online web with papers, tutorials, etc: https://www.mrpt.org/pbmap
- Namespace mrpt::pbmap


*/
