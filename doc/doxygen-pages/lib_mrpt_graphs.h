/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_graphs_grp [mrpt-graphs]

Graphs data structures (directed graphs, trees, graphs of pose constraints),
graphs algorithms

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a
href="modules.html" >See all modules</a> </small> <br>

# Library `mrpt-graphs`

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-graphs-dev

See: \ref mrpt_from_cmake

Graph-related stuff: generic directed graphs (mrpt::graphs::CDirectedGraph) and
trees (mrpt::graphs::CDirectedTree).

Graphs of pose constraints are also defined in this library, via a generic
template mrpt::graphs::CNetworkOfPoses, capable of reading and writing to both
binary and <a href="http://www.mrpt.org/Robotics_file_formats" >text pose-graph
file</a> formats.

Predefined typedefs exist for:
 - mrpt::graphs::CNetworkOfPoses2D     -> Edges are 2D graphs (x,y,phi), without
uncertainty.
 - mrpt::graphs::CNetworkOfPoses3D     -> Edges are 3D graphs
(x,y,z,yaw,pitch,roll),  without uncertainty.
 - mrpt::graphs::CNetworkOfPoses2DInf  -> Edges are 2D graphs (x,y,phi), with an
inverse covariance (information) matrix.
 - mrpt::graphs::CNetworkOfPoses3DInf  -> Edges are 3D graphs
(x,y,z,yaw,pitch,roll), with an inverse covariance (information) matrix.



*/
