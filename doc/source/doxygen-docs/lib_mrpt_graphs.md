\defgroup mrpt_graphs_grp [mrpt-graphs]

Graphs data structures (directed graphs, trees, graphs of pose constraints),
graphs algorithms

[TOC]

# Library mrpt-graphs

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-graphs-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

Graph-related stuff: generic directed graphs (mrpt::graphs::CDirectedGraph) and
trees (mrpt::graphs::CDirectedTree).

Graphs of pose constraints are also defined in this library, via a generic
template mrpt::graphs::CNetworkOfPoses, capable of reading and writing to both
binary and [text pose-graph file](Robotics_file_formats.html) formats.

Predefined typedefs exist for:
 - mrpt::graphs::CNetworkOfPoses2D     -> Edges are 2D graphs (x,y,phi), without
uncertainty.
 - mrpt::graphs::CNetworkOfPoses3D     -> Edges are 3D graphs
(x,y,z,yaw,pitch,roll),  without uncertainty.
 - mrpt::graphs::CNetworkOfPoses2DInf  -> Edges are 2D graphs (x,y,phi), with an
inverse covariance (information) matrix.
 - mrpt::graphs::CNetworkOfPoses3DInf  -> Edges are 3D graphs
(x,y,z,yaw,pitch,roll), with an inverse covariance (information) matrix.

# Library contents
