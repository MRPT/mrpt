/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_graphs_grp [mrpt-graphs]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-graphs</code></h2>
<hr>

Graph-related stuff: generic directed graphs (mrpt::graphs::CDirectedGraph) and trees (mrpt::graphs::CDirectedTree).

Graphs of pose constraints are also defined in this library, via a generic template mrpt::graphs::CNetworkOfPoses, capable
of reading and writing to both binary and <a href="http://www.mrpt.org/Robotics_file_formats" >text pose-graph file</a> formats.
Predefined typedefs exist for:
 - mrpt::graphs::CNetworkOfPoses2D     -> Edges are 2D graphs (x,y,phi), without uncertainty.
 - mrpt::graphs::CNetworkOfPoses3D     -> Edges are 3D graphs (x,y,z,yaw,pitch,roll),  without uncertainty.
 - mrpt::graphs::CNetworkOfPoses2DInf  -> Edges are 2D graphs (x,y,phi), with an inverse covariance (information) matrix.
 - mrpt::graphs::CNetworkOfPoses3DInf  -> Edges are 3D graphs (x,y,z,yaw,pitch,roll), with an inverse covariance (information) matrix.



*/

