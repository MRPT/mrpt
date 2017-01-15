/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_pbmap_grp [mrpt-pbmap]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-pbmap</code></h2>
<hr>

This library implements the functionality to build Plane-based Maps (PbMaps) from a set of point clouds plus their corresponding poses, which might be given by e.g. the odometry of a robot.

A PbMap consists of a set of planar entities (patches) described by geometric features (shape, relative position, etc.) and/or radiometric features (dominant color). It is organized as an annotated, undirected graph, where nodes stand for planar patches and edges connect neighbor planes when the distance between their closest points is under a threshold.

A method for PbMap place recognition (useful for re-localization or loop closure) is also implemented. This method relies on an interpretation tree which applies geometric constraints to efficiently match sets of neighboring planes.

Refer to the <b><a href="pbmap-guide.pdf" >PbMap Guide (PDF)</a></b>.

See:
- Online web with papers, tutorials, etc: http://www.mrpt.org/pbmap
- Namespace mrpt::pbmap


*/

