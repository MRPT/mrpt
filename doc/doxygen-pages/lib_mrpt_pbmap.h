/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_pbmap_grp [mrpt-pbmap]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-pbmap</code></h2>
<hr>

This library implements the functionality to build Plane-based Maps (PbMaps) from a set of point clouds plus their corresponding poses, which might be given by e.g. the odometry of a robot.

A PbMap consists of a set of planar entities (patches) described by geometric features (shape, relative position, etc.) and/or radiometric features (dominant color). It is organized as an annotated, undirected graph, where nodes stand for planar patches and edges connect neighbor planes when the distance between their closest points is under a threshold.

A method for PbMap place recognition (useful for re-localization or loop closure) is also implemented. This method relies on an interpretation tree to efficiently match sets of neighboring planes. Such interpretation tree applies geometric and radiometric restrictions in the form of both unary and binary constraints. The unary constraints check the individual correspondence of two planes by comparing directly their features (e.g. size, color), while the binary constraints validate if two pairs of connected planes present the same geometric relationship (e.g. perpendicularity).

Refer to the <b><a href="pbmap-guide.pdf" >PbMap Guide (PDF)</a></b>.

See:
- Namespace mrpt::pbmap


*/

