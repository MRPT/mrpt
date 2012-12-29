/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

/** \defgroup mrpt_maps_grp [mrpt-maps]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-maps</code></h2>
<hr>

This library includes (almost) all the maps usable for localization or mapping
in the rest of MRPT classes.

Interesting starting points:
<ul>
<li> To see the list of existing metric maps, see the classes inheriting from
the base virtual mrpt::slam::CMetricMap. <b>Note:</b> There are two special
maps which are not declared here in mrpt-maps, but 
in <a href="mrpt-vision.html" >mrpt-vision</a>, due to their
heavy dependence on computer vision functions. </li>
<li> This library includes an embedded version of the ANN library for 
kd-tree fast construction and queries. It's used internally in CPointsMap, but 
could be used directly by the user. See ANNkd_tree. </li>
<li> This library also adds new classes to the namespace mrpt::opengl, which 
couldn't be included in the library mrpt-opengl due to its heavy dependence on
map classes declared here. The classes are:
mrpt::opengl::CAngularObservationMesh,
mrpt::opengl::CPlanarLaserScan </li> 
</ul> 

Please, note that there is another very important map class 
(mrpt::slam::CMultiMetricMap) which is not in mrpt-maps, but in the library mrpt-slam 
(the reason is that there is not another reasonable way to factor MRPT into small libraries).


See the list of classes in mrpt::slam (not all those classes are in mrpt-maps).




*/

