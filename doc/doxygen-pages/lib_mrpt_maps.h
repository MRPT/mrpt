/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
the base virtual mrpt::maps::CMetricMap. <b>Note:</b> There are two special
maps which are not declared here in mrpt-maps, but 
in <a href="mrpt-vision.html" >mrpt-vision</a>, due to their
heavy dependence on computer vision functions. </li>
<li> This library also adds new classes to the namespace mrpt::opengl, which 
couldn't be included in the library mrpt-opengl due to its heavy dependence on
map classes declared here. The classes are:
mrpt::opengl::CAngularObservationMesh,
mrpt::opengl::CPlanarLaserScan </li> 
</ul> 

Please, note that there is another very important map class 
(mrpt::maps::CMultiMetricMap) which is not in mrpt-maps, but in the library mrpt-slam 
(the reason is that there is not another reasonable way to factor MRPT into small libraries).

See the list of classes in mrpt::maps


*/

