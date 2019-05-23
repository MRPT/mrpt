/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_maps_grp [mrpt-maps]

Map representations for localization and SLAM.

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a
href="modules.html" >See all modules</a> </small> <br>

# Library `mrpt-maps`

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-maps-dev

See: \ref mrpt_from_cmake

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

mrpt::maps::CMultiMetricMap provides a versatile container for multiple metric
maps, that behaves as if they were a single metric map. This includes updating
the map contents from observations, fusing the information from all maps while
evaluating an observation likelihood, etc.

See the list of classes in mrpt::maps

*/
