/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page dep-opengl-glut External dependency: opengl+glut
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>opengl</code> and <code>glut</code></h2>
<hr>

This optional library is required to render 3D graphics. It is fundamentally used by the classes in mrpt::opengl.

Note however that nothing prevents you from creating objects in mrpt::opengl, manipulating them, building 3D scenes, saving them to disk, etc. even without OpenGL support. 
An exception will be raised only if you try to render them in a mrpt::gui::CDisplayWindow3D or by any other mean.

*/

