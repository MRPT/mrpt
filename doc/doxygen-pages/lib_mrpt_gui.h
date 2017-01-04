/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \defgroup mrpt_gui_grp [mrpt-gui]

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a href="modules.html" >See all modules</a> </small>
<br>

<h2>Library <code>mrpt-gui</code></h2>
<hr>

This library provides three classes that represent GUI windows, each having a 
specific specialized purpose:

<ul>
<li> <b> mrpt::gui::CDisplayWindow :</b> Displays 2D bitmap images, and optionally sets of points over them, etc.  </li>
<li> <b> mrpt::gui::CDisplayWindow3D :</b> A powerful 3D rendering window capable of displaying a mrpt::opengl::COpenGLScene. 
It features mouse navigation, Alt+Enter fullscreen switching, multiple viewports, etc. See this <a href="http://www.mrpt.org/Tutorial_3D_Scenes" >tutorial</a>.</li>
<li> <b> mrpt::gui::CDisplayWindowPlots :</b> Displays one or more 2D vectorial graphs, in a manner very similar to MATLAB "plot" commands. </li>
</ul>

All these window classes inherits from mrpt::gui::CBaseGUIWindow, which provides a set of methods
and variables common to all the classes. 
It allow moving/resizing the windows, polling for key strokes, etc. 
Note events-driven applications can be also implemented since it also implements 
the mrpt::utils::CObservable pattern, emitting events as described in 
the description of mrpt::gui::CBaseGUIWindow.


All the classes in this library are in the namespace mrpt::gui

*/

