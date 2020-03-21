/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_gui_grp [mrpt-gui]

GUI support: 2D plots (Matlab-like), 3D rendering viewports, etc.

<small> <a href="index.html#libs">Back to list of all libraries</a> | <a
href="modules.html" >See all modules</a> </small> <br>

# Library `mrpt-gui`

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-gui-dev

See: \ref mrpt_from_cmake

This library provides these classes that represent GUI windows, each having a
specific specialized purpose:

 - mrpt::gui::CDisplayWindow: Displays 2D bitmap images, and
optionally sets of points over them, etc.
   ![mrpt::gui::CDisplayWindow screenshot](preview_CDisplayWindow.jpg)

 - mrpt::gui::CDisplayWindow3D: A powerful 3D rendering window capable of
displaying a mrpt::opengl::COpenGLScene, or efficiently displaying 2D images
using graphics card acceleration. It features mouse navigation, Alt+Enter
fullscreen switching, multiple viewports, etc. See [this
tutorial](https://www.mrpt.org/Tutorial_3D_Scenes).
   ![mrpt::gui::CDisplayWindow3D screenshot](preview_CDisplayWindow3D.png)

 - mrpt::gui::CDisplayWindowPlots: Displays one or more 2D vectorial graphs,
in a manner very similar to MATLAB "plot" commands.
   ![mrpt::gui::CDisplayWindowPlots screenshot](preview_CDisplayWindowPlots.png)

 - mrpt::gui::CDisplayWindowGUI: UI capable of complex controls, subwindows,
menus, etc. powered by the nanogui library.
 ![mrpt::gui::CDisplayWindowGUI screenshot](preview_CDisplayWindowGUI.png)


All these window classes inherits from mrpt::gui::CBaseGUIWindow, which provides
a set of methods and variables common to all the classes. It allow
moving/resizing the windows, polling for key strokes, etc. Note events-driven
applications can be also implemented since it also implements the
mrpt::system::CObservable pattern, emitting events as described in the
description of mrpt::gui::CBaseGUIWindow.

All the classes in this library are in the namespace mrpt::gui

*/
