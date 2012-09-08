/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

