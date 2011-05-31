/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_internals_H
#define opengl_internals_H

#define GL_GLEXT_PROTOTYPES 1
#define GLEW_STATIC

#include <mrpt/config.h>
#include <mrpt/utils/utils_defs.h>

#if MRPT_HAS_OPENGL_GLUT
	#ifdef MRPT_OS_WINDOWS
		// WINDOWS:
		#if defined(_MSC_VER)
			#pragma warning(disable:4505)
		#endif
		#include <windows.h>

		#include <GL/glew.h>
	#endif	// MRPT_OS_WINDOWS

	#include <GL/glut.h>

	/* Jerome Monceaux : bilock@gmail.com
	 * Add inclusion of otherlibs/freeglut/GL/glut.h
	 * because GLUT_INIT_STATE is detected as undefined 
	 * under osx
	 */
	#ifdef __APPLE__
		#include <otherlibs/freeglut/GL/freeglut_std.h>
	#endif

	#ifdef HAVE_FREEGLUT_EXT_H
		#include <GL/freeglut_ext.h>
	#endif


	// gl-ext
	#ifdef MRPT_OS_WINDOWS
		#include "glext/glext.h"
	#else
		#include <GL/glext.h>
	#endif

#endif // MRPT_HAS_OPENGL_GLUT

#endif
