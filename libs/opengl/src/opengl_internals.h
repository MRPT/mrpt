/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#define GL_GLEXT_PROTOTYPES 1
#define GLEW_STATIC

#include <mrpt/config.h>

#if MRPT_HAS_OPENGL_GLUT
#ifdef _WIN32
// WINDOWS:
#if defined(_MSC_VER)
#pragma warning(disable : 4505)
#endif
#include <windows.h>
#include <GL/glew.h>
#endif  // _WIN32

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <OpenGL/glext.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
// gl-ext
#ifdef _WIN32
#include "glext/glext.h"
#else
#include <GL/glext.h>
#endif
#endif

/* Jerome Monceaux : bilock@gmail.com
 * Add inclusion of otherlibs/freeglut/GL/glut.h
 * because GLUT_INIT_STATE is detected as undefined
 * under osx
 */
#ifdef __APPLE__
//#include <otherlibs/freeglut/GL/freeglut_std.h>
#ifndef GLUT_INIT_STATE
#define GLUT_INIT_STATE 0x007C
#endif
#else
#ifdef HAVE_FREEGLUT_EXT_H
#include <GL/freeglut_ext.h>
#endif
#endif

#endif  // MRPT_HAS_OPENGL_GLUT
