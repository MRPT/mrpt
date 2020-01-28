/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#define GL_GLEXT_PROTOTYPES 1
#if !defined(GLEW_STATIC)
#define GLEW_STATIC
#endif

#include <mrpt/config.h>

#if MRPT_HAS_OPENGL_GLUT
#ifdef _WIN32
// WINDOWS:
#if defined(_MSC_VER)
#pragma warning(disable : 4505)
#endif
#include <GL/glew.h>
#include <windows.h>
#endif  // _WIN32

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
// gl-ext
#ifdef _WIN32
#include "glext/glext.h"
#else
#include <GL/glext.h>
#endif
#endif

/* Jerome Monceaux : bilock@gmail.com
 * Add inclusion of 3rdparty/freeglut/GL/glut.h
 * because GLUT_INIT_STATE is detected as undefined
 * under osx
 */
#ifdef __APPLE__
//#include <3rdparty/freeglut/GL/freeglut_std.h>
#ifndef GLUT_INIT_STATE
#define GLUT_INIT_STATE 0x007C
#endif
#else
#ifdef HAVE_FREEGLUT_EXT_H
#include <GL/freeglut_ext.h>
#endif
#endif

namespace mrpt::opengl
{
MRPT_TODO("****** Convert into a class with dtor to free resources***");
inline GLuint make_buffer(
	GLenum target, const void* buffer_data, GLsizei buffer_size)
{
	GLuint buffer;
	glGenBuffers(1, &buffer);
	glBindBuffer(target, buffer);
	glBufferData(target, buffer_size, buffer_data, GL_STATIC_DRAW);
	return buffer;
}
#define BUFFER_OFFSET(offset) (reinterpret_cast<GLvoid*>(offset))

}  // namespace mrpt::opengl

#endif  // MRPT_HAS_OPENGL_GLUT
