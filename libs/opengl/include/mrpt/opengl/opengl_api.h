/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#define GL_GLEXT_PROTOTYPES 1
#if !defined(GLEW_STATIC)
#define GLEW_STATIC
#endif

#include <mrpt/config.h>

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
#ifdef _WIN32
// WINDOWS:
#if defined(_MSC_VER)
#pragma warning(disable : 4505)
#endif
#include <windows.h>
//
#include <GL/glew.h>
#endif	// _WIN32

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/gl3.h>
#include <OpenGL/glext.h>
// From: https://stackoverflow.com/a/22119409/1631514
#define glGenVertexArrays glGenVertexArraysAPPLE
#define glBindVertexArray glBindVertexArrayAPPLE
#define glDeleteVertexArrays glDeleteVertexArraysAPPLE
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
// #include <3rdparty/freeglut/GL/freeglut_std.h>
#ifndef GLUT_INIT_STATE
#define GLUT_INIT_STATE 0x007C
#endif
#else
#if defined(HAVE_FREEGLUT_EXT_H) && !(__EMSCRIPTEN__)
#include <GL/freeglut_ext.h>
#endif
#endif

#if MRPT_HAS_EGL
// #define GL_GLEXT_PROTOTYPES // already def above
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

#if HAVE_GLES_GL_H
#include <GLES/gl.h>
#if HAVE_GLES_GLEXT_H
#include <GLES/glext.h>
#endif
#endif

#if HAVE_GLES2_GL2_H
#include <GLES2/gl2.h>
#if HAVE_GLES2_GL2EXT_H
#include <GLES2/gl2ext.h>
#endif
#endif

#if HAVE_GLES3_GL3_H
#include <GLES3/gl3.h>
#if HAVE_GLES3_GL3EXT_H
#include <GLES3/gl3ext.h>
#endif
#endif

namespace mrpt::opengl
{
void checkOpenGLErr_impl(
	unsigned int glErrorCode, const char* filename, int lineno);
}

/** Checks glGetError and throws an exception if an error situation is found
 */
#define CHECK_OPENGL_ERROR()                                                   \
	{                                                                          \
		auto openglErr = glGetError();                                         \
		if (openglErr != GL_NO_ERROR)                                          \
			mrpt::opengl::checkOpenGLErr_impl(openglErr, __FILE__, __LINE__);  \
	}

#endif	// MRPT_HAS_OPENGL_GLUT
