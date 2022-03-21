/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/core/exceptions.h>
#include <mrpt/opengl/COpenGLVertexArrayObject.h>
#include <mrpt/opengl/opengl_api.h>

#include <thread>

using namespace mrpt::opengl;

COpenGLVertexArrayObject::COpenGLVertexArrayObject()
{
	m_impl = std::make_shared<RAII_Impl>();
}

COpenGLVertexArrayObject::RAII_Impl::~RAII_Impl()
{
	// Free resources:
	destroy();
}

void COpenGLVertexArrayObject::RAII_Impl::create()
{
	destroy();
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	GLuint buffer;
	glGenVertexArrays(1, &buffer);
	this->buffer_id = buffer;
	this->created_from = std::this_thread::get_id();
	created = true;
#endif
}

void COpenGLVertexArrayObject::RAII_Impl::destroy()
{
	if (!created) return;
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	if (created_from == std::this_thread::get_id())
	{
		release();
		glDeleteVertexArrays(1, &buffer_id);
	}
#endif
	buffer_id = 0;
	created = false;
}

void COpenGLVertexArrayObject::RAII_Impl::bind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	ASSERT_(created);
	glBindVertexArray(buffer_id);
#endif
}

void COpenGLVertexArrayObject::RAII_Impl::release()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	if (!created) return;
	if (created_from != std::this_thread::get_id()) return;

	glBindVertexArray(0);
#endif
}
