/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/core/exceptions.h>
#include <mrpt/opengl/VertexArrayObject.h>
#include <mrpt/opengl/opengl_api.h>

#include <thread>

using namespace mrpt::opengl;

VertexArrayObject::VertexArrayObject() {}

VertexArrayObject::RAII_Impl::~RAII_Impl()
{
	// Free resources:
	destroy();
}

void VertexArrayObject::RAII_Impl::create()
{
	destroy();
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	GLuint buffer;
	glGenVertexArrays(1, &buffer);
	m_state.get().buffer_id = buffer;
	m_state.get().created = true;
#endif
}

void VertexArrayObject::RAII_Impl::destroy()
{
	if (!m_state.get().created) return;
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	release();
	glDeleteVertexArrays(1, &m_state.get().buffer_id);
#endif
	m_state.get().buffer_id = 0;
	m_state.get().created = false;
}

void VertexArrayObject::RAII_Impl::bind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	ASSERT_(m_state.get().created);
	glBindVertexArray(m_state.get().buffer_id);
#endif
}

void VertexArrayObject::RAII_Impl::release()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	if (!m_state.get().created) return;
	glBindVertexArray(0);
#endif
}
