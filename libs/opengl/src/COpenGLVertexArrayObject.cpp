/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/core/exceptions.h>
#include <mrpt/opengl/COpenGLVertexArrayObject.h>
#include <mrpt/opengl/gl_utils.h>
#include "opengl_internals.h"

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
#if MRPT_HAS_OPENGL_GLUT
	GLuint buffer;
	glGenVertexArrays(1, &buffer);
	this->buffer_id = buffer;
	created = true;
#endif
}

void COpenGLVertexArrayObject::RAII_Impl::destroy()
{
	if (!created) return;
#if MRPT_HAS_OPENGL_GLUT
	release();
	glDeleteVertexArrays(1, &buffer_id);
#endif
	buffer_id = 0;
	created = false;
}

void COpenGLVertexArrayObject::RAII_Impl::bind()
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(created);
	glBindVertexArray(buffer_id);
#endif
}

void COpenGLVertexArrayObject::RAII_Impl::release()
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(created);
	glBindVertexArray(0);
#endif
}
