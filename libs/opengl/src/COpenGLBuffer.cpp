/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
//
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/opengl/COpenGLBuffer.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/system/backtrace.h>

#include <cstdlib>
#include <iostream>

using namespace mrpt::opengl;

COpenGLBuffer::COpenGLBuffer(const COpenGLBuffer::Type type)
{
	m_impl = std::make_shared<RAII_Impl>(type);
}

COpenGLBuffer::RAII_Impl::RAII_Impl(COpenGLBuffer::Type t) : type(t) {}
COpenGLBuffer::RAII_Impl::~RAII_Impl()
{
	// Free opengl resources:
	destroy();
}

void COpenGLBuffer::RAII_Impl::create()
{
	destroy();
#if MRPT_HAS_OPENGL_GLUT
	GLuint buffer;
	glGenBuffers(1, &buffer);
	this->buffer_id = buffer;
	this->created_from = std::this_thread::get_id();
	created = true;
#endif
}

void COpenGLBuffer::RAII_Impl::destroy()
{
	if (!created) return;

	static const bool showErrs =
		(::getenv("MRPT_REVEAL_OPENGL_BUFFER_LEAKS") != nullptr);

#if MRPT_HAS_OPENGL_GLUT
	if (created_from == std::this_thread::get_id())
	{
		release();
		glDeleteBuffers(1, &buffer_id);
	}
	else if (showErrs)
	{
		// at least, emit a warning:
		static double tLast = 0;
		auto tNow = mrpt::Clock::toDouble(mrpt::Clock::now());
		if (tNow - tLast > 2.0)
		{
			tLast = tNow;

			mrpt::system::TCallStackBackTrace bt;
			mrpt::system::getCallStackBackTrace(bt);

			std::cerr << "[COpenGLBuffer::RAII_Impl] *Warning* Leaking memory "
						 "since Buffer was acquired from a different thread "
						 "and cannot free it from this thread, call stack:"
					  << bt.asString() << std::endl;
		}
	}
#endif
	buffer_id = 0;
	created = false;
}

void COpenGLBuffer::RAII_Impl::bind()
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(created);
	glBindBuffer(static_cast<GLenum>(type), buffer_id);
#endif
}

void COpenGLBuffer::RAII_Impl::release()
{
#if MRPT_HAS_OPENGL_GLUT
	if (!created) return;
	if (created_from != std::this_thread::get_id()) return;

	glBindBuffer(static_cast<GLenum>(type), 0);
#endif
}

void COpenGLBuffer::RAII_Impl::allocate(const void* data, int byteCount)
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(created);
	glBufferData(
		static_cast<GLenum>(type), byteCount, data, static_cast<GLenum>(usage));
#endif
}
