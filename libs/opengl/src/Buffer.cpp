/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/core/Clock.h>
#include <mrpt/core/backtrace.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/Buffer.h>
#include <mrpt/opengl/opengl_api.h>

#include <cstdlib>
#include <iostream>

using namespace mrpt::opengl;

Buffer::Buffer(const Buffer::Type type) : m_impl(type) {}

Buffer::RAII_Impl::RAII_Impl(Buffer::Type t) : type(t) {}
Buffer::RAII_Impl::~RAII_Impl()
{
	// Free opengl resources:
	destroy();
}

void Buffer::RAII_Impl::create()
{
	destroy();
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	GLuint buffer;
	glGenBuffers(1, &buffer);
	this->buffer_id = buffer;
	this->created_from = std::this_thread::get_id();
	created = true;
#endif
}

void Buffer::RAII_Impl::destroy()
{
	if (!created) return;

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	static const bool showErrs =
		(::getenv("MRPT_REVEAL_OPENGL_BUFFER_LEAKS") != nullptr);

	if (created_from == std::this_thread::get_id())
	{
		unbind();
		glDeleteBuffers(1, &buffer_id);
	}
	else if (showErrs)
	{
		// at least, emit a warning:
		static thread_local double tLast = 0;
		auto tNow = mrpt::Clock::nowDouble();
		if (tNow - tLast > 2.0)
		{
			tLast = tNow;

			mrpt::TCallStackBackTrace bt;
			mrpt::callStackBackTrace(bt);

			std::cerr << "[Buffer::RAII_Impl] *Warning* Leaking memory "
						 "since Buffer was acquired from a different thread "
						 "and cannot free it from this thread, call stack:"
					  << bt.asString() << std::endl;
		}
	}
#endif
	buffer_id = 0;
	created = false;
}

void Buffer::RAII_Impl::bind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	ASSERT_(created);
	glBindBuffer(static_cast<GLenum>(type), buffer_id);
#endif
}

void Buffer::RAII_Impl::unbind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	if (!created) return;
	if (created_from != std::this_thread::get_id()) return;

	glBindBuffer(static_cast<GLenum>(type), 0);
#endif
}

void Buffer::RAII_Impl::allocate(const void* data, int byteCount)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	ASSERT_(created);
	glBufferData(
		static_cast<GLenum>(type), byteCount, data, static_cast<GLenum>(usage));
#endif
}
