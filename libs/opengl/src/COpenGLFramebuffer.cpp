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
#include <mrpt/core/backtrace.h>
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/COpenGLFramebuffer.h>
#include <mrpt/opengl/opengl_api.h>

using namespace mrpt::opengl;

#if MRPT_HAS_OPENGL_GLUT
static bool isExtensionSupported([[maybe_unused]] const std::string& extension)
{
	MRPT_START
	for (int index = 0;; index++)
	{
		const auto extName = glGetStringi(GL_EXTENSIONS, index);
		if (!extName) break;
		const auto sExt = std::string(reinterpret_cast<const char*>(extName));
		// std::cout << sExt << "\n";
		if (sExt == extension) return true;
	}
	MRPT_END
	return false;
}
#endif

COpenGLFramebuffer::COpenGLFramebuffer() : m_impl(std::make_shared<RAII_Impl>())
{
}

void COpenGLFramebuffer::RAII_Impl::create(
	unsigned int width, unsigned int height, int nSamples)
{
#if MRPT_HAS_OPENGL_GLUT

	if (!isExtensionSupported("GL_EXT_framebuffer_object"))
		THROW_EXCEPTION(
			"Framebuffer Object extension unsupported "
			"(GL_EXT_framebuffer_object)");

	m_width = width;
	m_height = height;
	m_Samples = nSamples;

	const auto oldFBs = CurrentBinding();

	// Render buffer: RGB
	glGenRenderbuffers(1, &m_Color);
	CHECK_OPENGL_ERROR();
	glBindRenderbuffer(GL_RENDERBUFFER, m_Color);
	CHECK_OPENGL_ERROR();

	if (nSamples <= 1)
	{
		glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, m_width, m_height);
		CHECK_OPENGL_ERROR();
	}
	else
	{
		glRenderbufferStorageMultisample(
			GL_RENDERBUFFER, nSamples, GL_RGBA8, m_width, m_height);
		CHECK_OPENGL_ERROR();
	}

	// Render buffer: DEPTH
	glGenRenderbuffers(1, &m_Depth);
	CHECK_OPENGL_ERROR();
	glBindRenderbuffer(GL_RENDERBUFFER, m_Depth);
	CHECK_OPENGL_ERROR();

	if (nSamples <= 1)
	{
		glRenderbufferStorage(
			GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, m_width, m_height);
		CHECK_OPENGL_ERROR();
	}
	else
	{
		glRenderbufferStorageMultisample(
			GL_RENDERBUFFER, nSamples, GL_DEPTH24_STENCIL8, m_width, m_height);
		CHECK_OPENGL_ERROR();
	}

	// Frame buffer:
	glGenFramebuffers(1, &m_Framebuffer);
	CHECK_OPENGL_ERROR();

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
	CHECK_OPENGL_ERROR();

	glFramebufferRenderbuffer(
		GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, m_Color);
	CHECK_OPENGL_ERROR();

	glFramebufferRenderbuffer(
		GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_Depth);
	CHECK_OPENGL_ERROR();

	glFramebufferRenderbuffer(
		GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, m_Depth);
	CHECK_OPENGL_ERROR();

	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	CHECK_OPENGL_ERROR();
	glReadBuffer(GL_COLOR_ATTACHMENT0);
	CHECK_OPENGL_ERROR();

	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
		THROW_EXCEPTION("Could not create framebuffer object.");

	m_created = true;
	m_created_from = std::this_thread::get_id();

	// Restore:
	Bind(oldFBs);
#else
	THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}

void COpenGLFramebuffer::RAII_Impl::destroy()
{
	if (!m_created) return;

#if MRPT_HAS_OPENGL_GLUT
	static const bool showErrs =
		(::getenv("MRPT_REVEAL_OPENGL_BUFFER_LEAKS") != nullptr);

	if (m_created_from == std::this_thread::get_id())
	{
		unbind();

		glDeleteRenderbuffers(1, &m_Color);
		CHECK_OPENGL_ERROR();
		glDeleteRenderbuffers(1, &m_Depth);
		CHECK_OPENGL_ERROR();
		glDeleteFramebuffers(1, &m_Framebuffer);
		CHECK_OPENGL_ERROR();
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

			std::cerr
				<< "[COpenGLFramebuffer::RAII_Impl] *Warning* Leaking OpenGL "
				   "resources: FBO was created from a different thread "
				   "and cannot free it from this thread, call stack:"
				<< bt.asString() << std::endl;
		}
	}
#endif
	m_Color = m_Depth = 0;
	m_Framebuffer = 0;
	m_created = false;
}

FrameBufferBinding COpenGLFramebuffer::RAII_Impl::bind()
{
#if MRPT_HAS_OPENGL_GLUT
	const FrameBufferBinding ids = CurrentBinding();

	glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
	CHECK_OPENGL_ERROR();
	if (m_Samples > 1)
	{
		glEnable(GL_MULTISAMPLE);
		CHECK_OPENGL_ERROR();
	}

	return ids;
#else
	THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}

void COpenGLFramebuffer::RAII_Impl::unbind()
{
#if MRPT_HAS_OPENGL_GLUT
	if (m_Samples > 1) glDisable(GL_MULTISAMPLE);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	CHECK_OPENGL_ERROR();
#endif
}

void COpenGLFramebuffer::blit()
{
#if MRPT_HAS_OPENGL_GLUT
	glBindFramebuffer(GL_READ_FRAMEBUFFER, m_impl->m_Framebuffer);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	glDrawBuffer(GL_BACK);

	glBlitFramebuffer(
		0, 0, m_impl->m_width, m_impl->m_height, 0, 0, m_impl->m_width,
		m_impl->m_height, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT,
		GL_NEAREST);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif
}

void COpenGLFramebuffer::Bind(const FrameBufferBinding& ids)
{
#if MRPT_HAS_OPENGL_GLUT
	glBindFramebuffer(GL_READ_FRAMEBUFFER, ids.readFbId);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, ids.drawFbId);
#endif
}

FrameBufferBinding COpenGLFramebuffer::CurrentBinding()
{
#if MRPT_HAS_OPENGL_GLUT
	GLint drawFboId = 0, readFboId = 0;
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFboId);
	glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFboId);

	FrameBufferBinding ids;
	ids.drawFbId = drawFboId;
	ids.readFbId = readFboId;
	return ids;
#else
	THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}
