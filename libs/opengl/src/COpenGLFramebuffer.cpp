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
#include <mrpt/core/backtrace.h>
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/COpenGLFramebuffer.h>
#include <mrpt/opengl/opengl_api.h>

using namespace mrpt::opengl;

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
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

void COpenGLFramebuffer::RAII_Impl::create(
	unsigned int width, unsigned int height, int nSamples)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	if (!isExtensionSupported("GL_EXT_framebuffer_object"))
		THROW_EXCEPTION(
			"Framebuffer Object extension unsupported "
			"(GL_EXT_framebuffer_object)");

	auto& _ = m_state.get();

	_.m_width = width;
	_.m_height = height;
	_.m_Samples = nSamples;

	const auto oldFBs = CurrentBinding();

	// Render buffer: RGB
	glGenRenderbuffers(1, &_.m_Color);
	CHECK_OPENGL_ERROR_IN_DEBUG();
	glBindRenderbuffer(GL_RENDERBUFFER, _.m_Color);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	if (nSamples <= 1)
	{
		glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, _.m_width, _.m_height);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}
	else
	{
		glRenderbufferStorageMultisample(
			GL_RENDERBUFFER, nSamples, GL_RGBA8, _.m_width, _.m_height);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	// Render buffer: DEPTH
	glGenRenderbuffers(1, &_.m_Depth);
	CHECK_OPENGL_ERROR_IN_DEBUG();
	glBindRenderbuffer(GL_RENDERBUFFER, _.m_Depth);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	if (nSamples <= 1)
	{
		glRenderbufferStorage(
			GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, _.m_width, _.m_height);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}
	else
	{
		glRenderbufferStorageMultisample(
			GL_RENDERBUFFER, nSamples, GL_DEPTH24_STENCIL8, _.m_width,
			_.m_height);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	// Frame buffer:
	glGenFramebuffers(1, &_.m_Framebuffer);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindFramebuffer(GL_FRAMEBUFFER, _.m_Framebuffer);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glFramebufferRenderbuffer(
		GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _.m_Color);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glFramebufferRenderbuffer(
		GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _.m_Depth);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glFramebufferRenderbuffer(
		GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, _.m_Depth);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glReadBuffer(GL_COLOR_ATTACHMENT0);
	CHECK_OPENGL_ERROR();

	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
		THROW_EXCEPTION("Could not create framebuffer object.");

	_.m_created = true;

	// Restore:
	Bind(oldFBs);
#else
	THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}

void COpenGLFramebuffer::RAII_Impl::destroy()
{
	auto& _ = m_state.get();

	if (!_.m_created) return;

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	unbind();

	glDeleteRenderbuffers(1, &_.m_Color);
	CHECK_OPENGL_ERROR_IN_DEBUG();
	glDeleteRenderbuffers(1, &_.m_Depth);
	CHECK_OPENGL_ERROR_IN_DEBUG();
	glDeleteFramebuffers(1, &_.m_Framebuffer);
	CHECK_OPENGL_ERROR();
#endif
	_.m_Color = _.m_Depth = 0;
	_.m_Framebuffer = 0;
	_.m_created = false;
}

FrameBufferBinding COpenGLFramebuffer::RAII_Impl::bind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	const FrameBufferBinding ids = CurrentBinding();

	auto& _ = m_state.get();

	glBindFramebuffer(GL_FRAMEBUFFER, _.m_Framebuffer);
	CHECK_OPENGL_ERROR_IN_DEBUG();
	if (_.m_Samples > 1)
	{
		glEnable(GL_MULTISAMPLE);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	return ids;
#else
	THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}

void COpenGLFramebuffer::RAII_Impl::unbind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	auto& _ = m_state.get();
	if (_.m_Samples > 1) glDisable(GL_MULTISAMPLE);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	CHECK_OPENGL_ERROR();
#endif
}

void COpenGLFramebuffer::blit()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	auto& _ = m_impl.m_state.get();

	glBindFramebuffer(GL_READ_FRAMEBUFFER, _.m_Framebuffer);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	glDrawBuffer(GL_BACK);

	glBlitFramebuffer(
		0, 0, _.m_width, _.m_height, 0, 0, _.m_width, _.m_height,
		GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif
}

void COpenGLFramebuffer::Bind(const FrameBufferBinding& ids)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	glBindFramebuffer(GL_READ_FRAMEBUFFER, ids.readFbId);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, ids.drawFbId);
#endif
}

FrameBufferBinding COpenGLFramebuffer::CurrentBinding()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
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
