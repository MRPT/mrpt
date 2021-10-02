/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/CGLFrameBuffer.h>
#include <mrpt/opengl/opengl_api.h>

using namespace mrpt::opengl;

void CGLFramebuffer::init(unsigned int width, unsigned int height, int nSamples)
{
#if MRPT_HAS_OPENGL_GLUT

	m_width = width;
	m_height = height;
	m_Samples = nSamples;

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

	glGenFramebuffers(1, &m_Framebuffer);
	CHECK_OPENGL_ERROR();
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

	release();
#else
	THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}

void CGLFramebuffer::free()
{
#if MRPT_HAS_OPENGL_GLUT
	glDeleteRenderbuffers(1, &m_Color);
	glDeleteRenderbuffers(1, &m_Depth);
	m_Color = m_Depth = 0;
#endif
}

void CGLFramebuffer::bind()
{
#if MRPT_HAS_OPENGL_GLUT
	glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
	if (m_Samples > 1) glEnable(GL_MULTISAMPLE);

#endif
}

void CGLFramebuffer::release()
{
#if MRPT_HAS_OPENGL_GLUT
	if (m_Samples > 1) glDisable(GL_MULTISAMPLE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

#endif
}

void CGLFramebuffer::blit()
{
#if MRPT_HAS_OPENGL_GLUT
	glBindFramebuffer(GL_READ_FRAMEBUFFER, m_Framebuffer);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	glDrawBuffer(GL_BACK);

	glBlitFramebuffer(
		0, 0, m_width, m_height, 0, 0, m_width, m_height,
		GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

#endif
}
