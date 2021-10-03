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
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/opengl_api.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using mrpt::img::CImage;

/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CFBORender::CFBORender(
	unsigned int width, unsigned int height, const bool skip_glut_window)
{
#if MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT

	MRPT_START

	if (!skip_glut_window)
	{
		// check a previous initialization of the GLUT
		if (!glutGet(GLUT_INIT_STATE))
		{
			// create the context (a little trick)
			int argc = 1;
			char* argv[1] = {nullptr};
			glutInit(&argc, argv);
		}

		// create a hidden window
		m_win = glutCreateWindow("CFBORender");
		glutHideWindow();
	}

	// -------------------------------
	// Create frame buffer object:
	// -------------------------------
	m_fb.create(width, height);
	const auto oldFB = m_fb.bind();

	// -------------------------------
	// Create texture:
	// -------------------------------
	glGenTextures(1, &m_texRGB);
	CHECK_OPENGL_ERROR();

	glBindTexture(GL_TEXTURE_2D, m_texRGB);
	CHECK_OPENGL_ERROR();

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	CHECK_OPENGL_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	CHECK_OPENGL_ERROR();

	glTexImage2D(
		GL_TEXTURE_2D, 0, GL_RGB, m_fb.width(), m_fb.height(), 0, GL_RGB,
		GL_UNSIGNED_BYTE, nullptr);
	CHECK_OPENGL_ERROR();

	// bind this texture to the current framebuffer obj. as color_attachement_0
	glFramebufferTexture2D(
		GL_FRAMEBUFFER,	 // DRAW + READ FB
		GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texRGB, 0);
	CHECK_OPENGL_ERROR();

	// unbind:
	COpenGLFramebuffer::Bind(oldFB);

	MRPT_END
#else
	THROW_EXCEPTION("MRPT compiled without OpenCV and/or OpenGL support!!");
#endif
}

CFBORender::~CFBORender()
{
#if MRPT_HAS_OPENGL_GLUT
	// delete the current texture, the framebuffer object and the GLUT window
	glDeleteTextures(1, &m_texRGB);
	m_fb.destroy();
	if (m_win != 0) glutDestroyWindow(m_win);
#endif
}

void CFBORender::internal_render_RGBD(
	[[maybe_unused]] const COpenGLScene& scene,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::img::CImage>& optoutRGB,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::math::CMatrixFloat>&
		optoutDepth)
{
#if MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT

	MRPT_START

	ASSERT_(optoutRGB.has_value() || optoutDepth.has_value());

	const auto oldFBs = m_fb.bind();

	// Get former viewport
	GLint oldViewport[4];
	glGetIntegerv(GL_VIEWPORT, oldViewport);

	// change viewport size (in pixels)
	glViewport(0, 0, m_fb.width(), m_fb.height());
	CHECK_OPENGL_ERROR();

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindTexture(GL_TEXTURE_2D, m_texRGB);
	CHECK_OPENGL_ERROR();

	glEnable(GL_DEPTH_TEST);
	CHECK_OPENGL_ERROR();

	scene.getViewport()->updateMatricesFromCamera();

	// ---------------------------
	// Render:
	// ---------------------------
	scene.getViewport()->render(m_fb.width(), m_fb.height(), 0, 0);

	// ---------------------------
	// RGB
	// ---------------------------
	if (optoutRGB.has_value())
	{
		auto& outRGB = optoutRGB.value().get();
		// resize the outRGB if it is necessary
		if (outRGB.isEmpty() ||
			outRGB.getWidth() != static_cast<size_t>(m_fb.width()) ||
			outRGB.getHeight() != static_cast<size_t>(m_fb.height()) ||
			outRGB.getChannelCount() != 3)
		{  // resize:
			outRGB.resize(m_fb.width(), m_fb.height(), mrpt::img::CH_RGB);
		}

		// check the outRGB size
		ASSERT_(!outRGB.isEmpty());
		ASSERT_EQUAL_(outRGB.getWidth(), static_cast<size_t>(m_fb.width()));
		ASSERT_EQUAL_(outRGB.getHeight(), static_cast<size_t>(m_fb.height()));
		ASSERT_EQUAL_(outRGB.getChannelCount(), 3);

		// TODO NOTE: This should fail if the image has padding bytes. See
		// glPixelStore() etc.
		glReadPixels(
			0, 0, m_fb.width(), m_fb.height(), GL_BGR_EXT, GL_UNSIGNED_BYTE,
			outRGB(0, 0));
		CHECK_OPENGL_ERROR();

		// Flip vertically:
		outRGB.flipVertical();
	}

	// ---------------------------
	// Depth
	// ---------------------------
	if (optoutDepth.has_value())
	{
		auto& outDepth = optoutDepth.value().get();
		outDepth.resize(m_fb.height(), m_fb.width());

		glReadPixels(
			0, 0, m_fb.width(), m_fb.height(), GL_DEPTH_COMPONENT, GL_FLOAT,
			outDepth.data());
		CHECK_OPENGL_ERROR();

		// Transform from OpenGL clip depths into linear distances:
		const auto mats = scene.getViewport()->getRenderMatrices();

		const float zn = mats.getLastClipZNear();
		const float zf = mats.getLastClipZFar();

		// Depth buffer -> linear depth:
		const auto linearDepth = [zn, zf](float depthSample) -> float {
			depthSample = 2.0 * depthSample - 1.0;
			float zLinear = 2.0 * zn * zf / (zf + zn - depthSample * (zf - zn));
			return zLinear;
		};

		for (auto& d : outDepth)
		{
			if (d == 1) d = 0;	// no "echo return"
			else
				d = linearDepth(d);
		}

		// flip lines:
		std::vector<float> bufLine(m_fb.width());
		const auto bytesPerLine = sizeof(float) * m_fb.width();
		for (unsigned int y = 0; y < m_fb.height() / 2; y++)
		{
			const int yFlipped = m_fb.height() - 1 - y;
			::memcpy(bufLine.data(), &outDepth(y, 0), bytesPerLine);
			::memcpy(&outDepth(y, 0), &outDepth(yFlipped, 0), bytesPerLine);
			::memcpy(&outDepth(yFlipped, 0), bufLine.data(), bytesPerLine);
		}
	}

	//'unbind' the frambuffer object, so subsequent drawing ops are not
	// drawn into the FBO.
	COpenGLFramebuffer::Bind(oldFBs);

	// Restore viewport:
	glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);

	MRPT_END
#endif
}

void CFBORender::render_RGB(const COpenGLScene& scene, CImage& outRGB)
{
	internal_render_RGBD(scene, outRGB, std::nullopt);
}

void CFBORender::render_RGBD(
	const COpenGLScene& scene, mrpt::img::CImage& outRGB,
	mrpt::math::CMatrixFloat& outDepth)
{
	internal_render_RGBD(scene, outRGB, outDepth);
}
void CFBORender::render_depth(
	const COpenGLScene& scene, mrpt::math::CMatrixFloat& outDepth)
{
	internal_render_RGBD(scene, std::nullopt, outDepth);
}
