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
	: m_width(width), m_height(height), m_win_used(!skip_glut_window)
{
#if MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT

	MRPT_START

	if (m_win_used)
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

	// call after creating the hidden window
	if (!isExtensionSupported("GL_EXT_framebuffer_object"))
		THROW_EXCEPTION("Framebuffer Object extension unsupported");

// In win32 we have to load the pointers to the functions:
#ifdef _WIN32
	glGenFramebuffersEXT =
		(PFNGLGENFRAMEBUFFERSEXTPROC)wglGetProcAddress("glGenFramebuffersEXT");
	glDeleteFramebuffersEXT = (PFNGLDELETEFRAMEBUFFERSEXTPROC)wglGetProcAddress(
		"glDeleteFramebuffersEXT");
	glBindFramebufferEXT =
		(PFNGLBINDFRAMEBUFFEREXTPROC)wglGetProcAddress("glBindFramebufferEXT");
	glFramebufferTexture2DEXT =
		(PFNGLFRAMEBUFFERTEXTURE2DEXTPROC)wglGetProcAddress(
			"glFramebufferTexture2DEXT");

	ASSERT_(glGenFramebuffersEXT != nullptr);
	ASSERT_(glDeleteFramebuffersEXT != nullptr);
	ASSERT_(glBindFramebufferEXT != nullptr);
	ASSERT_(glFramebufferTexture2DEXT != nullptr);
#endif

	// -------------------------------
	// RGB FBO
	// -------------------------------

	// gen the frambuffer object (FBO), similar manner as a texture
	glGenFramebuffersEXT(1, &m_fbo_rgb);

	// change viewport size (in pixels)
	glViewport(0, 0, m_width, m_height);

	// make textures
	glGenTextures(1, &m_texRGB);
	glGenRenderbuffers(1, &m_bufDepth);

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo_rgb);

	// RGB:
	glBindTexture(GL_TEXTURE_2D, m_texRGB);
	CHECK_OPENGL_ERROR();

	glTexImage2D(
		GL_TEXTURE_2D, 0, GL_RGB, m_width, m_height, 0, GL_RGB,
		GL_UNSIGNED_BYTE, nullptr);
	CHECK_OPENGL_ERROR();

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	CHECK_OPENGL_ERROR();

	// bind this texture to the current framebuffer obj. as color_attachement_0
	glFramebufferTexture2DEXT(
		GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texRGB, 0);
	CHECK_OPENGL_ERROR();

	glBindRenderbuffer(GL_RENDERBUFFER, m_bufDepth);
	CHECK_OPENGL_ERROR();

	glRenderbufferStorage(
		GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_width, m_height);
	CHECK_OPENGL_ERROR();

	glFramebufferRenderbuffer(
		GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_bufDepth);
	CHECK_OPENGL_ERROR();

	//
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);  //'unbind' framebuffer
	CHECK_OPENGL_ERROR();

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
	glDeleteFramebuffersEXT(1, &m_fbo_rgb);
	if (m_win_used) glutDestroyWindow(m_win);
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

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindTexture(GL_TEXTURE_2D, 0);
	CHECK_OPENGL_ERROR();
	// glEnable(GL_TEXTURE_2D);
	// CHECK_OPENGL_ERROR();
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo_rgb);
	CHECK_OPENGL_ERROR();

	glEnable(GL_DEPTH_TEST);
	CHECK_OPENGL_ERROR();

	// ---------------------------
	// Render:
	// ---------------------------
	scene.render();

	// ---------------------------
	// RGB
	// ---------------------------
	if (optoutRGB.has_value())
	{
		auto& outRGB = optoutRGB.value().get();
		// resize the outRGB if it is necessary
		if (outRGB.isEmpty() ||
			outRGB.getWidth() != static_cast<size_t>(m_width) ||
			outRGB.getHeight() != static_cast<size_t>(m_height) ||
			outRGB.getChannelCount() != 3)
		{  // resize:
			outRGB.resize(m_width, m_height, mrpt::img::CH_RGB);
		}

		// check the outRGB size
		ASSERT_(!outRGB.isEmpty());
		ASSERT_EQUAL_(outRGB.getWidth(), static_cast<size_t>(m_width));
		ASSERT_EQUAL_(outRGB.getHeight(), static_cast<size_t>(m_height));
		ASSERT_EQUAL_(outRGB.getChannelCount(), 3);

		// TODO NOTE: This should fail if the image has padding bytes. See
		// glPixelStore() etc.
		glReadPixels(
			0, 0, m_width, m_height, GL_BGR_EXT, GL_UNSIGNED_BYTE,
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
		outDepth.resize(m_height, m_width);

		glReadPixels(
			0, 0, m_width, m_height, GL_DEPTH_COMPONENT, GL_FLOAT,
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
		std::vector<float> bufLine(m_width);
		const auto bytesPerLine = sizeof(float) * m_width;
		for (int y = 0; y < m_height / 2; y++)
		{
			const int yFlipped = m_height - 1 - y;
			::memcpy(bufLine.data(), &outDepth(y, 0), bytesPerLine);
			::memcpy(&outDepth(y, 0), &outDepth(yFlipped, 0), bytesPerLine);
			::memcpy(&outDepth(yFlipped, 0), bufLine.data(), bytesPerLine);
		}
	}

	//'unbind' the frambuffer object, so subsequent drawing ops are not
	// drawn into the FBO. '0' means "windowing system provided framebuffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

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

bool CFBORender::isExtensionSupported([
	[maybe_unused]] const std::string& extension)
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START
	for (int index = 0;; index++)
	{
		const auto extName = glGetStringi(GL_EXTENSIONS, index);
		if (!extName) break;

		if (std::string(reinterpret_cast<const char*>(extName)) == extension)
			return true;
	}
	MRPT_END
#endif
	return false;
}
