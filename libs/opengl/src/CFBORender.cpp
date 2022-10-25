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
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/opengl_api.h>
//
#include <mrpt/config.h>

#include <mutex>
#include <unordered_map>

#define FBO_USE_LUT
//#define FBO_PROFILER

#ifdef FBO_PROFILER
#include <mrpt/system/CTimeLogger.h>
#endif

#if MRPT_HAS_EGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

#define HAVE_FBO (MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT && MRPT_HAS_EGL)

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using mrpt::img::CImage;

const thread_local bool MRPT_FBORENDER_SHOW_DEVICES =
	mrpt::get_env<bool>("MRPT_FBORENDER_SHOW_DEVICES");

const thread_local bool MRPT_FBORENDER_USE_LUT =
	mrpt::get_env<bool>("MRPT_FBORENDER_USE_LUT", true);

class OpenGLDepth2Linear_LUTs
{
   public:
	constexpr static std::size_t NUM_ENTRIES = 1 << 18;

	static OpenGLDepth2Linear_LUTs& Instance()
	{
		thread_local OpenGLDepth2Linear_LUTs lut;
		return lut;
	}

	using lut_t = std::vector<float>;

	lut_t& lut_from_zn_zf(float zn, float zf)
	{
		const auto p = std::pair<float, float>(zn, zf);
		// reuse?
		if (auto it = m_pool.find(p); it != m_pool.end()) return it->second;

		// create new:
		auto& lut = m_pool[p];
		lut.resize(NUM_ENTRIES);

		const auto linearDepth = [zn, zf](float depthSample) -> float {
			if (depthSample == 1.0f) return 0.0f;  // no echo

			depthSample = 2.0f * depthSample - 1.0f;
			float zLinear =
				2.0f * zn * zf / (zf + zn - depthSample * (zf - zn));
			return zLinear;
		};

		for (size_t i = 0; i < NUM_ENTRIES; i++)
		{
			float f = -1.0f + 2.0f * static_cast<float>(i) / (NUM_ENTRIES - 1);
			lut.at(i) = linearDepth(f);
		}

		return lut;
	}

   private:
	struct MyHash
	{
		template <typename T>
		std::size_t operator()(const std::pair<T, T>& x) const
		{
			return std::hash<T>()(x.first) ^ std::hash<T>()(x.second);
		}
	};

	std::unordered_map<std::pair<float, float>, lut_t, MyHash> m_pool;
};

/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CFBORender::CFBORender(
	unsigned int width, unsigned int height, const bool skip_create_egl_context)
{
#if HAVE_FBO

	MRPT_START

	if (!skip_create_egl_context)
	{
		static const EGLint configAttribs[] = {
			EGL_SURFACE_TYPE,
			EGL_PBUFFER_BIT,
			EGL_BLUE_SIZE,
			8,
			EGL_GREEN_SIZE,
			8,
			EGL_RED_SIZE,
			8,
			EGL_DEPTH_SIZE,
			8,
			EGL_RENDERABLE_TYPE,
			EGL_OPENGL_BIT,
			EGL_NONE};

		constexpr int pbufferWidth = 9;
		constexpr int pbufferHeight = 9;

		static const EGLint pbufferAttribs[] = {
			EGL_WIDTH, pbufferWidth, EGL_HEIGHT, pbufferHeight, EGL_NONE,
		};

		static const int MAX_DEVICES = 4;
		EGLDeviceEXT eglDevs[MAX_DEVICES];
		EGLint numDevices;

		PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
			(PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");

		eglQueryDevicesEXT(MAX_DEVICES, eglDevs, &numDevices);

		if (MRPT_FBORENDER_SHOW_DEVICES)
			printf("Detected %d devices\n", numDevices);

		PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
			(PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress(
				"eglGetPlatformDisplayEXT");

		EGLDisplay eglDpy =
			eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, eglDevs[0], 0);

		// 1. Initialize EGL
		if (eglDpy == EGL_NO_DISPLAY)
		{ THROW_EXCEPTION("Failed to get EGL display"); }

		EGLint major, minor;

		if (eglInitialize(eglDpy, &major, &minor) == EGL_FALSE)
		{
			THROW_EXCEPTION_FMT(
				"Failed to initialize EGL display: %x\n", eglGetError());
		}

		// 2. Select an appropriate configuration
		EGLint numConfigs;
		EGLConfig eglCfg;

		eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs);
		if (numConfigs != 1)
		{
			THROW_EXCEPTION_FMT(
				"Failed to choose exactly 1 config, chose %d\n", numConfigs);
		}

		// 3. Create a surface
		EGLSurface eglSurf =
			eglCreatePbufferSurface(eglDpy, eglCfg, pbufferAttribs);

		// 4. Bind the API
		if (!eglBindAPI(EGL_OPENGL_API))
		{ THROW_EXCEPTION("no opengl api in egl"); }

		// 5. Create a context and make it current
		eglBindAPI(EGL_OPENGL_API);

		EGLContext eglCtx =
			eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, NULL);

		eglMakeCurrent(eglDpy, eglSurf, eglSurf, eglCtx);

		m_eglDpy = eglDpy;
	}

	// -------------------------------
	// Create frame buffer object:
	// -------------------------------
	m_fb.create(width, height);	 // TODO: Multisample doesn't work...
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
	THROW_EXCEPTION(
		"This class requires MRPT built with: OpenCV; OpenGL, and EGL.");
#endif
}

CFBORender::~CFBORender()
{
#if HAVE_FBO
	// delete the current texture, the framebuffer object and the EGL context
	glDeleteTextures(1, &m_texRGB);
	m_fb.destroy();
	// Terminate EGL when finished:
	if (m_eglDpy) eglTerminate(m_eglDpy);
#endif
}

void CFBORender::internal_render_RGBD(
	[[maybe_unused]] const COpenGLScene& scene,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::img::CImage>& optoutRGB,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::math::CMatrixFloat>&
		optoutDepth)
{
#if HAVE_FBO

	MRPT_START

	ASSERT_(optoutRGB.has_value() || optoutDepth.has_value());

#ifdef FBO_PROFILER
	thread_local mrpt::system::CTimeLogger profiler(true, "FBO_RENDERER");

	using namespace std::string_literals;
	const std::string sSec =
		mrpt::format("%ux%u_", m_fb.width(), m_fb.height());

	auto tleR =
		mrpt::system::CTimeLoggerEntry(profiler, sSec + ".prepAndRender"s);
#endif

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

	// ---------------------------
	// Render:
	// ---------------------------
	for (const auto& viewport : scene.viewports())
		viewport->render(m_fb.width(), m_fb.height(), 0, 0);

#ifdef FBO_PROFILER
	tleR.stop();
#endif

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

#ifdef FBO_PROFILER
		auto tle1 = mrpt::system::CTimeLoggerEntry(
			profiler, sSec + ".glReadPixels_rgb"s);
#endif

		// TODO NOTE: This should fail if the image has padding bytes. See
		// glPixelStore() etc.
		glReadPixels(
			0, 0, m_fb.width(), m_fb.height(), GL_BGR_EXT, GL_UNSIGNED_BYTE,
			outRGB(0, 0));
		CHECK_OPENGL_ERROR();

#ifdef FBO_PROFILER
		tle1.stop();
		auto tle2 =
			mrpt::system::CTimeLoggerEntry(profiler, sSec + ".flip_rgb"s);
#endif

		// Flip vertically:
		outRGB.flipVertical();
	}

	// ---------------------------
	// Depth
	// ---------------------------
	if (optoutDepth.has_value())
	{
		auto& outDepth = optoutDepth.value().get();

#ifdef FBO_PROFILER
		auto tle1 = mrpt::system::CTimeLoggerEntry(
			profiler, sSec + ".glReadPixels_float"s);
#endif

		outDepth.resize(m_fb.height(), m_fb.width());

		glReadPixels(
			0, 0, m_fb.width(), m_fb.height(), GL_DEPTH_COMPONENT, GL_FLOAT,
			outDepth.data());
		CHECK_OPENGL_ERROR();

#ifdef FBO_PROFILER
		tle1.stop();
#endif

		// Transform from OpenGL clip depths into linear distances:
		const auto mats = scene.getViewport()->getRenderMatrices();
		const float zn = mats.getLastClipZNear();
		const float zf = mats.getLastClipZFar();

		const OpenGLDepth2Linear_LUTs::lut_t* lut = nullptr;

#if !defined(FBO_USE_LUT)
		const bool do_use_lut = false;
#else
		bool do_use_lut = MRPT_FBORENDER_USE_LUT;
		// dont use LUT for really small image areas:
		if (m_fb.height() * m_fb.width() < 10000) { do_use_lut = false; }
#endif

		if (do_use_lut)
			lut = &OpenGLDepth2Linear_LUTs::Instance().lut_from_zn_zf(zn, zf);

#ifdef FBO_PROFILER
		auto tle2 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".linear"s);
#endif

		if (!do_use_lut)
		{
			// Depth buffer -> linear depth:
			const auto linearDepth = [zn, zf](float depthSample) -> float {
				depthSample = 2.0 * depthSample - 1.0;
				float zLinear =
					2.0 * zn * zf / (zf + zn - depthSample * (zf - zn));
				return zLinear;
			};
			for (auto& d : outDepth)
			{
				if (d == 1) d = 0;	// no "echo return"
				else
					d = linearDepth(d);
			}
		}
		else
		{
			// map d in [-1.0f,+1.0f] ==> real depth values:
			for (auto& d : outDepth)
			{
				d = (*lut)
					[(d + 1.0f) * (OpenGLDepth2Linear_LUTs::NUM_ENTRIES - 1) /
					 2];
			}
		}

#ifdef FBO_PROFILER
		tle2.stop();
		auto tle3 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".flip"s);
#endif

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
#ifdef FBO_PROFILER
		tle3.stop();
#endif
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
