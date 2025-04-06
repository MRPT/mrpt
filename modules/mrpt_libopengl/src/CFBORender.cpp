/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
//
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/OpenGLDepth2LinearLUTs.h>
#include <mrpt/opengl/opengl_api.h>
//
#include <mrpt/config.h>

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

CFBORender::CFBORender(const Parameters& p) : m_params(p)
{
#if HAVE_FBO

  MRPT_START

  if (p.create_EGL_context)
  {
    // clang-format off
		std::vector<EGLint> configAttribs = {
			EGL_SURFACE_TYPE,    EGL_PBUFFER_BIT,
			EGL_BLUE_SIZE,       p.blueSize,
			EGL_GREEN_SIZE,      p.greenSize,
			EGL_RED_SIZE,        p.redSize,
			EGL_DEPTH_SIZE,      p.depthSize
		};
    // clang-format on
    if (p.conformantOpenGLES2)
    {
      configAttribs.push_back(EGL_CONFORMANT);
      configAttribs.push_back(EGL_OPENGL_ES2_BIT);
    }
    if (p.renderableOpenGLES2)
    {
      configAttribs.push_back(EGL_RENDERABLE_TYPE);
      configAttribs.push_back(EGL_OPENGL_ES2_BIT);
    }
    configAttribs.push_back(EGL_NONE);

    constexpr int pbufferWidth = 9;
    constexpr int pbufferHeight = 9;

    static const EGLint pbufferAttribs[] = {
        EGL_WIDTH, pbufferWidth, EGL_HEIGHT, pbufferHeight, EGL_NONE,
    };

    static const int MAX_DEVICES = 32;
    EGLDeviceEXT eglDevs[MAX_DEVICES];
    EGLint numDevices = 0;

    PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
        (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
    ASSERT_(eglQueryDevicesEXT);

    eglQueryDevicesEXT(MAX_DEVICES, eglDevs, &numDevices);

    if (MRPT_FBORENDER_SHOW_DEVICES)
    {
      printf("[mrpt EGL] Detected %d devices\n", numDevices);

      PFNEGLQUERYDEVICESTRINGEXTPROC eglQueryDeviceStringEXT =
          (PFNEGLQUERYDEVICESTRINGEXTPROC)eglGetProcAddress("eglQueryDeviceStringEXT");
      ASSERT_(eglQueryDeviceStringEXT);

      for (int i = 0; i < numDevices; i++)
      {
        const char* devExts = eglQueryDeviceStringEXT(eglDevs[i], EGL_EXTENSIONS);

        printf("[mrpt EGL] Device #%i. Extensions: %s\n", i, devExts ? devExts : "(None)");
      }
    }

    ASSERT_LT_(p.deviceIndexToUse, numDevices);

    PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");

    m_eglDpy = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, eglDevs[p.deviceIndexToUse], 0);

    // 1. Initialize EGL
    if (m_eglDpy == EGL_NO_DISPLAY)
    {
      THROW_EXCEPTION("Failed to get EGL display");
    }

    EGLint major, minor;

    if (eglInitialize(m_eglDpy, &major, &minor) == EGL_FALSE)
    {
      THROW_EXCEPTION_FMT("Failed to initialize EGL display: %x\n", eglGetError());
    }

    // printf("[mrpt EGL] version: %i.%i\n", major, minor);

    // 2. Select an appropriate configuration
    EGLint numConfigs;

    eglChooseConfig(m_eglDpy, configAttribs.data(), &m_eglCfg, 1, &numConfigs);
    if (numConfigs != 1)
    {
      THROW_EXCEPTION_FMT("Failed to choose exactly 1 config, chose %d\n", numConfigs);
    }

    // 3. Create a surface
    m_eglSurf = eglCreatePbufferSurface(m_eglDpy, m_eglCfg, pbufferAttribs);

    // 4. Bind the API
    if (!eglBindAPI(p.bindOpenGLES_API ? EGL_OPENGL_ES_API : EGL_OPENGL_API))
    {
      // error:
      THROW_EXCEPTION("no opengl api in egl");
    }

    // 5. Create a context and make it current

    // clang-format off
		std::vector<EGLint> ctxAttribs = {
			EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
			EGL_CONTEXT_OPENGL_DEBUG, p.contextDebug ? EGL_TRUE: EGL_FALSE
		};
    // clang-format on
    if (p.contextMajorVersion != 0 && p.contextMinorVersion != 0)
    {
      ctxAttribs.push_back(EGL_CONTEXT_MAJOR_VERSION);
      ctxAttribs.push_back(p.contextMajorVersion);
      ctxAttribs.push_back(EGL_CONTEXT_MINOR_VERSION);
      ctxAttribs.push_back(p.contextMinorVersion);
    }
    ctxAttribs.push_back(EGL_NONE);

    m_eglContext = eglCreateContext(m_eglDpy, m_eglCfg, EGL_NO_CONTEXT, ctxAttribs.data());

    ASSERT_(m_eglContext != EGL_NO_CONTEXT);

    eglMakeCurrent(m_eglDpy, m_eglSurf, m_eglSurf, m_eglContext);
  }

  // -------------------------------
  // Create frame buffer object:
  // -------------------------------
  m_fb.create(p.width, p.height);  // TODO: Multisample doesn't work...
  const auto oldFB = m_fb.bind();

  // -------------------------------
  // Create texture:
  // -------------------------------
  glGenTextures(1, &m_texRGB);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glBindTexture(GL_TEXTURE_2D, m_texRGB);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  CHECK_OPENGL_ERROR_IN_DEBUG();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glTexImage2D(
      GL_TEXTURE_2D, 0, GL_RGB, m_fb.width(), m_fb.height(), 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // bind this texture to the current framebuffer obj. as color_attachement_0
  glFramebufferTexture2D(
      GL_FRAMEBUFFER,  // DRAW + READ FB
      GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texRGB, 0);
  CHECK_OPENGL_ERROR();

  // unbind:
  FrameBuffer::Bind(oldFB);

  MRPT_END
#else
  THROW_EXCEPTION("This class requires MRPT built with: OpenCV; OpenGL, and EGL.");
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
    [[maybe_unused]] const Scene& scene,
    [[maybe_unused]] const mrpt::optional_ref<mrpt::img::CImage>& optoutRGB,
    [[maybe_unused]] const mrpt::optional_ref<mrpt::math::CMatrixFloat>& optoutDepth)
{
#if HAVE_FBO

  MRPT_START

  ASSERT_(optoutRGB.has_value() || optoutDepth.has_value());

#ifdef FBO_PROFILER
  thread_local mrpt::system::CTimeLogger profiler(true, "FBO_RENDERER");

  using namespace std::string_literals;
  const std::string sSec = mrpt::format("%ux%u_", m_fb.width(), m_fb.height());

  auto tleR = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".prepAndRender"s);
#endif

  const auto oldFBs = m_fb.bind();

  // change viewport size (in pixels)
  // glViewport(0, 0, m_fb.width(), m_fb.height());
  // CHECK_OPENGL_ERROR_IN_DEBUG();

  // bind the framebuffer, fbo, so operations will now occur on it
  glBindTexture(GL_TEXTURE_2D, m_texRGB);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glEnable(GL_DEPTH_TEST);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // ---------------------------
  // Render:
  // ---------------------------
  for (const auto& viewport : scene.viewports())
    viewport->render(m_fb.width(), m_fb.height(), 0, 0, &m_renderFromCamera);

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
    if (outRGB.isEmpty() || outRGB.getWidth() != static_cast<size_t>(m_fb.width()) ||
        outRGB.getHeight() != static_cast<size_t>(m_fb.height()) || outRGB.getChannelCount() != 3)
    {  // resize:
      outRGB.resize(m_fb.width(), m_fb.height(), mrpt::img::CH_RGB);
    }

    // check the outRGB size
    ASSERT_(!outRGB.isEmpty());
    ASSERT_EQUAL_(outRGB.getWidth(), static_cast<size_t>(m_fb.width()));
    ASSERT_EQUAL_(outRGB.getHeight(), static_cast<size_t>(m_fb.height()));
    ASSERT_EQUAL_(outRGB.getChannelCount(), 3);

#ifdef FBO_PROFILER
    auto tle1 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".glReadPixels_rgb"s);
#endif

    // TODO NOTE: This should fail if the image has padding bytes. See
    // glPixelStore() etc.
    glReadPixels(0, 0, m_fb.width(), m_fb.height(), GL_BGR_EXT, GL_UNSIGNED_BYTE, outRGB(0, 0));
    CHECK_OPENGL_ERROR();

#ifdef FBO_PROFILER
    tle1.stop();
    auto tle2 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".flip_rgb"s);
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
    auto tle1 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".glReadPixels_float"s);
#endif

    outDepth.resize(m_fb.height(), m_fb.width());

    glReadPixels(0, 0, m_fb.width(), m_fb.height(), GL_DEPTH_COMPONENT, GL_FLOAT, outDepth.data());
    CHECK_OPENGL_ERROR();

#ifdef FBO_PROFILER
    tle1.stop();
#endif

    using depth_lut_t = OpenGLDepth2LinearLUTs<18>;

    // Dont create LUT if we are not about to use it anyway:
    if (!m_params.raw_depth)
    {
      // Transform from OpenGL clip depths into linear distances:
      const auto mats = scene.getViewport()->getRenderMatrices();
      const float zn = mats.getLastClipZNear();
      const float zf = mats.getLastClipZFar();

      const depth_lut_t::lut_t* lut = nullptr;

#if !defined(FBO_USE_LUT)
      const bool do_use_lut = false;
#else
      bool do_use_lut = MRPT_FBORENDER_USE_LUT;
      // dont use LUT for really small image areas:
      if (m_fb.height() * m_fb.width() < 10000)
      {
        do_use_lut = false;
      }
#endif

      if (do_use_lut)
      {
#ifdef FBO_PROFILER
        auto tle_lut = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".get_lut"s);
#endif
        lut = &depth_lut_t::Instance().lut_from_zn_zf(zn, zf);
      }

#ifdef FBO_PROFILER
      auto tle2 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".linear"s);
#endif

      if (!do_use_lut)
      {
        // Depth buffer -> linear depth:
        const auto linearDepth = [zn, zf](float depthSample) -> float
        {
          depthSample = 2.0 * depthSample - 1.0;
          float zLinear = 2.0 * zn * zf / (zf + zn - depthSample * (zf - zn));
          return zLinear;
        };
        for (auto& d : outDepth)
        {
          if (d == 1)
            d = 0;  // no "echo return"
          else
            d = linearDepth(d);
        }
      }
      else
      {
        // map d in [-1.0f,+1.0f] ==> real depth values:
        for (auto& d : outDepth)
        {
          d = (*lut)[(d + 1.0f) * (depth_lut_t::NUM_ENTRIES - 1) / 2];
        }
      }
#ifdef FBO_PROFILER
      tle2.stop();
#endif
    }

#ifdef FBO_PROFILER
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
  FrameBuffer::Bind(oldFBs);

  MRPT_END
#endif
}

void CFBORender::render_RGB(const Scene& scene, CImage& outRGB)
{
  internal_render_RGBD(scene, outRGB, std::nullopt);
}

void CFBORender::render_RGBD(
    const Scene& scene, mrpt::img::CImage& outRGB, mrpt::math::CMatrixFloat& outDepth)
{
  internal_render_RGBD(scene, outRGB, outDepth);
}
void CFBORender::render_depth(const Scene& scene, mrpt::math::CMatrixFloat& outDepth)
{
  internal_render_RGBD(scene, std::nullopt, outDepth);
}
