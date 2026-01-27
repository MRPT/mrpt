/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/get_env.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/OpenGLDepth2LinearLUTs.h>
#include <mrpt/opengl/config.h>
#include <mrpt/opengl/opengl_api.h>

#define FBO_USE_LUT
// #define FBO_PROFILER

#ifdef FBO_PROFILER
#include <mrpt/system/CTimeLogger.h>
#endif

#if MRPT_HAS_EGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

#define HAVE_FBO (MRPT_HAS_OPENGL_GLUT && MRPT_HAS_EGL)

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using mrpt::img::CImage;

namespace
{
const thread_local bool MRPT_FBORENDER_SHOW_DEVICES =
    mrpt::get_env<bool>("MRPT_FBORENDER_SHOW_DEVICES");

const thread_local bool MRPT_FBORENDER_USE_LUT =
    mrpt::get_env<bool>("MRPT_FBORENDER_USE_LUT", true);
}  // namespace

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

    auto eglQueryDevicesEXT = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(  // NOLINT
        eglGetProcAddress("eglQueryDevicesEXT"));
    ASSERT_(eglQueryDevicesEXT);

    eglQueryDevicesEXT(MAX_DEVICES, eglDevs, &numDevices);

    if (MRPT_FBORENDER_SHOW_DEVICES)
    {
      printf("[mrpt EGL] Detected %d devices\n", numDevices);

      auto eglQueryDeviceStringEXT = reinterpret_cast<PFNEGLQUERYDEVICESTRINGEXTPROC>(  // NOLINT
          eglGetProcAddress("eglQueryDeviceStringEXT"));
      ASSERT_(eglQueryDeviceStringEXT);

      for (int i = 0; i < numDevices; i++)
      {
        const char* devExts = eglQueryDeviceStringEXT(eglDevs[i], EGL_EXTENSIONS);
        printf(
            "[mrpt EGL] Device #%i. Extensions: %s\n", i, devExts != nullptr ? devExts : "(None)");
      }
    }

    ASSERT_LT_(p.deviceIndexToUse, numDevices);

    auto eglGetPlatformDisplayEXT = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(  // NOLINT
        eglGetProcAddress("eglGetPlatformDisplayEXT"));
    ASSERT_(eglGetPlatformDisplayEXT);

    m_eglDpy = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, eglDevs[p.deviceIndexToUse], 0);

    // 1. Initialize EGL
    if (m_eglDpy == EGL_NO_DISPLAY)
    {
      THROW_EXCEPTION("Failed to get EGL display");
    }

    EGLint major = 0;
    EGLint minor = 0;

    if (eglInitialize(m_eglDpy, &major, &minor) == EGL_FALSE)
    {
      THROW_EXCEPTION_FMT("Failed to initialize EGL display: %x\n", eglGetError());
    }

    // 2. Select an appropriate configuration
    EGLint numConfigs = 0;

    eglChooseConfig(m_eglDpy, configAttribs.data(), &m_eglCfg, 1, &numConfigs);
    if (numConfigs != 1)
    {
      THROW_EXCEPTION_FMT("Failed to choose exactly 1 config, chose %d\n", numConfigs);
    }

    // 3. Create a surface
    m_eglSurf = eglCreatePbufferSurface(m_eglDpy, m_eglCfg, pbufferAttribs);

    // 4. Bind the API
    if (0 == eglBindAPI(p.bindOpenGLES_API ? EGL_OPENGL_ES_API : EGL_OPENGL_API))
    {
      THROW_EXCEPTION("no opengl api in egl");
    }

    // 5. Create a context and make it current
    // clang-format off
    std::vector<EGLint> ctxAttribs = {
        EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
        EGL_CONTEXT_OPENGL_DEBUG, p.contextDebug ? EGL_TRUE : EGL_FALSE
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

    if (eglMakeCurrent(m_eglDpy, m_eglSurf, m_eglSurf, m_eglContext) == EGL_FALSE)
    {
      EGLint err = eglGetError();
      THROW_EXCEPTION_FMT("eglMakeCurrent failed: 0x%X", err);
    }
  }

  // -------------------------------
  // Create frame buffer object:
  // -------------------------------
  m_fb.create(p.width, p.height);
  const auto oldFB = m_fb.bind();

  // -------------------------------
  // Create texture:
  // -------------------------------
  glGenTextures(1, &m_texRGB);
  CHECK_OPENGL_ERROR();

  glBindTexture(GL_TEXTURE_2D, m_texRGB);
  CHECK_OPENGL_ERROR();

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  CHECK_OPENGL_ERROR_IN_DEBUG();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glTexImage2D(
      GL_TEXTURE_2D, 0, GL_RGB, m_fb.width(), m_fb.height(), 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Bind this texture to the current framebuffer obj. as color_attachment_0
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texRGB, 0);
  CHECK_OPENGL_ERROR();

  // Unbind
  FrameBuffer::Bind(oldFB);

  MRPT_END
#else
  THROW_EXCEPTION("This class requires MRPT built with: OpenCV, OpenGL, and EGL.");
#endif
}

CFBORender::~CFBORender()
{
#if HAVE_FBO
  // Clear compiled scene first (releases GPU resources)
  m_compiledScene.reset();

  // Delete the current texture and framebuffer object
  if (m_texRGB != 0)
  {
    glDeleteTextures(1, &m_texRGB);
  }
  m_fb.destroy();

  // Terminate EGL when finished
  if (m_eglDpy)
  {
    eglTerminate(m_eglDpy);
  }
#endif
}

void CFBORender::ensureCompiledScene(const mrpt::viz::Scene& scene)
{
  // Check if we need to create or recreate the compiled scene
  auto scenePtr = scene.shared_from_this();
  auto lastScenePtr = m_lastScene.lock();

  if (!m_compiledScene || lastScenePtr.get() != scenePtr.get())
  {
    // Different scene or first time - create new compiled scene
    m_compiledScene = std::make_unique<CompiledScene>();
    m_compiledScene->compile(scene);
    m_lastScene = scenePtr;
  }
  else
  {
    // Same scene - just update if needed
    m_compiledScene->updateIfNeeded();
  }
}

void CFBORender::internal_render_RGBD(
    [[maybe_unused]] const mrpt::viz::Scene& scene,
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

  // Ensure compiled scene is ready
  ensureCompiledScene(scene);

  // Apply camera override if set
  if (m_useCameraOverride)
  {
    auto mainVp = m_compiledScene->getViewport("main");
    if (mainVp)
    {
      mainVp->updateCamera(m_cameraOverride);
    }
  }

  // Bind the framebuffer
  const auto oldFBs = m_fb.bind();

  glBindTexture(GL_TEXTURE_2D, m_texRGB);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glEnable(GL_DEPTH_TEST);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // ---------------------------
  // Render using CompiledScene
  // ---------------------------
  m_compiledScene->render(
      static_cast<int>(m_fb.width()), static_cast<int>(m_fb.height()),
      0,  // offsetX
      0   // offsetY
  );

#ifdef FBO_PROFILER
  tleR.stop();
#endif

  // ---------------------------
  // RGB output
  // ---------------------------
  if (optoutRGB.has_value())
  {
    auto& outRGB = optoutRGB.value().get();

    // Resize the outRGB if necessary
    if (outRGB.isEmpty() || outRGB.getWidth() != static_cast<size_t>(m_fb.width()) ||
        outRGB.getHeight() != static_cast<size_t>(m_fb.height()) || outRGB.channels() != 3)
    {
      outRGB.resize(m_fb.width(), m_fb.height(), mrpt::img::CH_RGB);
    }

    ASSERT_(!outRGB.isEmpty());
    ASSERT_EQUAL_(outRGB.getWidth(), static_cast<size_t>(m_fb.width()));
    ASSERT_EQUAL_(outRGB.getHeight(), static_cast<size_t>(m_fb.height()));
    ASSERT_EQUAL_(outRGB.channels(), 3);

#ifdef FBO_PROFILER
    auto tle1 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".glReadPixels_rgb"s);
#endif

    glReadPixels(
        0, 0, m_fb.width(), m_fb.height(), GL_BGR_EXT, GL_UNSIGNED_BYTE,
        outRGB.ptrLine<uint8_t>(0));
    CHECK_OPENGL_ERROR();

#ifdef FBO_PROFILER
    tle1.stop();
    auto tle2 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".flip_rgb"s);
#endif

    // Flip vertically (OpenGL has origin at bottom-left)
    outRGB.flipVertical();
  }

  // ---------------------------
  // Depth output
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

    // Convert to linear depth if requested
    if (!m_params.raw_depth)
    {
      // Get clip planes from the compiled viewport's render matrices
      auto mainVp = m_compiledScene->getViewport("main");
      float zn = 0.01f;
      float zf = 1000.0f;

      if (mainVp)
      {
        const auto& mats = mainVp->getRenderMatrices();
        zn = mats.getLastClipZNear();
        zf = mats.getLastClipZFar();
      }

      convertDepthToLinear(outDepth, zn, zf);
    }

#ifdef FBO_PROFILER
    auto tle3 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".flip"s);
#endif

    // Flip lines (OpenGL has origin at bottom-left)
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

  // Unbind the framebuffer object
  FrameBuffer::Bind(oldFBs);

  MRPT_END
#endif
}

void CFBORender::convertDepthToLinear(mrpt::math::CMatrixFloat& depth, float zn, float zf) const
{
  using depth_lut_t = OpenGLDepth2LinearLUTs<18>;

  const depth_lut_t::lut_t* lut = nullptr;

#if !defined(FBO_USE_LUT)
  const bool do_use_lut = false;
#else
  bool do_use_lut = MRPT_FBORENDER_USE_LUT;
  // Don't use LUT for really small image areas
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
    // Depth buffer -> linear depth (no LUT)
    const auto linearDepth = [zn, zf](float depthSample) -> float
    {
      depthSample = 2.0f * depthSample - 1.0f;
      float zLinear = 2.0f * zn * zf / (zf + zn - depthSample * (zf - zn));
      return zLinear;
    };

    for (auto& d : depth)
    {
      if (d == 1.0f)
      {
        d = 0.0f;  // No "echo return" - max depth
      }
      else
      {
        d = linearDepth(d);
      }
    }
  }
  else
  {
    // Map d in [0,1] ==> real depth values using LUT
    for (auto& d : depth)
    {
      d = (*lut)[static_cast<size_t>((d + 1.0f) * (depth_lut_t::NUM_ENTRIES - 1) / 2)];
    }
  }
}

void CFBORender::render_RGB(const mrpt::viz::Scene& scene, CImage& outRGB)
{
  internal_render_RGBD(scene, outRGB, std::nullopt);
}

void CFBORender::render_RGBD(
    const mrpt::viz::Scene& scene, mrpt::img::CImage& outRGB, mrpt::math::CMatrixFloat& outDepth)
{
  internal_render_RGBD(scene, outRGB, outDepth);
}

void CFBORender::render_depth(const mrpt::viz::Scene& scene, mrpt::math::CMatrixFloat& outDepth)
{
  internal_render_RGBD(scene, std::nullopt, outDepth);
}