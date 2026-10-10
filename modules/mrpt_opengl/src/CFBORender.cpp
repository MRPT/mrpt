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
#include <mrpt/img/camera_geometry.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/OpenGLDepth2LinearLUTs.h>
#include <mrpt/opengl/config.h>
#include <mrpt/opengl/opengl_api.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

#define FBO_USE_LUT
// #define FBO_PROFILER

#ifdef FBO_PROFILER
#include <mrpt/system/CTimeLogger.h>
#endif

#if MRPT_HAS_EGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

#define HAVE_FBO (MRPT_HAS_OPENGL && MRPT_HAS_EGL)

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

#if HAVE_FBO
/** Creates an RGB texture and attaches it as the color buffer of the currently
 * bound framebuffer. GL_SRGB8, so GL_FRAMEBUFFER_SRGB can encode linear to
 * sRGB on write. */
unsigned int createColorTextureForFBO(unsigned int width, unsigned int height)
{
  unsigned int tex = 0;
  glGenTextures(1, &tex);
  CHECK_OPENGL_ERROR();

  glBindTexture(GL_TEXTURE_2D, tex);
  CHECK_OPENGL_ERROR();

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
  CHECK_OPENGL_ERROR();

  return tex;
}
#endif
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
  m_texRGB = createColorTextureForFBO(m_fb.width(), m_fb.height());

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

  // Delete the current textures and framebuffer objects
  for (unsigned int tex : {m_texRGB, m_texScene, m_texLUT})
  {
    if (tex != 0)
    {
      glDeleteTextures(1, &tex);
    }
  }
  if (m_postVAO != 0)
  {
    glDeleteVertexArrays(1, &m_postVAO);
  }
  m_postProgram.reset();
  m_fbScene.destroy();
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
  // Check if we need to create or recreate the compiled scene. The Scene may
  // be stack-allocated (no shared_from_this()), so a new one may reuse the
  // address of a previous one: isCompiledFrom() also checks its viewports.
  if (!m_compiledScene || !m_compiledScene->isCompiledFrom(scene))
  {
    // Different scene or first time - create new compiled scene
    m_compiledScene = std::make_unique<CompiledScene>();
    m_compiledScene->setAutoUpdate(false);  // We manage updates explicitly
    m_compiledScene->compile(scene);
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

  // If we own an EGL context, make it current.
  // Otherwise, assume the caller (e.g. a GUI thread) already has a valid GL context.
  if (m_eglContext != EGL_NO_CONTEXT)
  {
    if (eglGetCurrentContext() != m_eglContext)
    {
      if (eglMakeCurrent(m_eglDpy, m_eglSurf, m_eglSurf, m_eglContext) == EGL_FALSE)
      {
        EGLint err = eglGetError();
        THROW_EXCEPTION_FMT("eglMakeCurrent failed: 0x%X", err);
      }
    }
  }

  // Clear any stale errors after context switch
  clearOpenGLErrors();

  // Ensure compiled scene is ready
  ensureCompiledScene(scene);

  ASSERTMSG_(
      !m_distortedCamera || !optoutDepth.has_value(),
      "Lens distortion is only supported by render_RGB(), not by render_RGBD() or "
      "render_depth()");

  // With lens distortion or noise, the scene is rendered into m_fbScene, then
  // post-processed into m_fb:
  const bool postProcess = optoutRGB.has_value() && needsPostProcessing();
  if (postProcess)
  {
    preparePostProcessing();
  }
  FrameBuffer& sceneFB = postProcess ? m_fbScene : m_fb;

  // Camera: the override, if set. With lens distortion, only its pose is
  // kept, with the intrinsics of the enlarged ideal camera:
  if (auto mainVp = m_compiledScene->getViewport("main"); mainVp)
  {
    if (m_distortedCamera)
    {
      mrpt::viz::CCamera cam = m_cameraOverride;
      if (!m_useCameraOverride)
      {
        if (const auto vizVp = scene.getViewport("main"); vizVp)
        {
          cam = vizVp->resolveActiveCamera();
        }
      }
      cam.setProjectiveFromPinhole(m_idealCamera);
      mainVp->updateCamera(cam);
      m_viewportCameraReplaced = true;
    }
    else if (m_useCameraOverride)
    {
      mainVp->updateCamera(m_cameraOverride);
    }
    else if (m_viewportCameraReplaced)
    {
      // Lens distortion was cleared: restore the scene camera.
      if (const auto vizVp = scene.getViewport("main"); vizVp)
      {
        mainVp->updateCamera(vizVp->resolveActiveCamera());
      }
      m_viewportCameraReplaced = false;
    }
  }

  // Bind the framebuffer
  const auto oldFBs = sceneFB.bind();

  glBindTexture(GL_TEXTURE_2D, postProcess ? m_texScene : m_texRGB);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glEnable(GL_DEPTH_TEST);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Enable y-flip projection to avoid having to flip the final images:
  for (const auto& [_, viewport] : m_compiledScene->getViewports())
  {
    viewport->flipVerticalProjection(true);
  }

  // ---------------------------
  // Render using CompiledScene
  // ---------------------------
  m_compiledScene->render(
      static_cast<int>(sceneFB.width()), static_cast<int>(sceneFB.height()),
      0,  // offsetX
      0   // offsetY
  );

  for (const auto& [_, viewport] : m_compiledScene->getViewports())
  {
    viewport->flipVerticalProjection(false);
  }

#ifdef FBO_PROFILER
  tleR.stop();
#endif

  // ---------------------------
  // Depth output
  // ---------------------------
  if (optoutDepth.has_value())
  {
    auto& outDepth = optoutDepth.value().get();

#ifdef FBO_PROFILER
    auto tle1 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".glReadPixels_float"s);
#endif

    outDepth.resize(sceneFB.height(), sceneFB.width());

    glReadPixels(
        0, 0, sceneFB.width(), sceneFB.height(), GL_DEPTH_COMPONENT, GL_FLOAT, outDepth.data());
    CHECK_OPENGL_ERROR();

    // No manual flip needed: flipVerticalProjection(true) already handles
    // the vertical orientation (same as for RGB).

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
      bool isProjective = true;

      if (mainVp)
      {
        const auto& mats = mainVp->getRenderMatrices();
        zn = mats.getLastClipZNear();
        zf = mats.getLastClipZFar();
        isProjective = mats.is_projective || mats.pinhole_model.has_value();
      }

      convertDepthToLinear(outDepth, zn, zf, isProjective);
    }
  }

  // ---------------------------
  // RGB output
  // ---------------------------
  if (optoutRGB.has_value())
  {
    auto& outRGB = optoutRGB.value().get();

    if (postProcess)
    {
#ifdef FBO_PROFILER
      auto tlePost = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".postProcess"s);
#endif
      m_fb.bind();
      renderPostProcessing();
    }

    // Resize the outRGB if necessary
    if (outRGB.isEmpty() || outRGB.getWidth() != static_cast<size_t>(m_fb.width()) ||
        outRGB.getHeight() != static_cast<size_t>(m_fb.height()) || outRGB.channels() != 3 ||
        outRGB.getPixelDepth() != mrpt::img::PixelDepth::D8U)
    {
      outRGB.resize(m_fb.width(), m_fb.height(), mrpt::img::CH_RGB);
    }

    ASSERT_(!outRGB.isEmpty());
    ASSERT_EQUAL_(outRGB.getWidth(), static_cast<size_t>(m_fb.width()));
    ASSERT_EQUAL_(outRGB.getHeight(), static_cast<size_t>(m_fb.height()));
    ASSERT_EQUAL_(outRGB.channels(), 3);
    ASSERT_(outRGB.getPixelDepth() == mrpt::img::PixelDepth::D8U);

#ifdef FBO_PROFILER
    auto tle1 = mrpt::system::CTimeLoggerEntry(profiler, sSec + ".glReadPixels_rgb"s);
#endif

    // CImage rows are tightly packed, so do not let OpenGL pad them to 4 bytes:
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(
        0, 0, m_fb.width(), m_fb.height(), GL_RGB, GL_UNSIGNED_BYTE, outRGB.ptrLine<uint8_t>(0));
    CHECK_OPENGL_ERROR();

#ifdef FBO_PROFILER
    tle1.stop();
#endif
    // No manual flip needed: flipVerticalProjection(true) already handles it.
  }

  // Unbind the framebuffer object
  FrameBuffer::Bind(oldFBs);

  MRPT_END
#endif
}

void CFBORender::convertDepthToLinear(
    mrpt::math::CMatrixFloat& depth, float zn, float zf, bool isProjective) const
{
  if (!isProjective)
  {
    // Orthographic projection: window depth is linear in the eye distance.
    for (auto& d : depth)
    {
      d = (d == 1.0f) ? 0.0f /* no "echo return" */ : zn + d * (zf - zn);
    }
    return;
  }

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

void CFBORender::setLensDistortion(const mrpt::img::TCamera& distortedCamera)
{
  MRPT_START

  const auto& cam = distortedCamera;
  if (cam.distortion == mrpt::img::DistortionModel::none)
  {
    clearLensDistortion();
    return;
  }

  ASSERT_EQUAL_(cam.ncols, m_params.width);
  ASSERT_EQUAL_(cam.nrows, m_params.height);

  const int W = static_cast<int>(cam.ncols);
  const int H = static_cast<int>(cam.nrows);

  const char* tooStrongMsg =
      "Lens distortion too strong: the undistorted field of view would be more than 3 times the "
      "image size";

  // Where each output (distorted) pixel lands in an ideal pinhole image.
  // Same convention as the pinhole projection used to render: the center of
  // pixel (u,v) is at image coordinates (u+0.5,v+0.5).
  std::vector<float> lut(2 * static_cast<size_t>(W) * static_cast<size_t>(H));
  float minX = 0;
  auto maxX = static_cast<float>(W);
  float minY = 0;
  auto maxY = static_cast<float>(H);
  for (int v = 0; v < H; v++)
  {
    for (int u = 0; u < W; u++)
    {
      mrpt::img::TPixelCoordf und;
      mrpt::img::camera_geometry::undistort_point(
          mrpt::img::TPixelCoordf(static_cast<float>(u) + 0.5f, static_cast<float>(v) + 0.5f), und,
          cam);
      ASSERTMSG_(std::isfinite(und.x) && std::isfinite(und.y), tooStrongMsg);

      const size_t i = 2 * (static_cast<size_t>(v) * W + u);
      lut[i + 0] = und.x;
      lut[i + 1] = und.y;
      minX = std::min(minX, und.x);
      maxX = std::max(maxX, und.x);
      minY = std::min(minY, und.y);
      maxY = std::max(maxY, und.y);
    }
  }

  // Enlarge the ideal image to cover the whole distorted field of view, so
  // there are no black borders. Bounded, to keep the rendered image size sane:
  const auto margin = [&](float outside, int size)
  {
    const int px = static_cast<int>(std::ceil(outside)) + 1;
    ASSERTMSG_(px <= size, tooStrongMsg);
    return px;
  };
  const int left = margin(-minX, W);
  const int right = margin(maxX - static_cast<float>(W), W);
  const int top = margin(-minY, H);
  const int bottom = margin(maxY - static_cast<float>(H), H);

  mrpt::img::TCamera ideal = cam;
  ideal.distortion = mrpt::img::DistortionModel::none;
  ideal.dist.fill(0);
  ideal.ncols = static_cast<uint32_t>(W + left + right);
  ideal.nrows = static_cast<uint32_t>(H + top + bottom);
  ideal.cx(cam.cx() + left);
  ideal.cy(cam.cy() + top);

  // Normalized texture coordinates:
  const auto idealW = static_cast<float>(ideal.ncols);
  const auto idealH = static_cast<float>(ideal.nrows);
  for (size_t i = 0; i < lut.size(); i += 2)
  {
    lut[i + 0] = (lut[i + 0] + static_cast<float>(left)) / idealW;
    lut[i + 1] = (lut[i + 1] + static_cast<float>(top)) / idealH;
  }

  m_distortedCamera = cam;
  m_idealCamera = ideal;
  m_distortionLUT = std::move(lut);
  m_distortionLUTChanged = true;

  MRPT_END
}

void CFBORender::clearLensDistortion()
{
  m_distortedCamera.reset();
  m_distortionLUT.clear();
}

void CFBORender::setRGBNoise(float stdIntensityLevels, uint32_t seed)
{
  ASSERT_GE_(stdIntensityLevels, 0.0f);
  m_noiseStd = stdIntensityLevels;
  m_noiseSeed = seed;
  m_noiseFrameIndex = 0;
}

void CFBORender::preparePostProcessing()
{
#if HAVE_FBO
  MRPT_START

  // Scene framebuffer, the size of the (enlarged) ideal image:
  const unsigned int w = m_distortedCamera ? m_idealCamera.ncols : m_fb.width();
  const unsigned int h = m_distortedCamera ? m_idealCamera.nrows : m_fb.height();

  if (!m_fbScene.initialized() || m_fbScene.width() != w || m_fbScene.height() != h)
  {
    if (m_texScene != 0)
    {
      glDeleteTextures(1, &m_texScene);
      m_texScene = 0;
    }
    m_fbScene.destroy();
    m_fbScene.create(w, h);

    const auto oldFB = m_fbScene.bind();
    m_texScene = createColorTextureForFBO(w, h);
    FrameBuffer::Bind(oldFB);
  }

  if (m_distortedCamera && m_distortionLUTChanged)
  {
    if (m_texLUT == 0)
    {
      glGenTextures(1, &m_texLUT);
      CHECK_OPENGL_ERROR();
    }
    glBindTexture(GL_TEXTURE_2D, m_texLUT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RG32F, static_cast<GLsizei>(m_distortedCamera->ncols),
        static_cast<GLsizei>(m_distortedCamera->nrows), 0, GL_RG, GL_FLOAT, m_distortionLUT.data());
    CHECK_OPENGL_ERROR();
    m_distortionLUTChanged = false;
  }

  if (!m_postProgram)
  {
    m_postProgram = LoadDefaultShader(DefaultShaderID::FBO_RGB_POSTPROCESS);
  }
  if (m_postVAO == 0)
  {
    glGenVertexArrays(1, &m_postVAO);
    CHECK_OPENGL_ERROR();
  }

  MRPT_END
#endif
}

void CFBORender::renderPostProcessing()
{
#if HAVE_FBO
  MRPT_START

  GLint oldViewport[4];
  glGetIntegerv(GL_VIEWPORT, oldViewport);
  glViewport(0, 0, static_cast<GLsizei>(m_fb.width()), static_cast<GLsizei>(m_fb.height()));
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  auto& prog = *m_postProgram;
  prog.use();

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, m_texScene);
  glUniform1i(prog.uniformId("sceneTex"), 0);

  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, m_texLUT);
  glUniform1i(prog.uniformId("lutTex"), 1);
  glUniform1i(prog.uniformId("useLUT"), m_distortedCamera ? 1 : 0);

  glUniform1f(prog.uniformId("noiseStd"), m_noiseStd / 255.0f);
  glUniform1ui(prog.uniformId("noiseSeed"), m_noiseSeed);
  glUniform1ui(prog.uniformId("frameIndex"), m_noiseFrameIndex++);

  glBindVertexArray(m_postVAO);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glActiveTexture(GL_TEXTURE0);
  glEnable(GL_DEPTH_TEST);
  glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);

  MRPT_END
#endif
}
