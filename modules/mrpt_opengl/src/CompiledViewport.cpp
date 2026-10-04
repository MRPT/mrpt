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

#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/CompiledViewport.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/ShaderProgramManager.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CTextMessageCapable.h>
#include <mrpt/viz/CVisualObject.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <random>
#include <unordered_set>

#include "gltext.h"

using namespace mrpt::opengl;
using namespace mrpt::viz;
using namespace mrpt::math;

static const bool VIEWPORT_VERBOSE = mrpt::get_env<bool>("MRPT_VIEWPORT_VERBOSE", false);

// ========== CompiledViewport ==========

CompiledViewport::CompiledViewport(const std::string& name) : m_name(name)
{
  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport] Created: '" << m_name << "'\n";
  }
}

CompiledViewport::~CompiledViewport()
{
  if (VIEWPORT_VERBOSE) std::cout << "[CompiledViewport] Destroying: '" << m_name << "'\n";

  clearProxies();
  ssaoDestroy();
}

void CompiledViewport::updateFromVizViewport(const mrpt::viz::Viewport& vizVp)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  // Store source viewport for writing back rendered dimensions
  m_sourceVizViewport = &vizVp;

  // Copy viewport bounds
  vizVp.getViewportPosition(m_viewX, m_viewY, m_viewWidth, m_viewHeight);

  // Copy camera (an explicit CCamera object inserted into the viewport, if
  // any, takes precedence over the viewport's own default camera):
  updateCameraParams(vizVp.resolveActiveCamera());
  m_matricesNeedUpdate = true;

  // Copy lighting
  m_lightParams = vizVp.lightParameters();

  // Copy rendering options
  m_enablePolygonSmooth = vizVp.isPolygonNicestEnabled();
  m_backgroundColor = vizVp.getCustomBackgroundColor();
  m_isTransparent = vizVp.isTransparent();
  m_isVisible = vizVp.getViewportVisibility();

  // Copy border settings
  m_borderWidth = vizVp.getBorderSize();
  m_borderColor = vizVp.getBorderColor();

  // Copy clip planes
  vizVp.getViewportClipDistances(m_clipNear, m_clipFar);
  vizVp.getLightShadowClipDistances(m_lightShadowClipNear, m_lightShadowClipFar);

  // Copy shadow settings
  m_shadowsEnabled = vizVp.isShadowCastingEnabled();
  m_shadowMapSizeX = vizVp.getShadowMapSizeX();
  m_shadowMapSizeY = vizVp.getShadowMapSizeY();

  // Propagate SSAO enable flag
  m_ssaoEnabled = m_lightParams.ssao_enabled;

  // Copy special modes
  if (vizVp.isImageViewMode())
  {
    // Image view mode will be handled separately by setting m_imageViewProxy
    // from the compiled CTexturedPlane object
  }

  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::updateFromVizViewport] '" << m_name << "'\n"
              << "  Position: (" << m_viewX << ", " << m_viewY << ")\n"
              << "  Size: " << m_viewWidth << " x " << m_viewHeight << "\n"
              << "  Transparent: " << (m_isTransparent ? "yes" : "no") << "\n"
              << "  Shadows: " << (m_shadowsEnabled ? "yes" : "no") << "\n";
  }

  MRPT_END
}

void CompiledViewport::addProxy(const RenderableProxy::Ptr& proxy)
{
  MRPT_START

  if (!proxy)
  {
    return;
  }

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);
  m_proxies.push_back(proxy);

  MRPT_END
}

void CompiledViewport::removeProxy(const RenderableProxy::Ptr& proxy)
{
  MRPT_START

  if (!proxy)
  {
    return;
  }

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);
  m_proxies.erase(std::remove(m_proxies.begin(), m_proxies.end(), proxy), m_proxies.end());

  MRPT_END
}

void CompiledViewport::removeProxies(const std::vector<const RenderableProxy*>& proxies)
{
  MRPT_START

  if (proxies.empty())
  {
    return;
  }

  const std::unordered_set<const RenderableProxy*> toRemove(proxies.begin(), proxies.end());

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);
  m_proxies.erase(
      std::remove_if(
          m_proxies.begin(), m_proxies.end(),
          [&](const RenderableProxy::Ptr& p) { return toRemove.count(p.get()) != 0; }),
      m_proxies.end());

  MRPT_END
}

void CompiledViewport::clearProxies()
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);
  m_proxies.clear();

  MRPT_END
}

void CompiledViewport::setImageViewMode(RenderableProxy::Ptr imageProxy)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_imageViewProxy = imageProxy;

  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::setImageViewMode] '" << m_name
              << "' entered image view mode\n";
  }

  MRPT_END
}

void CompiledViewport::clearImageViewMode()
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_imageViewProxy.reset();

  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::clearImageViewMode] '" << m_name
              << "' exited image view mode\n";
  }

  MRPT_END
}

void CompiledViewport::setCloneMode(const std::string& clonedViewportName, bool cloneCamera)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_isCloned = true;
  m_isClonedCamera = cloneCamera;
  m_clonedViewportName = clonedViewportName;
  m_clonedCameraViewportName = cloneCamera ? clonedViewportName : std::string();

  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::setCloneMode] '" << m_name << "' cloning from '"
              << clonedViewportName << "'" << (cloneCamera ? " (with camera)" : "") << "\n";
  }

  MRPT_END
}

void CompiledViewport::setCloneCameraFrom(const std::string& viewportName)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_isClonedCamera = true;
  m_clonedCameraViewportName = viewportName;

  MRPT_END
}

void CompiledViewport::clearCloneCamera()
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_isClonedCamera = false;
  m_clonedCameraViewportName.clear();

  MRPT_END
}

void CompiledViewport::clearCloneMode()
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_isCloned = false;
  m_isClonedCamera = false;
  m_clonedViewportName.clear();
  m_clonedCameraViewportName.clear();

  MRPT_END
}

void CompiledViewport::enableShadows(
    bool enabled, unsigned int shadowMapSizeX, unsigned int shadowMapSizeY)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_shadowsEnabled = enabled;

  if (shadowMapSizeX > 0)
  {
    m_shadowMapSizeX = shadowMapSizeX;
  }
  if (shadowMapSizeY > 0)
  {
    m_shadowMapSizeY = shadowMapSizeY;
  }

  if (!enabled)
  {
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
    if (m_shadowMapFBO != 0)
    {
      glDeleteFramebuffers(1, &m_shadowMapFBO);
      m_shadowMapFBO = 0;
    }
    if (m_cascadeDepthArrayTexId != 0)
    {
      glDeleteTextures(1, &m_cascadeDepthArrayTexId);
      m_cascadeDepthArrayTexId = 0;
      m_cascadeDepthArrayLayers = 0;
    }
#endif
  }

  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::enableShadows] '" << m_name << "' shadows "
              << (enabled ? "enabled" : "disabled") << "\n";
  }

  MRPT_END
}

void CompiledViewport::updateCamera(const mrpt::viz::CCamera& camera)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  updateCameraParams(camera);
  m_matricesNeedUpdate = true;

  MRPT_END
}

void CompiledViewport::updateCameraParams(const mrpt::viz::CCamera& camera)
{
  m_camera.noProjection = camera.isNoProjection();
  m_camera.projective = camera.isProjective();
  m_camera.is6DOF = camera.is6DOFMode();
  m_camera.fovDeg = camera.getProjectiveFOVdeg();
  m_camera.zoomDistance = camera.getZoomDistance();
  m_camera.azimuthDeg = camera.getAzimuthDegrees();
  m_camera.elevationDeg = camera.getElevationDegrees();
  m_camera.pointingAt = camera.getPointingAt().cast<double>();
  m_camera.pose = camera.getPose();
  m_camera.pinhole = camera.getPinholeModel();
}

void CompiledViewport::setViewportBounds(double x, double y, double width, double height)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_viewX = x;
  m_viewY = y;
  m_viewWidth = width;
  m_viewHeight = height;

  MRPT_END
}

void CompiledViewport::getViewportBounds(double& x, double& y, double& width, double& height) const
{
  std::shared_lock<std::shared_mutex> lock(m_stateMtx.data);

  x = m_viewX;
  y = m_viewY;
  width = m_viewWidth;
  height = m_viewHeight;
}

void CompiledViewport::setClipPlanes(float nearPlane, float farPlane)
{
  MRPT_START
  ASSERT_GT_(farPlane, nearPlane);

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_clipNear = nearPlane;
  m_clipFar = farPlane;
  m_matricesNeedUpdate = true;

  MRPT_END
}

void CompiledViewport::getClipPlanes(float& nearPlane, float& farPlane) const
{
  std::shared_lock<std::shared_mutex> lock(m_stateMtx.data);

  nearPlane = m_clipNear;
  farPlane = m_clipFar;
}

void CompiledViewport::setBorder(unsigned int width, const mrpt::img::TColor& color)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_borderWidth = width;
  m_borderColor = color;

  // Border proxy will be created/updated during render if needed

  MRPT_END
}

void CompiledViewport::computePixelViewport(
    int windowWidth, int windowHeight, int offsetX, int offsetY)
{
  m_pixelX = offsetX + startFromRatio(m_viewX, windowWidth);
  m_pixelY = offsetY + startFromRatio(m_viewY, windowHeight);
  m_pixelWidth = sizeFromRatio(m_pixelX, m_viewWidth, windowWidth);
  m_pixelHeight = sizeFromRatio(m_pixelY, m_viewHeight, windowHeight);
}
int CompiledViewport::startFromRatio(double frac, int dimension)
{
  const bool doWrap = (frac < 0);
  const auto fracAbs = std::abs(frac);
  const int L = fracAbs > 1 ? static_cast<int>(fracAbs) : static_cast<int>(dimension * fracAbs);
  return doWrap ? dimension - L : L;
}
int CompiledViewport::sizeFromRatio(int startCoord, double dSize, int dimension)
{
  if (dSize > 1)
  {  // >1 -> absolute pixels
    return static_cast<int>(dSize);
  }

  if (dSize < 0)
  {
    // Negative: specify right/bottom edge instead of size
    if (dSize >= -1)
    {
      return static_cast<int>(-dimension * dSize - startCoord + 1);
    }

    return static_cast<int>(dimension + dSize - startCoord + 1);
  }

  // Otherwise: a fraction
  return static_cast<int>(dimension * dSize);
}
void CompiledViewport::updateMatrices()
{
  MRPT_START
  if (!m_matricesNeedUpdate)
  {
    return;
  }

  // Update viewport dimensions in matrices
  m_renderMatrices.viewport_width = m_pixelWidth;
  m_renderMatrices.viewport_height = m_pixelHeight;
  // Compute projection matrix based on camera type
  if (m_camera.noProjection)
  {
    m_renderMatrices.computeNoProjectionMatrix(m_clipNear, m_clipFar);
  }
  else
  {
    m_renderMatrices.is_projective = m_camera.projective;
    m_renderMatrices.FOV = m_camera.fovDeg;
    m_renderMatrices.eyeDistance = m_camera.zoomDistance;
    m_renderMatrices.pinhole_model = m_camera.pinhole;
    if (m_camera.is6DOF)
    {
      const auto& pose = m_camera.pose;
      m_renderMatrices.eye = pose.translation();

      // Compute pointing direction
      const auto viewDir = mrpt::poses::CPose3D::FromTranslation(0, 0, 1);
      const auto at = pose + viewDir.asTPose();
      m_renderMatrices.pointing = at.translation();

      // Extract up vector from rotation matrix
      pose.getRotationMatrix().extractColumn(1, m_renderMatrices.up);
      m_renderMatrices.up *= -1.0;  // -Y is up
    }
    else
    {
      // Orbit camera mode
      m_renderMatrices.pointing = m_camera.pointingAt;
      m_renderMatrices.azimuth = DEG2RAD(m_camera.azimuthDeg);
      m_renderMatrices.elev = DEG2RAD(m_camera.elevationDeg);

      const auto c2m_u = mrpt::math::TVector3D(
          cos(m_renderMatrices.azimuth) * cos(m_renderMatrices.elev),
          sin(m_renderMatrices.azimuth) * cos(m_renderMatrices.elev), sin(m_renderMatrices.elev));

      const double dis = std::max<double>(0.001, m_camera.zoomDistance);
      m_renderMatrices.eye = m_renderMatrices.pointing + c2m_u * dis;

      m_renderMatrices.up.x = -cos(m_renderMatrices.azimuth) * sin(m_renderMatrices.elev);
      m_renderMatrices.up.y = -sin(m_renderMatrices.azimuth) * sin(m_renderMatrices.elev);
      m_renderMatrices.up.z = cos(m_renderMatrices.elev);
    }

    m_renderMatrices.computeProjectionMatrix(m_clipNear, m_clipFar);
    m_renderMatrices.computeViewMatrix();
  }
  // Compute light projection matrices for shadows (cascaded).
  // Clamp the shadow far plane to the camera's far clip distance: cascades
  // sized beyond the visible frustum create enormous frustum bounding spheres
  // that map hundreds of metres to a single shadow-map texel, producing coarse
  // (pixelated) shadows even at 4096-px map resolution.
  if (m_shadowsEnabled)
  {
    const float shadowZmax = std::min(m_lightShadowClipFar, m_clipFar);
    // Cameras with a free pose have no orbit distance to size shadows with:
    constexpr float DEFAULT_SHADOW_DISTANCE_6DOF = 40.0f;
    float shadowDistance = m_lightParams.shadow_max_distance;
    if (shadowDistance <= 0 && m_camera.is6DOF)
    {
      shadowDistance = DEFAULT_SHADOW_DISTANCE_6DOF;
    }
    m_renderMatrices.computeCascadedLightProjectionMatrices(
        m_lightShadowClipNear, shadowZmax, m_lightParams,
        std::min(m_shadowMapSizeX, m_shadowMapSizeY), shadowDistance);
  }
  if (m_flipYProjection)
  {
    // Negate the Y row of the projection matrix. Done here, on the freshly
    // computed matrix, so the flip is applied exactly once.
    for (mrpt::math::matrix_index_t c = 0; c < m_renderMatrices.p_matrix.cols(); c++)
    {
      m_renderMatrices.p_matrix(1, c) *= -1.0f;
    }
  }
  m_renderMatrices.initialized = true;
  m_matricesNeedUpdate = false;
  MRPT_END
}
void CompiledViewport::render(
    int renderWidth,
    int renderHeight,
    int renderOffsetX,
    int renderOffsetY,
    ShaderProgramManager& shaderManager,
    const CompiledViewport* sourceViewport)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!m_isVisible)
  {
    return;
  }
  m_lastStats.reset();
  const auto tStart = mrpt::Clock::nowDouble();
  std::shared_lock<std::shared_mutex> lock(m_stateMtx.data);
  // Compute pixel viewport
  computePixelViewport(renderWidth, renderHeight, renderOffsetX, renderOffsetY);

  // Write back rendered viewport size for get3DRayForPixelCoord()
  if (m_sourceVizViewport)
  {
    m_sourceVizViewport->updateRenderedViewportSize(m_pixelWidth, m_pixelHeight);
  }

  // Update matrices if needed
  updateMatrices();

  if (m_flipYProjection)
  {
    // Compensate for the winding order change of the flipped projection
    glFrontFace(GL_CW);
  }

  // Save previous viewport
  GLint oldViewport[4];
  glGetIntegerv(GL_VIEWPORT, oldViewport);
  // Set this viewport
  glViewport(m_pixelX, m_pixelY, m_pixelWidth, m_pixelHeight);
  CHECK_OPENGL_ERROR_IN_DEBUG();
  // Setup scissor for clearing
  glScissor(m_pixelX, m_pixelY, m_pixelWidth, m_pixelHeight);
  glEnable(GL_SCISSOR_TEST);
  CHECK_OPENGL_ERROR_IN_DEBUG();
  // Clear buffers
  if (!m_isTransparent)
  {
    GLfloat prevColor[4];
    glGetFloatv(GL_COLOR_CLEAR_VALUE, prevColor);
    glClearColor(
        m_backgroundColor.R, m_backgroundColor.G, m_backgroundColor.B, m_backgroundColor.A);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glClearColor(prevColor[0], prevColor[1], prevColor[2], prevColor[3]);
  }
  else
  {
    glClear(GL_DEPTH_BUFFER_BIT);
  }
  glDisable(GL_SCISSOR_TEST);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Determine which proxies to render: for cloned viewports, use source's
  const std::vector<RenderableProxy::Ptr>* proxiesToRender = nullptr;
  if (sourceViewport)
  {
    proxiesToRender = &sourceViewport->getProxies();
  }

  // GPU gamma correction: enable sRGB framebuffer encode (linear→sRGB on
  // write) when requested. The texture internal formats (GL_SRGB8/GL_SRGB8_ALPHA8)
  // already handle the decode (sRGB→linear) at sampling time for free.
  if (m_lightParams.gamma_correction)
  {
    glEnable(GL_FRAMEBUFFER_SRGB);
  }

  // Render based on mode
  if (isImageViewMode())
  {
    renderImageView(shaderManager);
  }
  else
  {
    // Shadow map pass (if enabled)
    if (m_shadowsEnabled)
    {
      renderShadowMap(shaderManager);
    }
    // SSAO pre-pass (if enabled)
    if (m_ssaoEnabled)
    {
      renderSSAOGeometry(shaderManager);
      renderSSAOCompute(shaderManager);
      // Restore viewport after SSAO passes
      glViewport(m_pixelX, m_pixelY, m_pixelWidth, m_pixelHeight);
    }
    // Normal scene rendering
    renderNormalScene(shaderManager, false, proxiesToRender);
  }
  // Render 2D text message overlays
  renderTextOverlays(shaderManager);

  // Render border
  if (m_borderWidth > 0)
  {
    renderBorder(shaderManager);
  }

  if (m_lightParams.gamma_correction)
  {
    glDisable(GL_FRAMEBUFFER_SRGB);
  }

  if (m_flipYProjection)
  {
    glFrontFace(GL_CCW);  // Restore default
  }
  // Restore viewport
  glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
  m_lastStats.renderTimeMs = (mrpt::Clock::nowDouble() - tStart) * 1000.0;
  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::render] '" << m_name << "' " << m_lastStats.numProxiesRendered
              << " objects in " << m_lastStats.renderTimeMs << " ms\n";
  }
#endif
  MRPT_END
}
void CompiledViewport::renderImageView(ShaderProgramManager& shaderManager)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!m_imageViewProxy)
  {
    return;
  }

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Identity projection by default (quad covers [-1,1]x[-1,1])
  auto matrices = m_renderMatrices;
  matrices.matricesSetIdentity();

  // Aspect-ratio correction: scale the quad so the image fits without stretching
  // (same logic as mrpt v2 Viewport::renderImageView)
  if (auto src = m_imageViewProxy->getSourceObject(); src)
  {
    if (auto* ttSrc =
            dynamic_cast<const mrpt::viz::VisualObjectParams_TexturedTriangles*>(src.get());
        ttSrc && ttSrc->textureImageHasBeenAssigned())
    {
      const auto& img = ttSrc->getTextureImage();
      const size_t img_w = img.getWidth(), img_h = img.getHeight();
      if (img_w > 0 && img_h > 0 && m_pixelWidth > 0 && m_pixelHeight > 0)
      {
        const float img_ratio = static_cast<float>(img_w) / static_cast<float>(img_h);
        const float vw_ratio = static_cast<float>(m_pixelWidth) / static_cast<float>(m_pixelHeight);
        const float ratio = vw_ratio / img_ratio;

        auto& p = matrices.p_matrix;
        if (ratio > 1.0f)
          p(1, 1) *= ratio;
        else if (ratio > 0.0f)
          p(0, 0) /= ratio;

        // Normalize so neither dimension exceeds 1
        auto& p00 = p(0, 0);
        auto& p11 = p(1, 1);
        if (p00 > 0 && p11 > 0)
        {
          const float s = std::max(p00, p11);
          p00 /= s;
          p11 /= s;
        }

        matrices.pmv_matrix.asEigen() =
            matrices.p_matrix.asEigen() * matrices.v_matrix.asEigen() * matrices.m_matrix.asEigen();
      }
    }
  }

  // Render the textured quad
  auto shaderProg = shaderManager.getProgram(DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT);
  if (!shaderProg)
  {
    return;
  }
  shaderProg->use();

  // Upload the PMV matrix (identity for image view, possibly with aspect-ratio scale)
  const auto IS_TRANSPOSED = GL_TRUE;
  if (shaderProg->hasUniform("pmv_matrix"))
  {
    glUniformMatrix4fv(
        shaderProg->uniformId("pmv_matrix"), 1, IS_TRANSPOSED, matrices.pmv_matrix.data());
  }

  RenderContext rc;
  rc.shader = shaderProg.get();
  rc.shader_id = DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT;
  rc.state = &matrices;
  rc.lights = &m_lightParams;
  m_imageViewProxy->render(rc);
  m_lastStats.numProxiesRendered = 1;
  m_lastStats.numDrawCalls = 1;
#endif
  MRPT_END
}
// -----------------------------------------------------------------------
// SSAO helpers
// -----------------------------------------------------------------------
void CompiledViewport::ssaoInit()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  // --- Hemisphere kernel (tangent space, z-facing hemisphere) ---
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> randUnit(0.0f, 1.0f);
  std::uniform_real_distribution<float> randSigned(-1.0f, 1.0f);

  const int N = 64;
  m_ssaoKernel.resize(N * 3);
  for (int i = 0; i < N; i++)
  {
    float x = randSigned(rng);
    float y = randSigned(rng);
    float z = randUnit(rng);  // hemisphere: z >= 0
    // Normalize
    float len = std::sqrt(x * x + y * y + z * z);
    if (len < 1e-6f)
    {
      x = 0;
      y = 0;
      z = 1;
      len = 1;
    }
    x /= len;
    y /= len;
    z /= len;
    // Accelerating interpolation: cluster samples closer to the origin
    float scale = static_cast<float>(i) / N;
    scale = 0.1f + 0.9f * scale * scale;
    m_ssaoKernel[i * 3 + 0] = x * scale;
    m_ssaoKernel[i * 3 + 1] = y * scale;
    m_ssaoKernel[i * 3 + 2] = z * scale;
  }

  // --- 4x4 noise texture (random rotation vectors, RG only) ---
  float noiseData[4 * 4 * 3];
  for (int i = 0; i < 16; i++)
  {
    noiseData[i * 3 + 0] = randSigned(rng);
    noiseData[i * 3 + 1] = randSigned(rng);
    noiseData[i * 3 + 2] = 0.0f;
  }

  if (m_ssaoNoiseTex == 0) glGenTextures(1, &m_ssaoNoiseTex);
  glBindTexture(GL_TEXTURE_2D, m_ssaoNoiseTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 4, 4, 0, GL_RGB, GL_FLOAT, noiseData);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  // --- Dummy VAO for full-screen triangle ---
  if (m_ssaoDummyVAO == 0) glGenVertexArrays(1, &m_ssaoDummyVAO);
#endif
}

void CompiledViewport::ssaoCreateFBOs(int w, int h)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  // The caller's framebuffer must be current again when returning:
  const auto callerFBs = FrameBuffer::CurrentBinding();

  // Delete the previous G-buffer and AO buffers only: the kernel, noise
  // texture and VAO created by ssaoInit() are needed to render.
  ssaoDestroyFramebuffers();

  // --- G-buffer FBO ---
  glGenFramebuffers(1, &m_ssaoGBufferFBO);
  glBindFramebuffer(GL_FRAMEBUFFER, m_ssaoGBufferFBO);

  // Position texture (RGBA32F — need full precision for view-space positions)
  glGenTextures(1, &m_ssaoGPositionTex);
  glBindTexture(GL_TEXTURE_2D, m_ssaoGPositionTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, w, h, 0, GL_RGBA, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glFramebufferTexture2D(
      GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_ssaoGPositionTex, 0);

  // Normal texture (RGBA16F)
  glGenTextures(1, &m_ssaoGNormalTex);
  glBindTexture(GL_TEXTURE_2D, m_ssaoGNormalTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0, GL_RGBA, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, m_ssaoGNormalTex, 0);

  // Depth renderbuffer
  glGenRenderbuffers(1, &m_ssaoGDepthRBO);
  glBindRenderbuffer(GL_RENDERBUFFER, m_ssaoGDepthRBO);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_ssaoGDepthRBO);

  const GLenum drawBufs[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
  glDrawBuffers(2, drawBufs);

  // --- Raw AO FBO (single channel R16F) ---
  glGenFramebuffers(1, &m_ssaoRawFBO);
  glBindFramebuffer(GL_FRAMEBUFFER, m_ssaoRawFBO);
  glGenTextures(1, &m_ssaoRawTex);
  glBindTexture(GL_TEXTURE_2D, m_ssaoRawTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, w, h, 0, GL_RED, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_ssaoRawTex, 0);
  {
    const GLenum buf = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &buf);
  }

  // --- Blur AO FBO ---
  glGenFramebuffers(1, &m_ssaoBlurFBO);
  glBindFramebuffer(GL_FRAMEBUFFER, m_ssaoBlurFBO);
  glGenTextures(1, &m_ssaoBlurTex);
  glBindTexture(GL_TEXTURE_2D, m_ssaoBlurTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, w, h, 0, GL_RED, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_ssaoBlurTex, 0);
  {
    const GLenum buf = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &buf);
  }

  FrameBuffer::Bind(callerFBs);
  m_ssaoLastW = w;
  m_ssaoLastH = h;
#endif
}

void CompiledViewport::ssaoDestroyFramebuffers()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  auto del_tex = [](unsigned int& t)
  {
    if (t)
    {
      glDeleteTextures(1, &t);
      t = 0;
    }
  };
  auto del_fbo = [](unsigned int& f)
  {
    if (f)
    {
      glDeleteFramebuffers(1, &f);
      f = 0;
    }
  };
  auto del_rbo = [](unsigned int& r)
  {
    if (r)
    {
      glDeleteRenderbuffers(1, &r);
      r = 0;
    }
  };

  del_fbo(m_ssaoGBufferFBO);
  del_tex(m_ssaoGPositionTex);
  del_tex(m_ssaoGNormalTex);
  del_rbo(m_ssaoGDepthRBO);
  del_fbo(m_ssaoRawFBO);
  del_tex(m_ssaoRawTex);
  del_fbo(m_ssaoBlurFBO);
  del_tex(m_ssaoBlurTex);

  m_ssaoLastW = 0;
  m_ssaoLastH = 0;
#endif
}

void CompiledViewport::ssaoDestroy()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  ssaoDestroyFramebuffers();

  if (m_ssaoNoiseTex)
  {
    glDeleteTextures(1, &m_ssaoNoiseTex);
    m_ssaoNoiseTex = 0;
  }
  if (m_ssaoDummyVAO)
  {
    glDeleteVertexArrays(1, &m_ssaoDummyVAO);
    m_ssaoDummyVAO = 0;
  }
  m_ssaoKernel.clear();
#endif
}

void CompiledViewport::renderSSAOGeometry(ShaderProgramManager& shaderManager)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  // Lazy init of kernel/noise/VAO (once per context)
  if (m_ssaoKernel.empty()) ssaoInit();

  // (Re-)create FBOs when size changes
  if (m_pixelWidth != m_ssaoLastW || m_pixelHeight != m_ssaoLastH)
    ssaoCreateFBOs(m_pixelWidth, m_pixelHeight);

  const auto oldFBs = FrameBuffer::CurrentBinding();
  glBindFramebuffer(GL_FRAMEBUFFER, m_ssaoGBufferFBO);

  glViewport(0, 0, m_pixelWidth, m_pixelHeight);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Build render queue overriding all triangle shaders with SSAO_GEOMETRY
  RenderQueue queue;
  auto matrices = m_renderMatrices;
  buildRenderQueueSSAOGeom(queue, matrices);
  processRenderQueue(queue, shaderManager, matrices, m_lastStats);

  FrameBuffer::Bind(oldFBs);
#endif
  MRPT_END
}

void CompiledViewport::renderSSAOCompute(ShaderProgramManager& shaderManager)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  const auto oldFBs = FrameBuffer::CurrentBinding();

  // ----- AO compute pass -----
  glBindFramebuffer(GL_FRAMEBUFFER, m_ssaoRawFBO);
  glViewport(0, 0, m_pixelWidth, m_pixelHeight);
  glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);

  auto computeShader = shaderManager.getProgram(DefaultShaderID::SSAO_COMPUTE);
  if (computeShader)
  {
    computeShader->use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_ssaoGPositionTex);
    glUniform1i(computeShader->uniformId("gPosition"), 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_ssaoGNormalTex);
    glUniform1i(computeShader->uniformId("gNormal"), 1);

    glActiveTexture(GL_TEXTURE0 + SSAO_NOISE_TEXTURE_UNIT);
    glBindTexture(GL_TEXTURE_2D, m_ssaoNoiseTex);
    glUniform1i(computeShader->uniformId("noiseTexture"), SSAO_NOISE_TEXTURE_UNIT);

    // Kernel samples: upload the entire vec3 array at once using the base
    // location of "ssao_samples" (which maps to ssao_samples[0]).
    const int kernelSize = static_cast<int>(m_ssaoKernel.size() / 3);
    if (computeShader->hasUniform("ssao_samples"))
    {
      const int count = std::min(kernelSize, static_cast<int>(m_lightParams.ssao_kernel_size));
      glUniform3fv(computeShader->uniformId("ssao_samples"), count, m_ssaoKernel.data());
    }

    if (computeShader->hasUniform("ssao_kernel_size"))
      glUniform1i(
          computeShader->uniformId("ssao_kernel_size"),
          std::min(kernelSize, static_cast<int>(m_lightParams.ssao_kernel_size)));

    if (computeShader->hasUniform("ssao_radius"))
      glUniform1f(computeShader->uniformId("ssao_radius"), m_lightParams.ssao_radius);
    if (computeShader->hasUniform("ssao_bias"))
      glUniform1f(computeShader->uniformId("ssao_bias"), m_lightParams.ssao_bias);
    // Pass projection focal lengths (diagonal elements of projection matrix)
    // p_matrix is row-major, so [0][0]=data[0], [1][1]=data[5]
    if (computeShader->hasUniform("proj_fx"))
      glUniform1f(computeShader->uniformId("proj_fx"), m_renderMatrices.p_matrix(0, 0));
    if (computeShader->hasUniform("proj_fy"))
      glUniform1f(computeShader->uniformId("proj_fy"), m_renderMatrices.p_matrix(1, 1));
    if (computeShader->hasUniform("noiseScale"))
      glUniform2f(
          computeShader->uniformId("noiseScale"), static_cast<float>(m_pixelWidth) / 4.0f,
          static_cast<float>(m_pixelHeight) / 4.0f);

    glBindVertexArray(m_ssaoDummyVAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
  }

  // ----- Blur pass -----
  glBindFramebuffer(GL_FRAMEBUFFER, m_ssaoBlurFBO);
  glClear(GL_COLOR_BUFFER_BIT);

  auto blurShader = shaderManager.getProgram(DefaultShaderID::SSAO_BLUR);
  if (blurShader)
  {
    blurShader->use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_ssaoRawTex);
    glUniform1i(blurShader->uniformId("ssaoInput"), 0);

    glBindVertexArray(m_ssaoDummyVAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
  }

  glEnable(GL_DEPTH_TEST);
  FrameBuffer::Bind(oldFBs);
#endif
  MRPT_END
}

void CompiledViewport::buildRenderQueueSSAOGeom(RenderQueue& queue, const TRenderMatrices& matrices)
{
  MRPT_START
  for (const auto& proxy : m_proxies)
  {
    if (!proxy || !proxy->m_visible)
    {
      continue;
    }

    // Only render objects that have lit triangle shaders (they have normals)
    const auto shaderIDs = proxy->requiredShaders();
    const bool hasTriangles = std::any_of(
        shaderIDs.begin(), shaderIDs.end(),
        [](shader_id_t sid)
        {
          return sid == DefaultShaderID::TRIANGLES_LIGHT ||
                 sid == DefaultShaderID::TEXTURED_TRIANGLES_LIGHT ||
                 sid == DefaultShaderID::TRIANGLES_SHADOW_2ND ||
                 sid == DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_2ND;
        });
    if (!hasTriangles)
    {
      continue;
    }

    RenderQueueElement e;
    e.proxy = proxy.get();
    e.shader = DefaultShaderID::SSAO_GEOMETRY;
    e.m_matrix = proxy->m_modelMatrix;
    e.mv_matrix.asEigen() = matrices.v_matrix.asEigen() * e.m_matrix.asEigen();
    e.pmv_matrix.asEigen() = matrices.p_matrix.asEigen() * e.mv_matrix.asEigen();

    const auto& bb = proxy->localBoundingBox();
    if (bb && proxy->cullEligible() && !boxIntersectsClipVolume(*bb, e.pmv_matrix))
    {
      continue;
    }
    queue.opaque.push_back(e);
  }
  queue.sort();
  MRPT_END
}

void CompiledViewport::renderShadowMap(ShaderProgramManager& shaderManager)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  const int numCascades = m_renderMatrices.numShadowCascades;

  // Create/resize the cascade depth texture array
  if (m_cascadeDepthArrayTexId == 0 || m_cascadeDepthArrayLayers != numCascades ||
      m_cascadeDepthArraySizeX != m_shadowMapSizeX || m_cascadeDepthArraySizeY != m_shadowMapSizeY)
  {
    if (m_cascadeDepthArrayTexId != 0) glDeleteTextures(1, &m_cascadeDepthArrayTexId);
    glGenTextures(1, &m_cascadeDepthArrayTexId);
    glBindTexture(GL_TEXTURE_2D_ARRAY, m_cascadeDepthArrayTexId);
    glTexImage3D(
        GL_TEXTURE_2D_ARRAY, 0, GL_DEPTH_COMPONENT32F, m_shadowMapSizeX, m_shadowMapSizeY,
        numCascades, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    m_cascadeDepthArrayLayers = numCascades;
    m_cascadeDepthArraySizeX = m_shadowMapSizeX;
    m_cascadeDepthArraySizeY = m_shadowMapSizeY;
  }

  // Create a single FBO for rendering directly into texture array layers
  if (m_shadowMapFBO == 0)
  {
    glGenFramebuffers(1, &m_shadowMapFBO);
  }

  const auto oldFBs = FrameBuffer::CurrentBinding();
  glBindFramebuffer(GL_FRAMEBUFFER, m_shadowMapFBO);

  glEnable(GL_DEPTH_TEST);
  glViewport(0, 0, m_shadowMapSizeX, m_shadowMapSizeY);

  // Hardware polygon offset to prevent shadow acne (self-shadowing)
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(2.0f, 4.0f);

#if defined(GL_DEPTH_CLAMP)
  // Casters between the light and the cascade near plane must still cast
  // shadows (e.g. a ceiling high above the floor seen by the camera):
  // clamp their depth to the near plane instead of clipping them.
  glEnable(GL_DEPTH_CLAMP);
#endif

  // Render each cascade directly into the texture array layer
  for (int c = 0; c < numCascades; c++)
  {
    // Attach texture array layer as FBO depth target
    glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_cascadeDepthArrayTexId, 0, c);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    glClear(GL_DEPTH_BUFFER_BIT);

    // Set the current cascade's light_pv as the active one for the 1st pass
    m_renderMatrices.light_pv = m_renderMatrices.cascade_light_pv[c];
    m_renderMatrices.currentCascadeIndex = c;

    renderNormalScene(shaderManager, true);
  }

  glDisable(GL_POLYGON_OFFSET_FILL);
#if defined(GL_DEPTH_CLAMP)
  glDisable(GL_DEPTH_CLAMP);
#endif

  // Restore previous FBO and viewport
  FrameBuffer::Bind(oldFBs);
  glViewport(m_pixelX, m_pixelY, m_pixelWidth, m_pixelHeight);
#endif
  MRPT_END
}
void CompiledViewport::renderNormalScene(
    ShaderProgramManager& shaderManager,
    bool isShadowMapPass,
    const std::vector<RenderableProxy::Ptr>* proxiesToRender)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  // Setup OpenGL state
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_MULTISAMPLE);
#if !defined(EMSCRIPTEN)
  if (m_enablePolygonSmooth)
  {
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  }
  glEnable(GL_LINE_SMOOTH);
#endif
#if !defined(EMSCRIPTEN) && defined(GL_PROGRAM_POINT_SIZE)
  glEnable(GL_PROGRAM_POINT_SIZE);
#endif
  // Build render queue with frustum culling
  RenderQueue queue;
  auto matrices = m_renderMatrices;
  matrices.is1stShadowMapPass = isShadowMapPass;
  buildRenderQueue(queue, matrices, isShadowMapPass, m_lastStats, proxiesToRender);
  // Process render queue
  processRenderQueue(queue, shaderManager, matrices, m_lastStats);
#endif
  MRPT_END
}

void CompiledViewport::buildRenderQueue(
    RenderQueue& queue,
    const TRenderMatrices& matrices,
    bool isShadowMapPass,
    ViewportRenderStats& stats,
    const std::vector<RenderableProxy::Ptr>* proxiesToRender)
{
  MRPT_START
  const auto& proxies = proxiesToRender ? *proxiesToRender : m_proxies;
  for (const auto& proxy : proxies)
  {
    // Skip invisible objects: check effective visibility (accounts for
    // parent container visibility)
    if (!proxy || !proxy->m_visible)
    {
      continue;
    }

    // Skip if in shadow map pass and object doesn't cast shadows
    if (isShadowMapPass && !proxy->castsShadows())
    {
      continue;
    }

    RenderQueueElement e;
    e.proxy = proxy.get();
    e.m_matrix = proxy->m_modelMatrix;
    e.mv_matrix.asEigen() = matrices.v_matrix.asEigen() * e.m_matrix.asEigen();

    // Frustum culling: against the camera, or against the light frustum of
    // this shadow cascade. Casters between the light and the cascade near
    // plane still cast shadows (depth clamp), so that plane is not tested.
    const auto& bb = proxy->localBoundingBox();
    const bool checkCulling = bb.has_value() && proxy->cullEligible();
    if (isShadowMapPass)
    {
      if (checkCulling)
      {
        mrpt::math::CMatrixFloat44 lightPVM;
        lightPVM.asEigen() = matrices.light_pv.asEigen() * e.m_matrix.asEigen();
        constexpr unsigned int NEAR_PLANE = 16;
        if (!boxIntersectsClipVolume(*bb, lightPVM, 0x3F & ~NEAR_PLANE))
        {
          stats.numProxiesCulled++;
          continue;
        }
      }
    }
    else
    {
      e.pmv_matrix.asEigen() = matrices.p_matrix.asEigen() * e.mv_matrix.asEigen();
      if (checkCulling && !boxIntersectsClipVolume(*bb, e.pmv_matrix))
      {
        stats.numProxiesCulled++;
        continue;
      }
    }

    // Eye-space depth (larger is farther), for sorting:
    const auto& p = proxy->m_sortPointLocal;
    const auto& MV = e.mv_matrix;
    e.depth = -(MV(2, 0) * p.x + MV(2, 1) * p.y + MV(2, 2) * p.z + MV(2, 3));

    auto shaderIDs = proxy->requiredShaders();
    if (isShadowMapPass)
    {
      // Shadow 1st pass: only render depth using the shadow depth shader.
      shaderIDs.clear();
      shaderIDs.push_back(proxy->shadowMapShader());
    }
    else if (m_shadowsEnabled)
    {
      // Normal rendering with shadows enabled: replace lit shaders with
      // their shadow 2nd-pass variants
      for (auto& sid : shaderIDs)
      {
        if (sid == DefaultShaderID::TRIANGLES_LIGHT)
        {
          sid = DefaultShaderID::TRIANGLES_SHADOW_2ND;
        }
        else if (sid == DefaultShaderID::TEXTURED_TRIANGLES_LIGHT)
        {
          sid = DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_2ND;
        }
      }
    }

    auto& layer = isShadowMapPass          ? queue.opaque
                  : proxy->isBackground()  ? queue.background
                  : proxy->isTransparent() ? queue.transparent
                                           : queue.opaque;
    for (auto shaderID : shaderIDs)
    {
      e.shader = shaderID;
      layer.push_back(e);
    }

    stats.numProxiesRendered++;
  }
  queue.sort();
  MRPT_END
}

void CompiledViewport::setupShaderForPass(
    Program& shader, shader_id_t shaderID, const TRenderMatrices& matrices)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  shader.use();

  const auto IS_TRANSPOSED = GL_TRUE;

  // Upload per-shader uniforms that don't change per object (p, v)
  if (shader.hasUniform("p_matrix"))
  {
    glUniformMatrix4fv(shader.uniformId("p_matrix"), 1, IS_TRANSPOSED, matrices.p_matrix.data());
  }
  if (shader.hasUniform("v_matrix"))
  {
    glUniformMatrix4fv(shader.uniformId("v_matrix"), 1, IS_TRANSPOSED, matrices.v_matrix.data());
  }

  // Upload light_pv_matrix for shadow shaders (per-shader, not per-object)
  if (shader.hasUniform("light_pv_matrix"))
  {
    glUniformMatrix4fv(
        shader.uniformId("light_pv_matrix"), 1, IS_TRANSPOSED, matrices.light_pv.data());
  }

  // Bind cascaded shadow map texture array for 2nd pass shaders
  const bool isShadow2ndPass = shaderID == DefaultShaderID::TRIANGLES_SHADOW_2ND ||
                               shaderID == DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_2ND;
  if (isShadow2ndPass && m_cascadeDepthArrayTexId != 0)
  {
    glActiveTexture(GL_TEXTURE0 + SHADOW_MAP_TEXTURE_UNIT);
    glBindTexture(GL_TEXTURE_2D_ARRAY, m_cascadeDepthArrayTexId);
    if (shader.hasUniform("shadowMapArray"))
    {
      glUniform1i(shader.uniformId("shadowMapArray"), SHADOW_MAP_TEXTURE_UNIT);
    }

    // Upload cascade light_pv matrices (each one transposed)
    static const std::array<const char*, 4> cascadeNames = {
        "cascade_light_pv[0]", "cascade_light_pv[1]", "cascade_light_pv[2]", "cascade_light_pv[3]"};
    const int N = std::min<int>(matrices.numShadowCascades, cascadeNames.size());
    for (int c = 0; c < N; c++)
    {
      if (shader.hasUniform(cascadeNames[c]))
      {
        glUniformMatrix4fv(
            shader.uniformId(cascadeNames[c]), 1, IS_TRANSPOSED,
            matrices.cascade_light_pv[c].data());
      }
    }
    // Upload cascade far planes and count
    if (shader.hasUniform("num_shadow_cascades"))
    {
      glUniform1i(shader.uniformId("num_shadow_cascades"), matrices.numShadowCascades);
    }
    if (shader.hasUniform("cascade_far_planes"))
    {
      glUniform1fv(
          shader.uniformId("cascade_far_planes"), matrices.numShadowCascades,
          matrices.cascade_far_planes.data());
    }
  }

  // Bind SSAO blur texture for lit shaders (all shaders that declare ssao_enabled)
  if (shader.hasUniform("ssao_enabled"))
  {
    const bool ssaoActive = m_ssaoEnabled && m_ssaoBlurTex != 0;
    glUniform1i(shader.uniformId("ssao_enabled"), ssaoActive ? 1 : 0);
    if (ssaoActive)
    {
      glActiveTexture(GL_TEXTURE0 + SSAO_TEXTURE_UNIT);
      glBindTexture(GL_TEXTURE_2D, m_ssaoBlurTex);
      if (shader.hasUniform("ssaoTexture"))
      {
        glUniform1i(shader.uniformId("ssaoTexture"), SSAO_TEXTURE_UNIT);
      }
      if (shader.hasUniform("ssao_power"))
      {
        glUniform1f(shader.uniformId("ssao_power"), m_lightParams.ssao_power);
      }
      if (shader.hasUniform("ssao_ambient_floor"))
      {
        glUniform1f(shader.uniformId("ssao_ambient_floor"), m_lightParams.ssao_ambient_floor);
      }
    }
  }
#endif
}

void CompiledViewport::processRenderQueue(
    const RenderQueue& queue,
    ShaderProgramManager& shaderManager,
    const TRenderMatrices& matrices,
    ViewportRenderStats& stats)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL

  // Clear any prior GL errors
  clearOpenGLErrors();

  const auto IS_TRANSPOSED = GL_TRUE;

  // State of the current shader:
  Program::Ptr shader;
  shader_id_t shaderID = DefaultShaderID::NONE;
  bool isShadowPass = false;
  int loc_m = -1;
  int loc_mv = -1;
  int loc_pmv = -1;
  int loc_normal = -1;

  const auto uniformOrNone = [](const Program& p, const char* name)
  { return p.hasUniform(name) ? p.uniformId(name) : -1; };

  // Per-object render state: the per-pass matrices, plus the matrices of
  // each object, overwritten for each one.
  TRenderMatrices objState = matrices;

  for (const auto* layer : {&queue.opaque, &queue.background, &queue.transparent})
  {
    for (const auto& e : *layer)
    {
      if (!shader || e.shader != shaderID)
      {
        shaderID = e.shader;
        shader = shaderManager.getProgram(shaderID);
        if (!shader)
        {
          continue;
        }
        setupShaderForPass(*shader, shaderID, matrices);
        loc_m = uniformOrNone(*shader, "m_matrix");
        loc_mv = uniformOrNone(*shader, "mv_matrix");
        loc_pmv = uniformOrNone(*shader, "pmv_matrix");
        loc_normal = uniformOrNone(*shader, "normal_matrix");
        isShadowPass = shaderID == DefaultShaderID::TRIANGLES_SHADOW_1ST ||
                       shaderID == DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_1ST ||
                       shaderID == DefaultShaderID::TRIANGLES_SHADOW_2ND ||
                       shaderID == DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_2ND;
      }

      // Upload per-object matrix uniforms
      if (loc_m >= 0)
      {
        glUniformMatrix4fv(loc_m, 1, IS_TRANSPOSED, e.m_matrix.data());
      }
      if (loc_mv >= 0)
      {
        glUniformMatrix4fv(loc_mv, 1, IS_TRANSPOSED, e.mv_matrix.data());
      }
      if (loc_pmv >= 0)
      {
        glUniformMatrix4fv(loc_pmv, 1, IS_TRANSPOSED, e.pmv_matrix.data());
      }
      if (loc_normal >= 0)
      {
        // Normals transform with the inverse transpose of the model
        // rotation and scale, which differs from it for non-uniform scales.
        mrpt::math::CMatrixFloat33 N;
        N.asEigen() = e.m_matrix.asEigen().block<3, 3>(0, 0);
        if (std::abs(N.asEigen().determinant()) > 1e-12f)
        {
          N.asEigen() = N.asEigen().inverse().transpose().eval();
        }
        glUniformMatrix3fv(loc_normal, 1, IS_TRANSPOSED, N.data());
      }

      objState.m_matrix = e.m_matrix;
      objState.mv_matrix = e.mv_matrix;
      objState.pmv_matrix = e.pmv_matrix;

      RenderContext rc;
      rc.shader = shader.get();
      rc.shader_id = shaderID;
      rc.state = &objState;
      rc.lights = &m_lightParams;
      rc.isShadowMapPass = isShadowPass;

      e.proxy->render(rc);
      stats.numDrawCalls++;
    }
  }
#endif
  MRPT_END
}

void CompiledViewport::renderBorder(ShaderProgramManager& shaderManager)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (m_borderWidth == 0)
  {
    return;
  }
  auto shader = shaderManager.getProgram(DefaultShaderID::WIREFRAME);
  if (!shader)
  {
    return;
  }
  shader->use();

  // Build an orthographic projection: (0,0) bottom-left to (w,h) top-right
  const float w = static_cast<float>(m_pixelWidth);
  const float h = static_cast<float>(m_pixelHeight);

  // Orthographic projection matrix (column-major for OpenGL, but MRPT
  // matrices are row-major and uploaded with GL_TRUE transpose)
  mrpt::math::CMatrixFloat44 ortho = mrpt::math::CMatrixFloat44::Zero();
  ortho(0, 0) = 2.0f / w;
  ortho(1, 1) = 2.0f / h;
  ortho(2, 2) = -1.0f;
  ortho(0, 3) = -1.0f;
  ortho(1, 3) = -1.0f;
  ortho(3, 3) = 1.0f;

  // Identity model-view matrix
  mrpt::math::CMatrixFloat44 identity = mrpt::math::CMatrixFloat44::Identity();

  const auto IS_TRANSPOSED = GL_TRUE;

  glUniformMatrix4fv(shader->uniformId("p_matrix"), 1, IS_TRANSPOSED, ortho.data());
  glUniformMatrix4fv(shader->uniformId("mv_matrix"), 1, IS_TRANSPOSED, identity.data());

  // Render the border as 4 filled quads (2 triangles each) to avoid glLineWidth,
  // which is restricted to 1.0 in OpenGL 3.1 Core Profile (any other value
  // generates GL_INVALID_VALUE on Mesa/core profile drivers).
  //
  // Layout (bw = borderWidth):
  //   bottom strip: (0,0)-(w,bw)
  //   top strip:    (0,h-bw)-(w,h)
  //   left strip:   (0,bw)-(bw,h-bw)
  //   right strip:  (w-bw,bw)-(w,h-bw)
  const float bw = static_cast<float>(m_borderWidth);
  const mrpt::img::TColor bc = m_borderColor;

  // 4 strips × 2 triangles × 3 vertices = 24 vertices
  std::array<mrpt::math::TPoint3Df, 24> verts;
  std::array<mrpt::img::TColor, 24> cols;
  cols.fill(bc);

  auto addQuad =
      [&](int idx, float ax, float ay, float bxq, float byq, float cx, float cy, float dx, float dy)
  {
    // Two triangles: (a,b,c) and (a,c,d)
    verts[idx + 0] = {ax, ay, 0};
    verts[idx + 1] = {bxq, byq, 0};
    verts[idx + 2] = {cx, cy, 0};
    verts[idx + 3] = {ax, ay, 0};
    verts[idx + 4] = {cx, cy, 0};
    verts[idx + 5] = {dx, dy, 0};
  };

  addQuad(0, 0, 0, w, 0, w, bw, 0, bw);                       // bottom
  addQuad(6, 0, h - bw, w, h - bw, w, h, 0, h);               // top
  addQuad(12, 0, bw, bw, bw, bw, h - bw, 0, h - bw);          // left
  addQuad(18, w - bw, bw, w, bw, w, h - bw, w - bw, h - bw);  // right

  // Upload and draw
  m_borderVAO.createOnce();
  m_borderVAO.bind();

  m_borderVertexBuffer.createOnce();
  m_borderVertexBuffer.bind();
  m_borderVertexBuffer.allocate(verts.data(), sizeof(verts));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TPoint3Df), nullptr);

  m_borderColorBuffer.createOnce();
  m_borderColorBuffer.bind();
  m_borderColorBuffer.allocate(cols.data(), sizeof(cols));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(mrpt::img::TColor), nullptr);

  glDisable(GL_DEPTH_TEST);
  glDrawArrays(GL_TRIANGLES, 0, 24);
  glEnable(GL_DEPTH_TEST);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
  MRPT_END
}

void CompiledViewport::renderTextOverlays(ShaderProgramManager& shaderManager)
{
  MRPT_START
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!m_sourceVizViewport)
  {
    return;
  }
  const auto& textMsgs = m_sourceVizViewport->getTextMessages();
  std::shared_lock<std::shared_mutex> lckRead(textMsgs.mtx.data);

  if (textMsgs.messages.empty())
  {
    return;
  }
  auto shader = shaderManager.getProgram(DefaultShaderID::TRIANGLES_NO_LIGHT);
  if (!shader)
  {
    return;
  }
  shader->use();

  // Orthographic projection: (0,0) bottom-left to (w,h) top-right.
  // The pmv_matrix is the only uniform used by this shader.
  const float w = static_cast<float>(m_pixelWidth);
  const float h = static_cast<float>(m_pixelHeight);

  mrpt::math::CMatrixFloat44 ortho = mrpt::math::CMatrixFloat44::Zero();
  ortho(0, 0) = 2.0f / w;
  ortho(1, 1) = 2.0f / h;
  ortho(2, 2) = -1.0f;
  ortho(0, 3) = -1.0f;
  ortho(1, 3) = -1.0f;
  ortho(3, 3) = 1.0f;

  const auto IS_TRANSPOSED = GL_TRUE;

  glUniformMatrix4fv(shader->uniformId("pmv_matrix"), 1, IS_TRANSPOSED, ortho.data());

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Helper lambda: generate triangles for a text string, upload to GPU, draw
  auto drawTextBatch = [&](const std::vector<mrpt::viz::TTriangle>& tris)
  {
    if (tris.empty())
    {
      return;
    }
    const size_t vertexCount = tris.size() * 3;
    std::vector<mrpt::math::TPoint3Df> vertices;
    std::vector<mrpt::img::TColor> colors;
    vertices.reserve(vertexCount);
    colors.reserve(vertexCount);

    for (const auto& tri : tris)
    {
      for (int i = 0; i < 3; ++i)
      {
        vertices.push_back(tri.vertices[i].xyzrgba.pt);
        const auto& rgba = tri.vertices[i].xyzrgba;
        colors.emplace_back(rgba.r, rgba.g, rgba.b, rgba.a);
      }
    }

    m_textVAO.createOnce();
    m_textVAO.bind();

    m_textVertexBuffer.createOnce();
    m_textVertexBuffer.bind();
    m_textVertexBuffer.allocate(
        vertices.data(), static_cast<int>(sizeof(mrpt::math::TPoint3Df) * vertexCount));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TPoint3Df), nullptr);

    m_textColorBuffer.createOnce();
    m_textColorBuffer.bind();
    m_textColorBuffer.allocate(
        colors.data(), static_cast<int>(sizeof(mrpt::img::TColor) * vertexCount));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(mrpt::img::TColor), nullptr);

    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));
  };

  for (const auto& [id, msg] : textMsgs.messages)
  {
    if (msg.text.empty()) continue;

    // Compute pixel position from fractional/absolute coordinates.
    // X: [0,1) = fraction of width, >=1 = absolute pixels, <0 = from right
    // Y: [0,1) = fraction of height from bottom, >=1 = pixels from bottom,
    //    <0 = from top
    float px, py;

    if (msg.x >= 0 && msg.x < 1)
      px = static_cast<float>(msg.x * w);
    else if (msg.x < 0)
      px = static_cast<float>(w + msg.x);
    else
      px = static_cast<float>(msg.x);

    if (msg.y >= 0 && msg.y < 1)
      py = static_cast<float>(msg.y * h);
    else if (msg.y < 0)
      py = static_cast<float>(h + msg.y);
    else
      py = static_cast<float>(msg.y);

    internal::glSetFont(msg.vfont_name);

    const mrpt::img::TColor textColor(
        static_cast<uint8_t>(msg.color.R * 255), static_cast<uint8_t>(msg.color.G * 255),
        static_cast<uint8_t>(msg.color.B * 255), static_cast<uint8_t>(msg.color.A * 255));

    const float textScale = msg.vfont_scale;

    // Shadow pass (rendered first, behind main text)
    if (msg.draw_shadow)
    {
      std::vector<mrpt::viz::TTriangle> shadowTris;
      std::vector<mrpt::math::TPoint3Df> tmpLines;
      std::vector<mrpt::img::TColor> tmpLineColors;

      const mrpt::img::TColor shadowColor(
          static_cast<uint8_t>(msg.shadow_color.R * 255),
          static_cast<uint8_t>(msg.shadow_color.G * 255),
          static_cast<uint8_t>(msg.shadow_color.B * 255),
          static_cast<uint8_t>(msg.shadow_color.A * 255));

      const auto shadowPose = mrpt::poses::CPose3D::FromTranslation(px + 1.0, py - 1.0, 0);

      internal::glDrawTextTransformed(
          msg.text, shadowTris, tmpLines, tmpLineColors, shadowPose, textScale, shadowColor,
          msg.vfont_style, msg.vfont_spacing, msg.vfont_kerning);

      drawTextBatch(shadowTris);
    }

    // Main text pass
    std::vector<mrpt::viz::TTriangle> tris;
    std::vector<mrpt::math::TPoint3Df> tmpLines;
    std::vector<mrpt::img::TColor> tmpLineColors;

    const auto textPose = mrpt::poses::CPose3D::FromTranslation(px, py, 0);

    internal::glDrawTextTransformed(
        msg.text, tris, tmpLines, tmpLineColors, textPose, textScale, textColor, msg.vfont_style,
        msg.vfont_spacing, msg.vfont_kerning);

    drawTextBatch(tris);
  }

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glEnable(GL_DEPTH_TEST);

  CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
  MRPT_END
}
