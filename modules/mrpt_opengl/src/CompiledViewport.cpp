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

#include <Eigen/Dense>
#include <iostream>

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
}

void CompiledViewport::updateFromVizViewport(const mrpt::viz::Viewport& vizVp)
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  // Copy viewport bounds
  vizVp.getViewportPosition(m_viewX, m_viewY, m_viewWidth, m_viewHeight);

  // Copy camera
  m_camera = vizVp.getCamera();
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

void CompiledViewport::addProxy(
    const RenderableProxy::Ptr& proxy, const std::shared_ptr<mrpt::viz::CVisualObject>& sourceObj)
{
  MRPT_START

  if (!proxy)
  {
    return;
  }

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  // Store in main list
  m_proxies.push_back(proxy);

  // Organize by shader for efficient rendering
  const auto shaderIDs = proxy->requiredShaders();
  for (auto shaderID : shaderIDs)
  {
    m_proxiesByShader[shaderID].push_back(proxy);
  }

  // Track object-to-proxy mapping using weak_ptr
  if (sourceObj)
  {
    std::weak_ptr<mrpt::viz::CVisualObject> objWeak = sourceObj;
    m_objectToProxy[objWeak] = proxy;
  }

  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::addProxy] '" << m_name << "' now has " << m_proxies.size()
              << " proxies\n";
  }

  MRPT_END
}

size_t CompiledViewport::cleanupOrphanedProxies()
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  size_t numRemoved = 0;

  // Find proxies whose source objects have been deleted
  for (auto it = m_objectToProxy.begin(); it != m_objectToProxy.end();)
  {
    if (it->first.expired())
    {
      // Object was deleted - remove this proxy
      auto& proxy = it->second;

      // Remove from main list
      m_proxies.erase(std::remove(m_proxies.begin(), m_proxies.end(), proxy), m_proxies.end());

      // Remove from shader-organized lists
      for (auto& [shaderID, proxyList] : m_proxiesByShader)
      {
        proxyList.erase(std::remove(proxyList.begin(), proxyList.end(), proxy), proxyList.end());
      }

      it = m_objectToProxy.erase(it);
      numRemoved++;

      if (VIEWPORT_VERBOSE)
      {
        std::cout << "[CompiledViewport::cleanupOrphanedProxies] '" << m_name
                  << "' removed orphaned proxy\n";
      }
    }
    else
    {
      ++it;
    }
  }

  return numRemoved;

  MRPT_END
}

void CompiledViewport::updateProxiesForObject(
    const std::weak_ptr<mrpt::viz::CVisualObject>& weakObj,
    const mrpt::viz::CVisualObject* sourceObj,
    const mrpt::math::CMatrixFloat44& modelMatrix,
    bool effectiveVisible)
{
  MRPT_START

  // Find all proxies associated with this source object in this viewport
  // and update them. Note: m_objectToProxy only stores one proxy per obj,
  // but the proxies list may have multiple for the same source (multi-mixin).
  // We iterate m_proxies to find all that share the same source.
  for (auto& proxy : m_proxies)
  {
    if (!proxy) continue;

    auto src = proxy->getSourceObject();
    if (!src) continue;

    // Check if this proxy belongs to the same source object
    auto srcWeak = std::weak_ptr<mrpt::viz::CVisualObject>(src);
    if (!(std::owner_less<std::weak_ptr<mrpt::viz::CVisualObject>>{}(srcWeak, weakObj) ||
          std::owner_less<std::weak_ptr<mrpt::viz::CVisualObject>>{}(weakObj, srcWeak)))
    {
      // Same object. Update model matrix, visibility, and buffers
      proxy->m_modelMatrix = modelMatrix;
      proxy->m_visible = effectiveVisible;
      proxy->updateBuffers(sourceObj);
    }
  }

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

  // Remove from main list
  m_proxies.erase(std::remove(m_proxies.begin(), m_proxies.end(), proxy), m_proxies.end());

  // Remove from shader-organized lists
  for (auto& [shaderID, proxyList] : m_proxiesByShader)
  {
    proxyList.erase(std::remove(proxyList.begin(), proxyList.end(), proxy), proxyList.end());
  }

  // Remove from object mapping
  for (auto it = m_objectToProxy.begin(); it != m_objectToProxy.end();)
  {
    if (it->second == proxy)
    {
      it = m_objectToProxy.erase(it);
    }
    else
    {
      ++it;
    }
  }

  MRPT_END
}

void CompiledViewport::clearProxies()
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_proxies.clear();
  m_proxiesByShader.clear();
  m_objectToProxy.clear();

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

  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::setCloneMode] '" << m_name << "' cloning from '"
              << clonedViewportName << "'" << (cloneCamera ? " (with camera)" : "") << "\n";
  }

  MRPT_END
}

void CompiledViewport::clearCloneMode()
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_isCloned = false;
  m_isClonedCamera = false;
  m_clonedViewportName.clear();

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

  if (enabled && !m_shadowMapFBO)
  {
    // Shadow FBO will be created on first render
  }
  else if (!enabled)
  {
    m_shadowMapFBO.reset();
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

  m_camera = camera;
  m_matricesNeedUpdate = true;

  MRPT_END
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

bool CompiledViewport::CameraState::operator!=(const CameraState& other) const
{
  return pose != other.pose || zoomDistance != other.zoomDistance || azimuth != other.azimuth ||
         elevation != other.elevation || FOV != other.FOV || is6DOF != other.is6DOF;
}

bool CompiledViewport::LightState::operator!=(const LightState& other) const
{
  return direction != other.direction || ambient != other.ambient || diffuse != other.diffuse;
}

bool CompiledViewport::updateIfNeeded()
{
  MRPT_START

  bool anyUpdates = false;

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  // Clean up orphaned proxies first
  if (cleanupOrphanedProxies() > 0)
  {
    anyUpdates = true;
  }

  // Check camera changes
  CameraState currentCamera;
  currentCamera.pose = m_camera.getPose();
  currentCamera.zoomDistance = m_camera.getZoomDistance();
  currentCamera.azimuth = m_camera.getAzimuthDegrees();
  currentCamera.elevation = m_camera.getElevationDegrees();
  currentCamera.FOV = m_camera.getProjectiveFOVdeg();
  currentCamera.is6DOF = m_camera.is6DOFMode();

  if (currentCamera != m_lastCameraState)
  {
    m_lastCameraState = currentCamera;
    m_matricesNeedUpdate = true;
    anyUpdates = true;

    if (VIEWPORT_VERBOSE)
    {
      std::cout << "[CompiledViewport::updateIfNeeded] '" << m_name << "' camera changed\n";
    }
  }

  // Check lighting changes
  LightState currentLight;
  currentLight.direction = m_lightParams.direction;
  currentLight.ambient = {m_lightParams.ambient, m_lightParams.ambient, m_lightParams.ambient};
  currentLight.diffuse = {m_lightParams.diffuse, m_lightParams.diffuse, m_lightParams.diffuse};

  if (currentLight != m_lastLightState)
  {
    m_lastLightState = currentLight;
    anyUpdates = true;

    if (VIEWPORT_VERBOSE)
    {
      std::cout << "[CompiledViewport::updateIfNeeded] '" << m_name << "' lighting changed\n";
    }
  }

  return anyUpdates;

  MRPT_END
}

bool CompiledViewport::hasPendingUpdates() const
{
  std::shared_lock<std::shared_mutex> lock(m_stateMtx.data);

  return m_matricesNeedUpdate;
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
  if (m_camera.isNoProjection())
  {
    m_renderMatrices.computeNoProjectionMatrix(m_clipNear, m_clipFar);
  }
  else
  {
    m_renderMatrices.is_projective = m_camera.isProjective();
    m_renderMatrices.FOV = m_camera.getProjectiveFOVdeg();
    m_renderMatrices.eyeDistance = m_camera.getZoomDistance();
    if (m_camera.hasPinholeModel())
      m_renderMatrices.pinhole_model = m_camera.getPinholeModel();
    else
      m_renderMatrices.pinhole_model.reset();
    if (m_camera.is6DOFMode())
    {
      const auto pose = m_camera.getPose();
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
      m_renderMatrices.pointing = m_camera.getPointingAt();
      m_renderMatrices.azimuth = DEG2RAD(m_camera.getAzimuthDegrees());
      m_renderMatrices.elev = DEG2RAD(m_camera.getElevationDegrees());

      const auto c2m_u = mrpt::math::TVector3D(
          cos(m_renderMatrices.azimuth) * cos(m_renderMatrices.elev),
          sin(m_renderMatrices.azimuth) * cos(m_renderMatrices.elev), sin(m_renderMatrices.elev));

      const double dis = std::max<double>(0.001, m_camera.getZoomDistance());
      m_renderMatrices.eye = m_renderMatrices.pointing + c2m_u * dis;

      m_renderMatrices.up.x = -cos(m_renderMatrices.azimuth) * sin(m_renderMatrices.elev);
      m_renderMatrices.up.y = -sin(m_renderMatrices.azimuth) * sin(m_renderMatrices.elev);
      m_renderMatrices.up.z = cos(m_renderMatrices.elev);
    }

    m_renderMatrices.computeProjectionMatrix(m_clipNear, m_clipFar);
    m_renderMatrices.computeViewMatrix();
  }
  // Compute light projection matrices for shadows
  if (m_shadowsEnabled)
  {
    m_renderMatrices.computeLightProjectionMatrix(
        m_lightShadowClipNear, m_lightShadowClipFar, m_lightParams);
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
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!m_isVisible)
  {
    return;
  }
  m_lastStats.reset();
  const auto tStart = mrpt::Clock::nowDouble();
  std::shared_lock<std::shared_mutex> lock(m_stateMtx.data);
  // Compute pixel viewport
  computePixelViewport(renderWidth, renderHeight, renderOffsetX, renderOffsetY);
  // Update matrices if needed
  updateMatrices();

  if (m_flipYProjection)
  {
    // Negate Y row of projection matrix
    for (mrpt::math::matrix_index_t c = 0; c < m_renderMatrices.p_matrix.cols(); c++)
    {
      m_renderMatrices.p_matrix(1, c) *= -1.0f;
    }

    // Compensate for winding order change
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
    // Normal scene rendering
    renderNormalScene(shaderManager, false, proxiesToRender);
  }
  // Render border
  if (m_borderWidth > 0)
  {
    renderBorder(shaderManager);
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
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!m_imageViewProxy)
  {
    return;
  }
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // Simple orthographic projection for image display
  auto matrices = m_renderMatrices;
  matrices.matricesSetIdentity();
  // Render the textured quad
  RenderContext rc;
  rc.shader = shaderManager.getProgram(DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT).get();
  rc.shader_id = DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT;
  rc.state = &matrices;
  rc.lights = &m_lightParams;
  m_imageViewProxy->render(rc);
  m_lastStats.numProxiesRendered = 1;
  m_lastStats.numDrawCalls = 1;
#endif
  MRPT_END
}
void CompiledViewport::renderShadowMap(ShaderProgramManager& shaderManager)
{
  MRPT_START
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  // Create shadow FBO if needed
  if (!m_shadowMapFBO)
  {
    m_shadowMapFBO = std::make_unique<FrameBuffer>();
    m_shadowMapFBO->createDepthMap(m_shadowMapSizeX, m_shadowMapSizeY);
  }
  glEnable(GL_DEPTH_TEST);
  glViewport(0, 0, m_shadowMapSizeX, m_shadowMapSizeY);

  const auto oldFBs = m_shadowMapFBO->bind();
  glClear(GL_DEPTH_BUFFER_BIT);
  // Render scene from light's perspective (1st pass)
  renderNormalScene(shaderManager, true);
  m_shadowMapFBO->Bind(oldFBs);
  // Restore viewport
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
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
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
    if (!proxy)
    {
      continue;
    }

    // Skip invisible objects: check effective visibility (accounts for
    // parent container visibility propagated by updateDirtyObjectRecursive)
    if (!proxy->m_visible)
    {
      continue;
    }

    // Skip if in shadow map pass and object doesn't cast shadows
    if (isShadowMapPass && !proxy->castsShadows())
    {
      continue;
    }

    // TODO: Implement frustum culling using proxy->getBoundingBox()
    // For now, render everything

    const auto shaderIDs = proxy->requiredShaders();
    for (auto shaderID : shaderIDs)
    {
      // Create per-object render state with the object's model matrix
      auto objMatrices = matrices;
      objMatrices.m_matrix = proxy->m_modelMatrix;

      // Precompute derived matrices
      objMatrices.mv_matrix.asEigen() =
          objMatrices.v_matrix.asEigen() * objMatrices.m_matrix.asEigen();
      objMatrices.pmv_matrix.asEigen() =
          objMatrices.p_matrix.asEigen() * objMatrices.mv_matrix.asEigen();

      // Use depth of 0 for now (proper depth sorting would go here)
      queue[shaderID].emplace(0.0f, RenderQueueElement{proxy.get(), objMatrices});
    }

    stats.numProxiesRendered++;
  }
  MRPT_END
}
void CompiledViewport::processRenderQueue(
    const RenderQueue& queue,
    ShaderProgramManager& shaderManager,
    const TRenderMatrices& matrices,
    ViewportRenderStats& stats)
{
  MRPT_START
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  // Clear any prior GL errors
  while (glGetError() != GL_NO_ERROR)
  {
  }

  for (const auto& [shaderID, proxyMap] : queue)
  {
    auto shader = shaderManager.getProgram(shaderID);
    if (!shader)
    {
      continue;
    }
    shader->use();
    stats.numDrawCalls++;

    const auto IS_TRANSPOSED = GL_TRUE;

    // Cache uniform locations for this shader
    const bool has_p = shader->hasUniform("p_matrix");
    const bool has_v = shader->hasUniform("v_matrix");
    const bool has_m = shader->hasUniform("m_matrix");
    const bool has_mv = shader->hasUniform("mv_matrix");
    const bool has_pmv = shader->hasUniform("pmv_matrix");

    // Upload per-shader uniforms that don't change per object (p, v)
    if (has_p)
    {
      glUniformMatrix4fv(shader->uniformId("p_matrix"), 1, IS_TRANSPOSED, matrices.p_matrix.data());
    }

    if (has_v)
    {
      glUniformMatrix4fv(shader->uniformId("v_matrix"), 1, IS_TRANSPOSED, matrices.v_matrix.data());
    }

    // Render all proxies using this shader
    for (const auto& [depth, element] : proxyMap)
    {
      const auto& objState = element.renderState;

      // Upload per-object matrix uniforms
      if (has_m)
      {
        glUniformMatrix4fv(
            shader->uniformId("m_matrix"), 1, IS_TRANSPOSED, objState.m_matrix.data());
      }
      if (has_mv)
      {
        glUniformMatrix4fv(
            shader->uniformId("mv_matrix"), 1, IS_TRANSPOSED, objState.mv_matrix.data());
      }
      if (has_pmv)
      {
        glUniformMatrix4fv(
            shader->uniformId("pmv_matrix"), 1, IS_TRANSPOSED, objState.pmv_matrix.data());
      }

      RenderContext rc;
      rc.shader = shader.get();
      rc.shader_id = shaderID;
      rc.state = &objState;
      rc.lights = &m_lightParams;

      element.proxy->render(rc);
    }
  }
#endif
  MRPT_END
}

void CompiledViewport::renderBorder(ShaderProgramManager& shaderManager)
{
  MRPT_START
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (m_borderWidth == 0) return;

  auto shader = shaderManager.getProgram(DefaultShaderID::WIREFRAME);
  if (!shader) return;

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

  // Half-pixel inset so lines are fully inside the viewport
  const float hw = static_cast<float>(m_borderWidth) * 0.5f;
  const float x0 = hw;
  const float y0 = hw;
  const float x1 = w - hw;
  const float y1 = h - hw;

  // 4 vertices for GL_LINE_LOOP
  const std::array<mrpt::math::TPoint3Df, 4> verts = {
      mrpt::math::TPoint3Df{x0, y0, 0.0f},
      mrpt::math::TPoint3Df{x1, y0, 0.0f},
      mrpt::math::TPoint3Df{x1, y1, 0.0f},
      mrpt::math::TPoint3Df{x0, y1, 0.0f}
  };

  const mrpt::img::TColor bc = m_borderColor;
  const std::array<mrpt::img::TColor, 4> colors = {bc, bc, bc, bc};

  // Create / update border VBO
  m_borderVAO.createOnce();
  m_borderVAO.bind();

  m_borderVertexBuffer.createOnce();
  m_borderVertexBuffer.bind();
  m_borderVertexBuffer.allocate(verts.data(), sizeof(verts));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TPoint3Df), nullptr);

  m_borderColorBuffer.createOnce();
  m_borderColorBuffer.bind();
  m_borderColorBuffer.allocate(colors.data(), sizeof(colors));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(mrpt::img::TColor), nullptr);

  // Render border lines without depth test
  glDisable(GL_DEPTH_TEST);
  glLineWidth(static_cast<float>(m_borderWidth));

  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glLineWidth(1.0f);
  glEnable(GL_DEPTH_TEST);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
  MRPT_END
}
