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
#include <mrpt/opengl/FramebufferObject.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/ShaderProgramManager.h>
#include <mrpt/opengl/opengl_api.h>

#include <iostream>

using namespace mrpt::opengl;
using namespace mrpt::viz;
using namespace mrpt::math;

static const bool VIEWPORT_VERBOSE = mrpt::get_env<bool>("MRPT_VIEWPORT_VERBOSE", false);

// ========== CompiledViewport ==========

CompiledViewport::CompiledViewport(const std::string& name) : m_name(name)
{
  if (VIEWPORT_VERBOSE) std::cout << "[CompiledViewport] Created: '" << m_name << "'\n";
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
    RenderableProxy::Ptr proxy, const mrpt::viz::CVisualObject* sourceObj)
{
  MRPT_START

  if (!proxy) return;

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  // Store in main list
  m_proxies.push_back(proxy);

  // Organize by shader for efficient rendering
  const auto shaderIDs = proxy->requiredShaders();
  for (auto shaderID : shaderIDs)
  {
    m_proxiesByShader[shaderID].push_back(proxy);
  }

  // Track object-to-proxy mapping
  if (sourceObj)
  {
    m_objectToProxy[sourceObj] = proxy;
  }

  if (VIEWPORT_VERBOSE)
  {
    std::cout << "[CompiledViewport::addProxy] '" << m_name << "' now has " << m_proxies.size()
              << " proxies\n";
  }

  MRPT_END
}

void CompiledViewport::removeProxy(RenderableProxy::Ptr proxy)
{
  MRPT_START

  if (!proxy) return;

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
      it = m_objectToProxy.erase(it);
    else
      ++it;
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
    std::cout << "[CompiledViewport::setImageViewMode] '" << m_name
              << "' entered image view mode\n";

  MRPT_END
}

void CompiledViewport::clearImageViewMode()
{
  MRPT_START

  std::unique_lock<std::shared_mutex> lock(m_stateMtx.data);

  m_imageViewProxy.reset();

  if (VIEWPORT_VERBOSE)
    std::cout << "[CompiledViewport::clearImageViewMode] '" << m_name
              << "' exited image view mode\n";

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
    std::cout << "[CompiledViewport::setCloneMode] '" << m_name << "' cloning from '"
              << clonedViewportName << "'" << (cloneCamera ? " (with camera)" : "") << "\n";

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

  if (shadowMapSizeX > 0) m_shadowMapSizeX = shadowMapSizeX;
  if (shadowMapSizeY > 0) m_shadowMapSizeY = shadowMapSizeY;

  if (enabled && !m_shadowMapFBO)
  {
    // Shadow FBO will be created on first render
  }
  else if (!enabled)
  {
    m_shadowMapFBO.reset();
  }

  if (VIEWPORT_VERBOSE)
    std::cout << "[CompiledViewport::enableShadows] '" << m_name << "' shadows "
              << (enabled ? "enabled" : "disabled") << "\n";

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
      std::cout << "[CompiledViewport::updateIfNeeded] '" << m_name << "' camera changed\n";
  }

  // Check lighting changes
  LightState currentLight;
  currentLight.direction = m_lightParams.direction;
  currentLight.ambient = m_lightParams.ambient;
  currentLight.diffuse = m_lightParams.diffuse;

  if (currentLight != m_lastLightState)
  {
    m_lastLightState = currentLight;
    anyUpdates = true;

    if (VIEWPORT_VERBOSE)
      std::cout << "[CompiledViewport::updateIfNeeded] '" << m_name << "' lighting changed\n";
  }

  return anyUpdates;

  MRPT_END
}

bool CompiledViewport::hasPendingUpdates() const
{
  std::shared_lock<std::shared_mutex> lock(m_stateMtx.data);

  return m_matricesNeedUpdate;
}
