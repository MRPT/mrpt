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
#pragma once

#include <mrpt/containers/NonCopiableData.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/opengl/FrameBuffer.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/RenderableProxy.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/TLightParameters.h>
#include <mrpt/viz/Viewport.h>

#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

namespace mrpt::opengl
{
class ShaderProgramManager;

/** Rendering statistics for a single viewport.
 * \ingroup mrpt_opengl_grp
 */
struct ViewportRenderStats
{
  size_t numProxiesRendered = 0;
  size_t numProxiesCulled = 0;
  size_t numDrawCalls = 0;
  double renderTimeMs = 0.0;

  void reset()
  {
    numProxiesRendered = 0;
    numProxiesCulled = 0;
    numDrawCalls = 0;
    renderTimeMs = 0.0;
  }
};

/** A compiled, GPU-ready representation of a mrpt::viz::Viewport.
 *
 * This class is the rendering engine for a single viewport. It maintains:
 * - Compiled proxies for all renderable objects in the viewport
 * - Camera state and projection matrices
 * - Lighting parameters
 * - Special rendering modes (image view, cloned view, etc.)
 * - Shadow mapping framebuffers (if enabled)
 *
 * Key features:
 * - **Normal 3D rendering**: Standard scene with objects, camera, lighting
 * - **Image view mode**: Efficient rendering of 2D images (for video streams)
 * - **Cloned viewports**: Share objects/camera from another viewport
 * - **Shadow mapping**: Two-pass rendering for directional shadows
 * - **Frustum culling**: Automatically skips objects outside camera view
 *
 * The viewport can operate in several modes:
 * 1. Normal: Renders its own objects with its own camera
 * 2. Image view: Displays a textured quad (for images/video)
 * 3. Cloned objects: Renders objects from another viewport
 * 4. Cloned camera: Uses camera from another viewport
 *
 * \sa CompiledScene, RenderableProxy, mrpt::viz::Viewport
 * \ingroup mrpt_opengl_grp
 */
class CompiledViewport
{
 public:
  using Ptr = std::shared_ptr<CompiledViewport>;

  /** Constructor.
   * \param name Viewport name (must match the name in viz::Viewport)
   */
  explicit CompiledViewport(const std::string& name);

  ~CompiledViewport();

  /** @name Viewport Configuration
   * @{ */

  /** Synchronizes this compiled viewport with its source viz::Viewport.
   *
   * Copies all configuration: camera, lights, rendering options, viewport
   * bounds, special modes, etc.
   *
   * This is called automatically by CompiledScene during compilation.
   */
  void updateFromVizViewport(const mrpt::viz::Viewport& vizVp);

  /** Returns the viewport name */
  const std::string& getName() const { return m_name; }

  /** @} */

  /** @name Proxy Management
   * @{ */

  /** Adds a renderable proxy to this viewport.
   *
   * \param proxy The GPU-side representation of an object
   * \param sourceObj The original viz object (tracked via weak_ptr)
   */
  void addProxy(
      const RenderableProxy::Ptr& proxy,
      const std::shared_ptr<mrpt::viz::CVisualObject>& sourceObj);

  /** Removes proxies whose source objects have been deleted.
   * \return Number of orphaned proxies removed
   */
  size_t cleanupOrphanedProxies();

  /** Removes a proxy from this viewport */
  void removeProxy(const RenderableProxy::Ptr& proxy);

  /** Removes all proxies */
  void clearProxies();

  /** Number of proxies in this viewport */
  size_t getProxyCount() const { return m_proxies.size(); }

  /** @} */

  /** @name Rendering
   * @{ */

  /** Renders this viewport.
   *
   * \param renderWidth Full window width in pixels
   * \param renderHeight Full window height in pixels
   * \param renderOffsetX X offset for multi-window rendering
   * \param renderOffsetY Y offset for multi-window rendering
   * \param shaderManager Shader program manager for binding programs
   *
   * This handles:
   * - Viewport positioning and clipping
   * - Background color clearing
   * - Shadow map rendering (if enabled)
   * - Normal scene rendering
   * - Image view rendering (if in image mode)
   * - Text overlay rendering
   * - Viewport border rendering
   */
  void render(
      int renderWidth,
      int renderHeight,
      int renderOffsetX,
      int renderOffsetY,
      ShaderProgramManager& shaderManager);

  /** @} */

  /** @name State Updates
   * @{ */

  /** Checks if viewport configuration has changed and updates if needed.
   *
   * This monitors changes in:
   * - Camera position/orientation
   * - Lighting parameters
   * - Viewport dimensions
   *
   * \return true if any updates were performed
   */
  bool updateIfNeeded();

  /** Returns true if there are pending configuration changes */
  bool hasPendingUpdates() const;

  /** Forces regeneration of all projection/view matrices */
  void forceMatrixUpdate() { m_matricesNeedUpdate = true; }

  /** @} */

  /** @name Special Rendering Modes
   * @{ */

  /** Enables/disables image view mode.
   *
   * When enabled, the viewport displays a single textured quad instead
   * of 3D objects. Used for efficient video/image display.
   *
   * \param imageProxy The textured quad proxy (created from CTexturedPlane)
   */
  void setImageViewMode(RenderableProxy::Ptr imageProxy);

  /** Disables image view mode, returns to normal 3D rendering */
  void clearImageViewMode();

  /** Returns true if viewport is in image view mode */
  bool isImageViewMode() const { return m_imageViewProxy != nullptr; }

  /** Sets this viewport to clone objects from another viewport.
   *
   * \param clonedViewportName Name of viewport to clone from
   * \param cloneCamera If true, also clone camera settings
   */
  void setCloneMode(const std::string& clonedViewportName, bool cloneCamera = false);

  /** Disables cloning */
  void clearCloneMode();

  /** Returns true if this viewport clones objects from another */
  bool isCloningObjects() const { return m_isCloned; }

  /** Returns true if this viewport clones camera from another */
  bool isCloningCamera() const { return m_isClonedCamera; }

  /** Returns name of cloned viewport, or empty if not cloning */
  const std::string& getClonedViewportName() const { return m_clonedViewportName; }

  /** @} */

  /** @name Shadow Mapping
   * @{ */

  /** Enables/disables shadow casting for this viewport.
   *
   * \param enabled Enable shadow rendering
   * \param shadowMapSizeX Shadow map texture width (default 2048)
   * \param shadowMapSizeY Shadow map texture height (default 2048)
   */
  void enableShadows(
      bool enabled, unsigned int shadowMapSizeX = 2048, unsigned int shadowMapSizeY = 2048);

  /** Returns true if shadow casting is enabled */
  bool areShadowsEnabled() const { return m_shadowsEnabled; }

  /** @} */

  /** @name Camera and Matrices
   * @{ */

  /** Updates camera state from a viz::CCamera */
  void updateCamera(const mrpt::viz::CCamera& camera);

  /** Returns current render matrices (projection, view, etc.) */
  const TRenderMatrices& getRenderMatrices() const { return m_renderMatrices; }

  /** Direct access to render matrices (for manual manipulation) */
  TRenderMatrices& getRenderMatrices() { return m_renderMatrices; }

  /** @} */

  /** @name Configuration Access
   * @{ */

  /** Viewport position and size (normalized or pixel coordinates) */
  void setViewportBounds(double x, double y, double width, double height);
  void getViewportBounds(double& x, double& y, double& width, double& height) const;

  /** Set near/far clip planes */
  void setClipPlanes(float nearPlane, float farPlane);
  void getClipPlanes(float& nearPlane, float& farPlane) const;

  /** Set background color */
  void setBackgroundColor(const mrpt::img::TColorf& color) { m_backgroundColor = color; }
  const mrpt::img::TColorf& getBackgroundColor() const { return m_backgroundColor; }

  /** Set transparent rendering (doesn't clear color buffer) */
  void setTransparent(bool transparent) { m_isTransparent = transparent; }
  bool isTransparent() const { return m_isTransparent; }

  /** Set viewport border */
  void setBorder(unsigned int width, const mrpt::img::TColor& color);
  unsigned int getBorderWidth() const { return m_borderWidth; }
  const mrpt::img::TColor& getBorderColor() const { return m_borderColor; }

  /** Set viewport visibility */
  void setVisible(bool visible) { m_isVisible = visible; }
  bool isVisible() const { return m_isVisible; }

  /** Access lighting parameters */
  mrpt::viz::TLightParameters& lightParameters() { return m_lightParams; }
  const mrpt::viz::TLightParameters& lightParameters() const { return m_lightParams; }

  /** Last rendering statistics */
  const ViewportRenderStats& lastRenderStats() const { return m_lastStats; }

  /** @} */

 private:
  /** Viewport name (matches viz::Viewport) */
  std::string m_name;

  /** All renderable proxies in this viewport, organized by shader type.
   * This allows efficient batching: all objects using the same shader
   * are rendered together to minimize state changes.
   */
  std::map<shader_id_t, std::vector<RenderableProxy::Ptr>> m_proxiesByShader;

  /** All proxies in insertion order (for debug/iteration) */
  std::vector<RenderableProxy::Ptr> m_proxies;

  /** Mapping from source objects to their proxies (using weak_ptr for safety).
   * This allows detection of deleted source objects for cleanup.
   */
  std::map<
      std::weak_ptr<mrpt::viz::CVisualObject>,
      RenderableProxy::Ptr,
      std::owner_less<std::weak_ptr<mrpt::viz::CVisualObject>>>
      m_objectToProxy;

  /** @name Viewport Configuration
   * @{ */

  /** Viewport position (0-1 normalized or >1 pixels) */
  double m_viewX = 0.0, m_viewY = 0.0, m_viewWidth = 1.0, m_viewHeight = 1.0;

  /** Computed viewport in pixels (updated each frame) */
  int m_pixelX = 0, m_pixelY = 0, m_pixelWidth = 0, m_pixelHeight = 0;

  /** Near/far clip planes */
  float m_clipNear = 0.01f, m_clipFar = 1000.0f;

  /** Light shadow clip planes */
  float m_lightShadowClipNear = 0.01f, m_lightShadowClipFar = 1000.0f;

  /** Background color */
  mrpt::img::TColorf m_backgroundColor{0.4f, 0.4f, 0.4f, 1.0f};

  /** Transparent rendering (don't clear color buffer) */
  bool m_isTransparent = false;

  /** Border rendering */
  unsigned int m_borderWidth = 0;
  mrpt::img::TColor m_borderColor{255, 255, 255, 255};
  RenderableProxy::Ptr m_borderProxy;  // Lines proxy for border

  /** Viewport visibility */
  bool m_isVisible = true;

  /** OpenGL rendering options */
  bool m_enablePolygonSmooth = true;

  /** @} */

  /** @name Camera and Lighting
   * @{ */

  /** Current camera configuration */
  mrpt::viz::CCamera m_camera;

  /** Cached camera state (for detecting changes) */
  struct CameraState
  {
    mrpt::math::TPose3D pose;
    double zoomDistance = 1.0;
    double azimuth = 0.0;
    double elevation = 0.0;
    double FOV = 60.0;
    bool is6DOF = false;

    bool operator!=(const CameraState& other) const;
  };
  CameraState m_lastCameraState;

  /** Lighting parameters */
  mrpt::viz::TLightParameters m_lightParams;

  /** Cached lighting state (for detecting changes) */
  struct LightState
  {
    mrpt::math::TVector3Df direction;
    mrpt::img::TColorf ambient;
    mrpt::img::TColorf diffuse;

    bool operator!=(const LightState& other) const;
  };
  LightState m_lastLightState;

  /** Render matrices (projection, view, model, etc.) */
  TRenderMatrices m_renderMatrices;

  /** Flag indicating matrices need recomputation */
  bool m_matricesNeedUpdate = true;

  /** @} */

  /** @name Special Rendering Modes
   * @{ */

  /** Image view mode: efficient 2D image rendering */
  RenderableProxy::Ptr m_imageViewProxy;

  /** Cloned viewport mode */
  bool m_isCloned = false;
  bool m_isClonedCamera = false;
  std::string m_clonedViewportName;

  /** @} */

  /** @name Shadow Mapping
   * @{ */

  /** Shadow casting enabled */
  bool m_shadowsEnabled = false;

  /** Shadow map dimensions */
  unsigned int m_shadowMapSizeX = 2048;
  unsigned int m_shadowMapSizeY = 2048;

  /** Shadow map framebuffer (created on demand) */
  std::unique_ptr<FrameBuffer> m_shadowMapFBO;

  /** @} */

  /** @name Rendering State
   * @{ */

  /** Statistics from last render */
  ViewportRenderStats m_lastStats;

  /** Thread-safe access to mutable state */
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_stateMtx;

  /** @} */

  /** @name Internal Rendering Methods
   * @{ */

  /** Computes pixel coordinates from normalized viewport bounds */
  void computePixelViewport(int windowWidth, int windowHeight, int offsetX, int offsetY);

  /** Updates projection and view matrices from camera */
  void updateMatrices();

  /** Performs shadow map rendering (1st pass) */
  void renderShadowMap(ShaderProgramManager& shaderManager);

  /** Performs normal scene rendering */
  void renderNormalScene(ShaderProgramManager& shaderManager, bool isShadowMapPass);

  /** Renders in image view mode */
  void renderImageView(ShaderProgramManager& shaderManager);

  /** Renders viewport border */
  void renderBorder(ShaderProgramManager& shaderManager);

  /** Builds render queue with frustum culling */
  void buildRenderQueue(
      RenderQueue& queue,
      const TRenderMatrices& matrices,
      bool isShadowMapPass,
      ViewportRenderStats& stats);

  /** Processes render queue (binds shaders, renders objects) */
  void processRenderQueue(
      const RenderQueue& queue,
      ShaderProgramManager& shaderManager,
      const TRenderMatrices& matrices,
      ViewportRenderStats& stats);

  /** Helper: converts normalized/pixel viewport coordinates */
  static int startFromRatio(double frac, int dimension);
  static int sizeFromRatio(int startCoord, double size, int dimension);

  /** @} */

 public:
  // Disable copy/move
  CompiledViewport(const CompiledViewport&) = delete;
  CompiledViewport& operator=(const CompiledViewport&) = delete;
  CompiledViewport(CompiledViewport&&) = delete;
  CompiledViewport& operator=(CompiledViewport&&) = delete;
};

}  // namespace mrpt::opengl
