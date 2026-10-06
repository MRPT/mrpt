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
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/opengl/Buffer.h>
#include <mrpt/opengl/FrameBuffer.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/RenderableProxy.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/VertexArrayObject.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/TLightParameters.h>
#include <mrpt/viz/Viewport.h>

#include <array>
#include <map>
#include <memory>
#include <optional>
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
  /** Cube faces of point/spot light shadow maps rendered (not reused from a
   * previous frame). */
  size_t numPointShadowFacesRendered = 0;
  double renderTimeMs = 0.0;

  void reset()
  {
    numProxiesRendered = 0;
    numProxiesCulled = 0;
    numDrawCalls = 0;
    numPointShadowFacesRendered = 0;
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
   *
   * \param vizVp The source viewport. A non-owning pointer is kept to allow
   *        writing back the rendered viewport size for
   *        get3DRayForPixelCoord().
   */
  void updateFromVizViewport(const mrpt::viz::Viewport& vizVp);

  /** Sets the lights of the mrpt::viz::CLight objects in this viewport, in
   * world coordinates, which are appended to those of the source viewport.
   * This is called automatically by CompiledScene during compilation. */
  void setSceneLights(const std::vector<mrpt::viz::TLight>& lights);

  /** Returns the viewport name */
  const std::string& getName() const { return m_name; }

  /** @} */

  /** @name Proxy Management
   * @{ */

  /** Adds a renderable proxy to this viewport. Its model matrix, visibility,
   * etc. are kept up to date by the owner (CompiledScene).
   *
   * \param proxy The GPU-side representation of an object
   */
  void addProxy(const RenderableProxy::Ptr& proxy);

  /** Removes a proxy from this viewport */
  void removeProxy(const RenderableProxy::Ptr& proxy);

  /** Removes a set of proxies from this viewport, in one pass. */
  void removeProxies(const std::vector<const RenderableProxy*>& proxies);

  /** Removes all proxies */
  void clearProxies();

  /** Number of proxies in this viewport */
  size_t getProxyCount() const { return m_proxies.size(); }

  /** Access to the list of proxies (used by cloned viewports) */
  const std::vector<RenderableProxy::Ptr>& getProxies() const { return m_proxies; }

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
   * \param sourceViewport For cloned viewports, the source viewport whose
   *        proxies should be rendered. nullptr for normal viewports.
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
      ShaderProgramManager& shaderManager,
      const CompiledViewport* sourceViewport = nullptr);

  /** @} */

  /** @name State Updates
   * @{ */

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

  /** Makes this viewport use the camera of another one, while still rendering
   * its own objects.
   * \param viewportName Name of the viewport whose camera is used */
  void setCloneCameraFrom(const std::string& viewportName);

  /** Stops using the camera of another viewport */
  void clearCloneCamera();

  /** Disables cloning, of both objects and camera */
  void clearCloneMode();

  /** Returns true if this viewport clones objects from another */
  bool isCloningObjects() const { return m_isCloned; }

  /** Returns true if this viewport clones camera from another */
  bool isCloningCamera() const { return m_isClonedCamera; }

  /** Returns name of cloned viewport, or empty if not cloning */
  const std::string& getClonedViewportName() const { return m_clonedViewportName; }

  /** Returns the name of the viewport whose camera is used, or empty if the
   * viewport has its own camera */
  const std::string& getCameraSourceViewportName() const { return m_clonedCameraViewportName; }

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

  /** Flip vertically at projection level (useful for FBO rendering) */
  void flipVerticalProjection(bool flipEnabled)
  {
    if (flipEnabled != m_flipYProjection)
    {
      m_flipYProjection = flipEnabled;
      m_matricesNeedUpdate = true;
    }
  }

  /** Flip vertically at projection level (useful for FBO rendering) */
  [[nodiscard]] bool flipVerticalProjection() const { return m_flipYProjection; }

  /** @} */

 private:
  /** Viewport name (matches viz::Viewport) */
  std::string m_name;

  /** All proxies, in insertion order. The render queue sorts them for each
   * pass. */
  std::vector<RenderableProxy::Ptr> m_proxies;

  /** @name Viewport Configuration
   * @{ */

  /** Non-owning pointer to the source viz::Viewport, for writing back
   * rendered dimensions via updateRenderedViewportSize(). */
  const mrpt::viz::Viewport* m_sourceVizViewport = nullptr;

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
  VertexArrayObject m_borderVAO;
  Buffer m_borderVertexBuffer{Buffer::Type::Vertex};
  Buffer m_borderColorBuffer{Buffer::Type::Vertex};

  /** Text overlay rendering */
  VertexArrayObject m_textVAO;
  Buffer m_textVertexBuffer{Buffer::Type::Vertex};
  Buffer m_textColorBuffer{Buffer::Type::Vertex};

  /** Viewport visibility */
  bool m_isVisible = true;

  /** OpenGL rendering options */
  bool m_enablePolygonSmooth = true;

  /**  Flip vertically at projection level */
  bool m_flipYProjection = false;

  /** @} */

  /** @name Camera and Lighting
   * @{ */

  /** Current camera configuration. A copy of the parameters only: copying
   * a CCamera object would count as a change in the scene. */
  struct CameraParams
  {
    bool noProjection = false;
    bool projective = true;
    bool is6DOF = false;
    float fovDeg = 30.0f;
    float zoomDistance = 10.0f;
    float azimuthDeg = 0;
    float elevationDeg = 0;
    mrpt::math::TPoint3D pointingAt{0, 0, 0};
    mrpt::math::TPose3D pose;
    std::optional<mrpt::img::TCamera> pinhole;
  };
  CameraParams m_camera;

  void updateCameraParams(const mrpt::viz::CCamera& camera);

  /** Lighting parameters, with the lights actually used for rendering: those
   * of the source viewport and of the scene, selected with selectLights(). */
  mrpt::viz::TLightParameters m_lightParams;

  /** All lights of the source viewport */
  std::vector<mrpt::viz::TLight> m_viewportLights;

  /** Lights of the CLight objects in the scene, in world coordinates */
  std::vector<mrpt::viz::TLight> m_sceneLights;

  /** Fills m_lightParams.lights with up to MAX_LIGHTS lights: all directional
   * lights, then the point/spot lights closest to the camera. */
  void selectLights();

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
  std::string m_clonedCameraViewportName;

  /** @} */

  /** @name Shadow Mapping
   * @{ */

  /** Shadow casting enabled */
  bool m_shadowsEnabled = false;

  /** Shadow map dimensions */
  unsigned int m_shadowMapSizeX = 2048;
  unsigned int m_shadowMapSizeY = 2048;

  /** Shadow map FBO used to render directly into texture array layers */
  unsigned int m_shadowMapFBO = 0;

  /** GL_TEXTURE_2D_ARRAY holding all cascade depth maps.
   *  Created/resized on demand in renderShadowMap(). */
  unsigned int m_cascadeDepthArrayTexId = 0;
  int m_cascadeDepthArrayLayers = 0;
  unsigned int m_cascadeDepthArraySizeX = 0;
  unsigned int m_cascadeDepthArraySizeY = 0;

  /** GL_TEXTURE_2D_ARRAY holding the cube shadow maps of point/spot lights
   *  (six layers per light). Created/resized on demand in
   *  renderPointShadowMaps(). */
  unsigned int m_pointShadowArrayTexId = 0;
  int m_pointShadowArrayLayers = 0;
  unsigned int m_pointShadowArraySize = 0;

  /** A cube shadow map in m_pointShadowArrayTexId */
  struct PointShadowCube
  {
    int lightIndex = -1;  //!< Index in m_lightParams.lights
    float zNear = 0;
    float zFar = 0;
    /** Of the light and the objects in each face frustum, to reuse faces */
    std::array<uint64_t, 6> faceSignatures{};
  };
  std::vector<PointShadowCube> m_pointShadowCubes;

  /** True while rendering point light shadow maps (culling against all the
   *  cube face frustum planes) */
  bool m_pointShadowPass = false;

  /** @} */

  /** @name SSAO
   * @{ */

  /** Enabled flag (copied from TLightParameters::ssao_enabled each frame) */
  bool m_ssaoEnabled = false;

  /** G-buffer FBO with two float color attachments (position, normal) */
  unsigned int m_ssaoGBufferFBO = 0;
  unsigned int m_ssaoGPositionTex = 0;  ///< GL_RGB16F view-space position
  unsigned int m_ssaoGNormalTex = 0;    ///< GL_RGB16F view-space normal
  unsigned int m_ssaoGDepthRBO = 0;     ///< Depth renderbuffer

  /** AO computation FBO (raw noisy AO, single channel) */
  unsigned int m_ssaoRawFBO = 0;
  unsigned int m_ssaoRawTex = 0;  ///< GL_R16F

  /** AO blur FBO (blurred AO) */
  unsigned int m_ssaoBlurFBO = 0;
  unsigned int m_ssaoBlurTex = 0;  ///< GL_R16F

  /** 4x4 tiled random rotation noise texture */
  unsigned int m_ssaoNoiseTex = 0;

  /** Hemisphere sample kernel (up to 64 vec3) */
  std::vector<float> m_ssaoKernel;  ///< flat: x0,y0,z0, x1,y1,z1, ...

  /** Size at which the SSAO G-buffer was last created */
  int m_ssaoLastW = 0, m_ssaoLastH = 0;

  /** Dummy VAO for full-screen triangle draws */
  unsigned int m_ssaoDummyVAO = 0;

  /** Build the SSAO kernel and noise texture (called once) */
  void ssaoInit();

  /** Create/recreate SSAO FBOs for the current viewport size */
  void ssaoCreateFBOs(int w, int h);

  /** Deletes the G-buffer and AO framebuffers, keeping the kernel and noise */
  void ssaoDestroyFramebuffers();
  /** Deletes everything created by ssaoInit() and ssaoCreateFBOs() */
  void ssaoDestroy();

  /** Render SSAO geometry pre-pass into G-buffer */
  void renderSSAOGeometry(ShaderProgramManager& shaderManager);

  /** Compute raw AO from G-buffer, then blur */
  void renderSSAOCompute(ShaderProgramManager& shaderManager);

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

  /** Performs shadow map rendering (1st pass).
   * \param proxiesToRender If non-null, use these proxies instead of m_proxies
   *        (used for cloned viewports). */
  void renderShadowMap(
      ShaderProgramManager& shaderManager,
      const std::vector<RenderableProxy::Ptr>* proxiesToRender = nullptr);

  /** Frees the GPU resources of all shadow maps */
  void releaseShadowMaps();

  /** Frees the GPU resources of the point/spot light cube shadow maps */
  void releasePointShadowMaps();

  /** Renders (or reuses the faces where nothing changed) the cube shadow maps of the
   * point/spot lights with TLight::cast_shadows.
   * \param proxiesToRender As in renderShadowMap() */
  void renderPointShadowMaps(
      ShaderProgramManager& shaderManager,
      const std::vector<RenderableProxy::Ptr>* proxiesToRender = nullptr);

  /** Performs normal scene rendering.
   * \param proxiesToRender If non-null, use these proxies instead of m_proxies
   *        (used for cloned viewports). */
  void renderNormalScene(
      ShaderProgramManager& shaderManager,
      bool isShadowMapPass,
      const std::vector<RenderableProxy::Ptr>* proxiesToRender = nullptr);

  /** Renders in image view mode */
  void renderImageView(ShaderProgramManager& shaderManager);

  /** Renders viewport border */
  void renderBorder(ShaderProgramManager& shaderManager);

  /** Renders 2D text message overlays from CTextMessageCapable */
  void renderTextOverlays(ShaderProgramManager& shaderManager);

  /** Builds render queue with frustum culling.
   * \param proxiesToRender If non-null, use these proxies instead of m_proxies. */
  void buildRenderQueue(
      RenderQueue& queue,
      const TRenderMatrices& matrices,
      bool isShadowMapPass,
      ViewportRenderStats& stats,
      const std::vector<RenderableProxy::Ptr>* proxiesToRender = nullptr);

  /** Builds render queue for the SSAO geometry pre-pass (triangle proxies only,
   *  all overridden to the SSAO_GEOMETRY shader). */
  void buildRenderQueueSSAOGeom(RenderQueue& queue, const TRenderMatrices& matrices);

  /** Binds a shader and uploads the uniforms that are common to all objects
   * drawn with it in this pass. */
  void setupShaderForPass(Program& shader, shader_id_t shaderID, const TRenderMatrices& matrices);

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
