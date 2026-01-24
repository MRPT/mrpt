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

#include <mrpt/core/optional_ref.h>
#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/opengl/FrameBuffer.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/Scene.h>

namespace mrpt::opengl
{
/** Render 3D scenes off-screen directly to RGB and/or RGB+D images.
 *
 * Main methods:
 * - render_RGB(): Renders a scene into an RGB image.
 * - render_RGBD(): Renders a scene into an RGB and depth images.
 *
 * To define a background color, define it in your
 * `scene.getViewport()->setCustomBackgroundColor()`. You can add overlaid text
 * messages, see base class CTextMessageCapable.
 *
 * The SE(3) pose from which the scene is rendered is defined by the scene
 * `"main"` viewport camera pose, or can be overridden via setCamera().
 *
 * Architecture (MRPT 3.0):
 * - Uses mrpt::viz::Scene as the abstract scene graph
 * - Internally maintains a CompiledScene for GPU-side representation
 * - Automatically recompiles when the scene changes (dirty flags)
 *
 * See \ref gui_fbo_render_example for code examples.
 *
 * \sa \ref opengl_offscreen_render_example , \ref gui_fbo_render_example
 * \ingroup mrpt_opengl_grp
 */
class CFBORender
{
 public:
  /** Parameters for CFBORender constructor */
  struct Parameters
  {
    Parameters() = default;

    Parameters(unsigned int Width, unsigned int Height) : width(Width), height(Height) {}

    unsigned int width = 800;   //!< Width of images to render.
    unsigned int height = 600;  //!< Height of images to render.

    /** If false (default), depth values returned in
     * CFBORender::render_RGBD() or CFBORender::render_depth() are real
     * depth values (e.g. units=meters).
     * If this is "true", raw OpenGL depth values in the range [0,1] are
     * left in the returned depth matrix, so it is the user responsibility
     * to map those logarithm depths to linear ones. Useful when only
     * a subset of all depths are required.
     */
    bool raw_depth = false;

    /** By default, each CFBORender constructor will create its own EGL
     * context, which enables using them in different threads, use in
     * head-less applications, etc.
     *
     * Set this to false to save that effort, only if it is ensured that
     * render calls will always happen from a thread where OpenGL has been
     * already initialized and a context created.
     */
    bool create_EGL_context = true;

    /** Can be used to select a particular GPU (or software-emulated)
     * device.
     *
     * Create a CFBORender object with the environment variable
     * MRPT_FBORENDER_SHOW_DEVICES=true to see a list of available and
     * detected GPU devices.
     */
    int deviceIndexToUse = 0;

    int blueSize = 8, redSize = 8, greenSize = 8, depthSize = 24;
    bool conformantOpenGLES2 = false;  //!< Default: EGL_OPENGL_ES_BIT
    bool renderableOpenGLES2 = false;  //!< Default: EGL_OPENGL_ES_BIT
    bool bindOpenGLES_API = false;     //!< Default: EGL_OPENGL_API
    int contextMajorVersion = 0;       //!< 0=default
    int contextMinorVersion = 0;       //!< 0=default
    /// See https://www.khronos.org/opengl/wiki/Debug_Output
    bool contextDebug = false;
  };

  /** Main constructor */
  explicit CFBORender(const Parameters& p);

  /** Convenience constructor with just dimensions */
  CFBORender(unsigned int width = 800, unsigned int height = 600) :
      CFBORender(Parameters(width, height))
  {
  }

  /** Destructor */
  ~CFBORender();

  // Non-copyable, non-movable
  CFBORender(const CFBORender&) = delete;
  CFBORender& operator=(const CFBORender&) = delete;
  CFBORender(CFBORender&&) = delete;
  CFBORender& operator=(CFBORender&&) = delete;

  /** @name Camera Control
   * @{ */

  /** Set the camera to use for rendering, overriding the scene's viewport camera.
   *
   * If not called, the camera from the scene's "main" viewport is used.
   */
  void setCamera(const mrpt::viz::CCamera& camera)
  {
    m_cameraOverride = camera;
    m_useCameraOverride = true;
  }

  /** Clear any camera override, reverting to using the scene's viewport camera. */
  void clearCameraOverride() { m_useCameraOverride = false; }

  /** Get a mutable reference to the camera override.
   * \note Only valid if setCamera() was previously called.
   */
  [[nodiscard]] mrpt::viz::CCamera& getCameraOverride() { return m_cameraOverride; }

  /** Get the camera override (const version). */
  [[nodiscard]] const mrpt::viz::CCamera& getCameraOverride() const { return m_cameraOverride; }

  /** Returns true if a camera override is set. */
  [[nodiscard]] bool hasCameraOverride() const { return m_useCameraOverride; }

  /** @} */

  /** @name Rendering Methods
   * @{ */

  /** Render the scene and get the rendered RGB image.
   *
   * Resizes the image buffer if necessary to the configured render resolution.
   *
   * \param scene The abstract viz scene to render
   * \param outRGB Output RGB image (resized if needed)
   *
   * \note The scene is compiled on first call, then incrementally updated.
   *
   * \sa render_RGBD()
   */
  void render_RGB(const mrpt::viz::Scene& scene, mrpt::img::CImage& outRGB);

  /** Render the scene and get the rendered RGB and depth images.
   *
   * Resizes the provided buffers if necessary to the configured render
   * resolution.
   *
   * The output depth image is in linear depth distance units (e.g. "meters").
   * Note that values are depth, not range, that is, it's the "+z" coordinate
   * of a point as seen from the camera, with +Z pointing forward in the view
   * direction (the common convention in computer vision).
   *
   * Pixels without any observed object in the valid viewport {clipMin,
   * clipMax} range will be returned with a depth of `0.0`.
   *
   * \param scene The abstract viz scene to render
   * \param outRGB Output RGB image (resized if needed)
   * \param outDepth Output depth matrix (resized if needed)
   *
   * \sa render_RGB(), Parameters::raw_depth
   */
  void render_RGBD(
      const mrpt::viz::Scene& scene, mrpt::img::CImage& outRGB, mrpt::math::CMatrixFloat& outDepth);

  /** Like render_RGBD(), but only renders the depth image.
   *
   * \param scene The abstract viz scene to render
   * \param outDepth Output depth matrix (resized if needed)
   *
   * \sa render_RGBD()
   */
  void render_depth(const mrpt::viz::Scene& scene, mrpt::math::CMatrixFloat& outDepth);

  /** @} */

  /** @name Status and Configuration
   * @{ */

  /** Returns the current render width in pixels */
  [[nodiscard]] unsigned int width() const { return m_fb.width(); }

  /** Returns the current render height in pixels */
  [[nodiscard]] unsigned int height() const { return m_fb.height(); }

  /** Returns the parameters used to construct this renderer */
  [[nodiscard]] const Parameters& getParameters() const { return m_params; }

  /** Force recompilation of the scene on next render.
   *
   * Normally not needed as changes are detected automatically via dirty flags.
   */
  void invalidateCompiledScene() { m_compiledScene.reset(); }

  /** @} */

 protected:
  // EGL context (opaque pointers to avoid including EGL headers)
  void* m_eglDpy = nullptr;
  void* m_eglCfg = nullptr;
  void* m_eglContext = nullptr;
  void* m_eglSurf = nullptr;

  unsigned int m_texRGB = 0;

  const Parameters m_params;

  FrameBuffer m_fb;

  // Camera override
  mrpt::viz::CCamera m_cameraOverride;
  bool m_useCameraOverride = false;

  // Compiled scene (lazy-initialized on first render)
  std::unique_ptr<CompiledScene> m_compiledScene;

  // Weak reference to last rendered scene (to detect scene changes)
  std::weak_ptr<const mrpt::viz::Scene> m_lastScene;

  /** Internal rendering implementation */
  void internal_render_RGBD(
      const mrpt::viz::Scene& scene,
      const mrpt::optional_ref<mrpt::img::CImage>& outRGB,
      const mrpt::optional_ref<mrpt::math::CMatrixFloat>& outDepth);

  /** Ensures the compiled scene is up-to-date with the source scene */
  void ensureCompiledScene(const mrpt::viz::Scene& scene);

  /** Converts raw OpenGL depth buffer to linear depth values */
  void convertDepthToLinear(mrpt::math::CMatrixFloat& depth, float zNear, float zFar) const;
};

}  // namespace mrpt::opengl