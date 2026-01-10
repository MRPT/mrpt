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

#include <mrpt/containers/PerThreadDataHolder.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CImage.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/CObservable.h>
#include <mrpt/system/mrptEvent.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CTextMessageCapable.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/TLightParameters.h>
#include <mrpt/viz/viz_frwds.h>

namespace mrpt::img
{
class CImage;
}
namespace mrpt::viz
{
/** A viewport within a Scene, containing a set of OpenGL objects to
 *render.
 *   This class has protected constructor, thus it cannot be created by users.
 *Use Scene::createViewport instead.
 *  A viewport has these "operation modes":
 *		- Normal (default): It renders the contained objects.
 *		- Cloned: It clones the objects from another viewport. See \a
 *setCloneView()
 *		- Image mode: It renders an image (e.g. from a video stream) efficiently
 *using a textued quad. See \a setImageView().
 *
 * In any case, the viewport can be resized to only fit a part of the entire
 *parent viewport.
 *  There will be always at least one viewport in a Scene named "main".
 *
 * This class can be observed (see mrpt::system::CObserver) for the following
 *events (see mrpt::system::mrptEvent):
 *   - mrpt::viz::mrptEventGLPreRender
 *   - mrpt::viz::mrptEventGLPostRender
 *
 * Two directional light sources at infinity are created by default, with
 *directions (-1,-1,-1) and (1,2,1), respectively.
 *
 * Lighting parameters are accessible via lightParameters().
 *
 *  Refer to mrpt::viz::Scene for further details.
 * \ingroup mrpt_viz_grp
 */
class Viewport :
    public mrpt::serialization::CSerializable,
    public mrpt::system::CObservable,
    public mrpt::viz::CTextMessageCapable
{
  DEFINE_SERIALIZABLE(Viewport, mrpt::viz)
  friend class Scene;

 public:
  /** @name Viewport "modes"
    @{ */

  /** Set this viewport as a clone of some other viewport, given its name - as
   * a side effect, current list of internal OpenGL objects is cleared.
   *  By default, only the objects are cloned, not the camera. See
   * \sa resetCloneView
   */
  void setCloneView(const std::string& clonedViewport);

  /** Set this viewport into "image view"-mode, where an image is efficiently
   * drawn (fitting the viewport area) using an OpenGL textured quad.
   *  Call this method with the new image to update the displayed image (but
   * recall to first lock the parent openglscene's critical section, then do
   * the update, then release the lock, and then issue a window repaint).
   *  Internally, the texture is drawn using a mrpt::viz::CTexturedPlane
   *  The viewport can be reverted to behave like a normal viewport by
   * calling setNormalMode()
   *
   * \param[in] transparentBackground This method can also make the viewport
   * transparent (default), so the area not filled with the image still allows
   * seeing an underlying viewport.
   */
  void setImageView(const mrpt::img::CImage& img, bool transparentBackground = true);

  /** \overload With move semantics */
  void setImageView(mrpt::img::CImage&& img, bool transparentBackground = true);

  /** Returns true if setImageView() has been called on this viewport */
  [[nodiscard]] bool isImageViewMode() const { return !!m_imageViewPlane; }

  /** Reset the viewport to normal mode: rendering its own objects.
   * \sa setCloneView, setNormalMode
   */
  void resetCloneView() { setNormalMode(); }

  /** If set to true, and setCloneView() has been called, this viewport will
   * be rendered using the camera of the cloned viewport.
   */
  void setCloneCamera(bool enable);

  /** Use the camera of another viewport.
   *  Note this works even for viewports not in "clone" mode, so you can
   *  render different scenes but using the same camera.
   */
  void setClonedCameraFrom(const std::string& viewPortName)
  {
    m_isClonedCamera = true;
    m_clonedCameraViewport = viewPortName;
  }

  /** Resets the viewport to a normal 3D viewport \sa setCloneView,
   * setImageView */
  void setNormalMode();

  void setViewportVisibility(bool visible) { m_isViewportVisible = visible; }
  [[nodiscard]] bool getViewportVisibility() const { return m_isViewportVisible; }

  /** @} */
  // end of Set the "viewport mode"

  /** @name OpenGL global settings that affect rendering all objects in the
   scene/viewport
    @{ */

  /** Sets glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST) is enabled, or GL_FASTEST
   * otherwise. */
  void enablePolygonNicest(bool enable = true) { m_OpenGL_enablePolygonNicest = enable; }
  [[nodiscard]] bool isPolygonNicestEnabled() const { return m_OpenGL_enablePolygonNicest; }

  [[nodiscard]] const TLightParameters& lightParameters() const { return m_light; }
  [[nodiscard]] TLightParameters& lightParameters() { return m_light; }

  /** @} */

  /** @name Change or read viewport properties (except "viewport modes")
    @{ */

  /** Returns the name of the viewport */
  [[nodiscard]] std::string getName() { return m_name; }

  /** Change the viewport position and dimension on the rendering window.
   *  X & Y coordinates here can have two interpretations:
   *    - If in the range [0,1], they are factors with respect to the actual
   *window sizes (i.e. width=1 means the entire width of the rendering
   *window).
   *    - If >1, they are interpreted as pixels.
   *
   *  width & height can be interpreted as:
   *		- If >1, they are the size of the viewport in that dimension, in
   *pixels.
   *		- If in [0,1], they are the size of the viewport in that dimension,
   *in
   *a factor of the width/height.
   *		- If in [-1,0[, the size is computed such as the right/top border
   *ends
   *up in the given coordinate, interpreted as a factor (e.g. -1: up to the
   *end of the viewport, -0.5: up to the middle of it).
   *		- If <-1 the size is computed such as the right/top border ends up
   *in
   *the given absolute coordinate (e.g. -200: up to the row/column 200px).
   *
   * \note (x,y) specify the lower left corner of the viewport rectangle.
   * \sa getViewportPosition
   */
  void setViewportPosition(const double x, const double y, const double width, const double height);

  /** Get the current viewport position and dimension on the rendering window.
   *  X & Y coordinates here can have two interpretations:
   *    - If in the range [0,1], they are factors with respect to the actual
   * window sizes (i.e. width=1 means the entire width of the rendering
   * window).
   *    - If >1, they are interpreted as pixels.
   * \note (x,y) specify the lower left corner of the viewport rectangle.
   * \sa setViewportPosition
   */
  void getViewportPosition(double& x, double& y, double& width, double& height);

  /** Set the min/max clip depth distances of the rendering frustum (default:
   * 0.1 - 1000)
   * \sa getViewportClipDistances
   */
  void setViewportClipDistances(const float clip_min, const float clip_max);

  /** Get the current min/max clip depth distances of the rendering frustum
   * (default: 0.1 - 1000)
   * \sa setViewportClipDistances
   */
  void getViewportClipDistances(float& clip_min, float& clip_max) const;

  void setLightShadowClipDistances(const float clip_min, const float clip_max);
  void getLightShadowClipDistances(float& clip_min, float& clip_max) const;

  /** Set the border size ("frame") of the viewport (default=0) */
  void setBorderSize(unsigned int lineWidth) { m_borderWidth = lineWidth; }
  [[nodiscard]] unsigned int getBorderSize() const { return m_borderWidth; }

  void setBorderColor(const mrpt::img::TColor& c) { m_borderColor = c; }
  [[nodiscard]] const mrpt::img::TColor& getBorderColor() const { return m_borderColor; }

  /** Return whether the viewport will be rendered transparent over previous
   * viewports.
   */
  [[nodiscard]] bool isTransparent() { return m_isTransparent; }

  /** Set the transparency, that is, whether the viewport will be rendered
   * transparent over previous viewports (default=false).
   */
  void setTransparent(bool trans) { m_isTransparent = trans; }

  /** Defines the viewport background color */
  void setCustomBackgroundColor(const mrpt::img::TColorf& color) { m_background_color = color; }

  [[nodiscard]] mrpt::img::TColorf getCustomBackgroundColor() const { return m_background_color; }

  /** Enables or disables rendering of shadows cast by the unidirectional
   * light.
   * \param enabled Set to true to enable shadow casting
   *         (default at ctor=false).
   * \param SHADOW_MAP_SIZE_X Width of the shadow cast map (1st pass of
   *         rendering with shadows). Larger values are slower but gives
   *         more precise shadows. Default=2048x2048.
   *         Zero means do not change.
   * \param SHADOW_MAP_SIZE_Y Like SHADOW_MAP_SIZE_X but defines the height.
   *
   */
  void enableShadowCasting(
      bool enabled = true, unsigned int SHADOW_MAP_SIZE_X = 0, unsigned int SHADOW_MAP_SIZE_Y = 0);

  [[nodiscard]] bool isShadowCastingEnabled() const { return m_shadowsEnabled; }

  /** @} */  // end of Change or read viewport properties

  /** @name Contained objects set/get/search
    @{ */

  using const_iterator = ListVisualObjects::const_iterator;
  using iterator = ListVisualObjects::iterator;

  [[nodiscard]] const_iterator begin() const { return m_objects.begin(); }
  [[nodiscard]] const_iterator end() const { return m_objects.end(); }
  [[nodiscard]] iterator begin() { return m_objects.begin(); }
  [[nodiscard]] iterator end() { return m_objects.end(); }

  /** Delete all internal obejcts * \sa insert */
  void clear();

  /** Insert a new object into the list.
   *  The object MUST NOT be deleted, it will be deleted automatically by
   * this object when not required anymore.
   */
  void insert(const CVisualObject::Ptr& newObject);

  /** Changes the point of view of the camera, from a given pose.
   * \sa getCurrentCameraPose
   */
  void setCurrentCameraFromPose(mrpt::poses::CPose3D& p);

  /** Returns the first object with a given name, or nullptr if not found.
   */
  [[nodiscard]] CVisualObject::Ptr getByName(const std::string& str);

  /** Returns the i'th object of a given class (or of a descendant class), or
  nullptr (an empty smart pointer) if not found.
  *  Example:
  * \code
     CSphere::Ptr obs = view.getByClass<CSphere>();
  * \endcode
  * By default (ith=0), the first observation is returned.
  */
  template <typename T>
  [[nodiscard]] typename T::Ptr getByClass(size_t ith = 0) const
  {
    MRPT_START
    size_t foundCount = 0;
    for (const auto& o : m_objects)
      if (const auto f = std::dynamic_pointer_cast<T>(o); f)
      {
        if (foundCount++ == ith) return f;
      }

    // If not found directly, search recursively:
    for (const auto& o : m_objects)
    {
      if (auto obj = std::dynamic_pointer_cast<CSetOfObjects>(o); obj)
      {
        if (auto f = obj->template getByClass<T>(ith); f) return f;
      }
    }
    return typename T::Ptr();  // Not found: return empty smart pointer
    MRPT_END
  }

  /** Removes the given object from the scene (it also deletes the object to
   * free its memory).
   */
  void removeObject(const CVisualObject::Ptr& obj);

  /** Number of objects contained. */
  [[nodiscard]] size_t size() const { return m_objects.size(); }

  [[nodiscard]] bool empty() const { return m_objects.empty(); }

  /** Get a reference to the camera associated with this viewport. */
  [[nodiscard]] mrpt::viz::CCamera& getCamera() { return m_camera; }

  /** Get a reference to the camera associated with this viewport. */
  [[nodiscard]] const mrpt::viz::CCamera& getCamera() const { return m_camera; }

  mrpt::math::TBoundingBox getBoundingBox() const;

  /** @} */  // end of Contained objects set/get/search

  /** Destructor: clears all objects. */
  ~Viewport() override;

  /** Constructor, invoked from Scene only.
   */
  Viewport(Scene* parent = nullptr, const std::string& name = std::string(""));

  /** Render the objects in this viewport (called from Scene) */
  void render(
      const int render_width,
      const int render_height,
      const int render_offset_x = 0,
      const int render_offset_y = 0,
      const CCamera* forceThisCamera = nullptr) const;

 protected:
  /** Retrieves a list of all objects in text form.
   * 	\deprecated Prefer asYAML() (since MRPT 2.1.3) */
  void dumpListOfObjects(std::vector<std::string>& lst) const;

  /** Prints all viewport objects in human-readable YAML form.
   * \note (New in MRPT 2.1.3) */
  [[nodiscard]] mrpt::containers::yaml asYAML() const;

  /** The camera associated to the viewport */
  mrpt::viz::CCamera m_camera;

  /** The scene that contains this viewport. */
  mrpt::safe_ptr<Scene> m_parent;

  /** Set by setCloneView */
  bool m_isCloned = false;

  /** Set by setCloneCamera */
  bool m_isClonedCamera = false;

  bool m_isViewportVisible = true;

  /** Only if m_isCloned=true */
  std::string m_clonedViewport;

  /** If m_isClonedCamera && !m_isCloned, take the camera from another view,
   * to render a different scene. */
  std::string m_clonedCameraViewport;

  /** The viewport's name */
  std::string m_name;

  /** Whether to clear color buffer. */
  bool m_isTransparent = false;

  /** Default=0, the border around the viewport. */
  uint32_t m_borderWidth = 0;

  mrpt::img::TColor m_borderColor{255, 255, 255, 255};

  /** The viewport position [0,1] */
  double m_view_x{0}, m_view_y{0}, m_view_width{1}, m_view_height{1};

  /** The min/max clip depth distances (default: 0.01 - 1000) */
  float m_clip_min = 0.01f, m_clip_max = 1000.0f;

  /** The near/far plane clip distances for unidirectional light shadow
   * casting */
  float m_lightShadowClipMin = 0.01f, m_lightShadowClipMax = 1000.0f;

  mrpt::img::TColorf m_background_color = {0.4f, 0.4f, 0.4f};

  /** The image to display, after calling \a setImageView() */
  mrpt::viz::CTexturedPlane::Ptr m_imageViewPlane;

  mutable mrpt::viz::CSetOfLines::Ptr m_borderLines;

  const CCamera* internalResolveActiveCamera(const CCamera* forceThisCamera = nullptr) const;

  mrpt::viz::ListVisualObjects m_objects;

  void internal_enableImageView(bool transparentBackground);

  // OpenGL global settings:
  bool m_OpenGL_enablePolygonNicest{true};

  TLightParameters m_light;

  bool m_shadowsEnabled = false;
  uint32_t m_ShadowMapSizeX = 2048, m_ShadowMapSizeY = 2048;
};

/**
 * Inserts an openGL object into a viewport. Allows call chaining.
 * \sa mrpt::viz::Viewport::insert
 */
Viewport::Ptr& operator<<(Viewport::Ptr& s, const CVisualObject::Ptr& r);

/**
 * Inserts any iterable set of openGL objects into a viewport. Allows call
 * chaining.
 * \sa mrpt::viz::Viewport::insert
 */
Viewport::Ptr& operator<<(Viewport::Ptr& s, const std::vector<CVisualObject::Ptr>& v);

}  // namespace mrpt::viz
