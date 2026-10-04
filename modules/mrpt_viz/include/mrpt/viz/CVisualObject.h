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
#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/viz/TLightParameters.h>
#include <mrpt/viz/TTriangle.h>
#include <mrpt/viz/viz_frwds.h>

#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <shared_mutex>

namespace mrpt::viz
{
/** Enum for cull face modes in triangle-based shaders.
 *  \sa CVisualObjectShaderTriangles, CVisualObjectShaderTexturedTriangles
 *  \ingroup mrpt_viz_grp
 */
enum class TCullFace : uint8_t
{
  /** The default: culls none, so all front and back faces are visible. */
  NONE = 0,
  /** Skip back faces (those that are NOT seen in the CCW direction) */
  BACK,
  /** Skip front faces (those that ARE seen in the CCW direction) */
  FRONT
};

/** How the alpha channel of a texture is used, as glTF alphaMode.
 *  \sa VisualObjectParams_TexturedTriangles::setAlphaMode()
 *  \ingroup mrpt_viz_grp
 */
enum class TAlphaMode : uint8_t
{
  /** The default: Mask if the texture alpha is (nearly) binary, Opaque if it
   * is fully opaque, or Blend otherwise. */
  Auto = 0,
  /** Alpha is ignored. */
  Opaque,
  /** Cutout: fragments with alpha below the cutoff are discarded, and the
   * rest are fully opaque. Correct in any drawing order, also in shadows and
   * depth images. Best for foliage, fences, etc. */
  Mask,
  /** Alpha blending, for semi-transparent surfaces. */
  Blend
};

/** Number of changes made so far to any object or container of any scene, in
 * this process: object data, poses, visibility, and insertions or removals.
 * Renderers compare it against the value they last saw, to skip re-checking a
 * whole scene graph when nothing changed.
 * \sa notifySceneChange()
 * \ingroup mrpt_viz_grp
 */
[[nodiscard]] uint64_t sceneChangeCount();

/** Increments sceneChangeCount(). Called by all methods that modify objects
 * or the structure of a scene graph.
 * \ingroup mrpt_viz_grp
 */
void notifySceneChange();

/** Number of changes made so far to the structure of any scene graph, in this
 * process: insertions or removals of objects or viewports, or whole objects
 * being assigned (which may replace the contents of containers).
 * \sa notifySceneStructureChange()
 * \ingroup mrpt_viz_grp
 */
[[nodiscard]] uint64_t sceneStructureChangeCount();

/** Increments both sceneStructureChangeCount() and sceneChangeCount().
 * \ingroup mrpt_viz_grp
 */
void notifySceneStructureChange();

/** The base class of 3D objects that can be directly rendered through OpenGL.
 *  In this class there are a set of common properties to all 3D objects,
 *mainly:
 * - Its SE(3) pose (x,y,z,yaw,pitch,roll), relative to the parent object,
 * or the global frame of reference for root objects (inserted into a
 *mrpt::viz::Scene).
 * - A name: A name that can be optionally assigned to objects for
 *easing its reference.
 * - A RGBA color: This field will be used in simple elements (points,
 *lines, text,...) but is ignored in more complex objects that carry their own
 *color information (triangle sets,...)
 * - Shininess: See materialShininess(float)
 *
 * See the main class opengl::Scene
 *
 *
 * RENDERING FLOW
 * ===============
 * 1. User modifies a viz object:
 *    - Geometry or appearance (e.g. box.setBoxCorners(...), setColor()):
 *      this calls notifyChange(), which increments dataVersion().
 *    - Pose, scale, visibility or shadow casting: this calls
 *      notifyTransformChange(), which increments transformVersion() only, so
 *      renderers do not regenerate any buffer.
 *
 * 2. CompiledScene::updateIfNeeded() is called before rendering
 *    → For each object whose dataVersion() changed:
 *       a) Call sourceObj->updateBuffersIfNeeded()  ← populates viz buffers
 *       b) Call proxy->updateBuffers(sourceObj)  ← Uploads to GPU
 *    → For each object whose transformVersion() changed, only its model
 *      matrix and visibility are updated.
 *
 * 3. Rendering proceeds with the updated GPU buffers
 *
 *  \sa opengl::Scene, mrpt::viz
 * \ingroup mrpt_viz_grp
 */
class CVisualObject : public mrpt::serialization::CSerializable
{
  DEFINE_VIRTUAL_SERIALIZABLE(CVisualObject, mrpt::viz)

  friend class mrpt::viz::Viewport;
  friend class mrpt::viz::CSetOfObjects;

 public:
 protected:
  struct State
  {
    std::string name;
    bool show_name = false;

    /** RGBA components in the range [0,255] */
    mrpt::img::TColor color = {0xff, 0xff, 0xff, 0xff};

    float materialShininess = 0.2f;

    /** Specular exponent for Blinn-Phong lighting (higher = sharper highlight).
     *  Typical values: 8 (rough), 32 (default, plastic), 128 (metal/mirror). */
    float materialSpecularExponent = 16.0f;

    /** Emissive color: light emitted by the object regardless of scene
     *  lighting. Default is black (no emission). Useful for displays,
     *  indicator lights, laser beams, etc. */
    mrpt::img::TColorf materialEmissive{0, 0, 0, 0};

    /** SE(3) pose wrt the parent coordinate reference. This class
     * automatically holds the cached 3x3 rotation matrix for quick load
     * into opengl stack. */
    mrpt::poses::CPose3D pose;

    /** Scale components to apply to the object (default=1) */
    float scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f;

    bool visible = true;  //!< Is the object visible? (default=true)

    bool castShadows = true;

    mrpt::math::TPoint3Df representativePoint{0, 0, 0};
  };

  /// All relevant rendering state that needs to get protected by m_stateMtx
  State m_state;
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_stateMtx;

 public:
  /** @name Changes the appearance of the object to render
    @{ */

  /** Changes the name of the object */
  void setName(const std::string& n)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.name = n;
    lckWrite.unlock();
    // The name is drawn as a label if enableShowName() is on:
    notifyChange();
  }
  /** Returns the name of the object */
  std::string getName() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.name;
  }

  /** Is the object visible? \sa setVisibility */
  bool isVisible() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.visible;
  }
  /** Set object visibility (default=true) \sa isVisible */
  void setVisibility(bool visible = true)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.visible = visible;
    lckWrite.unlock();
    notifyTransformChange();
  }

  /** Does the object cast shadows? (default=true) */
  bool castShadows() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.castShadows;
  }
  /** Enable/disable casting shadows by this object (default=true).
   *  \note The argument is not defaulted on purpose: it would make the
   *  no-argument call resolve to this setter instead of the getter above
   *  for any non-const object. */
  void castShadows(bool doCast)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.castShadows = doCast;
    lckWrite.unlock();
    notifyTransformChange();
  }

  /** Enables or disables showing the name of the object as a label when
   * rendering */
  void enableShowName(bool showName = true)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.show_name = showName;
    lckWrite.unlock();
    notifyChange();
  }
  /** \sa enableShowName */
  bool isShowNameEnabled() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.show_name;
  }

  /** Defines the SE(3) (pose=translation+rotation) of the object with respect
   * to its parent */
  CVisualObject& setPose(const mrpt::poses::CPose3D& o);
  /// \overload
  CVisualObject& setPose(const mrpt::poses::CPose2D& o);
  /// \overload
  CVisualObject& setPose(const mrpt::math::TPose3D& o);
  /// \overload
  CVisualObject& setPose(const mrpt::math::TPose2D& o);
  /// \overload
  CVisualObject& setPose(const mrpt::poses::CPoint3D& o);
  /// \overload
  CVisualObject& setPose(const mrpt::poses::CPoint2D& o);

  /** Returns the 3D pose of the object as TPose3D */
  mrpt::math::TPose3D getPose() const;

  /** Atomically reads pose, scale, and visibility under a single lock.
   * Use this when you need a consistent snapshot of the object's transform. */
  struct PoseAndScale
  {
    mrpt::poses::CPose3D pose;
    float scaleX = 1, scaleY = 1, scaleZ = 1;
    bool visible = true;
  };
  PoseAndScale getPoseAndScale() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return {m_state.pose, m_state.scale_x, m_state.scale_y, m_state.scale_z, m_state.visible};
  }

  /** Returns a const ref to the 3D pose of the object as mrpt::poses::CPose3D
   * (which explicitly contains the 3x3 rotation matrix) */
  mrpt::poses::CPose3D getCPose() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.pose;
  }

  /** Changes the location of the object, keeping untouched the orientation
   * \return a ref to this */
  CVisualObject& setLocation(double x, double y, double z)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.pose.x(x);
    m_state.pose.y(y);
    m_state.pose.z(z);
    lckWrite.unlock();
    notifyTransformChange();
    return *this;
  }

  /** Changes the location of the object, keeping untouched the orientation
   * \return a ref to this  */
  CVisualObject& setLocation(const mrpt::math::TPoint3D& p)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.pose.x(p.x);
    m_state.pose.y(p.y);
    m_state.pose.z(p.z);
    lckWrite.unlock();
    notifyTransformChange();
    return *this;
  }

  /** Get color components as floats in the range [0,1] */
  mrpt::img::TColorf getColor() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return mrpt::img::TColorf(m_state.color);
  }

  /** Get color components as uint8_t in the range [0,255] */
  mrpt::img::TColor getColor_u8() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.color;
  }

  /** Set alpha (transparency) color component in the range [0,1]
   *  \return a ref to this */
  CVisualObject& setColorA(const float a) { return setColorA_u8(f2u8(a)); }

  /** Set alpha (transparency) color component in the range [0,255]
   *  \return a ref to this */
  virtual CVisualObject& setColorA_u8(const uint8_t a)
  {
    m_stateMtx.data.lock();
    m_state.color.A = a;
    notifyChange();
    m_stateMtx.data.unlock();
    return *this;
  }

  /** Material shininess (for specular lights in shaders that support it),
   *  between 0.0f (none) to 1.0f (shiny) */
  float materialShininess() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.materialShininess;
  }

  /** Material shininess (for specular lights in shaders that support it),
   *  between 0.0f (none) to 1.0f (shiny) */
  void materialShininess(float shininess)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.materialShininess = shininess;
    lckWrite.unlock();
    notifyChange();
  }

  /** Blinn-Phong specular exponent. Higher values produce a smaller, sharper
   *  specular highlight. Typical values: 8 (rough), 32 (default), 128 (metal).
   */
  float materialSpecularExponent() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.materialSpecularExponent;
  }

  /** Blinn-Phong specular exponent. Higher values produce a smaller, sharper
   *  specular highlight. Typical values: 8 (rough), 32 (default), 128 (metal).
   */
  void materialSpecularExponent(float exponent)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.materialSpecularExponent = exponent;
    notifyChange();
  }

  /** Emissive color: light emitted by the object regardless of scene
   *  lighting. Default is black (no emission). Useful for displays,
   *  indicator lights, laser beams, warning signs, etc.
   *  The emissive term is added to the final lighting equation as:
   *  finalColor = emissive + (ambient + diffuse + specular) * materialColor
   */
  [[nodiscard]] mrpt::img::TColorf materialEmissive() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.materialEmissive;
  }

  /** \overload */
  void materialEmissive(const mrpt::img::TColorf& color)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.materialEmissive = color;
    lckWrite.unlock();
    notifyChange();
  }

  /** Scale to apply to the object, in all three axes (default=1)  \return a
   * ref to this */
  CVisualObject& setScale(float s)
  {
    m_stateMtx.data.lock();
    m_state.scale_x = m_state.scale_y = m_state.scale_z = s;
    m_stateMtx.data.unlock();
    notifyTransformChange();
    return *this;
  }

  /** Scale to apply to the object in each axis (default=1)  \return a ref to
   * this */
  CVisualObject& setScale(float sx, float sy, float sz)
  {
    m_stateMtx.data.lock();
    m_state.scale_x = sx;
    m_state.scale_y = sy;
    m_state.scale_z = sz;
    m_stateMtx.data.unlock();
    notifyTransformChange();
    return *this;
  }
  /** Get the current scaling factor in one axis */
  float getScaleX() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.scale_x;
  }
  /** Get the current scaling factor in one axis */
  float getScaleY() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.scale_y;
  }
  /** Get the current scaling factor in one axis */
  float getScaleZ() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.scale_z;
  }

  /** Changes the default object color \return a ref to this */
  CVisualObject& setColor(const mrpt::img::TColorf& c)
  {
    return setColor_u8(mrpt::img::TColor(f2u8(c.R), f2u8(c.G), f2u8(c.B), f2u8(c.A)));
  }

  /** Set the color components of this object (R,G,B,Alpha, in the range 0-1)
   * \return a ref to this */
  CVisualObject& setColor(float R, float G, float B, float A = 1)
  {
    return setColor_u8(f2u8(R), f2u8(G), f2u8(B), f2u8(A));
  }

  /*** Changes the default object color \return a ref to this */
  virtual CVisualObject& setColor_u8(const mrpt::img::TColor& c);

  /** Set the color components of this object (R,G,B,Alpha, in the range
   * 0-255)  \return a ref to this */
  CVisualObject& setColor_u8(uint8_t R, uint8_t G, uint8_t B, uint8_t A = 255)
  {
    return setColor_u8(mrpt::img::TColor(R, G, B, A));
  }

  /** @} */

  /** Return false if this object should never be checked for being culled out
   * (=not rendered if its bbox are out of the screen limits).
   * For example, skyboxes or other special effects.
   */
  virtual bool cullElegible() const { return true; }

  /** Used from Scene::asYAML().
   * \note (New in MRPT 2.4.2) */
  virtual void toYAMLMap(mrpt::containers::yaml& propertiesMap) const;

  /** Should return true if enqueueForRenderRecursive() is defined since
   *  the object has inner children. Examples: CSetOfObjects, CAssimpModel.
   */
  virtual bool isCompositeObject() const { return false; }

  /** Returns internal children for composite objects (e.g. CAxis text labels).
   * Only meaningful if isCompositeObject() returns true.
   * \note Not for CSetOfObjects — those are handled as containers. */
  virtual const std::deque<std::shared_ptr<CVisualObject>>& getInternalChildren() const
  {
    static const std::deque<std::shared_ptr<CVisualObject>> empty;
    return empty;
  }

  /** Must be called after any change to the geometry or appearance of the
   * object, so renderers regenerate its buffers (see updateBuffers()) before
   * the next frame. \sa notifyTransformChange() */
  void notifyChange() const
  {
    {
      std::unique_lock<std::shared_mutex> lckWrite(m_outdatedStateMtx.data);
      m_cachedLocalBBox.reset();
    }
    m_dataVersion.increment();
  }

  /** Must be called after a change in the pose, scale, visibility or shadow
   * casting of the object: renderers update where and whether it is drawn,
   * without regenerating its buffers. \sa notifyChange() */
  void notifyTransformChange() const { m_transformVersion.increment(); }

  /** Reset the dirty flag set with notifyChange().
   * \deprecated Prefer using the version-counter API:
   * dataVersion() / hasToUpdateBuffersSince(). Kept for
   * backwards compatibility; now a no-op.
   */
  void clearChangedFlag() const
  {
    // No-op: dirty tracking is now done via version counters
    // in each CompiledScene, not via a shared boolean flag.
  }

  void notifyBBoxChange() const { m_cachedLocalBBox.reset(); }

  /** Returns whether notifyChange() has been invoked since the last call
   * to renderUpdateBuffers(), meaning the latter needs to be called again
   * before rendering.
   * \note Prefer hasToUpdateBuffersSince() for multi-consumer scenarios.
   */
  bool hasToUpdateBuffers() const
  {
    // Kept for backwards compatibility: always returns true if version > 0
    // (i.e., object has ever been modified). New code should use
    // hasToUpdateBuffersSince().
    return dataVersion() > 0;
  }

  /** Returns the current data version counter. Incremented on each
   * notifyChange() call. Use this with hasToUpdateBuffersSince() for
   * multi-consumer dirty tracking. */
  uint64_t dataVersion() const { return m_dataVersion.get(); }

  /** Returns the current counter of changes in pose, scale, visibility or
   * shadow casting. Incremented on each notifyTransformChange() call. */
  uint64_t transformVersion() const { return m_transformVersion.get(); }

  /** Returns true if this object's data has changed since the given
   * version. Each consumer (e.g., CompiledScene) should store the last
   * version it processed and pass it here. */
  bool hasToUpdateBuffersSince(uint64_t sinceVersion) const
  {
    return dataVersion() != sinceVersion;
  }

  /** Calls updateBuffers() only if the object data changed since its last
   * call through this method, so several renderers of the same object do not
   * regenerate its buffers once each. */
  void updateBuffersIfNeeded() const;

  /// Called by the rendering system to update internal geometry buffers.
  ///
  /// Derived classes should override this to populate their data buffers
  /// (triangles, points, lines) when the object geometry changes.
  ///
  /// This is called automatically when hasToUpdateBuffers() returns true,
  /// which happens after notifyChange() was called.
  ///
  /// The base implementation does nothing; derived classes should override.
  ///
  /// \note Thread safety: implementations should lock the appropriate mutexes
  /// when writing to shared buffers.
  virtual void updateBuffers() const {}

  /** Simulation of ray-trace, given a pose. Returns true if the ray
   * effectively collisions with the object (returning the distance to the
   * origin of the ray in "dist"), or false in other case. "dist" variable
   * yields undefined behaviour when false is returned
   */
  virtual bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const;

  /** Evaluates the bounding box of this object (including possible
   * children) in the coordinate frame of my parent object,
   * i.e. if this object pose changes, the bbox returned here will change too.
   * This is in contrast with the local bbox returned by getBoundingBoxLocal()
   */
  auto getBoundingBox() const -> mrpt::math::TBoundingBox
  {
    return getBoundingBoxLocal().compose(getCPose());
  }

  /** Evaluates the bounding box of this object (including possible
   * children) in the coordinate frame of my parent object,
   * i.e. if this object pose changes, the bbox returned here will change too.
   * This is in contrast with the local bbox returned by getBoundingBoxLocal()
   */
  auto getBoundingBoxLocal() const -> mrpt::math::TBoundingBox;

  /// \overload Fastest method, returning a copy of the float version of
  /// the bbox. const refs are not returned for multi-thread safety.
  auto getBoundingBoxLocalf() const -> mrpt::math::TBoundingBoxf;

  /** Provide a representative point (in object local coordinates), used to
   * sort objects by eye-distance while rendering with transparencies
   * (Default=[0,0,0]) */
  virtual mrpt::math::TPoint3Df getLocalRepresentativePoint() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.representativePoint;
  }

  /** See getLocalRepresentativePoint() */
  void setLocalRepresentativePoint(const mrpt::math::TPoint3Df& p)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.representativePoint = p;
  }

  /** Returns or constructs (in its first invocation) the associated
   * mrpt::viz::CText object representing the label of the object.
   * \sa enableShowName()
   */
  mrpt::viz::CText& labelObject() const;

  /** Returns the shared_ptr to the label object (creating it if needed).
   * \sa labelObject(), enableShowName()
   */
  std::shared_ptr<mrpt::viz::CText> labelObjectPtr() const;

 protected:
  void writeToStreamRender(mrpt::serialization::CArchive& out) const;
  void readFromStreamRender(mrpt::serialization::CArchive& in);

  /** Must be implemented by derived classes to provide the updated bounding
   * box in the object local frame of coordinates.
   * This will be called only once after each time the derived class reports
   * to notifyChange() that the object geometry changed.
   *
   * \sa getBoundingBox(), getBoundingBoxLocal(), getBoundingBoxLocalf()
   */
  [[nodiscard]] virtual mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const = 0;

  /** Change counter read by renderers to detect dirty objects. Assigning
   * one object onto another replaces its whole state, so the target bumps its
   * own counter rather than inheriting the source's value, which could match
   * what a renderer already saw and leave the change undetected. */
  struct ChangeCounter
  {
    ChangeCounter() = default;
    ChangeCounter(const ChangeCounter& o) : m_value(o.get()) {}
    ChangeCounter(ChangeCounter&& o) noexcept : m_value(o.get()) {}
    ~ChangeCounter() = default;
    ChangeCounter& operator=(const ChangeCounter&)
    {
      increment();
      notifySceneStructureChange();
      return *this;
    }
    ChangeCounter& operator=(ChangeCounter&&) noexcept
    {
      increment();
      notifySceneStructureChange();
      return *this;
    }

    [[nodiscard]] uint64_t get() const { return m_value.load(std::memory_order_acquire); }
    void increment()
    {
      m_value.fetch_add(1, std::memory_order_acq_rel);
      notifySceneChange();
    }

   private:
    std::atomic<uint64_t> m_value{1};
  };

  /** dataVersion() at the last call to updateBuffersIfNeeded(), or 0 if
   * never called. A copied object has to regenerate its own buffers. */
  struct BuffersVersion
  {
    BuffersVersion() = default;
    BuffersVersion(const BuffersVersion&) {}
    BuffersVersion(BuffersVersion&&) noexcept {}
    ~BuffersVersion() = default;
    BuffersVersion& operator=(const BuffersVersion&)
    {
      value = 0;
      return *this;
    }
    BuffersVersion& operator=(BuffersVersion&&) noexcept
    {
      value = 0;
      return *this;
    }

    std::atomic<uint64_t> value{0};
  };

  mutable ChangeCounter m_dataVersion;
  mutable ChangeCounter m_transformVersion;
  mutable BuffersVersion m_buffersVersion;
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_outdatedStateMtx;

  mutable std::optional<mrpt::math::TBoundingBoxf> m_cachedLocalBBox;

  /** Optional pointer to a mrpt::viz::CText */
  mutable std::shared_ptr<mrpt::viz::CText> m_label_obj;
};

/** A list of smart pointers to renderizable objects */
using ListVisualObjects = std::deque<CVisualObject::Ptr>;

class VisualObjectParams_Triangles : public virtual CVisualObject
{
 public:
  VisualObjectParams_Triangles() = default;

  [[nodiscard]] bool isLightEnabled() const { return m_enableLight; }
  void enableLight(bool enable = true)
  {
    m_enableLight = enable;
    CVisualObject::notifyChange();
  }

  /** Control whether to render the FRONT, BACK, or BOTH (default) set of
   * faces. Refer to docs for glCullFace().
   * Example: If set to `cullFaces(TCullFace::BACK);`, back faces will not be
   * drawn ("culled")
   */
  void cullFaces(const TCullFace& cf)
  {
    m_cullface = cf;
    CVisualObject::notifyChange();
  }
  [[nodiscard]] TCullFace cullFaces() const { return m_cullface; }

  /** @name Raw access to triangle shader buffer data
   * @{ */
  [[nodiscard]] const auto& shaderTrianglesBuffer() const { return m_triangles; }
  [[nodiscard]] auto& shaderTrianglesBufferMutex() const { return m_trianglesMtx; }
  /** @} */

 protected:
  void params_serialize(mrpt::serialization::CArchive& out) const;
  void params_deserialize(mrpt::serialization::CArchive& in);

  /** List of triangles  \sa TTriangle */
  mutable std::vector<mrpt::viz::TTriangle> m_triangles;
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_trianglesMtx;

  /** Returns the bounding box of m_triangles, or (0,0,0)-(0,0,0) if empty. */
  [[nodiscard]] const mrpt::math::TBoundingBoxf trianglesBoundingBox() const;

 private:
  bool m_enableLight = true;
  TCullFace m_cullface = TCullFace::NONE;
};

class VisualObjectParams_TexturedTriangles : public virtual CVisualObject
{
 public:
  VisualObjectParams_TexturedTriangles() = default;

  /** Assigns a texture and a transparency image, and enables transparency (If
   * the images are not 2^N x 2^M, they will be internally filled to its
   * dimensions to be powers of two)
   * \note Images are copied, the original ones can be deleted.
   */
  void assignImage(const mrpt::img::CImage& img, const mrpt::img::CImage& imgAlpha);

  /** Assigns a texture image, and disable transparency.
   * \note Images are copied, the original ones can be deleted. */
  void assignImage(const mrpt::img::CImage& img);

  /** Similar to assignImage, but the passed images are moved in (move
   * semantic). */
  void assignImage(mrpt::img::CImage&& img, mrpt::img::CImage&& imgAlpha);

  /** Similar to assignImage, but with move semantics. */
  void assignImage(mrpt::img::CImage&& img);

  [[nodiscard]] bool isLightEnabled() const { return m_enableLight; }
  void enableLight(bool enable = true)
  {
    m_enableLight = enable;
    CVisualObject::notifyChange();
  }

  /** Control whether to render the FRONT, BACK, or BOTH (default) set of
   * faces. Refer to docs for glCullFace().
   * Example: If set to `cullFaces(TCullFace::BACK);`, back faces will not be
   * drawn ("culled")
   */
  void cullFaces(const TCullFace& cf)
  {
    m_cullface = cf;
    CVisualObject::notifyChange();
  }
  [[nodiscard]] TCullFace cullFaces() const { return m_cullface; }

  [[nodiscard]] const mrpt::img::CImage& getTextureImage() const { return m_textureImage; }

  [[nodiscard]] const mrpt::img::CImage& getTextureAlphaImage() const
  {
    return m_textureImageAlpha;
  }

  [[nodiscard]] bool textureImageHasBeenAssigned() const { return m_textureImageAssigned; }

  /** Sets how the texture alpha channel is used (default: Auto).
   * \sa setAlphaCutoff(), effectiveAlphaCutoff() */
  void setAlphaMode(TAlphaMode mode)
  {
    m_alphaMode = mode;
    CVisualObject::notifyChange();
  }
  [[nodiscard]] TAlphaMode alphaMode() const { return m_alphaMode; }

  /** Alpha threshold for TAlphaMode::Mask (default: 0.5) */
  void setAlphaCutoff(float cutoff)
  {
    m_alphaCutoff = cutoff;
    CVisualObject::notifyChange();
  }
  [[nodiscard]] float alphaCutoff() const { return m_alphaCutoff; }

  /** The alpha cutoff for the shaders: alphaCutoff() if the effective alpha
   * mode is Mask (set explicitly, or detected by Auto), a negative value if
   * Opaque was set explicitly (alpha is ignored), or 0 otherwise (blending).
   */
  [[nodiscard]] float effectiveAlphaCutoff() const;

  /** Assigns a normal map image for tangent-space normal mapping.
   * The image should encode normals in tangent space as RGB where
   * (128,128,255) represents the unperturbed surface normal.
   * Normals follow the OpenGL convention (green points to the image top, as
   * in Blender or glTF). For DirectX-style normal maps, invert the green
   * channel first.
   * \note Images are copied, the original ones can be deleted. */
  void assignNormalMap(const mrpt::img::CImage& img);

  /** Similar to assignNormalMap, but with move semantics. */
  void assignNormalMap(mrpt::img::CImage&& img);

  [[nodiscard]] const mrpt::img::CImage& getNormalMapImage() const { return m_normalMapImage; }
  [[nodiscard]] bool normalMapHasBeenAssigned() const { return m_normalMapAssigned; }

  /** Enable linear interpolation of textures (default=false, use nearest
   * pixel) */
  void enableTextureLinearInterpolation(bool enable) { m_textureInterpolate = enable; }
  [[nodiscard]] bool textureLinearInterpolation() const { return m_textureInterpolate; }

  void enableTextureMipMap(bool enable) { m_textureUseMipMaps = enable; }
  [[nodiscard]] bool textureMipMap() const { return m_textureUseMipMaps; }

  /** @name Raw access to textured-triangle shader buffer data
   * @{ */
  [[nodiscard]] const auto& shaderTexturedTrianglesBuffer() const { return m_triangles; }
  [[nodiscard]] auto& shaderTexturedTrianglesBufferMutex() const { return m_trianglesMtx; }
  /** @} */

 protected:
  void params_serialize(mrpt::serialization::CArchive& out) const;
  void params_deserialize(mrpt::serialization::CArchive& in);

  /** List of triangles  \sa TTriangle */
  mutable std::vector<mrpt::viz::TTriangle> m_triangles;
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_trianglesMtx;

  /** Returns the bounding box of m_triangles, or (0,0,0)-(0,0,0) if empty. */
  [[nodiscard]] mrpt::math::TBoundingBoxf trianglesBoundingBox() const;

  void writeToStreamTexturedObject(mrpt::serialization::CArchive& out) const;
  void readFromStreamTexturedObject(mrpt::serialization::CArchive& in);

 private:
  bool m_enableLight = true;
  TCullFace m_cullface = TCullFace::NONE;

  bool m_textureImageAssigned = false;
  mutable mrpt::img::CImage m_textureImage{4, 4};
  mutable mrpt::img::CImage m_textureImageAlpha;

  /** Of the texture using "m_textureImageAlpha" */
  mutable bool m_enableTransparency{false};
  bool m_textureInterpolate = false;
  bool m_textureUseMipMaps = true;

  bool m_normalMapAssigned = false;
  mutable mrpt::img::CImage m_normalMapImage;

  TAlphaMode m_alphaMode = TAlphaMode::Auto;
  float m_alphaCutoff = 0.5f;
  /** Alpha mode detected from the texture, for TAlphaMode::Auto */
  TAlphaMode m_detectedAlphaMode = TAlphaMode::Opaque;
  void detectAlphaMode();
};

class VisualObjectParams_Lines : public virtual CVisualObject
{
 public:
  VisualObjectParams_Lines() = default;

  void setLineWidth(float w)
  {
    m_lineWidth = w;
    CVisualObject::notifyChange();
  }
  [[nodiscard]] float getLineWidth() const { return m_lineWidth; }

  void enableAntiAliasing(bool enable = true)
  {
    m_antiAliasing = enable;
    CVisualObject::notifyChange();
  }
  [[nodiscard]] bool isAntiAliasingEnabled() const { return m_antiAliasing; }

  /// @name Raw access to line shader buffer data
  /// @{
  [[nodiscard]] const auto& shaderLinesVertexPointBuffer() const { return m_vertex_buffer_data; }
  [[nodiscard]] const auto& shaderLinesVertexColorBuffer() const { return m_color_buffer_data; }
  [[nodiscard]] auto& shaderLinesBufferMutex() const { return m_linesMtx; }
  /// @}

 protected:
  void params_serialize(mrpt::serialization::CArchive& out) const;
  void params_deserialize(mrpt::serialization::CArchive& in);

  /// Line segment vertices (pairs of points form line segments)
  mutable std::vector<mrpt::math::TPoint3Df> m_vertex_buffer_data;
  /// Per-vertex colors
  mutable std::vector<mrpt::img::TColor> m_color_buffer_data;
  /// Mutex for thread-safe access to buffers
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_linesMtx;

  /// Returns the bounding box of m_vertex_buffer_data, or empty if no lines.
  [[nodiscard]] mrpt::math::TBoundingBoxf linesBoundingBox() const;

 private:
  float m_lineWidth = 1.0f;
  bool m_antiAliasing = false;
};

class VisualObjectParams_Points : public virtual CVisualObject
{
 public:
  VisualObjectParams_Points() = default;

  /** By default is 1.0. \sa enableVariablePointSize() */
  void setPointSize(float p) { m_pointSize = p; }
  [[nodiscard]] float getPointSize() const { return m_pointSize; }

  /** Enable/disable variable eye distance-dependent point size (default=true)
   */
  void enableVariablePointSize(bool enable = true) { m_variablePointSize = enable; }
  [[nodiscard]] bool isEnabledVariablePointSize() const { return m_variablePointSize; }

  /** see CRenderizableShaderPoints for a discussion of this parameter. */
  void setVariablePointSize_k(float v) { m_variablePointSize_K = v; }
  [[nodiscard]] float getVariablePointSize_k() const { return m_variablePointSize_K; }

  /** see CRenderizableShaderPoints for a discussion of this parameter. */
  void setVariablePointSize_DepthScale(float v) { m_variablePointSize_DepthScale = v; }
  [[nodiscard]] float getVariablePointSize_DepthScale() const
  {
    return m_variablePointSize_DepthScale;
  }

  /** @name Raw access to point shader buffer data
   * @{ */
  const auto& shaderPointsVertexPointBuffer() const { return m_vertex_buffer_data; }
  const auto& shaderPointsVertexColorBuffer() const { return m_color_buffer_data; }
  auto& shaderPointsBuffersMutex() const { return m_pointsMtx; }

  /** @} */

 protected:
  void params_serialize(mrpt::serialization::CArchive& out) const;
  void params_deserialize(mrpt::serialization::CArchive& in);

  mutable std::vector<mrpt::math::TPoint3Df> m_vertex_buffer_data;
  mutable std::vector<mrpt::img::TColor> m_color_buffer_data;
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_pointsMtx;

  /** Returns the bounding box of m_vertex_buffer_data, or (0,0,0)-(0,0,0) if
   * empty. */
  const mrpt::math::TBoundingBoxf verticesBoundingBox() const;

 private:
  float m_pointSize = 1.0f;
  bool m_variablePointSize = true;
  float m_variablePointSize_K = 0.1f;
  float m_variablePointSize_DepthScale = 0.1f;
};

}  // namespace mrpt::viz

MRPT_ENUM_TYPE_BEGIN(mrpt::viz::TCullFace)
using namespace mrpt::viz;
MRPT_FILL_ENUM_MEMBER(TCullFace, NONE);
MRPT_FILL_ENUM_MEMBER(TCullFace, BACK);
MRPT_FILL_ENUM_MEMBER(TCullFace, FRONT);
MRPT_ENUM_TYPE_END()

MRPT_ENUM_TYPE_BEGIN(mrpt::viz::TAlphaMode)
using namespace mrpt::viz;
MRPT_FILL_ENUM_MEMBER(TAlphaMode, Auto);
MRPT_FILL_ENUM_MEMBER(TAlphaMode, Opaque);
MRPT_FILL_ENUM_MEMBER(TAlphaMode, Mask);
MRPT_FILL_ENUM_MEMBER(TAlphaMode, Blend);
MRPT_ENUM_TYPE_END()
