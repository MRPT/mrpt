/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/NonCopiableData.h>
#include <mrpt/containers/PerThreadDataHolder.h>
#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TLightParameters.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/TEnumType.h>

#include <deque>
#include <optional>
#include <shared_mutex>

#ifdef MRPT_OPENGL_PROFILER
#include <mrpt/system/CTimeLogger.h>
#endif

namespace mrpt::opengl
{
/** Enum for cull face modes in triangle-based shaders.
 *  \sa CRenderizableShaderTriangles, CRenderizableShaderTexturedTriangles
 *  \ingroup mrpt_opengl_grp
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

/** The base class of 3D objects that can be directly rendered through OpenGL.
 *  In this class there are a set of common properties to all 3D objects,
 *mainly:
 * - Its SE(3) pose (x,y,z,yaw,pitch,roll), relative to the parent object,
 * or the global frame of reference for root objects (inserted into a
 *mrpt::opengl::Scene).
 * - A name: A name that can be optionally asigned to objects for
 *easing its reference.
 * - A RGBA color: This field will be used in simple elements (points,
 *lines, text,...) but is ignored in more complex objects that carry their own
 *color information (triangle sets,...)
 * - Shininess: See materialShininess(float)
 *
 * See the main class opengl::Scene
 *
 *  \sa opengl::Scene, mrpt::opengl
 * \ingroup mrpt_opengl_grp
 */
class CRenderizable : public mrpt::serialization::CSerializable
{
  DEFINE_VIRTUAL_SERIALIZABLE(CRenderizable)

  friend class mrpt::opengl::Viewport;
  friend class mrpt::opengl::CSetOfObjects;

 protected:
  struct State
  {
    std::string name = {};
    bool show_name = false;

    /** RGBA components in the range [0,255] */
    mrpt::img::TColor color = {0xff, 0xff, 0xff, 0xff};

    float materialShininess = 0.2f;

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
  }

  /** Does the object cast shadows? (default=true) */
  bool castShadows() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.castShadows;
  }
  /** Enable/disable casting shadows by this object (default=true) */
  void castShadows(bool doCast = true)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.castShadows = doCast;
  }

  /** Enables or disables showing the name of the object as a label when
   * rendering */
  void enableShowName(bool showName = true)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.show_name = showName;
  }
  /** \sa enableShowName */
  bool isShowNameEnabled() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.show_name;
  }

  /** Defines the SE(3) (pose=translation+rotation) of the object with respect
   * to its parent */
  CRenderizable& setPose(const mrpt::poses::CPose3D& o);
  /// \overload
  CRenderizable& setPose(const mrpt::poses::CPose2D& o);
  /// \overload
  CRenderizable& setPose(const mrpt::math::TPose3D& o);
  /// \overload
  CRenderizable& setPose(const mrpt::math::TPose2D& o);
  /// \overload
  CRenderizable& setPose(const mrpt::poses::CPoint3D& o);
  /// \overload
  CRenderizable& setPose(const mrpt::poses::CPoint2D& o);

  /** Returns the 3D pose of the object as TPose3D */
  mrpt::math::TPose3D getPose() const;

  /** Returns a const ref to the 3D pose of the object as mrpt::poses::CPose3D
   * (which explicitly contains the 3x3 rotation matrix) */
  mrpt::poses::CPose3D getCPose() const
  {
    std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
    return m_state.pose;
  }

  /** Changes the location of the object, keeping untouched the orientation
   * \return a ref to this */
  CRenderizable& setLocation(double x, double y, double z)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.pose.x(x);
    m_state.pose.y(y);
    m_state.pose.z(z);
    return *this;
  }

  /** Changes the location of the object, keeping untouched the orientation
   * \return a ref to this  */
  CRenderizable& setLocation(const mrpt::math::TPoint3D& p)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
    m_state.pose.x(p.x);
    m_state.pose.y(p.y);
    m_state.pose.z(p.z);
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
  CRenderizable& setColorA(const float a) { return setColorA_u8(f2u8(a)); }

  /** Set alpha (transparency) color component in the range [0,255]
   *  \return a ref to this */
  virtual CRenderizable& setColorA_u8(const uint8_t a)
  {
    m_stateMtx.data.lock();
    m_state.color.A = a;
    m_stateMtx.data.unlock();
    notifyChange();
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
  }

  /** Scale to apply to the object, in all three axes (default=1)  \return a
   * ref to this */
  CRenderizable& setScale(float s)
  {
    m_stateMtx.data.lock();
    m_state.scale_x = m_state.scale_y = m_state.scale_z = s;
    m_stateMtx.data.unlock();
    notifyChange();
    return *this;
  }

  /** Scale to apply to the object in each axis (default=1)  \return a ref to
   * this */
  CRenderizable& setScale(float sx, float sy, float sz)
  {
    m_stateMtx.data.lock();
    m_state.scale_x = sx;
    m_state.scale_y = sy;
    m_state.scale_z = sz;
    m_stateMtx.data.unlock();
    notifyChange();
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
  CRenderizable& setColor(const mrpt::img::TColorf& c)
  {
    return setColor_u8(mrpt::img::TColor(f2u8(c.R), f2u8(c.G), f2u8(c.B), f2u8(c.A)));
  }

  /** Set the color components of this object (R,G,B,Alpha, in the range 0-1)
   * \return a ref to this */
  CRenderizable& setColor(float R, float G, float B, float A = 1)
  {
    return setColor_u8(f2u8(R), f2u8(G), f2u8(B), f2u8(A));
  }

  /*** Changes the default object color \return a ref to this */
  virtual CRenderizable& setColor_u8(const mrpt::img::TColor& c);

  /** Set the color components of this object (R,G,B,Alpha, in the range
   * 0-255)  \return a ref to this */
  CRenderizable& setColor_u8(uint8_t R, uint8_t G, uint8_t B, uint8_t A = 255)
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

  /** Default constructor:  */
  CRenderizable() = default;
  ~CRenderizable() override;

  /** Context for calls to render() */
  struct RenderContext
  {
    RenderContext() = default;

    const mrpt::opengl::TRenderMatrices* state = nullptr;
    const mrpt::opengl::Program* shader = nullptr;
    mrpt::opengl::shader_id_t shader_id;
    const mrpt::opengl::TLightParameters* lights = nullptr;

    mutable std::optional<TCullFace> activeCullFace;

    /// The light that is currently bound to the shader uniforms:
    mutable std::optional<const mrpt::opengl::TLightParameters*> activeLights;
    /// The texture that is currently bound to the shader uniforms:
    mutable std::optional<int> activeTextureUnit;
  };

  /** Implements the rendering of 3D objects in each class derived from
   * CRenderizable. This can be called more than once (one per required shader
   * program) if the object registered several shaders. \sa
   * renderUpdateBuffers
   */
  virtual void render(const RenderContext& rc) const = 0;

  /** Process all children objects recursively, if the object is a container
   *  \param wholeInView If true, it means that the render engine has already
   * verified that the whole bounding box lies within the visible part of the
   * viewport, so further culling checks can be discarded.
   */
  virtual void enqueueForRenderRecursive(
      [[maybe_unused]] const mrpt::opengl::TRenderMatrices& state,
      [[maybe_unused]] RenderQueue& rq,   //
      [[maybe_unused]] bool wholeInView,  //
      [[maybe_unused]] bool is1stShadowMapPass) const
  {
    // do nothing
  }
  /** Should return true if enqueueForRenderRecursive() is defined since
   *  the object has inner children. Examples: CSetOfObjects, CAssimpModel.
   */
  virtual bool isCompositeObject() const { return false; }

  /** Called whenever m_outdatedBuffers is true: used to re-generate
   * OpenGL vertex buffers, etc. before they are sent for rendering in
   * render() */
  virtual void renderUpdateBuffers() const = 0;

  /** Returns the ID of the OpenGL shader program required to render this
   * class. \sa DefaultShaderID
   */
  virtual shader_list_t requiredShaders() const
  {
    THROW_EXCEPTION("Not implemented in derived class!");
  }

  /** Calls renderUpdateBuffers() and clear the flag that is set with
   * notifyChange() */
  void updateBuffers() const
  {
    renderUpdateBuffers();
    std::unique_lock<std::shared_mutex> lckWrite(m_outdatedStateMtx.data);
    const_cast<CRenderizable&>(*this).m_outdatedBuffersState.get().outdatedBuffers = false;
  }

  /** Call to enable calling renderUpdateBuffers() before the next
   * render() rendering iteration. */
  void notifyChange() const
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_outdatedStateMtx.data);
    m_cachedLocalBBox.reset();
    const_cast<CRenderizable&>(*this).m_outdatedBuffersState.run_on_all(
        [](auto& state) { state.outdatedBuffers = true; });
  }

  void notifyBBoxChange() const { m_cachedLocalBBox.reset(); }

  /** Returns whether notifyChange() has been invoked since the last call
   * to renderUpdateBuffers(), meaning the latter needs to be called again
   * before rendering.
   */
  bool hasToUpdateBuffers() const
  {
    std::shared_lock<std::shared_mutex> lckWrite(m_outdatedStateMtx.data);
    return m_outdatedBuffersState.get().outdatedBuffers;
  }

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

  /** Returns or constructs (in its first invokation) the associated
   * mrpt::opengl::CText object representing the label of the object.
   * \sa enableShowName()
   */
  mrpt::opengl::CText& labelObject() const;

  /** Free opengl buffers */
  virtual void freeOpenGLResources() = 0;

  /** Initializes all textures (loads them into opengl memory). */
  virtual void initializeTextures() const {}

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
  virtual mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const = 0;

  struct OutdatedState
  {
    bool outdatedBuffers = true;
  };
  mrpt::containers::PerThreadDataHolder<OutdatedState> m_outdatedBuffersState;
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_outdatedStateMtx;

  mutable std::optional<mrpt::math::TBoundingBoxf> m_cachedLocalBBox;

  /** Optional pointer to a mrpt::opengl::CText */
  mutable std::shared_ptr<mrpt::opengl::CText> m_label_obj;
};

/** A list of smart pointers to renderizable objects */
using CListOpenGLObjects = std::deque<CRenderizable::Ptr>;

/** @name Miscellaneous rendering methods
@{ */

/** Processes, recursively, all objects in the list, classifying them by shader
 * programs into a list suitable to be used within processPendingRendering()
 *
 * For each object in the list:
 *   - checks visibility of each object
 *   - update the MODELVIEW matrix according to its coordinates
 *   - call its ::render()
 *   - shows its name (if enabled).
 *
 * \param skipCullChecks Will be true if the render engine already checked that
 *        the object lies within the viewport area, so it is pointless to waste
 *        more time checking.
 *
 * \note Used by CSetOfObjects and Viewport
 *
 * \sa processPendingRendering
 */
void enqueueForRendering(
    const mrpt::opengl::CListOpenGLObjects& objs,
    const mrpt::opengl::TRenderMatrices& state,
    RenderQueue& rq,
    const bool skipCullChecks,
    const bool is1stShadowMapPass,
    RenderQueueStats* stats = nullptr);

/** After enqueueForRendering(), actually executes the rendering tasks, grouped
 * shader by shader.
 *
 *  \note Used by Viewport
 */
void processRenderQueue(
    const RenderQueue& rq,
    std::map<shader_id_t, mrpt::opengl::Program::Ptr>& shaders,
    const mrpt::opengl::TLightParameters& lights,
    const std::optional<unsigned int>& depthMapTextureId = std::nullopt);

#ifdef MRPT_OPENGL_PROFILER
mrpt::system::CTimeLogger& opengl_profiler();
#endif

/** @} */

}  // namespace mrpt::opengl

MRPT_ENUM_TYPE_BEGIN(mrpt::opengl::TCullFace)
using namespace mrpt::opengl;
MRPT_FILL_ENUM_MEMBER(TCullFace, NONE);
MRPT_FILL_ENUM_MEMBER(TCullFace, BACK);
MRPT_FILL_ENUM_MEMBER(TCullFace, FRONT);
MRPT_ENUM_TYPE_END()
