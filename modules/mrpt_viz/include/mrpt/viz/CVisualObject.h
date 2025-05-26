/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/NonCopiableData.h>
#include <mrpt/containers/PerThreadDataHolder.h>
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

#include <deque>
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

/** The base class of 3D objects that can be directly rendered through OpenGL.
 *  In this class there are a set of common properties to all 3D objects,
 *mainly:
 * - Its SE(3) pose (x,y,z,yaw,pitch,roll), relative to the parent object,
 * or the global frame of reference for root objects (inserted into a
 *mrpt::viz::Scene).
 * - A name: A name that can be optionally asigned to objects for
 *easing its reference.
 * - A RGBA color: This field will be used in simple elements (points,
 *lines, text,...) but is ignored in more complex objects that carry their own
 *color information (triangle sets,...)
 * - Shininess: See materialShininess(float)
 *
 * See the main class opengl::Scene
 *
 *  \sa opengl::Scene, mrpt::viz
 * \ingroup mrpt_viz_grp
 */
class CVisualObject : public mrpt::serialization::CSerializable
{
  DEFINE_VIRTUAL_SERIALIZABLE(CVisualObject, mrpt::viz)

  friend class mrpt::viz::Viewport;
  friend class mrpt::viz::CSetOfObjects;

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
  CVisualObject& setScale(float s)
  {
    m_stateMtx.data.lock();
    m_state.scale_x = m_state.scale_y = m_state.scale_z = s;
    m_stateMtx.data.unlock();
    notifyChange();
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

  /** Default constructor:  */
  CVisualObject() = default;
  ~CVisualObject() override;

  /** Should return true if enqueueForRenderRecursive() is defined since
   *  the object has inner children. Examples: CSetOfObjects, CAssimpModel.
   */
  virtual bool isCompositeObject() const { return false; }

  /** Call to enable calling renderUpdateBuffers() before the next
   * render() rendering iteration. */
  void notifyChange() const
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_outdatedStateMtx.data);
    m_cachedLocalBBox.reset();
    const_cast<CVisualObject&>(*this).m_outdatedBuffersState.run_on_all(
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

  /** Returns or constructs (in its first invocation) the associated
   * mrpt::viz::CText object representing the label of the object.
   * \sa enableShowName()
   */
  mrpt::viz::CText& labelObject() const;

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

  struct OutdatedState
  {
    bool outdatedBuffers = true;
  };
  mrpt::containers::PerThreadDataHolder<OutdatedState> m_outdatedBuffersState;
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
  virtual ~VisualObjectParams_Triangles() = default;

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
  virtual ~VisualObjectParams_TexturedTriangles() = default;

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
  [[nodiscard]] const mrpt::math::TBoundingBoxf trianglesBoundingBox() const;

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
};

class VisualObjectParams_Lines : public virtual CVisualObject
{
 public:
  VisualObjectParams_Lines() = default;
  virtual ~VisualObjectParams_Lines() = default;

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

 protected:
  void params_serialize(mrpt::serialization::CArchive& out) const;
  void params_deserialize(mrpt::serialization::CArchive& in);

 private:
  float m_lineWidth = 1.0f;
  bool m_antiAliasing = false;
};

class VisualObjectParams_Points : public virtual CVisualObject
{
 public:
  VisualObjectParams_Points() = default;
  virtual ~VisualObjectParams_Points() = default;

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
