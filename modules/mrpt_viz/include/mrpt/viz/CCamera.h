/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TCamera.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/viz/CVisualObject.h>

#include <optional>

namespace mrpt::viz
{
/** Defines the intrinsic and extrinsic camera coordinates from which to render
 * a 3D scene.
 *
 *  By default, each viewport has its own camera, accessible via
 *  Viewport::getCamera(), but if a CCamera object is added as an object
 *  to be rendered, it will override the internal viewport camera.
 *
 *  Available projection models:
 *  - Projective model, parameterized via setProjectiveFOVdeg() (vertical field
 * of view, in degrees)
 *  - Projective model, by means of a computer vision pinhole intrinsic
 * parameter set: see setProjectiveFromPinhole()
 *  - Orthogonal projection model: use setProjectiveModel(false), or
 *  setOrthogonal(), or
 *  - No projection mode: use `setNoProjection()`. Viewport coordinates are
 *    fixed to bottomLeft=(-1,-1) to rightTop=(+1,+1).
 *
 * Defining the position and orientation of a camera is possible by:
 * - Using an "orbit" model: defined by a "pointing to" point, a distance to
 * object, and azimuth + elevation angles; or
 * - Directly giving the SE(3) camera pose, setting the set6DOFMode() to `true`
 * and storing the desired pose with CVisualObject::setPose(). Pose axis
 * convention is +Z pointing forwards, +X right, +Y down.
 *
 *  \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CCamera : public CVisualObject
{
  DEFINE_SERIALIZABLE(CCamera, mrpt::viz)
 public:
  CCamera() = default;
  ~CCamera() override = default;

  void toYAMLMap(mrpt::containers::yaml& propertiesMap) const override;

  /** @name Projection model (camera intrinsic parameters)
   *  @{ */

  /** Enable/Disable projective mode (vs. orthogonal). */
  void setProjectiveModel(bool v = true)
  {
    m_projectiveModel = v;
    m_useNoProjection = false;
  }

  /** Enable/Disable orthogonal mode (vs. projective)*/
  void setOrthogonal(bool v = true)
  {
    m_projectiveModel = !v;
    m_useNoProjection = false;
  }

  void setProjectiveFromPinhole(const mrpt::img::TCamera& camIntrinsics)
  {
    m_projectiveModel = true;
    m_useNoProjection = false;
    m_pinholeModel = camIntrinsics;
  }

  /** Disable all coordinate transformations and allow direct use of pixel
   * coordinates, that is, the projection matrix is the identity.
   *
   * In this mode, (-1,-1) is the bottom-left corner and (+1,+1) the top-right
   * corner, per OpenGL defaults. This mode can be disabled calling
   * setProjectiveModel() or setOrthogonal()
   */
  void setNoProjection() { m_useNoProjection = true; }

  /** Vertical field-of-View in degs, only when projectiveModel=true
   * (default=30 deg).
   */
  void setProjectiveFOVdeg(float ang) { m_projectiveFOVdeg = ang; }

  /** Field-of-View in degs, only when projectiveModel=true (default=30 deg).
   */
  [[nodiscard]] float getProjectiveFOVdeg() const { return m_projectiveFOVdeg; }

  [[nodiscard]] bool hasPinholeModel() const { return m_pinholeModel.has_value(); }

  const std::optional<mrpt::img::TCamera>& getPinholeModel() const { return m_pinholeModel; }

  [[nodiscard]] bool isProjective() const { return m_projectiveModel; }
  [[nodiscard]] bool isOrthogonal() const { return !m_projectiveModel; }
  [[nodiscard]] bool isNoProjection() const { return m_useNoProjection; }

  /** @} */

  /** @name Defines camera pose (camera extrinsic parameters)
   *  @{ */

  void setPointingAt(float x, float y, float z)
  {
    m_pointingX = x;
    m_pointingY = y;
    m_pointingZ = z;
  }

  template <class POSEORPOINT>
  void setPointingAt(const POSEORPOINT& p)
  {
    m_pointingX = p.x();
    m_pointingY = p.y();
    m_pointingZ = p.is3DPoseOrPoint() ? p.m_coords[2] : 0;
  }
  void setPointingAt(const mrpt::math::TPoint3D& p) { setPointingAt(d2f(p.x), d2f(p.y), d2f(p.z)); }

  [[nodiscard]] float getPointingAtX() const { return m_pointingX; }
  [[nodiscard]] float getPointingAtY() const { return m_pointingY; }
  [[nodiscard]] float getPointingAtZ() const { return m_pointingZ; }

  [[nodiscard]] mrpt::math::TPoint3Df getPointingAt() const
  {
    return {m_pointingX, m_pointingY, m_pointingZ};
  }

  [[nodiscard]] float getZoomDistance() const { return m_eyeDistance; }
  [[nodiscard]] float getAzimuthDegrees() const { return m_azimuthDeg; }
  [[nodiscard]] float getElevationDegrees() const { return m_elevationDeg; }
  [[nodiscard]] float getRollDegrees() const { return m_eyeRollDeg; }

  void setZoomDistance(float z) { m_eyeDistance = z; }
  void setAzimuthDegrees(float ang) { m_azimuthDeg = ang; }
  void setElevationDegrees(float ang) { m_elevationDeg = ang; }
  void setRollDegrees(float ang) { m_eyeRollDeg = ang; }

  /** Set 6DOFMode, if enabled camera is set according to its pose, set via
   *CVisualObject::setPose(). (default=false).
   * Conventionally, eye is set looking towards +Z axis, "down" is the +Y
   * axis, right is "+X" axis. In this mode azimuth/elevation are ignored.
   */
  void set6DOFMode(bool v) { m_6DOFMode = v; }

  [[nodiscard]] bool is6DOFMode() const { return m_6DOFMode; }

  /** @} */

  /** In this class, returns a fixed box (max,max,max), (-max,-max,-max). */
  [[nodiscard]] mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

 protected:
  float m_pointingX{0}, m_pointingY{0}, m_pointingZ{0};
  float m_eyeDistance{10};
  float m_azimuthDeg{45}, m_elevationDeg{45};
  float m_eyeRollDeg{0};

  /** If set to true (default), camera model is projective, otherwise, it's
   * orthogonal. */
  bool m_projectiveModel{true};

  /// See setNoProjection()
  bool m_useNoProjection = false;

  /** If defined, it overrides m_projectiveFOVdeg. */
  std::optional<mrpt::img::TCamera> m_pinholeModel;

  /** Field-of-View in degs, only when projectiveModel=true and there is no
   * pinhole model assigned. (default=30 deg).
   */
  float m_projectiveFOVdeg{30};

  /** If set to true, camera pose is used when rendering the viewport */
  bool m_6DOFMode{false};
};

}  // namespace mrpt::viz
