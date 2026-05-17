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

#include <mrpt/core/Stringifyable.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/CPose.h>
#include <mrpt/system/string_utils.h>

#include <tuple>

namespace mrpt::poses
{
class CPose3DQuat;

/** SE(3) rigid-body pose with cached rotation matrix — the preferred class for 3D transformations.
 *
 * Represents an element of SE(3) = SO(3) ⋉ R³. Internally stored as:
 * - A 3-vector `m_coords = (x, y, z)` — translation in metres.
 * - A 3×3 rotation matrix `m_ROT ∈ SO(3)` — always kept normalised.
 *
 * The corresponding 4x4 homogeneous matrix is:
 * \verbatim
 *   T = | R  t |   where R in SO(3), t = (x,y,z)^T
 *       | 0  1 |
 * \endverbatim
 *
 * **Angle parameterisation (intrinsic ZYX / Tait-Bryan):**
 * Yaw/Pitch/Roll angles (yaw=psi, pitch=theta, roll=phi) are derived on demand from `m_ROT`
 * via `getYawPitchRoll()`:
 *
 *  1. Rotate about the body Z axis by **yaw** (psi).
 *  2. Rotate about the *new* body Y axis by **pitch** (theta).
 *  3. Rotate about the *new* body X axis by **roll** (phi).
 *
 * So R = Rz(yaw) * Ry(pitch) * Rx(roll). This is equivalent to extrinsic rotations about
 * the **fixed** axes in the reverse order (X then Y then Z).
 * The representation is singular (gimbal lock) when pitch = +-pi/2.
 *
 * To change pose components, always use setFromValues() — **never** assign to `m_ROT` directly
 * using yaw/pitch/roll since the cached angles would become stale. The rotation matrix is
 * always the primary storage; yaw/pitch/roll are recomputed lazily.
 *
 * **Operator semantics (SE(3) group):**
 * - `a + b` (⊕): pose composition — maps the frame of `b` into the frame of `a`.
 *   In matrix form: `T_a · T_b`.
 * - `a - b` (⊖): relative pose — pose of `a` as seen from `b`. In matrix form: `T_b⁻¹ · T_a`.
 * - Unary `operator-`: SE(3) group inverse. **Not** elementwise negation of (x,y,z,yaw,pitch,roll).
 * - `pose + point`: applies the SE(3) transformation to a 3D point (composePoint).
 *
 * **Alternative representations:**
 * - As a unit quaternion + translation: CPose3DQuat / getAsQuaternion().
 * - As a Lie algebra twist (se(3) tangent vector): see mrpt::poses::Lie.
 *
 * **Serializable:** supports `mrpt::serialization::CArchive` and YAML schema serialization.
 *
 * ![CPose3D](CPose3D.gif)
 *
 * \note See also: "A tutorial on SE(3) transformation parameterizations and
 *   on-manifold optimization", J.L. Blanco. \cite blanco_se3_tutorial
 *
 * \ingroup poses_grp
 * \sa mrpt::math::TPose3D, CPoseOrPoint, CPoint3D, CPose3DQuat, mrpt::math::CQuaternion,
 * mrpt::poses::Lie
 */
class CPose3D :
    public CPose<CPose3D, 6>,
    public mrpt::serialization::CSerializable,
    public mrpt::Stringifyable
{
  DEFINE_SERIALIZABLE(CPose3D, mrpt::poses)
  DEFINE_SCHEMA_SERIALIZABLE()

 public:
  /** The translation vector [x,y,z] access directly or with x(), y(), z()
   * setter/getter methods. */
  mrpt::math::CVectorFixedDouble<3> m_coords;

 protected:
  /** The SO(3) rotation matrix — primary storage for orientation.
   *  Access via getRotationMatrix() / setRotationMatrix(); do **not** modify directly,
   *  as that would leave the cached yaw/pitch/roll angles inconsistent. */
  mrpt::math::CMatrixDouble33 m_ROT;

  /** Whether yaw/pitch/roll members are up-to-date since the last rotation
   * matrix update. */
  mutable bool m_ypr_uptodate{false};
  /** These variables are updated every time that the object rotation matrix
   * is modified (construction, loading from values, pose composition, etc )
   */
  mutable double m_yaw{0}, m_pitch{0}, m_roll{0};

  /** Rebuild the homog matrix from the angles. */
  void rebuildRotationMatrix();

  /** Updates Yaw/pitch/roll members from the m_ROT  */
  void updateYawPitchRoll() const
  {
    if (!m_ypr_uptodate)
    {
      m_ypr_uptodate = true;
      auto [y, p, r] = getYawPitchRoll();
      m_yaw = y;
      m_pitch = p;
      m_roll = r;
    }
  }

 public:
  /** @name Constructors
    @{ */

  /** Default constructor, with all the coordinates set to zero. */
  CPose3D();

  /** Constructor with Initialization of the pose, translation (x,y,z) in
   * meters, (yaw,pitch,roll) angles in radians.
   *
   * \sa FromXYZYawPitchRoll()
   */
  CPose3D(
      const double x,
      const double y,
      const double z,
      const double yaw = 0,
      const double pitch = 0,
      const double roll = 0);

  /** Returns the identity transformation */
  static CPose3D Identity() { return CPose3D(); }

  /** Builds a pose from a translation (x,y,z) in
   * meters and (yaw,pitch,roll) angles in radians. \note (New in MRPT 2.1.8)
   */
  static CPose3D FromXYZYawPitchRoll(
      double x, double y, double z, double yaw, double pitch, double roll)
  {
    return CPose3D(x, y, z, yaw, pitch, roll);
  }

  /** Builds a pose with a null translation and (yaw,pitch,roll) angles in
   * radians. \note (New in MRPT 2.1.8)
   */
  static CPose3D FromYawPitchRoll(double yaw, double pitch, double roll)
  {
    return CPose3D(.0, .0, .0, yaw, pitch, roll);
  }

  /** Builds a pose with a translation without rotation \note (New in
   * MRPT 2.1.8)
   */
  static CPose3D FromTranslation(double x, double y, double z)
  {
    return CPose3D(x, y, z, .0, .0, .0);
  }
  /** \overload \note (New in MRPT 2.3.3)
   */
  static CPose3D FromTranslation(const mrpt::math::TPoint3D& t)
  {
    return CPose3D(t.x, t.y, t.z, .0, .0, .0);
  }

  /** Constructor from a 4x4 homogeneous matrix - the passed matrix can be
   * actually of any size larger than or equal 3x4, since only those first
   * values are used (the last row of a homogeneous 4x4 matrix are always
   * fixed). */
  explicit CPose3D(const math::CMatrixDouble& m);

  /** Constructor from a 4x4 homogeneous matrix: */
  explicit CPose3D(const math::CMatrixDouble44& m);

  /** Builds a pose with a 4x4 homogeneous matrix
   * \note (New in MRPT 2.1.8)
   */
  template <class MATRIX>
  static CPose3D FromHomogeneousMatrix(const MATRIX& m)
  {
    return CPose3D(mrpt::math::CMatrixDouble44(m));
  }

  /** Constructor from a 3x3 rotation matrix and a the translation given as a
   * 3-vector, a 3-array, a CPoint3D or a mrpt::math::TPoint3D */
  template <class MATRIX33, class VECTOR3>
  CPose3D(const MATRIX33& rot, const VECTOR3& xyz) :
      m_ROT(mrpt::math::UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
  {
    ASSERT_EQUAL_(rot.rows(), 3);
    ASSERT_EQUAL_(rot.cols(), 3);
    ASSERT_EQUAL_(xyz.size(), 3);
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) m_ROT(r, c) = rot(r, c);
    for (int r = 0; r < 3; r++) m_coords[r] = xyz[r];
  }
  //! \overload
  CPose3D(const mrpt::math::CMatrixDouble33& rot, const mrpt::math::CVectorFixedDouble<3>& xyz) :
      m_coords(xyz), m_ROT(rot), m_ypr_uptodate(false)
  {
  }

  /** Builds a pose with a 3x3 SO(3) rotation matrix and a translation vector.
   * \note (New in MRPT 2.1.8)
   */
  template <class MATRIX, class VECTOR>
  static CPose3D FromRotationAndTranslation(const MATRIX& rot, const VECTOR& t)
  {
    return CPose3D(rot, t);
  }

  /** Constructor from a CPose2D object.
   */
  explicit CPose3D(const CPose2D&);

  /** Constructor from a CPoint3D object.
   */
  explicit CPose3D(const CPoint3D&);

  /** Constructor from lightweight object.
   */
  explicit CPose3D(const mrpt::math::TPose3D&);

  mrpt::math::TPose3D asTPose() const;

  /** Constructor from a quaternion (which only represents the 3D rotation
   * part) and a 3D displacement. */
  CPose3D(const mrpt::math::CQuaternionDouble& q, const double x, const double y, const double z);

  /** Constructor from a CPose3DQuat. */
  explicit CPose3D(const CPose3DQuat&);

  /** Builds a pose from a quaternion (and no translation).
   * \note (New in MRPT 2.1.8)
   */
  static CPose3D FromQuaternion(const mrpt::math::CQuaternionDouble& q)
  {
    return CPose3D(q, .0, .0, .0);
  }

  /** Builds a pose from a quaternion and a (x,y,z) translation.
   * \note (New in MRPT 2.1.8)
   */
  static CPose3D FromQuaternionAndTranslation(
      const mrpt::math::CQuaternionDouble& q, double x, double y, double z)
  {
    return CPose3D(q, x, y, z);
  }

  /** Builds a pose from a quaternion and a 3D translation.
   * \note (New in MRPT 2.3.3)
   */
  template <typename Point3DLike>
  static CPose3D FromQuaternionAndTranslation(
      const mrpt::math::CQuaternionDouble& q, const Point3DLike& pt)
  {
    return CPose3D(q, pt.x, pt.y, pt.z);
  }

  /** Fast constructor that leaves all the data uninitialized - call with
   * UNINITIALIZED_POSE as argument */
  CPose3D(TConstructorFlags_Poses) : m_ROT(mrpt::math::UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
  {
  }

  /** Constructor from an array with these 12 elements: [r11 r21 r31 r12 r22
   * r32 r13 r23 r33 tx ty tz]
   *  where r{ij} are the entries of the 3x3 rotation matrix and t{x,y,z} are
   * the 3D translation of the pose
   *  \sa setFrom12Vector, getAs12Vector
   */
  explicit CPose3D(const mrpt::math::CVectorFixedDouble<12>& vec12) :
      m_ROT(mrpt::math::UNINITIALIZED_MATRIX), m_ypr_uptodate(false)
  {
    setFrom12Vector(vec12);
  }

  /** @} */  // end Constructors

  /** @name Access SO(3), SE(3), R(3)
    @{ */

  /** Returns the corresponding 4x4 homogeneous transformation matrix for the
   * point(translation) or pose (translation+orientation).
   * \sa getInverseHomogeneousMatrix, getRotationMatrix
   * \deprecated Use getHomogeneousMatrix() returning by value instead.
   */
  [[deprecated("Use getHomogeneousMatrix() returning by value instead.")]] void
  getHomogeneousMatrix(mrpt::math::CMatrixDouble44& out_HM) const;

  /** Returns the corresponding 4x4 homogeneous transformation matrix.
   * \sa getInverseHomogeneousMatrix, getRotationMatrix
   */
  [[nodiscard]] mrpt::math::CMatrixDouble44 getHomogeneousMatrix() const;

  /** Get the 3x3 rotation matrix \sa getHomogeneousMatrix
   * \deprecated Use getRotationMatrix() returning by value instead.
   */
  [[deprecated("Use getRotationMatrix() returning by value instead.")]] void getRotationMatrix(
      mrpt::math::CMatrixDouble33& ROT) const
  {
    ROT = m_ROT;
  }
  //! \overload Returns rotation matrix by const reference (efficient, zero-copy).
  const mrpt::math::CMatrixDouble33& getRotationMatrix() const { return m_ROT; }

  /** Sets the 3x3 rotation matrix \sa getRotationMatrix, getHomogeneousMatrix
   */
  void setRotationMatrix(const mrpt::math::CMatrixDouble33& ROT)
  {
    m_ROT = ROT;
    m_ypr_uptodate = false;
  }

  /** Returns the (x,y,z) translational part of the SE(3) transformation. */
  [[nodiscard]] const mrpt::math::TPoint3D translation() const
  {
    return {m_coords[0], m_coords[1], m_coords[2]};
  }

  /** @} */  // end rot and HM

  /** @name Pose-pose and pose-point compositions and operators
    @{ */

  /** SE(3) group composition: `ret = this ⊕ b`.
   *
   *  If `this` is the pose of frame A in world W, and `b` is the pose of frame B in A,
   *  then `ret` is the pose of B in W. In matrix form: `T_this · T_b`.
   */
  CPose3D operator+(const CPose3D& b) const
  {
    CPose3D ret(UNINITIALIZED_POSE);
    ret.composeFrom(*this, b);
    return ret;
  }

  /** Applies the SE(3) transformation to a CPoint3D: `ret = this ⊕ b`. */
  CPoint3D operator+(const CPoint3D& b) const;

  /** Applies the SE(3) transformation to a CPoint2D (z=0): `ret = this ⊕ b`. */
  CPoint3D operator+(const CPoint2D& b) const;

  /** Computes the spherical (range, yaw, pitch) coordinates of a 3D point expressed
   *  in the global frame, as observed from the sensor frame defined by this pose.
   *  The angles follow the same ZY intrinsic convention as CPose3D. */
  void sphericalCoordinates(
      const mrpt::math::TPoint3D& point,
      double& out_range,
      double& out_yaw,
      double& out_pitch) const;

  /** Transforms a 3D point from the **local** frame of this pose into the **global** frame.
   *  Computes: g = R * l + t
   *
   *  Optional Jacobians (all w.r.t. the *input* quantities):
   *  - `out_jacobian_df_dpoint` (3x3): dg/dl = R.
   *  - `out_jacobian_df_dpose`  (3x6): dg/d(t, yaw, pitch, roll)
   *    in the yaw/pitch/roll parameterisation. Set `use_small_rot_approx=true` for the first-order
   *    linearisation (valid only for small rotations).
   *  - `out_jacobian_df_dse3`   (3x6): dg/dxi
   *    where xi is the 6D locally-Euclidean tangent-space vector of SE(3) (see \cite
   * blanco_se3_tutorial).
   *
   * \sa inverseComposePoint, composePoint(const TPoint3D&)
   */
  void composePoint(
      double lx,
      double ly,
      double lz,
      double& gx,
      double& gy,
      double& gz,
      mrpt::optional_ref<mrpt::math::CMatrixDouble33> out_jacobian_df_dpoint = std::nullopt,
      mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dpose = std::nullopt,
      mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dse3 = std::nullopt,
      bool use_small_rot_approx = false) const;

  /** An alternative, slightly more efficient way of doing `G = P (+) L`
   * with G and L being 3D points and P this 6D pose.
   * \note local_point is passed by value to allow global and local point to
   * be the same variable
   */
  void composePoint(
      const mrpt::math::TPoint3D& local_point, mrpt::math::TPoint3D& global_point) const
  {
    composePoint(
        local_point.x, local_point.y, local_point.z, global_point.x, global_point.y,
        global_point.z);
  }
  /** \overload Returns global point: `this (+) l` */
  mrpt::math::TPoint3D composePoint(const mrpt::math::TPoint3D& l) const
  {
    mrpt::math::TPoint3D g;
    composePoint(l, g);
    return g;
  }

  /** This version of the method assumes that the resulting point has no Z
   * component (use with caution!) */
  void composePoint(
      const mrpt::math::TPoint3D& local_point, mrpt::math::TPoint2D& global_point) const
  {
    double dummy_z;
    composePoint(
        local_point.x, local_point.y, local_point.z, global_point.x, global_point.y, dummy_z);
  }

  /** An alternative, slightly more efficient way of doing `G = P (+) L`
   * with G and L being 3D points and P this 6D pose.  */
  void composePoint(double lx, double ly, double lz, float& gx, float& gy, float& gz) const
  {
    double ggx, ggy, ggz;
    composePoint(lx, ly, lz, ggx, ggy, ggz);
    gx = d2f(ggx);
    gy = d2f(ggy);
    gz = d2f(ggz);
  }

  /** Rotates a vector (i.e. like composePoint(), but ignoring translation) */
  mrpt::math::TVector3D rotateVector(const mrpt::math::TVector3D& local) const;

  /** Inverse of rotateVector(), i.e. using the inverse rotation matrix */
  mrpt::math::TVector3D inverseRotateVector(const mrpt::math::TVector3D& global) const;

  /** Transforms a 3D point from the **global** frame into the **local** frame of this pose.
   *  Computes: l = R^T * (g - t)
   *
   *  Optional Jacobians follow the same conventions as composePoint():
   *  - `out_jacobian_df_dpoint` (3x3): dl/dg = R^T.
   *  - `out_jacobian_df_dpose`  (3×6): w.r.t. yaw/pitch/roll parameterisation.
   *  - `out_jacobian_df_dse3`   (3×6): w.r.t. SE(3) tangent-space vector ξ (see \cite
   * blanco_se3_tutorial). \sa composePoint, composeFrom
   */
  void inverseComposePoint(
      const double gx,
      const double gy,
      const double gz,
      double& lx,
      double& ly,
      double& lz,
      mrpt::optional_ref<mrpt::math::CMatrixDouble33> out_jacobian_df_dpoint = std::nullopt,
      mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dpose = std::nullopt,
      mrpt::optional_ref<mrpt::math::CMatrixDouble36> out_jacobian_df_dse3 = std::nullopt) const;

  /** \overload */
  void inverseComposePoint(const mrpt::math::TPoint3D& g, mrpt::math::TPoint3D& l) const
  {
    inverseComposePoint(g.x, g.y, g.z, l.x, l.y, l.z);
  }
  /** \overload Returns local point: `g` as seen from `this` pose */
  mrpt::math::TPoint3D inverseComposePoint(const mrpt::math::TPoint3D& g) const
  {
    mrpt::math::TPoint3D l;
    inverseComposePoint(g, l);
    return l;
  }

  /** overload for 2D points \exception If the z component of the result is
   * greater than some epsilon */
  void inverseComposePoint(
      const mrpt::math::TPoint2D& g, mrpt::math::TPoint2D& l, const double eps = 1e-6) const
  {
    double lz;
    inverseComposePoint(g.x, g.y, 0, l.x, l.y, lz);
    ASSERT_LT_(std::abs(lz), eps);
  }

  /** In-place SE(3) group composition: computes `this = A ⊕ B` (matrix form: `T_A · T_B`).
   *  Slightly more efficient than `this = A + B` as it avoids a temporary.
   *  \note A or B may safely alias `this`.
   */
  void composeFrom(const CPose3D& A, const CPose3D& B);

  /** Compound-assignment composition: `this = this ⊕ b`. `b` may safely alias `this`. */
  CPose3D& operator+=(const CPose3D& b)
  {
    composeFrom(*this, b);
    return *this;
  }

  /** In-place relative pose: computes `this = A ⊖ B = B⁻¹ ⊕ A` (matrix form: `T_B⁻¹ · T_A`).
   *  Slightly more efficient than `this = A - B` as it avoids a temporary.
   *  \note A or B may safely alias `this`.
   * \sa composeFrom, composePoint
   */
  void inverseComposeFrom(const CPose3D& A, const CPose3D& B);

  /** SE(3) relative pose: `ret = this ⊖ b = b⁻¹ ⊕ this`.
   *  Returns the pose of `this` expressed in the frame of `b`. */
  CPose3D operator-(const CPose3D& b) const
  {
    CPose3D ret(UNINITIALIZED_POSE);
    ret.inverseComposeFrom(*this, b);
    return ret;
  }

  /** Replaces this pose with its SE(3) group inverse in-place.
   *  If T = [R|t], then T⁻¹ = [Rᵀ | -Rᵀ·t].
   *  \note **Not** the same as negating all 6 components individually.
   * \sa operator-
   */
  void inverse();

  /** makes: this = p (+) this */
  void changeCoordinatesReference(const CPose3D& p) { composeFrom(p, CPose3D(*this)); }

  /** @} */  // compositions

  /** Return the opposite of the current pose instance by taking the negative
   * of all its components \a individually
   */
  CPose3D getOppositeScalar() const;

  /** @name Access and modify contents
    @{ */

  /** Scalar sum of all 6 components: This is different from poses composition,
   * which is implemented as "+" operators.
   * \sa normalizeAngles
   */
  void addComponents(const CPose3D& p);

  /** Rebuild the internal matrix & update the yaw/pitch/roll angles within
   * the ]-PI,PI] range (Must be called after using addComponents)
   * \sa addComponents
   */
  void normalizeAngles();

  /** Scalar multiplication of x,y,z,yaw,pitch & roll (angles will be wrapped
   * to the ]-pi,pi] interval). */
  void operator*=(const double s);

  /** Set the pose from a 3D position (meters) and yaw/pitch/roll angles
   * (radians) - This method recomputes the internal rotation matrix.
   * \sa getYawPitchRoll, setYawPitchRoll
   */
  void setFromValues(
      const double x0,
      const double y0,
      const double z0,
      const double yaw = 0,
      const double pitch = 0,
      const double roll = 0);

  /** Set the pose from a 3D position (meters) and a quaternion, stored as [x
   * y z qr qx qy qz] in a 7-element vector.
   * \sa setFromValues, getYawPitchRoll, setYawPitchRoll, CQuaternion,
   * getAsQuaternion
   */
  template <typename VECTORLIKE>
  void setFromXYZQ(const VECTORLIKE& v, size_t index_offset = 0)
  {
    ASSERT_GE_(v.size(), 7 + index_offset);
    // The 3x3 rotation part:
    mrpt::math::CQuaternion<typename VECTORLIKE::value_type> q(
        v[index_offset + 3], v[index_offset + 4], v[index_offset + 5], v[index_offset + 6]);
    q.rotationMatrixNoResize(m_ROT);
    m_ypr_uptodate = false;
    m_coords[0] = v[index_offset + 0];
    m_coords[1] = v[index_offset + 1];
    m_coords[2] = v[index_offset + 2];
  }

  /** Set the 3 angles of the 3D pose (in radians) - This method recomputes
   * the internal rotation coordinates matrix.
   * \sa getYawPitchRoll, setFromValues
   */
  void setYawPitchRoll(const double yaw_, const double pitch_, const double roll_)
  {
    setFromValues(x(), y(), z(), yaw_, pitch_, roll_);
  }

  /** Set pose from an array with these 12 elements: [r11 r21 r31 r12 r22 r32
   * r13 r23 r33 tx ty tz]
   *  where r{ij} are the entries of the 3x3 rotation matrix and t{x,y,z} are
   * the 3D translation of the pose
   *  \sa getAs12Vector
   */
  template <class ARRAYORVECTOR>
  void setFrom12Vector(const ARRAYORVECTOR& vec12)
  {
    m_ROT(0, 0) = vec12[0];
    m_ROT(0, 1) = vec12[3];
    m_ROT(0, 2) = vec12[6];
    m_ROT(1, 0) = vec12[1];
    m_ROT(1, 1) = vec12[4];
    m_ROT(1, 2) = vec12[7];
    m_ROT(2, 0) = vec12[2];
    m_ROT(2, 1) = vec12[5];
    m_ROT(2, 2) = vec12[8];
    m_ypr_uptodate = false;
    m_coords[0] = vec12[9];
    m_coords[1] = vec12[10];
    m_coords[2] = vec12[11];
  }

  /** Get the pose representation as an array with these 12 elements: [r11 r21
   * r31 r12 r22 r32 r13 r23 r33 tx ty tz]
   *  where r{ij} are the entries of the 3x3 rotation matrix and t{x,y,z} are
   * the 3D translation of the pose
   *  \sa setFrom12Vector
   */
  template <class ARRAYORVECTOR>
  void getAs12Vector(ARRAYORVECTOR& vec12) const
  {
    vec12[0] = m_ROT(0, 0);
    vec12[3] = m_ROT(0, 1);
    vec12[6] = m_ROT(0, 2);
    vec12[1] = m_ROT(1, 0);
    vec12[4] = m_ROT(1, 1);
    vec12[7] = m_ROT(1, 2);
    vec12[2] = m_ROT(2, 0);
    vec12[5] = m_ROT(2, 1);
    vec12[8] = m_ROT(2, 2);
    vec12[9] = m_coords[0];
    vec12[10] = m_coords[1];
    vec12[11] = m_coords[2];
  }

  /** Returns the three angles (yaw, pitch, roll), in radians, from the
   * rotation matrix.
   * \sa setFromValues, yaw, pitch, roll
   */
  /** Returns the three angles (yaw, pitch, roll), in radians, as a tuple.
   * \sa setFromValues, yaw, pitch, roll
   */
  [[nodiscard]] std::tuple<double, double, double> getYawPitchRoll() const;

  /** \deprecated Use getYawPitchRoll() returning a tuple instead. */
  [[deprecated("Use getYawPitchRoll() returning a tuple instead.")]] void getYawPitchRoll(
      double& yaw, double& pitch, double& roll) const;

  /** Get the YAW angle (in radians)  \sa setFromValues */
  double yaw() const
  {
    updateYawPitchRoll();
    return m_yaw;
  }
  /** Get the PITCH angle (in radians) \sa setFromValues */
  double pitch() const
  {
    updateYawPitchRoll();
    return m_pitch;
  }
  /** Get the ROLL angle (in radians) \sa setFromValues */
  double roll() const
  {
    updateYawPitchRoll();
    return m_roll;
  }

  /** Returns a 6x1 vector with [x y z yaw pitch roll]' */
  void asVector(vector_t& v) const;

  /** Converts the SO(3) rotation to a unit quaternion q = (qr, qx, qy, qz).
   *
   * The translation (x,y,z) is **ignored** — only `m_ROT` is converted.
   *
   * The mapping from intrinsic ZYX angles (yaw, pitch, roll) to the
   * Hamilton unit quaternion (scalar-first) is:
   * \verbatim
   *   Let cy=cos(yaw/2), sy=sin(yaw/2)
   *       cp=cos(pitch/2), sp=sin(pitch/2)
   *       cr=cos(roll/2),  sr=sin(roll/2)
   *
   *   q = [ cr*cp*cy + sr*sp*sy ]   (qr, real/scalar part)
   *       [ sr*cp*cy - cr*sp*sy ]   (qx)
   *       [ cr*sp*cy + sr*cp*sy ]   (qy)
   *       [ cr*cp*sy - sr*sp*cy ]   (qz)
   * \endverbatim
   *
   * \param[out] q The resulting unit quaternion in (qr, qx, qy, qz) / Hamilton / scalar-first
   * order.
   * \param[out] out_dq_dr  If provided, the 4x3 Jacobian dq/d(yaw,pitch,roll)
   *   is computed and stored here.
   */
  void getAsQuaternion(
      mrpt::math::CQuaternionDouble& q,
      mrpt::optional_ref<mrpt::math::CMatrixDouble43> out_dq_dr = std::nullopt) const;

  mrpt::math::CMatrixDouble33 jacobian_rodrigues_from_YPR() const;

  mrpt::math::CMatrixDouble66 jacobian_pose_rodrigues_from_YPR() const;

  double operator[](unsigned int i) const
  {
    updateYawPitchRoll();
    switch (i)
    {
      case 0:
        return m_coords[0];
      case 1:
        return m_coords[1];
      case 2:
        return m_coords[2];
      case 3:
        return m_yaw;
      case 4:
        return m_pitch;
      case 5:
        return m_roll;
      default:
        throw std::runtime_error("CPose3D::operator[]: Index of bounds.");
    }
  }
  // CPose3D CANNOT have a write [] operator, since it'd leave the object in
  // an inconsistent state (outdated rotation matrix).
  // Use setFromValues() instead.
  // double &operator[](unsigned int i)

  /** Returns a human-readable textual representation of the object (eg: "[x y
   * z yaw pitch roll]", angles in degrees.)
   * \sa fromString
   */
  std::string asString() const override
  {
    using mrpt::RAD2DEG;
    updateYawPitchRoll();
    return mrpt::format(
        "[%f %f %f %f %f %f]", m_coords[0], m_coords[1], m_coords[2], RAD2DEG(m_yaw),
        RAD2DEG(m_pitch), RAD2DEG(m_roll));
  }

  /** Set the current object value from a string generated by 'asString' (eg:
   * "[x y z yaw pitch roll]", angles in deg. )
   * \sa asString
   * \exception std::exception On invalid format
   */
  void fromString(const std::string& s);

  /** Same as fromString, but without requiring the square brackets in the
   * string */
  void fromStringRaw(const std::string& s);

  static CPose3D FromString(const std::string& s)
  {
    CPose3D o;
    o.fromString(s);
    return o;
  }

  /** Return true if the 6D pose represents a Z axis almost exactly vertical
   * (upwards or downwards), with a given tolerance (if set to 0 exact
   * horizontality is tested). */
  bool isHorizontal(const double tolerance = 0) const;

  /** The euclidean distance between two poses taken as two 6-length vectors
   * (angles in radians). */
  double distanceEuclidean6D(const CPose3D& o) const;

  /** @} */  // modif. components

  /** Used to emulate CPosePDF types, for example, in
   * mrpt::graphs::CNetworkOfPoses */
  using type_value = CPose3D;
  enum
  {
    is_3D_val = 1
  };
  static constexpr bool is_3D() { return is_3D_val != 0; }
  enum
  {
    rotation_dimensions = 3
  };
  enum
  {
    is_PDF_val = 0
  };
  static constexpr bool is_PDF() { return is_PDF_val != 0; }
  const type_value& getPoseMean() const { return *this; }
  type_value& getPoseMean() { return *this; }
  /** @name STL-like methods and typedefs
   @{   */
  /** The type of the elements */
  using value_type = double;
  using reference = double&;
  using const_reference = double;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  // size is constant
  static constexpr std::size_t static_size = 6;
  static constexpr size_type size() { return static_size; }
  static constexpr bool empty() { return false; }
  static constexpr size_type max_size() { return static_size; }
  static void resize(size_t n)
  {
    if (n != static_size)
    {
      throw std::logic_error(
          format("Try to change the size of CPose3D to %u.", static_cast<unsigned>(n)));
    }
  }
  /** @} */

};  // End of class def.

std::ostream& operator<<(std::ostream& o, const CPose3D& p);

/** Unary ⊖ operator: returns the SE(3) group inverse of `p`, i.e. T⁻¹ = [Rᵀ | -Rᵀ·t].
 *  \note **Not** the same as negating (x, y, z, yaw, pitch, roll) individually. */
CPose3D operator-(const CPose3D& p);

bool operator==(const CPose3D& p1, const CPose3D& p2);
bool operator!=(const CPose3D& p1, const CPose3D& p2);

}  // namespace mrpt::poses
