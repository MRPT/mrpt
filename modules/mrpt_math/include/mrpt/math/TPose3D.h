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

#include <mrpt/core/bits_math.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <mrpt/math/wrap2pi.h>

namespace mrpt::math
{
/** Lightweight 3D rigid-body pose — an element of SE(3).
 *
 * Parameterises the group SE(3) = SO(3) ⋉ R³ as the 6-vector `(x, y, z, yaw, pitch, roll)`:
 * - `(x, y, z)` — translation in metres.
 * - `(yaw, pitch, roll)` — orientation in **radians**, using the **intrinsic ZYX** convention
 *   (also known as Tait-Bryan angles):
 *   1. Rotate about the body Z axis by **yaw** (ψ).
 *   2. Rotate about the *new* body Y axis by **pitch** (θ).
 *   3. Rotate about the *new* body X axis by **roll** (φ).
 *
 *   Equivalently (same result, reversed order) as extrinsic rotations about the **fixed** axes
 *   X→Y→Z. The resulting SO(3) rotation matrix is R = Rz(yaw)·Ry(pitch)·Rx(roll).
 *
 * The equivalent 4x4 homogeneous matrix is:
 * \verbatim
 *   T = | R  t |   where R in SO(3), t = (x,y,z)^T
 *       | 0  1 |
 * \endverbatim
 *
 * **Composition (`operator+`)** implements the SE(3) group product:
 * `(this ⊕ b)` maps a point expressed in frame `b` into the frame of `this`.
 *
 * **Inverse (`operator-` unary)** computes the SE(3) group inverse T⁻¹.
 *
 * **Relative pose (`operator-` binary)**: `b - a = a⁻¹ ⊕ b`, the pose of `b` as seen
 * from `a`.
 *
 * This is a *lightweight* type without serialization or cached rotation matrix.
 * Prefer mrpt::poses::CPose3D when the rotation matrix is needed directly
 * or when composition is repeated with the same pose (avoids redundant trig).
 *
 * Coordinate access: `operator[]` with index 0→x, 1→y, 2→z, 3→yaw, 4→pitch, 5→roll.
 *
 * \sa mrpt::poses::CPose3D, TPose2D, TPoint3D
 * \ingroup geometry_grp
 */
struct TPose3D : public TPoseOrPoint, public internal::ProvideStaticResize<TPose3D>
{
  static constexpr std::size_t static_size = 6;

  /** Translation along X (metres). */
  double x{.0};
  /** Translation along Y (metres). */
  double y{.0};
  /** Translation along Z (metres). */
  double z{.0};
  /** Yaw angle psi in **radians**: rotation about the body (intrinsic) Z axis — first rotation.
   *  Range: any real value; conventionally wrapped to (-pi, pi]. */
  double yaw{.0};
  /** Pitch angle theta in **radians**: rotation about the (new) body Y axis — second rotation.
   *  Range: (-pi/2, pi/2) for a non-singular representation (gimbal-lock at +-pi/2). */
  double pitch{.0};
  /** Roll angle phi in **radians**: rotation about the (new) body X axis — third rotation.
   *  Range: any real value; conventionally wrapped to (-pi, pi]. */
  double roll{.0};

  /** Returns the identity transformation, T=eye(4) */
  static constexpr TPose3D Identity() { return TPose3D(); }

  /** Constructor from TPoint2D: copies (x,y), sets z=0, yaw=pitch=roll=0.
   * \sa TPoint2D
   */
  TPose3D(const TPoint2D& p);

  /** Constructor from TPose2D: copies (x,y), sets z=0, maps phi→yaw, pitch=roll=0.
   * \sa TPose2D
   */
  TPose3D(const TPose2D& p);

  /** Constructor from TPoint3D: copies (x,y,z) as translation; yaw=pitch=roll=0 (no rotation).
   * \sa TPoint3D
   */
  explicit TPose3D(const TPoint3D& p);

  /** Constructor from coordinates */
  constexpr TPose3D(double _x, double _y, double _z, double _yaw, double _pitch, double _roll) :
      x(_x), y(_y), z(_z), yaw(_yaw), pitch(_pitch), roll(_roll)
  {
  }
  /** Default fast constructor. Initializes to zeros. */
  constexpr TPose3D() = default;

  /** See fromString() for a description of the expected string format. */
  static TPose3D FromString(const std::string& s)
  {
    TPose3D o;
    o.fromString(s);
    return o;
  }

  /** Builds from the first 6 elements of a vector-like object: [x y z yaw
   * pitch roll]
   *
   * \tparam Vector It can be std::vector<double>, Eigen::VectorXd, etc.
   */
  template <typename Vector>
  static TPose3D FromVector(const Vector& v)
  {
    TPose3D o;
    for (int i = 0; i < 6; i++)
    {
      o[i] = v[i];
    }
    return o;
  }

  /** Coordinate access using operator[]. Order: x,y,z,yaw,pitch,roll */
  double& operator[](size_t i)
  {
    switch (i)
    {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      case 3:
        return yaw;
      case 4:
        return pitch;
      case 5:
        return roll;
      default:
        throw std::out_of_range("index out of range");
    }
  }
  /** Coordinate access using operator[]. Order: x,y,z,yaw,pitch,roll */
  constexpr double operator[](size_t i) const
  {
    switch (i)
    {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      case 3:
        return yaw;
      case 4:
        return pitch;
      case 5:
        return roll;
      default:
        throw std::out_of_range("index out of range");
    }
  }

  /** Returns the (x,y,z) translational part of the SE(3) transformation. */
  [[nodiscard]] mrpt::math::TPoint3D translation() const { return {x, y, z}; }

  /** Euclidean norm of the translational part: |(x,y,z)|. Angular coordinates are ignored. */
  [[nodiscard]] double norm() const { return std::sqrt(square(x) + square(y) + square(z)); }

  /** Gets the pose as a vector of doubles: [x y z yaw pitch roll]
   * \tparam Vector It can be std::vector<double>, Eigen::VectorXd, etc.
   */
  template <typename Vector>
  void asVector(Vector& v) const
  {
    v.resize(6);
    v[0] = x;
    v[1] = y;
    v[2] = z;
    v[3] = yaw;
    v[4] = pitch;
    v[5] = roll;
  }
  /// \overload
  template <typename Vector>
  Vector asVector() const
  {
    Vector v;
    asVector(v);
    return v;
  }

  /** Returns a human-readable textual representation of the object (eg: "[x y
   * z yaw pitch roll]", angles in degrees.)
   * \sa fromString
   */
  void asString(std::string& s) const;
  std::string asString() const
  {
    std::string s;
    asString(s);
    return s;
  }

  /** Converts the SO(3) rotation of this pose to a unit quaternion q = (qr, qx, qy, qz).
   *
   * The translation (x,y,z) is **ignored** — only the yaw/pitch/roll angles are converted.
   *
   * The mapping from intrinsic ZYX Euler angles (yaw=psi, pitch=theta, roll=phi) to the
   * Hamilton unit quaternion is:
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
   * The quaternion is stored in the order **(qr, qx, qy, qz)** (scalar-first / Hamilton
   * convention).
   *
   * \param[out] q The resulting unit quaternion.
   * \param[out] out_dq_dr If provided, the 4x3 Jacobian dq/d(yaw,pitch,roll)
   *   is computed and stored here.
   */
  void getAsQuaternion(
      mrpt::math::CQuaternion<double>& q,
      mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 4, 3>> out_dq_dr = std::nullopt) const;

  /** Transforms a point from the local frame of this pose into the global (world) frame.
   *  Equivalent to `g = R·l + t`, where R and t are the rotation matrix and translation of this
   * pose. \param l Point in the **local** frame. \param[out] g Point in the **global** frame. \sa
   * inverseComposePoint, composePoint(const TPoint3D&)
   */
  void composePoint(const TPoint3D& l, TPoint3D& g) const;
  /** \overload Returns transformed point directly. */
  [[nodiscard]] TPoint3D composePoint(const TPoint3D& l) const;

  /** Transforms a point from the global (world) frame into the local frame of this pose.
   *  Equivalent to `l = Rᵀ·(g − t)`.
   * \param g Point in the **global** frame.
   * \param[out] l Point in the **local** frame of this pose.
   * \sa composePoint
   */
  void inverseComposePoint(const TPoint3D& g, TPoint3D& l) const;
  /** \overload Returns transformed point directly. */
  [[nodiscard]] TPoint3D inverseComposePoint(const TPoint3D& g) const;

  /** SE(3) group composition: `result = this ⊕ other`.
   *  In matrix form: `T_result = T_this · T_other`.
   */
  void composePose(const TPose3D& other, TPose3D& result) const;

  /** SE(3) group composition — "⊕" operator: `ret = this ⊕ b`.
   *
   * Computes the pose obtained by applying `this` then `b`.
   * In matrix form: `T_ret = T_this · T_b`.
   * If `this` is the pose of frame A in the world W, and `b` is the pose of frame B
   * in frame A, then `ret` is the pose of B in W.
   * \sa CPose3D
   */
  mrpt::math::TPose3D operator+(const mrpt::math::TPose3D& b) const
  {
    mrpt::math::TPose3D ret;
    this->composePose(b, ret);
    return ret;
  }

  void getRotationMatrix(mrpt::math::CMatrixDouble33& R) const;
  [[nodiscard]] mrpt::math::CMatrixDouble33 getRotationMatrix() const
  {
    mrpt::math::CMatrixDouble33 R;
    getRotationMatrix(R);
    return R;
  }
  void getHomogeneousMatrix(mrpt::math::CMatrixDouble44& HG) const;
  [[nodiscard]] mrpt::math::CMatrixDouble44 getHomogeneousMatrix() const
  {
    mrpt::math::CMatrixDouble44 H;
    getHomogeneousMatrix(H);
    return H;
  }
  void getInverseHomogeneousMatrix(mrpt::math::CMatrixDouble44& HG) const;
  [[nodiscard]] mrpt::math::CMatrixDouble44 getInverseHomogeneousMatrix() const
  {
    mrpt::math::CMatrixDouble44 H;
    getInverseHomogeneousMatrix(H);
    return H;
  }
  void fromHomogeneousMatrix(const mrpt::math::CMatrixDouble44& HG);
  static void SO3_to_yaw_pitch_roll(
      const mrpt::math::CMatrixDouble33& R, double& yaw, double& pitch, double& roll);
  /** Set the current object value from a string generated by 'asString' (eg:
   * "[x y z yaw pitch roll]" with the three angles given in degrees. )
   * \sa asString
   * \exception std::exception On invalid format
   */
  void fromString(const std::string& s);
};

/** Unary ⊖ operator: returns the SE(3) group inverse T^{-1}.
 *  Note: this is **not** the elementwise negation of (x,y,z,yaw,pitch,roll). */
TPose3D operator-(const TPose3D& p);

/** Binary ⊖ operator: `b ⊖ a = a^{-1} ⊕ b` — the pose of `b` expressed in the frame of `a`.
 *  In matrix form: T_a^{-1} * T_b. */
TPose3D operator-(const TPose3D& b, const TPose3D& a);

/** Exact comparison between 3D poses, taking possible cycles into account */
inline bool operator==(const TPose3D& p1, const TPose3D& p2)
{
  return (p1.x == p2.x) && (p1.y == p2.y) && (p1.z == p2.z) &&
         (mrpt::math::wrapTo2Pi(p1.yaw) == mrpt::math::wrapTo2Pi(p2.yaw)) &&
         (mrpt::math::wrapTo2Pi(p1.pitch) == mrpt::math::wrapTo2Pi(p2.pitch)) &&
         (mrpt::math::wrapTo2Pi(p1.roll) == mrpt::math::wrapTo2Pi(p2.roll));  //-V550
}
/** Exact comparison between 3D poses, taking possible cycles into account */
inline bool operator!=(const TPose3D& p1, const TPose3D& p2)
{
  return (p1.x != p2.x) || (p1.y != p2.y) || (p1.z != p2.z) ||
         (mrpt::math::wrapTo2Pi(p1.yaw) != mrpt::math::wrapTo2Pi(p2.yaw)) ||
         (mrpt::math::wrapTo2Pi(p1.pitch) != mrpt::math::wrapTo2Pi(p2.pitch)) ||
         (mrpt::math::wrapTo2Pi(p1.roll) != mrpt::math::wrapTo2Pi(p2.roll));  //-V550
}

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPose3D, mrpt::math)
}  // namespace mrpt::typemeta
