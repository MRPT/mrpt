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

#include <mrpt/core/bits_math.h>  // hypot_fast()
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <mrpt/math/wrap2pi.h>

#include <vector>

namespace mrpt::math
{
/** Lightweight 2D rigid-body pose — an element of SE(2).
 *
 * Parameterises the group SE(2) = SO(2) ⋉ R² as the triplet `(x, y, φ)`:
 * - `(x, y)` — translation in metres.
 * - `φ` (phi) — counter-clockwise rotation angle in **radians**, measured from
 *   the global X axis.
 *
 * The equivalent 3x3 homogeneous matrix is:
 * \verbatim
 *   T = | cos(phi)  -sin(phi)  x |
 *       | sin(phi)   cos(phi)  y |
 *       |    0          0      1 |
 * \endverbatim
 *
 * **Composition (`operator+`)** implements the SE(2) group product:
 * `(this ⊕ b)` maps a point expressed in frame `b` into the frame of `this`.
 *
 * **Inverse composition (`operator-`)** implements `(this ⊖ b) = b⁻¹ ⊕ this`,
 * i.e. the pose of `this` expressed in the frame of `b`.
 *
 * This is a *lightweight* type without serialization or caching of trig values.
 * Prefer mrpt::poses::CPose2D when composition is called repeatedly with the same
 * pose (it caches cos/sin and supports serialization).
 *
 * Coordinate access: `operator[]` with index 0→x, 1→y, 2→phi.
 *
 * \sa mrpt::poses::CPose2D, TPose3D, TPoint2D
 * \ingroup geometry_grp
 */
struct TPose2D : public TPoseOrPoint, public internal::ProvideStaticResize<TPose2D>
{
  static constexpr std::size_t static_size = 3;

  /** Translation along the global X axis (metres). */
  double x{.0};
  /** Translation along the global Y axis (metres). */
  double y{.0};
  /** Heading angle phi in **radians**, counter-clockwise from the global X axis.
   *  Range: any real value; use normalizePhi() to wrap to (-pi, pi]. */
  double phi{.0};

  /** Returns the identity transformation */
  static constexpr TPose2D Identity() { return TPose2D(); }

  /** Constructor from TPoint2D: copies (x,y), sets phi=0 (pure translation, no rotation). */
  explicit TPose2D(const TPoint2D& p);

  /** Constructor from TPoint3D: copies (x,y), sets phi=0; the z coordinate is discarded. */
  explicit TPose2D(const TPoint3D& p);

  /** Constructor from TPose3D: copies (x,y), maps yaw→phi; z, pitch and roll are discarded.
   *  This is a projection from SE(3) onto SE(2); information is irreversibly lost
   *  when pitch ≠ 0 or roll ≠ 0.
   * \sa TPose3D
   */
  explicit TPose2D(const TPose3D& p);
  /**
   * Constructor from coordinates.
   */
  constexpr TPose2D(double xx, double yy, double Phi) : x(xx), y(yy), phi(Phi) {}
  /**
   * Default fast constructor. Initializes to zeros.
   */
  constexpr TPose2D() = default;

  /** Builds from the first 3 elements of a vector-like object: [x y phi]
   *
   * \tparam Vector It can be std::vector<double>, Eigen::VectorXd, etc.
   */
  template <typename Vector>
  static TPose2D FromVector(const Vector& v)
  {
    TPose2D o;
    for (int i = 0; i < 3; i++) o[i] = v[i];
    return o;
  }
  /** Coordinate access using operator[]. Order: x,y,phi */
  double& operator[](size_t i)
  {
    switch (i)
    {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return phi;
      default:
        throw std::out_of_range("index out of range");
    }
  }
  /** Coordinate access using operator[]. Order: x,y,phi */
  constexpr double operator[](size_t i) const
  {
    switch (i)
    {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return phi;
      default:
        throw std::out_of_range("index out of range");
    }
  }
  /** Gets the pose as a vector of doubles.
   * \tparam Vector It can be std::vector<double>, Eigen::VectorXd, etc.
   */
  template <typename Vector>
  void asVector(Vector& v) const
  {
    v.resize(3);
    v[0] = x;
    v[1] = y;
    v[2] = phi;
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
   * yaw]", yaw in degrees)
   * \sa fromString
   */
  void asString(std::string& s) const;
  std::string asString() const
  {
    std::string s;
    asString(s);
    return s;
  }

  /** SE(2) group composition — "⊕" operator: `ret = this ⊕ b`.
   *
   * Computes the pose obtained by first applying transformation `this`, then `b`.
   * In matrix form: `T_ret = T_this · T_b`.
   * If `this` is the pose of frame A in the world W, and `b` is the pose of frame B
   * in frame A, then `ret` is the pose of B in W.
   * \sa CPose2D, inverseComposeFrom
   */
  mrpt::math::TPose2D operator+(const mrpt::math::TPose2D& b) const;

  /** SE(2) inverse composition — "⊖" operator: `ret = this ⊖ b = b⁻¹ ⊕ this`.
   *
   * Returns the pose of `this` expressed in the reference frame of `b`.
   * In matrix form: `T_ret = T_b⁻¹ · T_this`.
   * Useful for computing relative poses: if both `this` and `b` are poses in the
   * world frame, `ret` is the pose of `this` as seen from `b`.
   * \sa CPose2D
   */
  mrpt::math::TPose2D operator-(const mrpt::math::TPose2D& b) const;

  /** Transforms a point from the local frame of this pose into the global (world) frame.
   *  Equivalent to `g = T_this · [l.x  l.y  1]ᵀ` (homogeneous coords).
   * \param l Point in the **local** frame.
   * \return Point in the **global** frame.
   * \sa inverseComposePoint
   */
  mrpt::math::TPoint2D composePoint(const TPoint2D l) const;

  /** Alias for composePoint(): `pose + point` applies the SE(2) transformation to the point. */
  mrpt::math::TPoint2D operator+(const mrpt::math::TPoint2D& b) const;

  /** Transforms a point from the global (world) frame into the local frame of this pose.
   *  Equivalent to `l = T_this⁻¹ · [g.x  g.y  1]ᵀ` (homogeneous coords).
   * \param g Point in the **global** frame.
   * \return Point in the **local** frame of this pose.
   * \sa composePoint
   */
  mrpt::math::TPoint2D inverseComposePoint(const TPoint2D g) const;

  /** Returns the (x,y) translational part of the SE(2) transformation. */
  [[nodiscard]] const mrpt::math::TPoint2D translation() const { return {x, y}; }

  /** Euclidean norm of the translational part: |(x,y)|. The angle phi is ignored. */
  double norm() const { return mrpt::hypot_fast(x, y); }
  /** Wraps phi to the canonical range (-pi, pi]. */
  void normalizePhi() { phi = mrpt::math::wrapToPi(phi); }
  /** Set the current object value from a string generated by 'asString' (eg:
   * "[0.02 1.04 -45.0]" )
   * \sa asString
   * \exception std::exception On invalid format
   */
  void fromString(const std::string& s);
  static TPose2D FromString(const std::string& s)
  {
    TPose2D o;
    o.fromString(s);
    return o;
  }
};

/** Exact comparison between 2D poses, taking possible cycles into account */
inline bool operator==(const TPose2D& p1, const TPose2D& p2)
{
  return (p1.x == p2.x) && (p1.y == p2.y) &&
         (mrpt::math::wrapTo2Pi(p1.phi) == mrpt::math::wrapTo2Pi(p2.phi));  //-V550
}
/** Exact comparison between 2D poses, taking possible cycles into account */
inline bool operator!=(const TPose2D& p1, const TPose2D& p2)
{
  return (p1.x != p2.x) || (p1.y != p2.y) ||
         (mrpt::math::wrapTo2Pi(p1.phi) != mrpt::math::wrapTo2Pi(p2.phi));  //-V550
}

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPose2D, mrpt::math)
}  // namespace mrpt::typemeta
