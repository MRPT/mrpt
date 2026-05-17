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
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/poses/CPose.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::poses
{
class CPose3D;
/** SE(2) rigid-body pose with cached trigonometry — the preferred class for repeated composition.
 *
 * Represents an element of SE(2) = SO(2) ⋉ R² as the triplet `(x, y, φ)`:
 * - `(x, y)` — translation in metres.
 * - `φ` (phi) — counter-clockwise heading angle in **radians**.
 *
 * The equivalent 3x3 homogeneous matrix is:
 * \verbatim
 *   T = | cos(phi)  -sin(phi)  x |
 *       | sin(phi)   cos(phi)  y |
 *       |    0          0      1 |
 * \endverbatim
 *
 * **Use this class instead of mrpt::math::TPose2D** when pose–point composition (`composePoint`,
 * `operator+`) is called repeatedly with the same pose. CPose2D caches `cos(φ)` and `sin(φ)`,
 * recomputing them only when `φ` changes, making repeated transforms significantly cheaper.
 *
 * **Operator semantics (SE(2) group):**
 * - `a + b` (⊕): pose composition — maps the frame of `b` into the frame of `a`.
 *   In matrix form: `T_a · T_b`.
 * - `a - b` (⊖): relative pose — the pose of `a` expressed in the frame of `b`.
 *   In matrix form: `T_b⁻¹ · T_a`. Equivalent to `b.inverse() + a`.
 * - `operator-` (unary): SE(2) group inverse. **Not** elementwise negation.
 * - `pose + point`: applies the SE(2) transformation to a point (composePoint).
 *
 * **Serializable:** supports `mrpt::serialization::CArchive` and YAML schema serialization.
 *
 *  <div align=center>
 *   <img src="CPose2D.gif">
 *  </div>
 *
 * \note See also: "A tutorial on SE(3) transformation parameterizations and
 *   on-manifold optimization", J.L. Blanco. \cite blanco_se3_tutorial
 * \sa mrpt::math::TPose2D, CPoseOrPoint, CPoint2D, CPose3D
 * \ingroup poses_grp
 */
class CPose2D :
    public CPose<CPose2D, 3>,
    public mrpt::serialization::CSerializable,
    public mrpt::Stringifyable
{
 public:
  DEFINE_SERIALIZABLE(CPose2D, mrpt::poses)
  DEFINE_SCHEMA_SERIALIZABLE()

 public:
  /** [x,y] */
  mrpt::math::CVectorFixedDouble<2> m_coords;

 protected:
  /** The orientation of the pose, in radians. */
  double m_phi{.0};
  /** Precomputed cos() & sin() of phi. */
  mutable double m_cosphi{1.0}, m_sinphi{.0};
  mutable bool m_cossin_uptodate{false};
  void update_cached_cos_sin() const;

 public:
  /** Default constructor (all coordinates to 0) */
  CPose2D();

  /** Constructor from an initial value of the pose.*/
  CPose2D(const double x, const double y, const double phi);

  /** Returns the identity transformation */
  static CPose2D Identity() { return CPose2D(); }

  /** Constructor from a CPoint2D object. */
  explicit CPose2D(const CPoint2D&);

  /** Projects a CPose3D into SE(2) by copying (x,y) and mapping yaw→phi; z, pitch and roll are
   *  discarded. **Information is irreversibly lost** when pitch ≠ 0 or roll ≠ 0. */
  explicit CPose2D(const CPose3D&);

  /** Constructor from lightweight object. */
  explicit CPose2D(const mrpt::math::TPose2D&);

  mrpt::math::TPose2D asTPose() const;

  /** Constructor from CPoint3D with information loss. */
  explicit CPose2D(const CPoint3D&);

  /** Fast constructor that leaves all the data uninitialized - call with
   * UNINITIALIZED_POSE as argument */
  CPose2D(TConstructorFlags_Poses) : m_cossin_uptodate(false) {}
  /** Get the phi angle of the 2D pose (in radians) */
  double phi() const { return m_phi; }
  //! \overload
  double& phi()
  {
    m_cossin_uptodate = false;
    return m_phi;
  }  //-V659

  /** Returns cos(φ), using a cached value that is recomputed only when φ changes.
   *  This avoids repeated calls to std::cos() in tight composition loops. */
  double phi_cos() const
  {
    update_cached_cos_sin();
    return m_cosphi;
  }
  /** Returns sin(φ), using a cached value that is recomputed only when φ changes.
   *  This avoids repeated calls to std::sin() in tight composition loops. */
  double phi_sin() const
  {
    update_cached_cos_sin();
    return m_sinphi;
  }

  /** Set the phi angle of the 2D pose (in radians) */
  void phi(double angle)
  {
    m_phi = angle;
    m_cossin_uptodate = false;
  }

  /** Increment the PHI angle (without checking the 2 PI range, call
   * normalizePhi is needed) */
  void phi_incr(const double Aphi)
  {
    m_phi += Aphi;
    m_cossin_uptodate = false;
  }

  /** Returns a 1x3 vector with [x y phi] */
  void asVector(vector_t& v) const;

  /** Returns the (x,y) translational part of the SE(2) transformation. */
  mrpt::math::TPoint2D translation() const { return {m_coords[0], m_coords[1]}; }

  /** Returns the corresponding 4x4 homogeneous transformation matrix for the
   * point(translation) or pose (translation+orientation).
   * \sa getInverseHomogeneousMatrix
   * \deprecated Use getHomogeneousMatrix() returning by value instead.
   */
  [[deprecated("Use getHomogeneousMatrix() returning by value instead.")]] void
  getHomogeneousMatrix(mrpt::math::CMatrixDouble44& out_HM) const;

  /** Returns the corresponding 4x4 homogeneous transformation matrix. */
  [[nodiscard]] mrpt::math::CMatrixDouble44 getHomogeneousMatrix() const;

  /** Returns the SE(2) 2x2 rotation matrix
   * \deprecated Use getRotationMatrix() returning by value instead.
   */
  [[deprecated("Use getRotationMatrix() returning by value instead.")]] void getRotationMatrix(
      mrpt::math::CMatrixDouble22& R) const;
  /** Returns the equivalent SE(3) 3x3 rotation matrix, with (2,2)=1.
   * \deprecated Use getRotationMatrix() returning by value instead.
   */
  [[deprecated("Use getRotationMatrix() returning by value instead.")]] void getRotationMatrix(
      mrpt::math::CMatrixDouble33& R) const;

  /** Returns the SE(2) 2x2 rotation matrix by value. */
  [[nodiscard]] mrpt::math::CMatrixDouble22 getRotationMatrix() const;

  template <class MATRIX22>
  MATRIX22 getRotationMatrixAs() const
  {
    MATRIX22 R;
    getRotationMatrix(R);
    return R;
  }

  /** SE(2) group composition: `ret = this ⊕ D`.
   *
   *  If `this` is the pose of frame A in world W, and `D` is the pose of frame B in frame A,
   *  then `ret` is the pose of B in W. In matrix form: `T_ret = T_this · T_D`.
   */
  CPose2D operator+(const CPose2D& D) const;

  /** In-place SE(2) group composition: computes `this = A ⊕ B` and stores the result in `this`.
   *  Slightly more efficient than `this = A + B` as it avoids a temporary.
   *  \note A or B may safely alias `this`. */
  void composeFrom(const CPose2D& A, const CPose2D& B);

  /** Lifts this SE(2) pose into SE(3) (z=0, pitch=roll=0), then composes with `D` in SE(3).
   *  Returns the full SE(3) result. */
  CPose3D operator+(const CPose3D& D) const;

  /** Applies the SE(2) transformation to a CPoint2D (pose–point composition): `ret = this ⊕ u`. */
  CPoint2D operator+(const CPoint2D& u) const;

  /** Transforms a 2D point from the local frame of this pose into the global frame.
   *  Computes: (gx, gy) = R * (lx, ly)^T + t
   * \param lx,ly  Point coordinates in the **local** frame.
   * \param[out] gx,gy  Point coordinates in the **global** frame.
   * \sa inverseComposePoint
   */
  void composePoint(double lx, double ly, double& gx, double& gy) const;

  /** \overload */
  void composePoint(const mrpt::math::TPoint2D& l, mrpt::math::TPoint2D& g) const;

  /** \overload for 3D points; the z coordinate passes through unmodified. */
  mrpt::math::TPoint3D composePoint(const mrpt::math::TPoint3D& l) const;

  /** \overload for 3D points; the z coordinate passes through unmodified. */
  void composePoint(const mrpt::math::TPoint3D& l, mrpt::math::TPoint3D& g) const;
  /** \overload for 3D points; the z coordinate passes through unmodified. */
  void composePoint(double lx, double ly, double lz, double& gx, double& gy, double& gz) const;

  /** Transforms a 2D point from the global frame into the local frame of this pose.
   *  Computes: (lx, ly) = R^T * ((gx, gy)^T - t)
   * \param gx,gy  Point coordinates in the **global** frame.
   * \param[out] lx,ly  Point coordinates in the **local** frame.
   * \sa composePoint
   */
  void inverseComposePoint(const double gx, const double gy, double& lx, double& ly) const;
  /** \overload */
  void inverseComposePoint(const mrpt::math::TPoint2D& g, mrpt::math::TPoint2D& l) const;
  /** \overload Returns the local-frame point directly. */
  mrpt::math::TPoint2D inverseComposePoint(const mrpt::math::TPoint2D& g) const;

  /** The operator `u' = this (+) u` is the pose/point compounding operator. */
  CPoint3D operator+(const CPoint3D& u) const;

  /** In-place relative pose: computes `this = A ⊖ B = B⁻¹ ⊕ A` and stores the result in `this`.
   *  Slightly more efficient than `this = A - B` as it avoids a temporary.
   *  \note A or B may safely alias `this`.
   * \sa composeFrom, composePoint
   */
  void inverseComposeFrom(const CPose2D& A, const CPose2D& B);

  /** Replaces this pose with its SE(2) group inverse in-place.
   *  After calling this, `original * this = Identity`.
   *  \note **Not** the same as negating all components individually.
   * \sa operator-
   */
  void inverse();

  /** SE(2) relative pose: `ret = this ⊖ b = b⁻¹ ⊕ this`.
   *  Returns the pose of `this` expressed in the frame of `b`. */
  CPose2D operator-(const CPose2D& b) const
  {
    CPose2D ret(UNINITIALIZED_POSE);
    ret.inverseComposeFrom(*this, b);
    return ret;
  }

  /** The operator `a (-) b` is the pose inverse compounding operator. */
  CPose3D operator-(const CPose3D& b) const;

  /** Scalar sum of components: This is different from poses
   *    composition, which is implemented as "+" operators in "CPose" derived
   * classes.
   */
  void AddComponents(const CPose2D& p);

  /** Scalar multiplication.
   */
  void operator*=(const double s);

  /** Make `this = this (+) b` */
  CPose2D& operator+=(const CPose2D& b);

  /** Forces "phi" to be in the range [-pi,pi];
   */
  void normalizePhi();

  /** Return the opposite of the current pose instance by taking the negative
   * of all its components \a individually
   */
  CPose2D getOppositeScalar() const;

  /** Returns a human-readable textual representation of the object (eg: "[x y
   * yaw]", yaw in degrees)
   * \sa fromString
   */
  std::string asString() const override;

  /** Set the current object value from a string generated by 'asString' (eg:
   * "[0.02 1.04 -0.8]" )
   * \sa asString
   * \exception std::exception On invalid format
   */
  void fromString(const std::string& s);
  /** Same as fromString, but without requiring the square brackets in the
   * string */
  void fromStringRaw(const std::string& s);

  static CPose2D FromString(const std::string& s)
  {
    CPose2D o;
    o.fromString(s);
    return o;
  }

  double operator[](unsigned int i) const
  {
    switch (i)
    {
      case 0:
        return m_coords[0];
      case 1:
        return m_coords[1];
      case 2:
        return m_phi;
      default:
        throw std::runtime_error("CPose2D::operator[]: Index of bounds.");
    }
  }
  double& operator[](unsigned int i)
  {
    switch (i)
    {
      case 0:
        return m_coords[0];
      case 1:
        return m_coords[1];
      case 2:
        return m_phi;
      default:
        throw std::runtime_error("CPose2D::operator[]: Index of bounds.");
    }
  }

  /** makes: this = p (+) this */
  void changeCoordinatesReference(const CPose2D& p) { composeFrom(p, CPose2D(*this)); }

  /** Returns the 2D distance from this pose/point to a 2D pose using the
   * Frobenius distance. */
  double distance2DFrobeniusTo(const CPose2D& p) const;

  /** Used to emulate CPosePDF types, for example, in
   * mrpt::graphs::CNetworkOfPoses */
  using type_value = CPose2D;
  enum
  {
    is_3D_val = 0
  };
  static constexpr bool is_3D() { return is_3D_val != 0; }
  enum
  {
    rotation_dimensions = 2
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
  static constexpr std::size_t static_size = 3;
  static constexpr size_type size() { return static_size; }
  static constexpr bool empty() { return false; }
  static constexpr size_type max_size() { return static_size; }
  static void resize(size_t n)
  {
    if (n != static_size)
    {
      throw std::logic_error(
          format("Try to change the size of CPose2D to %u.", static_cast<unsigned>(n)));
    }
  }

  /** @} */

};  // End of class def.

std::ostream& operator<<(std::ostream& o, const CPose2D& p);

/** Unary ⊖ operator: returns the SE(2) group inverse of `p`.
 *  \note **Not** the same as negating (x, y, phi) individually. \sa CPose2D::inverse() */
CPose2D operator-(const CPose2D& p);

/** Compose a 2D point from a new coordinate base given by a 2D pose. */
mrpt::math::TPoint2D operator+(const CPose2D& pose, const mrpt::math::TPoint2D& pnt);

bool operator==(const CPose2D& p1, const CPose2D& p2);
bool operator!=(const CPose2D& p1, const CPose2D& p2);

}  // namespace mrpt::poses
