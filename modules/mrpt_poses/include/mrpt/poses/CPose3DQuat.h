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
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose.h>
#include <mrpt/poses/poses_frwds.h>

namespace mrpt::poses
{
/** A class used to store a 3D pose as a translation (x,y,z) and a quaternion
 * (qr,qx,qy,qz).
 *
 * For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint,
 * or refer to the <a href="http://www.mrpt.org/2D_3D_Geometry"> 2D/3D Geometry
 * tutorial</a> in the wiki.
 *
 * To access the translation use x(), y() and z(). To access the rotation, use
 * CPose3DQuat::quat().
 *
 * This class also behaves like a STL container, since it has begin(), end(),
 * iterators, and can be accessed with the [] operator with indices running from
 * 0 to 6 to access the [x y z qr qx qy qz] as if they were a vector. Thus, a
 * CPose3DQuat can be used as a 7-vector anywhere the MRPT math functions expect
 * any kind of vector.
 *
 * This class and CPose3D are very similar, and they can be converted to each
 * other automatically via transformation constructors.
 *
 * \sa CPose3D (for a class based on a 4x4 matrix instead of a quaternion),
 * mrpt::math::TPose3DQuat, mrpt::poses::CPose3DQuatPDF for a probabilistic
 * version of this class, mrpt::math::CQuaternion, CPoseOrPoint
 * \ingroup poses_grp
 */
class CPose3DQuat :
    public CPose<CPose3DQuat, 7>,
    public mrpt::serialization::CSerializable,
    public mrpt::Stringifyable
{
  DEFINE_SERIALIZABLE(CPose3DQuat, mrpt::poses)
  DEFINE_SCHEMA_SERIALIZABLE()

 public:
  /** The translation vector [x,y,z] */
  mrpt::math::CVectorFixedDouble<3> m_coords;

  /** The quaternion [0]:w, [1-3]:x,y,z */
  mrpt::math::CQuaternionDouble m_quat;

 public:
  /** Read/Write access to the quaternion representing the 3D rotation. */
  mrpt::math::CQuaternionDouble& quat() { return m_quat; }

  /** Read-only access to the quaternion representing the 3D rotation. */
  const mrpt::math::CQuaternionDouble& quat() const { return m_quat; }

  /** Read/Write access to the translation vector in R^3. */
  mrpt::math::CVectorFixedDouble<3>& xyz() { return m_coords; }

  /** Read-only access to the translation vector in R^3. */
  const mrpt::math::CVectorFixedDouble<3>& xyz() const { return m_coords; }

  /** Default constructor, initialize translation to zeros and quaternion to
   * no rotation. */
  CPose3DQuat() : m_quat() { m_coords[0] = m_coords[1] = m_coords[2] = 0.; }

  /** Constructor which leaves all the quaternion members uninitialized, for
   * use when speed is critical; Use UNINITIALIZED_POSE as argument to this
   * constructor. */
  CPose3DQuat(mrpt::math::TConstructorFlags_Quaternions) :
      m_quat(mrpt::math::UNINITIALIZED_QUATERNION)
  {
  }

  /** \overload */
  CPose3DQuat(TConstructorFlags_Poses) : m_quat(mrpt::math::UNINITIALIZED_QUATERNION) {}

  /** Constructor with initialization of the pose - the quaternion is
   * normalized to ensure it's unitary. */
  CPose3DQuat(
      const double x,  // NOLINT
      const double y,
      const double z,
      const mrpt::math::CQuaternionDouble& q) :
      m_quat(q)
  {
    m_coords[0] = x;
    m_coords[1] = y;
    m_coords[2] = z;
    m_quat.normalize();
  }

  /** Constructor from a CPose3D */
  explicit CPose3DQuat(const CPose3D& p);

  /** Constructor from lightweight object. */
  CPose3DQuat(const mrpt::math::TPose3DQuat& p) : m_quat(p.qr, p.qx, p.qy, p.qz)
  {
    x() = p.x;
    y() = p.y;
    z() = p.z;
  }

  mrpt::math::TPose3DQuat asTPose() const;

  /** Constructor from a 4x4 homogeneous transformation matrix. */
  explicit CPose3DQuat(const mrpt::math::CMatrixDouble44& M);

  /** Returns the corresponding 4x4 homogeneous transformation matrix for the
   * point (translation) or pose (translation+orientation).
   * \sa getInverseHomogeneousMatrix
   */
  void getHomogeneousMatrix(mrpt::math::CMatrixDouble44& out_HM) const;

  /** Returns a 7x1 vector with [x y z qr qx qy qz]' */
  void asVector(vector_t& v) const;

  /** Makes \f$ this = A \oplus B \f$ this method is slightly more efficient
   * than "this = A + B;" since it avoids the temporary object.
   * \note A or B can be "this" without problems.
   * \sa inverseComposeFrom, composePoint
   */
  void composeFrom(const CPose3DQuat& A, const CPose3DQuat& B);

  /** Makes \f$ this = A \ominus B \f$ this method is slightly more efficient
   * than "this = A - B;" since it avoids the temporary object.
   * \note A or B can be "this" without problems.
   * \sa composeFrom, composePoint
   */
  void inverseComposeFrom(const CPose3DQuat& A, const CPose3DQuat& B);

  /** Computes the 3D point G such as \f$ G = this \oplus L \f$.
   * \sa composeFrom, inverseComposePoint
   */
  void composePoint(
      const double lx,
      const double ly,
      const double lz,
      double& gx,
      double& gy,
      double& gz,
      mrpt::math::CMatrixFixed<double, 3, 3>* out_jacobian_df_dpoint = nullptr,
      mrpt::math::CMatrixFixed<double, 3, 7>* out_jacobian_df_dpose = nullptr) const;

  /** Computes the 3D point L such as \f$ L = G \ominus this \f$.
   * \sa composePoint, composeFrom
   */
  void inverseComposePoint(
      const double gx,
      const double gy,
      const double gz,
      double& lx,
      double& ly,
      double& lz,
      mrpt::math::CMatrixFixed<double, 3, 3>* out_jacobian_df_dpoint = nullptr,
      mrpt::math::CMatrixFixed<double, 3, 7>* out_jacobian_df_dpose = nullptr) const;

  /** Computes the 3D point G such as \f$ G = this \oplus L \f$.
   * POINT1 and POINT2 can be anything supporting [0], [1], [2].
   * \sa composePoint
   */
  template <class POINT1, class POINT2>
  void composePoint(const POINT1& L, POINT2& G) const
  {
    composePoint(L[0], L[1], L[2], G[0], G[1], G[2]);
  }

  /** Computes the 3D point L such as \f$ L = G \ominus this \f$.
   * \sa inverseComposePoint
   */
  template <class POINT1, class POINT2>
  void inverseComposePoint(const POINT1& G, POINT2& L) const
  {
    inverseComposePoint(G[0], G[1], G[2], L[0], L[1], L[2]);
  }

  /** Computes the 3D point G such as \f$ G = this \oplus L \f$.
   * \sa composePoint
   */
  CPoint3D operator+(const CPoint3D& L) const
  {
    CPoint3D G;
    composePoint(L[0], L[1], L[2], G[0], G[1], G[2]);
    return G;
  }

  /** Computes the 3D point G such as \f$ G = this \oplus L \f$.
   * \sa composePoint
   */
  mrpt::math::TPoint3D operator+(const mrpt::math::TPoint3D& L) const
  {
    mrpt::math::TPoint3D G;
    composePoint(L[0], L[1], L[2], G[0], G[1], G[2]);
    return G;
  }

  /** Scalar multiplication (all x y z qr qx qy qz elements are multiplied by
   * the scalar). */
  virtual void operator*=(const double s);

  /** Make \f$ this = this \oplus b \f$ */
  CPose3DQuat& operator+=(const CPose3DQuat& b)
  {
    composeFrom(*this, b);
    return *this;
  }

  /** Return the composed pose \f$ ret = this \oplus p \f$ */
  CPose3DQuat operator+(const CPose3DQuat& p) const
  {
    CPose3DQuat ret;
    ret.composeFrom(*this, p);
    return ret;
  }

  /** Make \f$ this = this \ominus b \f$ */
  CPose3DQuat& operator-=(const CPose3DQuat& b)
  {
    inverseComposeFrom(*this, b);
    return *this;
  }

  /** Return the composed pose \f$ ret = this \ominus p \f$ */
  CPose3DQuat operator-(const CPose3DQuat& p) const
  {
    CPose3DQuat ret;
    ret.inverseComposeFrom(*this, p);
    return ret;
  }

  /** Convert this pose into its inverse, saving the result in itself.
   * \sa operator-
   */
  void inverse();

  /** Returns a human-readable textual representation of the object as:
   * "[x y z qw qx qy qz]"
   * \sa fromString
   */
  std::string asString() const override
  {
    return mrpt::format(
        "[%f %f %f %f %f %f %f]", m_coords[0], m_coords[1], m_coords[2], m_quat[0], m_quat[1],
        m_quat[2], m_quat[3]);
  }

  /** Set the current object value from a string generated by 'asString'
   * (e.g.: "[0.02 1.04 -0.8 1 0 0 0]")
   * \sa asString
   * \exception std::exception On invalid format
   */
  void fromString(const std::string& s);

  /** Same as fromString, but without requiring the square brackets in the
   * string. */
  void fromStringRaw(const std::string& s);

  static CPose3DQuat FromString(const std::string& s)
  {
    CPose3DQuat o;
    o.fromString(s);
    return o;
  }

  /** Read-only [] operator */
  double operator[](const std::size_t i) const
  {
    switch (i)
    {
      case 0:
        return m_coords[0];
      case 1:
        return m_coords[1];
      case 2:
        return m_coords[2];
      case 3:
        return m_quat[0];  // w
      case 4:
        return m_quat[1];  // x
      case 5:
        return m_quat[2];  // y
      case 6:
        return m_quat[3];  // z
      default:
        throw std::runtime_error("CPose3DQuat::operator[]: Index out of bounds.");
    }
  }

  /** Read/write [] operator */
  double& operator[](const std::size_t i)
  {
    switch (i)
    {
      case 0:
        return m_coords[0];
      case 1:
        return m_coords[1];
      case 2:
        return m_coords[2];
      case 3:
        return m_quat[0];
      case 4:
        return m_quat[1];
      case 5:
        return m_quat[2];
      case 6:
        return m_quat[3];
      default:
        throw std::runtime_error("CPose3DQuat::operator[]: Index out of bounds.");
    }
  }

  /** Computes the spherical coordinates of a 3D point as seen from the 6D
   * pose specified by this object. For the coordinate system see the top of
   * this page. If the matrix pointers are not nullptr, the Jacobians will be
   * also computed for the range-yaw-pitch variables wrt the passed 3D point
   * and this 7D pose.
   */
  void sphericalCoordinates(
      const mrpt::math::TPoint3D& point,
      double& out_range,
      double& out_yaw,
      double& out_pitch,
      mrpt::math::CMatrixFixed<double, 3, 3>* out_jacob_dryp_dpoint = nullptr,
      mrpt::math::CMatrixFixed<double, 3, 7>* out_jacob_dryp_dpose = nullptr) const;

  /** Used to emulate CPosePDF types, for example, in
   * mrpt::graphs::CNetworkOfPoses */
  using type_value = CPose3DQuat;
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
    is_PDF_val = 1
  };
  static constexpr bool is_PDF() { return is_PDF_val != 0; }

  [[nodiscard]] const type_value& getPoseMean() const { return *this; }
  [[nodiscard]] type_value& getPoseMean() { return *this; }

  /** @name STL-like methods and typedefs
   * @{ */

  /** The type of the elements */
  using value_type = double;
  using reference = double&;
  using const_reference = double;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  // Size is constant
  static constexpr std::size_t static_size = 7;
  static constexpr size_type size() { return static_size; }
  static constexpr bool empty() { return false; }
  static constexpr size_type max_size() { return static_size; }
  static void resize(size_t n)
  {
    if (n != static_size)
    {
      throw std::logic_error(
          format("Try to change the size of CPose3DQuat to %u.", static_cast<unsigned>(n)));
    }
  }

  void assign(size_t N, const double val)
  {
    if (N != 7)
    {
      throw std::runtime_error("CPose3DQuat::assign: Try to resize to length!=7.");
    }
    m_coords.fill(val);
    m_quat.fill(val);
  }

  // Forward declarations for iterator types
  template <bool IsConst>
  class iterator_impl;

  using iterator = iterator_impl<false>;
  using const_iterator = iterator_impl<true>;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  /** Unified iterator implementation using template parameterization for const-correctness.
   * This eliminates code duplication between iterator and const_iterator. */
  template <bool IsConst>
  class iterator_impl
  {
   public:
    // Iterator traits
    using difference_type = std::ptrdiff_t;
    using value_type = double;
    using pointer = std::conditional_t<IsConst, const double*, double*>;
    using reference = std::conditional_t<IsConst, const double&, double&>;
    using iterator_category = std::random_access_iterator_tag;

   private:
    using PoseType = std::conditional_t<IsConst, const CPose3DQuat, CPose3DQuat>;

    /** A reference to the source of this iterator */
    PoseType* m_obj = nullptr;
    /** The iterator points to this element. */
    difference_type m_cur_idx = 0;

    void check_limits([[maybe_unused]] bool allow_end = false) const
    {
#ifdef _DEBUG
      ASSERTMSG_(m_obj != nullptr, "Non-initialized iterator");
      if (m_cur_idx > (allow_end ? 7 : 6)) THROW_EXCEPTION("Index out of range in iterator.");
#endif
    }

   public:
    // Constructors
    iterator_impl() = default;
    iterator_impl(PoseType& obj, difference_type start_idx) : m_obj(&obj), m_cur_idx(start_idx)
    {
      check_limits(true); /* Don't report as error an iterator to end() */
    }

    // Allow conversion from non-const to const iterator
    template <bool WasConst, typename = std::enable_if_t<IsConst && !WasConst>>
    iterator_impl(const iterator_impl<WasConst>& other) :
        m_obj(other.m_obj), m_cur_idx(other.m_cur_idx)
    {
    }

    // Dereference operators
    reference operator*() const
    {
      check_limits();
      return (*m_obj)[m_cur_idx];
    }

    reference operator[](difference_type off) const { return (*m_obj)[m_cur_idx + off]; }

    // Increment/Decrement operators
    iterator_impl& operator++()
    {
      check_limits();
      ++m_cur_idx;
      return *this;
    }

    iterator_impl operator++(int)
    {
      iterator_impl it = *this;
      ++*this;
      return it;
    }

    iterator_impl& operator--()
    {
      --m_cur_idx;
      check_limits();
      return *this;
    }

    iterator_impl operator--(int)
    {
      iterator_impl it = *this;
      --*this;
      return it;
    }

    // Arithmetic operators
    iterator_impl& operator+=(difference_type off)
    {
      m_cur_idx += off;
      check_limits(true);
      return *this;
    }

    iterator_impl operator+(difference_type off) const
    {
      iterator_impl it = *this;
      it += off;
      return it;
    }

    iterator_impl& operator-=(difference_type off) { return (*this) += (-off); }

    iterator_impl operator-(difference_type off) const
    {
      iterator_impl it = *this;
      it -= off;
      return it;
    }

    difference_type operator-(const iterator_impl& it) const { return m_cur_idx - it.m_cur_idx; }

    // Comparison operators
    bool operator==(const iterator_impl& it) const
    {
      return m_obj == it.m_obj && m_cur_idx == it.m_cur_idx;
    }

    bool operator!=(const iterator_impl& it) const { return !(*this == it); }

    bool operator<(const iterator_impl& it) const { return m_cur_idx < it.m_cur_idx; }

    bool operator>(const iterator_impl& it) const { return m_cur_idx > it.m_cur_idx; }

    bool operator<=(const iterator_impl& it) const { return !(*this > it); }

    bool operator>=(const iterator_impl& it) const { return !(*this < it); }

    // Friend declaration to allow conversion constructor access
    template <bool>
    friend class iterator_impl;
  };  // end iterator_impl

  // Iterator access methods
  iterator begin() { return iterator(*this, 0); }
  iterator end() { return iterator(*this, static_size); }
  const_iterator begin() const { return const_iterator(*this, 0); }
  const_iterator end() const { return const_iterator(*this, static_size); }
  const_iterator cbegin() const { return const_iterator(*this, 0); }
  const_iterator cend() const { return const_iterator(*this, static_size); }
  reverse_iterator rbegin() { return reverse_iterator(end()); }
  const_reverse_iterator rbegin() const { return const_reverse_iterator(end()); }
  reverse_iterator rend() { return reverse_iterator(begin()); }
  const_reverse_iterator rend() const { return const_reverse_iterator(begin()); }
  const_reverse_iterator crbegin() const { return const_reverse_iterator(end()); }
  const_reverse_iterator crend() const { return const_reverse_iterator(begin()); }

  void swap(CPose3DQuat& o)
  {
    std::swap(o.m_coords, m_coords);
    o.m_quat.swap(m_quat);
  }

  /** @} */

};  // End of class def.

std::ostream& operator<<(std::ostream& o, const CPose3DQuat& p);

/** Unary - operator: return the inverse pose "-p" (Note that is NOT the same
 * as a pose with all its arguments multiplied by "-1") */
CPose3DQuat operator-(const CPose3DQuat& p);

/** Computes the 3D point L such as \f$ L = G \ominus this \f$.
 * \sa inverseComposePoint
 */
CPoint3D operator-(const CPoint3D& G, const CPose3DQuat& p);

/** Computes the 3D point L such as \f$ L = G \ominus this \f$.
 * \sa inverseComposePoint
 */
mrpt::math::TPoint3D operator-(const mrpt::math::TPoint3D& G, const CPose3DQuat& p);

bool operator==(const CPose3DQuat& p1, const CPose3DQuat& p2);
bool operator!=(const CPose3DQuat& p1, const CPose3DQuat& p2);

}  // namespace mrpt::poses