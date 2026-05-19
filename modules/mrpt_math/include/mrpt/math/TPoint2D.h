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
#include <mrpt/core/format.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <mrpt/math/math_frwds.h>  // CMatrixFixed

#include <cmath>  // sqrt
#include <tuple>
#include <type_traits>
#include <vector>

namespace mrpt::math
{
/** \addtogroup  geometry_grp
 * @{ */

template <typename T>
struct TPoint2D_data
{
  constexpr TPoint2D_data() = default;
  constexpr TPoint2D_data(T X, T Y) : x(X), y(Y) {}

  /** X,Y coordinates */
  T x, y;
};

/** Base template for TPoint2D (double) and TPoint2Df (float).
 *
 * Represents a point (or free vector) in the 2D Euclidean space R^2.
 * Coordinates are stored as `(x, y)`.
 *
 * This is a *lightweight* POD-like type intended for storage and arithmetic.
 * When pose-point composition (i.e. applying an SE(2) rigid transformation to a point)
 * is needed, use mrpt::poses::CPose2D or mrpt::math::TPose2D instead.
 *
 * \note `TVector2D` is a type alias for `TPoint2D` (both represent elements of R^2;
 *       the alias is provided for semantic clarity when the object is a free vector
 *       rather than a position).
 * \sa mrpt::poses::CPoint2D, TPose2D, TPoint3D
 * \ingroup geometry_grp
 */
template <typename T>
struct TPoint2D_ :
    public TPoseOrPoint,
    public TPoint2D_data<T>,
    public internal::ProvideStaticResize<TPoint2D_<T>>
{
  static constexpr std::size_t static_size = 2;

  /** Default constructor. Initializes to zeros  */
  constexpr TPoint2D_() : TPoint2D_data<T>{0, 0} {}
  /** Constructor from coordinates  */
  constexpr TPoint2D_(T xx, T yy) : TPoint2D_data<T>{xx, yy} {}

  /** Explicit constructor from coordinates.  */
  template <typename U>
  TPoint2D_(const TPoint2D_data<U>& p)
  {
    TPoint2D_data<T>::x = static_cast<T>(p.x);
    TPoint2D_data<T>::y = static_cast<T>(p.y);
  }

  /** Constructor from column vector. */
  template <typename U>
  explicit TPoint2D_(const mrpt::math::CMatrixFixed<U, 2, 1>& m)
  {
    TPoint2D_data<T>::x = static_cast<T>(m[0]);
    TPoint2D_data<T>::y = static_cast<T>(m[1]);
  }

  /** Constructor from TPose2D, retaining only the translational part (x,y); phi is discarded.
   * \sa TPose2D
   */
  explicit TPoint2D_(const TPose2D& p);
  /** Constructor from TPoint3D, retaining only (x,y); z is discarded.
   * \sa TPoint3D
   */
  explicit TPoint2D_(const TPoint3D_<T>& p);
  /** Constructor from TPose3D, retaining only the translational (x,y) part; z and all
   * angular coordinates are discarded.
   * \sa TPose3D
   */
  explicit TPoint2D_(const TPose3D& p);

  /** Builds from the first 2 elements of a vector-like object: [x y]
   *
   * \tparam Vector It can be std::vector<double>, Eigen::VectorXd, etc.
   */
  template <typename Vector>
  [[nodiscard]] static TPoint2D_<T> FromVector(const Vector& v)
  {
    TPoint2D_<T> o;
    for (int i = 0; i < 2; i++)
    {
      o[i] = static_cast<T>(v.at(i));
    }
    return o;
  }

  /** Return a copy of this object using type U for coordinates */
  template <typename U>
  [[nodiscard]] TPoint2D_<U> cast() const
  {
    return TPoint2D_<U>(static_cast<U>(this->x), static_cast<U>(this->y));
  }

  /** Coordinate access using operator[]. Order: x,y */
  T& operator[](size_t i)
  {
    switch (i)
    {
      case 0:
        return this->x;
      case 1:
        return this->y;
      default:
        throw std::out_of_range("index out of range");
    }
  }
  /** Coordinate access using operator[]. Order: x,y */
  constexpr T operator[](size_t i) const
  {
    switch (i)
    {
      case 0:
        return this->x;
      case 1:
        return this->y;
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
    v.resize(2);
    v[0] = TPoint2D_data<T>::x;
    v[1] = TPoint2D_data<T>::y;
  }
  /// \overload
  template <typename Vector>
  [[nodiscard]] Vector asVector() const
  {
    Vector v;
    asVector(v);
    return v;
  }

  bool operator<(const TPoint2D_& p) const;

  TPoint2D_& operator+=(const TPoint2D_& p)
  {
    this->x += p.x;
    this->y += p.y;
    return *this;
  }

  TPoint2D_& operator-=(const TPoint2D_& p)
  {
    this->x -= p.x;
    this->y -= p.y;
    return *this;
  }

  TPoint2D_& operator*=(T d)
  {
    this->x *= d;
    this->y *= d;
    return *this;
  }

  TPoint2D_& operator/=(T d)
  {
    ASSERT_(d != 0);
    this->x /= d;
    this->y /= d;
    return *this;
  }

  [[nodiscard]] constexpr TPoint2D_ operator+(const TPoint2D_& p) const
  {
    return {this->x + p.x, this->y + p.y};
  }

  [[nodiscard]] constexpr TPoint2D_ operator-(const TPoint2D_& p) const
  {
    return {this->x - p.x, this->y - p.y};
  }

  [[nodiscard]] constexpr TPoint2D_ operator*(T d) const { return {d * this->x, d * this->y}; }
  [[nodiscard]] constexpr TPoint2D_ operator/(T d) const
  {
    ASSERT_(d != 0);
    return {this->x / d, this->y / d};
  }
  /** Returns a human-readable textual representation of the object (eg:
   * "[0.02 1.04]" )
   * \sa fromString
   */
  void asString(std::string& s) const
  {
    s = mrpt::format("[%f %f]", double{this->x}, double{this->y});
  }

  [[nodiscard]] std::string asString() const
  {
    std::string s;
    asString(s);
    return s;
  }

  /** Set the current object value from a string generated by 'asString' (eg:
   * "[0.02 1.04]" )
   * \sa asString
   * \exception std::exception On invalid format
   */
  void fromString(const std::string& s);

  [[nodiscard]] static TPoint2D_ FromString(const std::string& s)
  {
    TPoint2D_ o;
    o.fromString(s);
    return o;
  }

  /** Method so std::tuple, std::tie() works with TPoint2D */
  template <size_t I>
  const T& get() const
  {
    if constexpr (I == 0)
    {
      return TPoint2D_data<T>::x;
    }
    return TPoint2D_data<T>::y;
  }

  [[nodiscard]] constexpr auto as_tuple() const
  {
    return std::tie(TPoint2D_data<T>::x, TPoint2D_data<T>::y);
  }

  /** Squared norm: `|v|^2 = x^2+y^2` */
  [[nodiscard]] T sqrNorm() const { return this->x * this->x + this->y * this->y; }

  /** Point norm: `|v| = sqrt(x^2+y^2)` */
  [[nodiscard]] T norm() const { return std::sqrt(sqrNorm()); }

  /** Returns this vector with unit length: v/norm(v) */
  [[nodiscard]] TPoint2D_<T> unitarize() const
  {
    const T n = norm();
    ASSERT_GT_(n, 0);
    const T f = 1 / n;
    return {TPoint2D_data<T>::x * f, TPoint2D_data<T>::y * f};
  }
};

/** Lightweight 2D point / free vector in R^2 (double precision).
 * Coordinate access via `x`, `y` members or `operator[]` (index 0→x, 1→y).
 * \sa mrpt::poses::CPoint2D, TPoint2Df, TVector2D
 * \ingroup geometry_grp
 */
using TPoint2D = TPoint2D_<double>;
/** Single-precision variant of TPoint2D. \ingroup geometry_grp */
using TPoint2Df = TPoint2D_<float>;

/** Type alias for a 2D free vector (same storage as TPoint2D; use this name when the
 *  object represents a direction or displacement rather than a position). */
using TVector2D = TPoint2D;
/** Single-precision variant of TVector2D. */
using TVector2Df = TPoint2Df;

/** Unary minus operator for 2D points/vectors. */
template <typename T>
constexpr TPoint2D_<T> operator-(const TPoint2D_<T>& p1)
{
  return {-p1.x, -p1.y};
}

/** scalar times vector operator. */
template <
    typename T,
    typename Scalar,
    std::enable_if_t<std::is_convertible_v<Scalar, T>>* = nullptr>
constexpr TPoint2D_<T> operator*(const Scalar scalar, const TPoint2D_<T>& p)
{
  const auto s = static_cast<T>(scalar);
  return {s * p.x, s * p.y};
}

/** Exact comparison between 2D points */
template <typename T>
constexpr bool operator==(const TPoint2D_<T>& p1, const TPoint2D_<T>& p2)
{
  return (p1.x == p2.x) && (p1.y == p2.y);  //-V550
}

/**  Exact comparison between 2D points */
template <typename T>
constexpr bool operator!=(const TPoint2D_<T>& p1, const TPoint2D_<T>& p2)
{
  return (p1.x != p2.x) || (p1.y != p2.y);  //-V550
}

/** @} */

}  // namespace mrpt::math

// Specializations so std::tuple, std::tie() works with TPoint2D
namespace std
{
template <typename T>
struct tuple_size<mrpt::math::TPoint2D_<T>> : integral_constant<size_t, 2>
{
};

template <size_t I, typename T>
struct tuple_element<I, mrpt::math::TPoint2D_<T>>
{
  using type = T;
};
}  // namespace std

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPoint2D, mrpt::math)
}  // namespace mrpt::typemeta
