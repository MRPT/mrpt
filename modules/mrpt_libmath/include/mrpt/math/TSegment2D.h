/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoseOrPoint.h>

namespace mrpt::math
{
/**
 * 2D segment, consisting of two points.
 * \sa TSegment3D,TLine2D,TPolygon2D,TPoint2D
 * \ingroup geometry_grp
 */
struct TSegment2D
{
 public:
  /** Constructor from both points */
  TSegment2D(const TPoint2D& p1, const TPoint2D& p2) : point1(p1), point2(p2) {}
  /// Static method, returns segment from two points \note New in MRPT 2.3.0
  static TSegment2D FromPoints(const TPoint2D& p1, const TPoint2D& p2)
  {
    return TSegment2D(p1, p2);
  }
  /** Fast default constructor. Initializes to (0,0)-(0,0) */
  TSegment2D() = default;

  /** Explicit constructor from 3D object, discarding the z. */
  explicit TSegment2D(const TSegment3D& s);

  /**
   * Origin point.
   */
  TPoint2D point1;
  /**
   * Destiny point.
   */
  TPoint2D point2;
  /**
   * Segment length.
   */
  double length() const;

  /** Absolute distance to point. */
  double distance(const TPoint2D& point) const;

  /** Distance with sign to point (sign indicates which side the point is) */
  double signedDistance(const TPoint2D& point) const;

  /**
   * Check whether a point is inside a segment.
   */
  bool contains(const TPoint2D& point) const;
  /** Access to points using operator[0-1] */
  TPoint2D& operator[](size_t i)
  {
    switch (i)
    {
      case 0:
        return point1;
      case 1:
        return point2;
      default:
        throw std::out_of_range("index out of range");
    }
  }
  /** Access to points using operator[0-1] */
  constexpr const TPoint2D& operator[](size_t i) const
  {
    switch (i)
    {
      case 0:
        return point1;
      case 1:
        return point2;
      default:
        throw std::out_of_range("index out of range");
    }
  }
  /**
   * Project into 3D space, setting the z to 0.
   */
  void generate3DObject(TSegment3D& s) const;
  /**
   * Segment's central point.
   */
  void getCenter(TPoint2D& p) const
  {
    p.x = (point1.x + point2.x) / 2;
    p.y = (point1.y + point2.y) / 2;
  }

  bool operator<(const TSegment2D& s) const;
};

inline bool operator==(const TSegment2D& s1, const TSegment2D& s2)
{
  return (s1.point1 == s2.point1) && (s1.point2 == s2.point2);
}

inline bool operator!=(const TSegment2D& s1, const TSegment2D& s2)
{
  return (s1.point1 != s2.point1) || (s1.point2 != s2.point2);
}

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, mrpt::math::TSegment2D& s);
mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const mrpt::math::TSegment2D& s);

/** Text streaming function */
std::ostream& operator<<(std::ostream& o, const TSegment2D& p);

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TSegment2D, mrpt::math)

}  // namespace mrpt::typemeta
