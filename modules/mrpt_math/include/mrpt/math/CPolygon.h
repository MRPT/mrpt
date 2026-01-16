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

#include <mrpt/math/TPolygon2D.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::math
{
/** A wrapper of a TPolygon2D class, implementing CSerializable.
 * \ingroup geometry_grp
 */
class CPolygon : public mrpt::serialization::CSerializable, public mrpt::math::TPolygon2D
{
  DEFINE_SERIALIZABLE(CPolygon, mrpt::math)

 public:
  /** Default constructor (empty polygon, 0 vertices) */
  CPolygon() = default;

  /** Add a new vertex to polygon */
  void add_vertex(double x, double y) { TPolygon2D::emplace_back(x, y); }

  /** Methods for accessing the vertices \sa verticesCount */
  [[nodiscard]] double get_vertex_x(size_t i) const
  {
    ASSERT_(i < TPolygon2D::size());
    return TPolygon2D::operator[](i).x;
  }
  [[nodiscard]] double get_vertex_y(size_t i) const
  {
    ASSERT_(i < TPolygon2D::size());
    return TPolygon2D::operator[](i).y;
  }

  /** Set all vertices at once. */
  void set_vertices(const std::vector<double>& x, const std::vector<double>& y);

  /** Set all vertices at once. Please use the std::vector version whenever
   * possible unless efficiency is really an issue */
  void set_vertices(size_t nVertices, const double* xs, const double* ys);

  /** Set all vertices at once. Please use the std::vector version whenever
   * possible unless efficiency is really an issue */
  void set_vertices(size_t nVertices, const float* xs, const float* ys);

  /** Get all vertices at once */
  void get_vertices(std::vector<double>& x, std::vector<double>& y) const;
};

}  // namespace mrpt::math
