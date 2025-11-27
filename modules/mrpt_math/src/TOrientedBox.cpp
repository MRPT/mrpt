/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/math/TOrientedBox.h>
#include <mrpt/serialization/CArchive.h>  // impl of << operator

#include <sstream>
#include <type_traits>

using namespace mrpt::math;

static_assert(std::is_copy_constructible_v<TOrientedBox>);
static_assert(std::is_copy_assignable_v<TOrientedBox>);

static_assert(std::is_copy_constructible_v<TOrientedBoxf>);
static_assert(std::is_copy_assignable_v<TOrientedBoxf>);

mrpt::serialization::CArchive& mrpt::math::operator>>(
    mrpt::serialization::CArchive& in, mrpt::math::TOrientedBox& bb)
{
  mrpt::math::TPose3D p;
  mrpt::math::TPoint3D s;
  in >> p >> s;
  bb.setPose(p);
  bb.setSize(s);
  return in;
}

mrpt::serialization::CArchive& mrpt::math::operator<<(
    mrpt::serialization::CArchive& out, const mrpt::math::TOrientedBox& bb)
{
  out << bb.pose() << bb.size();
  return out;
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
    mrpt::serialization::CArchive& in, mrpt::math::TOrientedBoxf& bb)
{
  mrpt::math::TPose3D p;
  mrpt::math::TPoint3Df s;
  in >> p >> s;
  bb.setPose(p);
  bb.setSize(s);
  return in;
}

mrpt::serialization::CArchive& mrpt::math::operator<<(
    mrpt::serialization::CArchive& out, const mrpt::math::TOrientedBoxf& bb)
{
  out << bb.pose() << bb.size();
  return out;
}

template <typename T>
const std::vector<mrpt::math::TPoint3D_<T>>& mrpt::math::TOrientedBox_<T>::vertices() const
{
  if (m_vertices)
  {
    return *m_vertices;
  }

  // Recalculate:
  auto& verts = m_vertices.emplace();

  verts.clear();
  verts.reserve(8);
  for (const int z : {-1, 1})
  {
    for (const int y : {-1, 1})
    {
      for (const int x : {-1, 1})
      {
        verts.push_back(
            m_pose.composePoint({x * m_size.x / 2.0, y * m_size.y / 2.0, z * m_size.z / 2.0}));
      }
    }
  }
  return *m_vertices;
}

template <typename T>
std::string mrpt::math::TOrientedBox_<T>::asString() const
{
  std::stringstream ss;
  ss << "center=" << m_pose << " size=" << m_size;
  return ss.str();
}

template <typename T>
TBoundingBox_<T> mrpt::math::TOrientedBox_<T>::getAxisAlignedBox() const
{
  auto bbox = mrpt::math::TBoundingBox_<T>::PlusMinusInfinity();

  for (const auto& point : vertices())
  {
    bbox.updateWithPoint(point);
  }
  return bbox;
}

template <typename T>
typename mrpt::math::TOrientedBox_<T>::plane_array_t
mrpt::math::TOrientedBox_<T>::getBoxPlanes() const
{
  using mrpt::math::TPlane;

  ///      4 +---------+ 6
  ///       /|        /|
  ///      / |       / |
  ///     /  |      /  |
  ///  5 +---------+ 7 |       +Z'  (up)
  ///     |   |    |   |       ^
  ///     |   |    |   |       |
  ///     | 0 +----|---+ 2     +--->  +Y'  (left)
  ///     |  /     |  /       /
  ///     | /      | /       v
  ///     |/       |/        +X'  (front)
  ///  1 +---------+ 3

  const auto& vertices = this->vertices();

  // NOTE: Planes defined with consistent winding order: normals pointing OUTWARDS
  return std::array<TPlane, mrpt::math::TOrientedBox_<T>::PLANES_PER_BOX>{
      TPlane::From3Points(vertices[7], vertices[5], vertices[1]),  // front
      TPlane::From3Points(vertices[6], vertices[7], vertices[3]),  // left
      TPlane::From3Points(vertices[5], vertices[4], vertices[0]),  // right
      TPlane::From3Points(vertices[4], vertices[5], vertices[7]),  // top
      TPlane::From3Points(vertices[4], vertices[6], vertices[0]),  // back
      TPlane::From3Points(vertices[0], vertices[2], vertices[1]),  // bottom
  };
}

// Explicit instantiations
template class mrpt::math::TOrientedBox_<float>;
template class mrpt::math::TOrientedBox_<double>;
