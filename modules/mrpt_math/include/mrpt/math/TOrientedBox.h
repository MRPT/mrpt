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

#include <mrpt/core/exceptions.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>

#include <optional>
#include <vector>

namespace mrpt::math
{
/** \addtogroup  geometry_grp
 * @{ */

/** \brief 3D oriented bounding box: defined by dimensions and pose
 *
 * \details The box is described by the SE(3) pose of its geometric
 * center, plus its lengths along the local (X',Y',Z') axes determined
 * by that pose.
 *
 * Numbering of vertices:
 * \code
 *      4 +---------+ 6
 *       /|        /|
 *      / |       / |
 *     /  |      /  |
 *  5 +---------+ 7 |       +Z'
 *     |   |    |   |       ^
 *     |   |    |   |       |
 *     | 0 +----|---+ 2     +--->  +Y'
 *     |  /     |  /       /
 *     | /      | /       v
 *     |/       |/        +X'
 *  1 +---------+ 3
 * \endcode
 */
template <typename T>
class TOrientedBox_
{
 public:
  /** The number of planes defined by a box */
  constexpr static std::size_t PLANES_PER_BOX = 6;

  using plane_array_t = std::array<mrpt::math::TPlane, PLANES_PER_BOX>;

  TOrientedBox_() = default;

  /** Constructor pose of the center and dimensions in local coordinates */
  TOrientedBox_(const mrpt::math::TPose3D& center_pose, const mrpt::math::TPoint3D_<T>& box_size) :
      m_pose(center_pose), m_size(box_size)
  {
  }

  TOrientedBox_(const TOrientedBox_<T>& o) : m_pose(o.pose()), m_size(o.size()) {}
  TOrientedBox_<T>& operator=(const TOrientedBox_<T>& o)
  {
    setPose(o.pose());
    setSize(o.size());
    return *this;
  }

  template <typename U>
  mrpt::math::TOrientedBox_<U> cast() const
  {
    return {m_pose, m_size.template cast<U>()};
  }

  /** Gets the 8 vertices (cached to avoid recalculation).
   * The order or vertices is ensured to be as defined above */
  [[nodiscard]] const std::vector<mrpt::math::TPoint3D_<T>>& vertices() const;

  /** Gets the box center pose */
  [[nodiscard]] const mrpt::math::TPose3D& pose() const { return m_pose; }

  /** Changes the the box center SE(3) pose */
  void setPose(const mrpt::math::TPose3D& p)
  {
    m_pose = p;
    m_vertices.reset();
  }

  /** Gets the box size (local coordinates) */
  [[nodiscard]] const mrpt::math::TPoint3D_<T>& size() const { return m_size; }

  /** Changes the box size (local coordinates) */
  void setSize(const mrpt::math::TPoint3D_<T>& size)
  {
    m_size = size;
    m_vertices.reset();
  }

  /** Returns a human-readable description of this box */
  [[nodiscard]] std::string asString() const;

  /// @brief Returns the axis-aligned bounding box enclosing this oriented box
  [[nodiscard]] TBoundingBox_<T> getAxisAlignedBox() const;

  /** Returns the 6 planes enclosing the 3D box.
   * The ordering of the planes is: [0]=front, [1]=left, [2]=right, [3]=top, [4]=back, [5]=bottom.
   * Plane normals are all pointing outwards
   */
  [[nodiscard]] plane_array_t getBoxPlanes() const;

  [[nodiscard]] bool operator==(const TOrientedBox_<T>& o) const
  {
    return m_pose == o.m_pose && m_size == o.m_size;
  }

  [[nodiscard]] bool operator!=(const TOrientedBox_<T>& o) const
  {
    return m_pose != o.m_pose || m_size != o.m_size;
  }

 private:
  mrpt::math::TPose3D m_pose;
  mrpt::math::TPoint3D_<T> m_size{0, 0, 0};
  mutable std::optional<std::vector<mrpt::math::TPoint3D_<T>>> m_vertices;
};

/** 3D oriented bounding box, defined by dimensions and pose
 * \sa mrpt::math::TBoundingBox
 */
using TOrientedBox = TOrientedBox_<double>;
using TOrientedBoxf = TOrientedBox_<float>;

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, mrpt::math::TOrientedBox& bb);
mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const mrpt::math::TOrientedBox& bb);

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, mrpt::math::TOrientedBoxf& bb);
mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const mrpt::math::TOrientedBoxf& bb);

/** @} */

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TOrientedBox, mrpt::math)
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TOrientedBoxf, mrpt::math)
}  // namespace mrpt::typemeta
