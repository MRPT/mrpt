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

#include <mrpt/containers/yaml.h>
#include <mrpt/core/round.h>           // round()
#include <mrpt/math/ops_containers.h>  // for << ops
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/viz/CPointCloudColoured.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;
using mrpt::serialization::CArchive;

IMPLEMENTS_SERIALIZABLE(CPointCloudColoured, CVisualObject, mrpt::viz)

uint8_t CPointCloudColoured::serializeGetVersion() const { return 4; }
void CPointCloudColoured::serializeTo(mrpt::serialization::CArchive& out) const
{
  std::shared_lock<std::shared_mutex> wfReadLock(VisualObjectParams_Points::m_pointsMtx.data);

  writeToStreamRender(out);
  out << m_points << m_point_colors;
  VisualObjectParams_Points::params_serialize(out);
}

void CPointCloudColoured::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    {
      THROW_EXCEPTION("Binary backward compatibility lost for this class.");
    }
    break;
    case 4:
    {
      readFromStreamRender(in);
      in >> m_points >> m_point_colors;

      VisualObjectParams_Points::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  wfWriteLock.unlock();
  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

/** Write an individual point (checks for "i" in the valid range only in Debug).
 */
void CPointCloudColoured::setPoint(size_t i, const TPointXYZfRGBAu8& p)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

#ifdef _DEBUG
  ASSERT_LT_(i, size_unprotected());
#endif
  m_points[i] = p.pt;
  auto& c = m_point_colors[i];
  c.R = p.r;
  c.G = p.g;
  c.B = p.b;
  c.A = p.a;

  // JL: TODO note: Well, this can be clearly done much more efficiently
  // but...I don't have time! :-(
  wfWriteLock.unlock();
  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

/** Inserts a new point into the point cloud. */
void CPointCloudColoured::push_back(float x, float y, float z, float R, float G, float B, float A)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

  m_points.emplace_back(x, y, z);
  m_point_colors.emplace_back(f2u8(R), f2u8(G), f2u8(B), f2u8(A));

  // JL: TODO note: Well, this can be clearly done much more efficiently
  // but...I don't have time! :-(
  wfWriteLock.unlock();
  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

void CPointCloudColoured::insertPoint(const mrpt::math::TPointXYZfRGBAu8& p)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

  m_points.emplace_back(p.pt);
  m_point_colors.emplace_back(p.r, p.g, p.b, p.a);

  wfWriteLock.unlock();
  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

// Do needed internal work if all points are new (octree rebuilt,...)
void CPointCloudColoured::markAllPointsAsNew() { CVisualObject::notifyChange(); }

mrpt::math::TBoundingBoxf CPointCloudColoured::internalBoundingBoxLocal() const
{
  // m_pointsMtx.data: already held by calls inside.
  if (empty()) return {};
  auto bb = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
  for (const auto& pt : m_points) bb.updateWithPoint(pt);

  return bb;
}

/** In a base class, reserve memory to prepare subsequent calls to
 * PLY_import_set_vertex */
void CPointCloudColoured::PLY_import_set_vertex_count(size_t N) { this->resize(N); }

/** In a base class, will be called after PLY_import_set_vertex_count() once for
 * each loaded point.
 *  \param pt_color Will be nullptr if the loaded file does not provide color
 * info.
 */
void CPointCloudColoured::PLY_import_set_vertex(
    size_t idx, const mrpt::math::TPoint3Df& pt, const mrpt::img::TColorf* pt_color)
{
  if (!pt_color)
    this->setPoint(idx, TPointXYZfRGBAu8(pt.x, pt.y, pt.z, 0xff, 0xff, 0xff));
  else
    this->setPoint(
        idx, TPointXYZfRGBAu8(
                 pt.x, pt.y, pt.z, f2u8(pt_color->R), f2u8(pt_color->G), f2u8(pt_color->B)));
}

/** In a base class, return the number of vertices */
size_t CPointCloudColoured::PLY_export_get_vertex_count() const { return this->size(); }

/** In a base class, will be called after PLY_export_get_vertex_count() once for
 * each exported point.
 *  \param pt_color Will be nullptr if the loaded file does not provide color
 * info.
 */
void CPointCloudColoured::PLY_export_get_vertex(
    size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color, mrpt::img::TColorf& pt_color) const
{
  std::shared_lock<std::shared_mutex> wfReadLock(VisualObjectParams_Points::m_pointsMtx.data);

  auto& p = m_points[idx];
  auto& p_color = m_point_colors[idx];
  p = pt;
  p_color = pt_color.asTColor();
  pt_has_color = true;
}

void CPointCloudColoured::recolorizeByCoordinate(
    const float coord_min,
    const float coord_max,
    const int coord_index,
    const mrpt::img::TColormap color_map)
{
  ASSERT_GE_(coord_index, 0);
  ASSERT_LT_(coord_index, 3);

  const float coord_range = coord_max - coord_min;
  const float coord_range_1 = coord_range != 0.0f ? 1.0f / coord_range : 1.0f;
  for (size_t i = 0; i < m_points.size(); i++)
  {
    float coord = .0f;
    switch (coord_index)
    {
      case 0:
        coord = m_points[i].x;
        break;
      case 1:
        coord = m_points[i].y;
        break;
      case 2:
        coord = m_points[i].z;
        break;
      default:
        THROW_EXCEPTION("Should not reach here");
    };
    const float col_idx = std::max(0.0f, std::min(1.0f, (coord - coord_min) * coord_range_1));
    const auto col = mrpt::img::colormap(color_map, col_idx);
    this->setPointColor_fast(i, col.R, col.G, col.B);
  }
}

void CPointCloudColoured::toYAMLMap(mrpt::containers::yaml& propertiesMap) const
{
  CVisualObject::toYAMLMap(propertiesMap);
  propertiesMap["point_count"] = m_points.size();
  propertiesMap["bounding_box"] = this->getBoundingBox().asString();
}
