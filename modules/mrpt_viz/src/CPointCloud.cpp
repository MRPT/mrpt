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
#include <mrpt/core/round.h>  // round()
#include <mrpt/math/ops_containers.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/viz/CPointCloud.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPointCloud, CVisualObject, mrpt::viz)

CPointCloud::CPointCloud() { markAllPointsAsNew(); }

void CPointCloud::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
  SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
  out["colorFromDepth"] = static_cast<int32_t>(m_colorFromDepth);
  out["pointSize"] = getPointSize();
  const auto N = m_points.size();
  out["N"] = static_cast<uint64_t>(N);
  for (size_t i = 0; i < N; i++)
  {
    out["xs"][i] = m_points[i].x;
    out["ys"][i] = m_points[i].y;
    out["zs"][i] = m_points[i].z;
  }
  out["colorFromDepth_min"]["R"] = m_colorFromDepth_min.R;
  out["colorFromDepth_min"]["G"] = m_colorFromDepth_min.G;
  out["colorFromDepth_min"]["B"] = m_colorFromDepth_min.B;
  out["colorFromDepth_max"]["R"] = m_colorFromDepth_max.R;
  out["colorFromDepth_max"]["G"] = m_colorFromDepth_max.G;
  out["colorFromDepth_max"]["B"] = m_colorFromDepth_max.B;
}
void CPointCloud::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
  uint8_t version;
  SCHEMA_DESERIALIZE_DATATYPE_VERSION();
  switch (version)
  {
    case 1:
    {
      /**
       *  currently below is being left to what is being set by
       *  the default constructor i.e.,CPointCloud::colNone
       */
      // m_colorFromDepth = static_cast<float>(in["colorDepth"]);
      setPointSize(static_cast<float>(in["pointSize"]));
      const size_t N = static_cast<uint64_t>(in["N"]);
      m_points.resize(N);
      for (size_t i = 0; i < N; i++)
      {
        m_points[i].x = static_cast<float>(in["xs"][i]);
        m_points[i].y = static_cast<float>(in["ys"][i]);
        m_points[i].z = static_cast<float>(in["zs"][i]);
      }
      m_colorFromDepth_min.R = static_cast<float>(in["colorFromDepth_min"]["R"]);
      m_colorFromDepth_min.G = static_cast<float>(in["colorFromDepth_min"]["G"]);
      m_colorFromDepth_min.B = static_cast<float>(in["colorFromDepth_min"]["B"]);
      m_colorFromDepth_max.R = static_cast<float>(in["colorFromDepth_max"]["R"]);
      m_colorFromDepth_max.G = static_cast<float>(in["colorFromDepth_max"]["G"]);
      m_colorFromDepth_max.B = static_cast<float>(in["colorFromDepth_max"]["B"]);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}
uint8_t CPointCloud::serializeGetVersion() const { return 7; }
void CPointCloud::serializeTo(mrpt::serialization::CArchive& out) const
{
  std::shared_lock<std::shared_mutex> wfReadLock(VisualObjectParams_Points::m_pointsMtx.data);

  writeToStreamRender(out);
  // Changed from bool to enum/int32_t in version 3.
  out.WriteAs<int32_t>(m_colorFromDepth);

  // out << m_xs << m_ys << m_zs;// was: v4
  out.WriteAs<uint32_t>(m_points.size());
  for (const auto& pt : m_points) out << pt;

  // New in version 2:
  out << m_colorFromDepth_min.R << m_colorFromDepth_min.G << m_colorFromDepth_min.B;
  out << m_colorFromDepth_max.R << m_colorFromDepth_max.G << m_colorFromDepth_max.B;

  // new v6:
  VisualObjectParams_Points::params_serialize(out);
}

void CPointCloud::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    {
      readFromStreamRender(in);
      if (version >= 3)
      {
        int32_t axis;
        in >> axis;
        m_colorFromDepth = Axis(axis);
      }
      else
      {
        bool colorFromZ;
        in >> colorFromZ;
        m_colorFromDepth = colorFromZ ? CPointCloud::colZ : CPointCloud::colNone;
      }
      if (version < 5)
      {
        std::vector<float> xs, ys, zs;
        in >> xs >> ys >> zs;
        wfWriteLock.unlock();  // avoid recursive lock
        this->setAllPoints(xs, ys, zs);
        wfWriteLock.lock();
      }
      else
      {
        // New in v5:
        auto N = in.ReadAs<uint32_t>();
        m_points.resize(N);
        for (auto& pt : m_points) in >> pt;
      }

      if (version >= 1 && version < 6) setPointSize(in.ReadAs<float>());

      if (version >= 2)
      {
        in >> m_colorFromDepth_min.R >> m_colorFromDepth_min.G >> m_colorFromDepth_min.B;
        in >> m_colorFromDepth_max.R >> m_colorFromDepth_max.G >> m_colorFromDepth_max.B;
      }
      else
      {
        m_colorFromDepth_min = TColorf(0, 0, 0);
        m_colorFromDepth_max = getColor();
      }

      if (version >= 4 && version < 7)
      {
        bool m_pointSmooth_ignored;
        in >> m_pointSmooth_ignored;
      }

      if (version >= 6) VisualObjectParams_Points::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  wfWriteLock.unlock();
  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

void CPointCloud::clear()
{
  {
    std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

    m_points.clear();
  }

  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

void CPointCloud::insertPoint(float x, float y, float z)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

  m_points.emplace_back(x, y, z);

  m_minmax_valid = false;

  // JL: TODO note: Well, this can be clearly done much more efficiently
  // but...I don't have time! :-(
  wfWriteLock.unlock();
  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

/** Write an individual point (checks for "i" in the valid range only in
 * Debug).
 */
void CPointCloud::setPoint(size_t i, const float x, const float y, const float z)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

  m_points.at(i) = {x, y, z};

  m_minmax_valid = false;

  // JL: TODO note: Well, this can be clearly done much more efficiently
  // but...I don't have time! :-(
  wfWriteLock.unlock();
  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

/*---------------------------------------------------------------
          setGradientColors
---------------------------------------------------------------*/
void CPointCloud::setGradientColors(
    const mrpt::img::TColorf& colorMin, const mrpt::img::TColorf& colorMax)
{
  m_colorFromDepth_min = colorMin;
  m_colorFromDepth_max = colorMax;
}

// Do needed internal work if all points are new (octree rebuilt,...)
void CPointCloud::markAllPointsAsNew()
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

  m_minmax_valid = false;
  CVisualObject::notifyChange();
}

/** In a base class, reserve memory to prepare subsequent calls to
 * PLY_import_set_vertex */
void CPointCloud::PLY_import_set_vertex_count(size_t N) { this->resize(N); }

/** In a base class, will be called after PLY_import_set_vertex_count() once
 * for each loaded point. \param pt_color Will be nullptr if the loaded file
 * does not provide color info.
 */
void CPointCloud::PLY_import_set_vertex(
    size_t idx,
    const mrpt::math::TPoint3Df& pt,
    [[maybe_unused]] const mrpt::img::TColorf* pt_color)
{
  this->setPoint(idx, pt.x, pt.y, pt.z);
}

/** In a base class, return the number of vertices */
size_t CPointCloud::PLY_export_get_vertex_count() const { return this->size(); }
/** In a base class, will be called after PLY_export_get_vertex_count() once
 * for each exported point. \param pt_color Will be nullptr if the loaded
 * file does not provide color info.
 */
void CPointCloud::PLY_export_get_vertex(
    size_t idx,
    mrpt::math::TPoint3Df& pt,
    bool& pt_has_color,
    [[maybe_unused]] mrpt::img::TColorf& pt_color) const
{
  std::shared_lock<std::shared_mutex> wfReadLock(VisualObjectParams_Points::m_pointsMtx.data);

  pt_has_color = false;

  pt = m_points[idx];
}

TBoundingBoxf CPointCloud::internalBoundingBoxLocal() const
{
  // m_pointsMtx.data: already held by calls inside.
  if (empty()) return {};
  auto bb = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
  for (const auto& pt : m_points) bb.updateWithPoint(pt);

  return bb;
}

void CPointCloud::setAllPoints(const std::vector<mrpt::math::TPoint3D>& pts)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(VisualObjectParams_Points::m_pointsMtx.data);

  const auto N = pts.size();
  m_points.resize(N);
  for (size_t i = 0; i < N; i++) m_points[i] = pts[i];
  m_minmax_valid = false;
  wfWriteLock.unlock();
  markAllPointsAsNew();
  CVisualObject::notifyChange();
}

void CPointCloud::toYAMLMap(mrpt::containers::yaml& propertiesMap) const
{
  CVisualObject::toYAMLMap(propertiesMap);
  propertiesMap["point_count"] = m_points.size();
}
