/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
//
#include <mrpt/containers/yaml.h>
#include <mrpt/core/round.h>  // round()
#include <mrpt/math/ops_containers.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace std;

float OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL_value = 0.10f;
size_t OCTREE_RENDER_MAX_POINTS_PER_NODE_value = 1e6;

float mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL()
{
  return OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL_value;
}
void mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL(float value)
{
  OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL_value = value;
}

size_t mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE()
{
  return OCTREE_RENDER_MAX_POINTS_PER_NODE_value;
}
void mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE(size_t value)
{
  OCTREE_RENDER_MAX_POINTS_PER_NODE_value = value;
}

IMPLEMENTS_SERIALIZABLE(CPointCloud, CRenderizable, mrpt::opengl)

CPointCloud::CPointCloud() { markAllPointsAsNew(); }

void CPointCloud::onUpdateBuffers_Points()
{
  std::unique_lock<std::shared_mutex> wfWriteLock(CRenderizableShaderPoints::m_pointsMtx.data);

  {
    mrpt::math::TPoint3Df tst[2];
    // was static_assert(), error in gcc9.1, cannot use ptr+3 in constexpr.
    ASSERTMSG_(&tst[1].x == (&tst[0].x + 3), "memory layout not as expected");
    ASSERTMSG_(&tst[1].y == (&tst[0].y + 3), "memory layout not as expected");
    ASSERTMSG_(&tst[1].z == (&tst[0].z + 3), "memory layout not as expected");
  }

  const auto N = m_points.size();

  octree_assure_uptodate();  // Rebuild octree if needed
  m_last_rendered_count_ongoing = 0;

  if (m_colorFromDepth != colNone)
  {
    if (!m_minmax_valid)
    {
      m_minmax_valid = true;
      if (!m_points.empty())
      {
        const float* vs =
            m_colorFromDepth == CPointCloud::colZ
                ? &m_points[0].z
                : (m_colorFromDepth == CPointCloud::colY ? &m_points[0].y : &m_points[0].x);
        m_min = m_max = vs[0];
        for (size_t i = 1; i < N; i++)
        {
          float v = vs[3 * i];
          if (v < m_min) m_min = v;
          if (v > m_max) m_max = v;
        }
      }
      else
        m_max = m_min = 0;
    }

    m_max_m_min = m_max - m_min;
    if (std::abs(m_max_m_min) < 1e-4)
      m_max_m_min = -1;
    else
      m_min = m_max - m_max_m_min * 1.01f;
    m_max_m_min_inv = 1.0f / m_max_m_min;
  }

  // Slopes of color interpolation:
  m_col_slop.R = m_colorFromDepth_max.R - m_colorFromDepth_min.R;
  m_col_slop.G = m_colorFromDepth_max.G - m_colorFromDepth_min.G;
  m_col_slop.B = m_colorFromDepth_max.B - m_colorFromDepth_min.B;

  m_col_slop_inv.R = m_col_slop.R != 0 ? 1.0f / m_col_slop.R : 0;
  m_col_slop_inv.G = m_col_slop.G != 0 ? 1.0f / m_col_slop.G : 0;
  m_col_slop_inv.B = m_col_slop.B != 0 ? 1.0f / m_col_slop.B : 0;

  // TODO: Restore rendering using octrees?
  // octree_render(*rc.state);  // Render all points recursively:

  // ------------------------------
  // Fill the shader buffers
  // ------------------------------
  // "CRenderizableShaderPoints::m_vertex_buffer_data" is already done, since
  // "m_points" is an alias for it.

  const auto myColor = getColor_u8();

  // color buffer:
  auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;
  cbd.clear();
  cbd.reserve(N);

  // color for each point:
  if (m_colorFromDepth != colNone && m_max_m_min > 0)
  {
    for (size_t i = 0; i < N; i++)
    {
      const float depthCol =
          (m_colorFromDepth == colX ? m_points[i].x
                                    : (m_colorFromDepth == colY ? m_points[i].y : m_points[i].z));

      float f = (depthCol - m_min) * m_max_m_min_inv;
      f = std::max(0.0f, min(1.0f, f));

      cbd.push_back(
          {f2u8(m_colorFromDepth_min.R + f * m_col_slop_inv.R),
           f2u8(m_colorFromDepth_min.G + f * m_col_slop_inv.G),
           f2u8(m_colorFromDepth_min.B + f * m_col_slop_inv.B), myColor.A});
    }
  }
  else
  {
    // all points: same color
    cbd.assign(N, myColor);
  }

  m_last_rendered_count = m_last_rendered_count_ongoing;
}

inline void CPointCloud::internal_render_one_point([[maybe_unused]] size_t i) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
#endif
}

/** Render a subset of points (required by octree renderer) */
void CPointCloud::render_subset(
    [[maybe_unused]] const bool all,
    [[maybe_unused]] const std::vector<size_t>& idxs,
    [[maybe_unused]] const float render_area_sqpixels) const
{
#if 0 && MRPT_HAS_OPENGL_GLUT
	// Disabled for now... (Feb 2020)

	const size_t N = (all ? m_xs.size() : idxs.size());
	const size_t decimation = mrpt::round(std::max(
		1.0f, d2f(
				  N / (OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL_value *
					   render_area_sqpixels))));

	m_last_rendered_count_ongoing += N / decimation;

	if (all)
	{
		for (size_t i = 0; i < N; i++) internal_render_one_point(i);
	}
	else
	{
		const size_t Np = idxs.size();
		for (size_t i = 0; i < Np; i += decimation)
			internal_render_one_point(idxs[i]);
	}
#endif
}
void CPointCloud::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
  SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
  out["colorFromDepth"] = static_cast<int32_t>(m_colorFromDepth);
  out["pointSize"] = m_pointSize;
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
  out["pointSmooth"] = m_pointSmooth;
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
      m_pointSize = static_cast<float>(in["pointSize"]);
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
      m_pointSmooth = static_cast<bool>(in["pointSmooth"]);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}
uint8_t CPointCloud::serializeGetVersion() const { return 6; }
void CPointCloud::serializeTo(mrpt::serialization::CArchive& out) const
{
  std::shared_lock<std::shared_mutex> wfReadLock(CRenderizableShaderPoints::m_pointsMtx.data);

  writeToStreamRender(out);
  // Changed from bool to enum/int32_t in version 3.
  out.WriteAs<int32_t>(m_colorFromDepth);

  // out << m_xs << m_ys << m_zs;// was: v4
  out.WriteAs<uint32_t>(m_points.size());
  for (const auto& pt : m_points) out << pt;

  // New in version 2:
  out << m_colorFromDepth_min.R << m_colorFromDepth_min.G << m_colorFromDepth_min.B;
  out << m_colorFromDepth_max.R << m_colorFromDepth_max.G << m_colorFromDepth_max.B;

  // New in version 4:
  out << m_pointSmooth;

  // new v6:
  CRenderizableShaderPoints::params_serialize(out);
}

void CPointCloud::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(CRenderizableShaderPoints::m_pointsMtx.data);

  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
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

      if (version >= 1 && version < 6)
        in >> m_pointSize;
      else
        m_pointSize = 1;

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

      if (version >= 4)
        in >> m_pointSmooth;
      else
        m_pointSmooth = false;

      if (version >= 6) CRenderizableShaderPoints::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  wfWriteLock.unlock();
  markAllPointsAsNew();
  CRenderizable::notifyChange();
}

void CPointCloud::clear()
{
  {
    std::unique_lock<std::shared_mutex> wfWriteLock(CRenderizableShaderPoints::m_pointsMtx.data);

    m_points.clear();
  }

  markAllPointsAsNew();
  CRenderizable::notifyChange();
}

void CPointCloud::insertPoint(float x, float y, float z)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(CRenderizableShaderPoints::m_pointsMtx.data);

  m_points.emplace_back(x, y, z);

  m_minmax_valid = false;

  // JL: TODO note: Well, this can be clearly done much more efficiently
  // but...I don't have time! :-(
  wfWriteLock.unlock();
  markAllPointsAsNew();
  CRenderizable::notifyChange();
}

/** Write an individual point (checks for "i" in the valid range only in
 * Debug).
 */
void CPointCloud::setPoint(size_t i, const float x, const float y, const float z)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(CRenderizableShaderPoints::m_pointsMtx.data);

  m_points.at(i) = {x, y, z};

  m_minmax_valid = false;

  // JL: TODO note: Well, this can be clearly done much more efficiently
  // but...I don't have time! :-(
  wfWriteLock.unlock();
  markAllPointsAsNew();
  CRenderizable::notifyChange();
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
  std::unique_lock<std::shared_mutex> wfWriteLock(CRenderizableShaderPoints::m_pointsMtx.data);

  m_minmax_valid = false;
  octree_mark_as_outdated();
  CRenderizable::notifyChange();
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
  std::shared_lock<std::shared_mutex> wfReadLock(CRenderizableShaderPoints::m_pointsMtx.data);

  pt_has_color = false;

  pt = m_points[idx];
}

void CPointCloud::setAllPoints(const std::vector<mrpt::math::TPoint3D>& pts)
{
  std::unique_lock<std::shared_mutex> wfWriteLock(CRenderizableShaderPoints::m_pointsMtx.data);

  const auto N = pts.size();
  m_points.resize(N);
  for (size_t i = 0; i < N; i++) m_points[i] = pts[i];
  m_minmax_valid = false;
  wfWriteLock.unlock();
  markAllPointsAsNew();
  CRenderizable::notifyChange();
}

void CPointCloud::toYAMLMap(mrpt::containers::yaml& propertiesMap) const
{
  CRenderizable::toYAMLMap(propertiesMap);
  propertiesMap["point_count"] = m_points.size();
}
