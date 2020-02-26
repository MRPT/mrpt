/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/core/round.h>  // round()
#include <mrpt/math/ops_containers.h>  // for << ops
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;
using mrpt::serialization::CArchive;

IMPLEMENTS_SERIALIZABLE(CPointCloudColoured, CRenderizable, mrpt::opengl)

void CPointCloudColoured::onUpdateBuffers_Points()
{
	octree_assure_uptodate();  // Rebuild octree if needed
	m_last_rendered_count_ongoing = 0;

	{
		mrpt::math::TPoint3Df tst[2];
		static_assert(
			&tst[1].x == (&tst[0].x + 3), "memory layout not as expected");
		static_assert(
			&tst[1].y == (&tst[0].y + 3), "memory layout not as expected");
		static_assert(
			&tst[1].z == (&tst[0].z + 3), "memory layout not as expected");
	}

	const auto N = m_points.size();

	octree_assure_uptodate();  // Rebuild octree if needed
	m_last_rendered_count_ongoing = 0;

	MRPT_TODO("Restore rendering using octrees");
	// octree_render(*rc.state);  // Render all points recursively:

	// ------------------------------
	// Fill the shader buffers
	// ------------------------------
	// "CRenderizableShaderPoints::m_vertex_buffer_data" is already done, since
	// "m_points" is an alias for it.

	// color buffer:
	auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;
	cbd.clear();
	cbd.reserve(N);

	// Point colors:
	cbd.assign(N, m_color);

	m_last_rendered_count = m_last_rendered_count_ongoing;
}

/** Render a subset of points (required by octree renderer) */
void CPointCloudColoured::render_subset(
	const bool all, const std::vector<size_t>& idxs,
	const float render_area_sqpixels) const
{
#if 0 && MRPT_HAS_OPENGL_GLUT
	// Disabled for now... (Feb 2020)
	const size_t N = all ? m_points.size() : idxs.size();
	const size_t decimation = mrpt::round(std::max(
		1.0f, d2f(N / (mrpt::global_settings::
						   OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL() *
					   render_area_sqpixels))));

	m_last_rendered_count_ongoing += N / decimation;

	m_last_rendered_count_ongoing +=
		(all ? m_points.size() : idxs.size()) / decimation;

	if (all)
	{
		for (size_t i = 0; i < N; i += decimation)
		{
			const TPointColour& p = m_points[i];
			glColor4ub(p.r, p.g, p.b, m_color.A);
			glVertex3f(p.pt.x, p.pt.y, p.pt.z);
		}
	}
	else
	{
		for (size_t i = 0; i < N; i += decimation)
		{
			const TPointColour& p = m_points[idxs[i]];
			glColor4ub(p.r, p.g, p.b, m_color.A);
			glVertex3f(p.pt.x, p.pt.y, p.pt.z);
		}
	}
#else
	MRPT_UNUSED_PARAM(all);
	MRPT_UNUSED_PARAM(idxs);
	MRPT_UNUSED_PARAM(render_area_sqpixels);
#endif
}

uint8_t CPointCloudColoured::serializeGetVersion() const { return 4; }
void CPointCloudColoured::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_points << m_point_colors << m_pointSize;
}

void CPointCloudColoured::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			THROW_EXCEPTION(
				"Binary backward compatibility lost for this class.");
		}
		break;
		case 4:
		{
			readFromStreamRender(in);
			in >> m_points >> m_point_colors >> m_pointSize;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	markAllPointsAsNew();
}

/** Write an individual point (checks for "i" in the valid range only in Debug).
 */
void CPointCloudColoured::setPoint(size_t i, const TPointXYZfRGBAu8& p)
{
#ifdef _DEBUG
	ASSERT_BELOW_(i, size());
#endif
	m_points[i] = p.pt;
	auto& c = m_point_colors[i];
	c.R = p.r;
	c.G = p.g;
	c.B = p.b;
	c.A = p.a;

	// JL: TODO note: Well, this can be clearly done much more efficiently
	// but...I don't have time! :-(
	markAllPointsAsNew();
}

/** Inserts a new point into the point cloud. */
void CPointCloudColoured::push_back(
	float x, float y, float z, float R, float G, float B, float A)
{
	m_points.emplace_back(x, y, z);
	m_point_colors.emplace_back(f2u8(R), f2u8(G), f2u8(B), f2u8(A));

	// JL: TODO note: Well, this can be clearly done much more efficiently
	// but...I don't have time! :-(
	markAllPointsAsNew();
}

// Do needed internal work if all points are new (octree rebuilt,...)
void CPointCloudColoured::markAllPointsAsNew() { octree_mark_as_outdated(); }
/** In a base class, reserve memory to prepare subsequent calls to
 * PLY_import_set_vertex */
void CPointCloudColoured::PLY_import_set_vertex_count(const size_t N)
{
	this->resize(N);
}

/** In a base class, will be called after PLY_import_set_vertex_count() once for
 * each loaded point.
 *  \param pt_color Will be nullptr if the loaded file does not provide color
 * info.
 */
void CPointCloudColoured::PLY_import_set_vertex(
	const size_t idx, const mrpt::math::TPoint3Df& pt,
	const mrpt::img::TColorf* pt_color)
{
	if (!pt_color)
		this->setPoint(
			idx, TPointXYZfRGBAu8(pt.x, pt.y, pt.z, 0xff, 0xff, 0xff));
	else
		this->setPoint(
			idx, TPointXYZfRGBAu8(
					 pt.x, pt.y, pt.z, f2u8(pt_color->R), f2u8(pt_color->G),
					 f2u8(pt_color->B)));
}

/** In a base class, return the number of vertices */
size_t CPointCloudColoured::PLY_export_get_vertex_count() const
{
	return this->size();
}

/** In a base class, will be called after PLY_export_get_vertex_count() once for
 * each exported point.
 *  \param pt_color Will be nullptr if the loaded file does not provide color
 * info.
 */
void CPointCloudColoured::PLY_export_get_vertex(
	const size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color,
	mrpt::img::TColorf& pt_color) const
{
	auto& p = m_points[idx];
	auto& p_color = m_point_colors[idx];
	p = pt;
	p_color = pt_color.asTColor();
	pt_has_color = true;
}

void CPointCloudColoured::recolorizeByCoordinate(
	const float coord_min, const float coord_max, const int coord_index,
	const mrpt::img::TColormap color_map)
{
	ASSERT_ABOVEEQ_(coord_index, 0);
	ASSERT_BELOW_(coord_index, 3);

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
		};
		const float col_idx =
			std::max(0.0f, std::min(1.0f, (coord - coord_min) * coord_range_1));
		float r, g, b;
		mrpt::img::colormap(color_map, col_idx, r, g, b);
		this->setPointColor_fast(i, r, g, b);
	}
}
