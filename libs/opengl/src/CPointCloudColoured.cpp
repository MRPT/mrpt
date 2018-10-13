/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/core/round.h>  // round()
#include <mrpt/serialization/CArchive.h>
#include <mrpt/math/ops_containers.h>  // for << ops
#include <mrpt/serialization/stl_serialization.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;
using mrpt::serialization::CArchive;

IMPLEMENTS_SERIALIZABLE(CPointCloudColoured, CRenderizable, mrpt::opengl)

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CPointCloudColoured::render() const
{
#if MRPT_HAS_OPENGL_GLUT
	octree_assure_uptodate();  // Rebuild octree if needed
	m_last_rendered_count_ongoing = 0;

	// Info needed by octree renderer:
	gl_utils::TRenderInfo ri;
	gl_utils::getCurrentRenderingInfo(ri);

	if (m_color.A != 255)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	glPointSize(m_pointSize);

	if (m_pointSmooth)
		glEnable(GL_POINT_SMOOTH);
	else
		glDisable(GL_POINT_SMOOTH);

	// Disable lighting for point clouds:
	glDisable(GL_LIGHTING);

	glBegin(GL_POINTS);
	octree_render(ri);  // Render all points recursively:
	glEnd();

	glEnable(GL_LIGHTING);

	// Undo flags:
	if (m_color.A != 255) glDisable(GL_BLEND);

	if (m_pointSmooth) glDisable(GL_POINT_SMOOTH);

	m_last_rendered_count = m_last_rendered_count_ongoing;

	checkOpenGLError();
#endif
}

/** Render a subset of points (required by octree renderer) */
void CPointCloudColoured::render_subset(
	const bool all, const std::vector<size_t>& idxs,
	const float render_area_sqpixels) const
{
#if MRPT_HAS_OPENGL_GLUT
	const size_t N = all ? m_points.size() : idxs.size();
	const size_t decimation = mrpt::round(std::max(
		1.0f, static_cast<float>(
				  N / (mrpt::global_settings::
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
			glColor4f(p.R, p.G, p.B, m_color.A * 1.0f / 255.f);
			glVertex3f(p.x, p.y, p.z);
		}
	}
	else
	{
		for (size_t i = 0; i < N; i += decimation)
		{
			const TPointColour& p = m_points[idxs[i]];
			glColor4f(p.R, p.G, p.B, m_color.A * 1.0f / 255.f);
			glVertex3f(p.x, p.y, p.z);
		}
	}
#else
	MRPT_UNUSED_PARAM(all);
	MRPT_UNUSED_PARAM(idxs);
	MRPT_UNUSED_PARAM(render_area_sqpixels);
#endif
}

uint8_t CPointCloudColoured::serializeGetVersion() const { return 2; }
void CPointCloudColoured::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_points;
	out << m_pointSize;
	out << m_pointSmooth;  // Added in v2
}

void CPointCloudColoured::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 1:
		case 2:
		{
			readFromStreamRender(in);
			in >> m_points >> m_pointSize;

			if (version >= 2)
				in >> m_pointSmooth;
			else
				m_pointSmooth = false;
		}
		break;
		case 0:
		{
			readFromStreamRender(in);

			// Old vector_serializable:
			uint32_t n;
			in >> n;
			m_points.resize(n);
			for (uint32_t i = 0; i < n; i++) in >> m_points[i];

			in >> m_pointSize;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	markAllPointsAsNew();
}

CArchive& mrpt::opengl::operator>>(
	CArchive& in, CPointCloudColoured::TPointColour& o)
{
	in >> o.x >> o.y >> o.z >> o.R >> o.G >> o.B;
	return in;
}

CArchive& mrpt::opengl::operator<<(
	CArchive& out, const CPointCloudColoured::TPointColour& o)
{
	out << o.x << o.y << o.z << o.R << o.G << o.B;
	return out;
}

/** Write an individual point (checks for "i" in the valid range only in Debug).
 */
void CPointCloudColoured::setPoint(size_t i, const TPointColour& p)
{
#ifdef _DEBUG
	ASSERT_BELOW_(i, size());
#endif
	m_points[i] = p;

	// JL: TODO note: Well, this can be clearly done much more efficiently
	// but...I don't have time! :-(
	markAllPointsAsNew();
}

/** Inserts a new point into the point cloud. */
void CPointCloudColoured::push_back(
	float x, float y, float z, float R, float G, float B)
{
	m_points.push_back(TPointColour(x, y, z, R, G, B));

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
		this->setPoint(idx, TPointColour(pt.x, pt.y, pt.z, 1, 1, 1));
	else
		this->setPoint(
			idx, TPointColour(
					 pt.x, pt.y, pt.z, pt_color->R, pt_color->G, pt_color->B));
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
	const TPointColour& p = m_points[idx];
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
	pt_color.R = p.R;
	pt_color.G = p.G;
	pt_color.B = p.B;
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
