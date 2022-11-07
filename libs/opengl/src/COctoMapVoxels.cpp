/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(COctoMapVoxels, CRenderizable, mrpt::opengl)

/** Ctor */
COctoMapVoxels::COctoMapVoxels() : m_grid_color(0xE0, 0xE0, 0xE0, 0x90) {}
/** Clears everything */
void COctoMapVoxels::clear()
{
	m_voxel_sets.clear();
	m_grid_cubes.clear();

	CRenderizable::notifyChange();
}

void COctoMapVoxels::setBoundingBox(
	const mrpt::math::TPoint3D& bb_min, const mrpt::math::TPoint3D& bb_max)
{
	m_bb_min = bb_min;
	m_bb_max = bb_max;
}

void COctoMapVoxels::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::POINTS:
			if (m_showVoxelsAsPoints) CRenderizableShaderPoints::render(rc);
			break;
		case DefaultShaderID::TRIANGLES:
			if (!m_showVoxelsAsPoints) CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			if (m_show_grids) CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void COctoMapVoxels::renderUpdateBuffers() const
{
	CRenderizableShaderPoints::renderUpdateBuffers();
	CRenderizableShaderTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

// See: http://www.songho.ca/opengl/gl_vertexarray.html

// cube ///////////////////////////////////////////////////////////////////////
//        v6----- v5
// +Z    /|      /|
// A    v1------v0|
// |    | |     | |
// |    | |v7---|-|v4   / -X
// |    |/      |/     /
// |    v2------v3    L +X
// ----------------------> +Y
//

static const uint8_t grid_line_indices[] = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6,
											6, 7, 7, 4, 0, 5, 1, 6, 2, 7, 3, 4};

static const uint8_t cube_indices[2 * 6 * 3] = {
	// front:
	0, 1, 2, 0, 2, 3,
	// right:
	3, 4, 0, 4, 5, 0,
	// top
	0, 5, 6, 6, 1, 0,
	// left
	1, 6, 7, 7, 2, 1,
	// bottom
	7, 3, 4, 2, 3, 7,
	// back
	4, 7, 6, 5, 6, 4};

// normal array: one per triangle
static const mrpt::math::TPoint3Df normals_cube[6 * 2] = {
	{1, 0, 0},	{1, 0, 0},	// v0,v1,v2,v3 (front)
	{0, 1, 0},	{0, 1, 0},	// v0,v3,v4,v5 (right)
	{0, 0, 1},	{0, 0, 1},	// v0,v5,v6,v1 (top)
	{0, -1, 0}, {0, -1, 0},	 // v1,v6,v7,v2 (left)
	{0, 0, -1}, {0, 0, -1},	 // v7,v4,v3,v2 (bottom)
	{-1, 0, 0}, {-1, 0, 0}};  // v4,v7,v6,v5 (back)

void COctoMapVoxels::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	std::unique_lock<std::shared_mutex> wfWriteLock(
		CRenderizableShaderWireFrame::m_wireframeMtx.data);

	vbd.clear();

	CRenderizableShaderWireFrame::setLineWidth(m_grid_width);

	const size_t nGrids = m_grid_cubes.size();
	for (size_t i = 0; i < nGrids; i++)
	{
		const TGridCube& c = m_grid_cubes[i];

		const mrpt::math::TPoint3Df vs[8] = {
			{c.max.x, c.max.y, c.max.z}, {c.max.x, c.min.y, c.max.z},
			{c.max.x, c.min.y, c.min.z}, {c.max.x, c.max.y, c.min.z},
			{c.min.x, c.max.y, c.min.z}, {c.min.x, c.max.y, c.max.z},
			{c.min.x, c.min.y, c.max.z}, {c.min.x, c.min.y, c.min.z}};

		const auto& gli = grid_line_indices;

		// glDrawElements(GL_LINES, , GL_UNSIGNED_BYTE, grid_line_indices);
		for (size_t k = 0; k < sizeof(gli) / sizeof(gli[0]); k += 2)
		{
			vbd.emplace_back(vs[gli[k]]);
			vbd.emplace_back(vs[gli[k + 1]]);
		}
	}

	cbd.assign(vbd.size(), m_grid_color);
}

void COctoMapVoxels::onUpdateBuffers_Triangles()
{
	std::unique_lock<std::shared_mutex> trisWriteLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;

	tris.clear();

	for (const auto& m_voxel_set : m_voxel_sets)
	{
		if (!m_voxel_set.visible) continue;

		const std::vector<TVoxel>& voxels = m_voxel_set.voxels;
		const size_t N = voxels.size();
		for (size_t j = 0; j < N; j++)
		{
			const mrpt::img::TColor& vx_j_col = voxels[j].color;
			// Was: glColor4ub(vx_j_col.R, vx_j_col.G, vx_j_col.B, vx_j_col.A);

			const mrpt::math::TPoint3Df& c = voxels[j].coords;
			const float L = voxels[j].side_length * 0.5f;

			// Render as cubes:
			const mrpt::math::TPoint3Df vs[8] = {
				{c.x + L, c.y + L, c.z + L}, {c.x + L, c.y - L, c.z + L},
				{c.x + L, c.y - L, c.z - L}, {c.x + L, c.y + L, c.z - L},
				{c.x - L, c.y + L, c.z - L}, {c.x - L, c.y + L, c.z + L},
				{c.x - L, c.y - L, c.z + L}, {c.x - L, c.y - L, c.z - L}};

			// Was: glDrawElements(
			// GL_TRIANGLES, sizeof(cube_indices) / sizeof(cube_indices[0]),
			// GL_UNSIGNED_BYTE, cube_indices);
			// glDrawElements(GL_LINES, , GL_UNSIGNED_BYTE, grid_line_indices);

			const auto& ci = cube_indices;
			const auto& ns = normals_cube;

			for (size_t k = 0; k < sizeof(ci) / sizeof(ci[0]); k += 3)
			{
				mrpt::opengl::TTriangle tri(
					// vertices:
					vs[ci[k]], vs[ci[k + 1]], vs[ci[k + 2]],
					// normals:
					ns[k / 3], ns[k / 3], ns[k / 3]);

				for (int p = 0; p < 3; p++)
				{
					tri.vertices[p].xyzrgba.r = vx_j_col.R;
					tri.vertices[p].xyzrgba.g = vx_j_col.G;
					tri.vertices[p].xyzrgba.b = vx_j_col.B;
					tri.vertices[p].xyzrgba.a = vx_j_col.A;
				}

				tris.emplace_back(std::move(tri));
			}
		}
	}
}

void COctoMapVoxels::onUpdateBuffers_Points()
{
	auto& vbd = CRenderizableShaderPoints::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;
	std::unique_lock<std::shared_mutex> wfWriteLock(
		CRenderizableShaderPoints::m_pointsMtx.data);

	for (const auto& m_voxel_set : m_voxel_sets)
	{
		if (!m_voxel_set.visible) continue;

		const std::vector<TVoxel>& voxels = m_voxel_set.voxels;
		const size_t N = voxels.size();
		for (size_t j = 0; j < N; j++)
		{
			const mrpt::img::TColor& vx_j_col = voxels[j].color;
			const mrpt::math::TPoint3D& c = voxels[j].coords;

			vbd.emplace_back(c);
			cbd.emplace_back(vx_j_col);
		}
	}
}

DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TInfoPerVoxelSet)
DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TGridCube)
DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TVoxel)

namespace mrpt::opengl
{
using mrpt::serialization::CArchive;
CArchive& operator<<(CArchive& out, const COctoMapVoxels::TInfoPerVoxelSet& a)
{
	out << a.visible << a.voxels;
	return out;
}
CArchive& operator>>(CArchive& in, COctoMapVoxels::TInfoPerVoxelSet& a)
{
	in >> a.visible >> a.voxels;
	return in;
}

CArchive& operator<<(CArchive& out, const COctoMapVoxels::TGridCube& a)
{
	out << a.min << a.max;
	return out;
}
CArchive& operator>>(CArchive& in, COctoMapVoxels::TGridCube& a)
{
	in >> a.min >> a.max;
	return in;
}

CArchive& operator<<(CArchive& out, const COctoMapVoxels::TVoxel& a)
{
	out << a.coords << a.side_length << a.color;
	return out;
}
CArchive& operator>>(CArchive& in, COctoMapVoxels::TVoxel& a)
{
	in >> a.coords >> a.side_length >> a.color;
	return in;
}
}  // end of namespace mrpt::opengl

uint8_t COctoMapVoxels::serializeGetVersion() const { return 3; }
void COctoMapVoxels::serializeTo(CArchive& out) const
{
	writeToStreamRender(out);

	out << m_voxel_sets << m_grid_cubes << m_bb_min << m_bb_max
		<< m_enable_lighting << m_showVoxelsAsPoints << m_showVoxelsAsPointsSize
		<< m_show_grids << m_grid_width << m_grid_color
		<< m_enable_cube_transparency  // added in v1
		<< uint32_t(m_visual_mode);	 // added in v2
	CRenderizableShaderTriangles::params_serialize(out);  // v3
}

void COctoMapVoxels::serializeFrom(CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			readFromStreamRender(in);

			in >> m_voxel_sets >> m_grid_cubes >> m_bb_min >> m_bb_max >>
				m_enable_lighting >> m_showVoxelsAsPoints >>
				m_showVoxelsAsPointsSize >> m_show_grids >> m_grid_width >>
				m_grid_color;

			if (version >= 1) in >> m_enable_cube_transparency;
			else
				m_enable_cube_transparency = false;

			if (version >= 2)
			{
				uint32_t i;
				in >> i;
				m_visual_mode =
					static_cast<COctoMapVoxels::visualization_mode_t>(i);
			}
			else
				m_visual_mode = COctoMapVoxels::COLOR_FROM_OCCUPANCY;

			if (version >= 3)
				CRenderizableShaderTriangles::params_deserialize(in);
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};

	CRenderizable::notifyChange();
}

auto COctoMapVoxels::internalBoundingBoxLocal() const
	-> mrpt::math::TBoundingBoxf
{
	return {m_bb_min, m_bb_max};
}

bool sort_voxels_z(
	const COctoMapVoxels::TVoxel& a, const COctoMapVoxels::TVoxel& b)
{
	return a.coords.z < b.coords.z;
}

void COctoMapVoxels::sort_voxels_by_z()
{
	for (auto& m_voxel_set : m_voxel_sets)
	{
		std::sort(
			m_voxel_set.voxels.begin(), m_voxel_set.voxels.end(),
			&sort_voxels_z);
	}
}
