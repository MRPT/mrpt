/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPlanarLaserScan, CRenderizable, mrpt::opengl)

void CPlanarLaserScan::clear()
{
	CRenderizable::notifyChange();
	m_scan.resizeScan(0);
}

void CPlanarLaserScan::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::TRIANGLES_NO_LIGHT:
			if (m_enable_surface) CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			if (m_enable_line) CRenderizableShaderWireFrame::render(rc);
			break;
		case DefaultShaderID::POINTS:
			if (m_enable_points) CRenderizableShaderPoints::render(rc);
			break;
	};
}
void CPlanarLaserScan::renderUpdateBuffers() const
{
	// Load into cache:
	if (!m_cache_valid)
	{
		m_cache_valid = true;
		m_cache_points.clear();
		m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
		m_cache_points.insertionOptions.isPlanarMap = false;

		m_cache_points.insertObservation(m_scan);
	}

	CRenderizableShaderPoints::renderUpdateBuffers();
	CRenderizableShaderTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

void CPlanarLaserScan::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	std::unique_lock<std::shared_mutex> wfWriteLock(
		CRenderizableShaderWireFrame::m_wireframeMtx.data);

	vbd.clear();
	cbd.clear();

	size_t n;
	const float *x, *y, *z;
	m_cache_points.getPointsBuffer(n, x, y, z);
	if (!n) return;

	for (size_t i = 0; i < n - 1; i++)
	{
		vbd.emplace_back(x[i], y[i], z[i]);
		vbd.emplace_back(x[i + 1], y[i + 1], z[i + 1]);
	}

	cbd.assign(
		vbd.size(),
		mrpt::img::TColorf(m_line_R, m_line_G, m_line_B, m_line_A).asTColor());
}

void CPlanarLaserScan::onUpdateBuffers_Triangles()
{
	std::unique_lock<std::shared_mutex> trisWriteLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;

	tris.clear();

	size_t n;
	const float *x, *y, *z;
	m_cache_points.getPointsBuffer(n, x, y, z);
	if (!n) return;

	using P3f = mrpt::math::TPoint3Df;

	for (size_t i = 0; i < n - 1; i++)
	{
		tris.emplace_back(
			P3f(m_scan.sensorPose.x(), m_scan.sensorPose.y(),
				m_scan.sensorPose.z()),
			P3f(x[i], y[i], z[i]), P3f(x[i + 1], y[i + 1], z[i + 1]));
	}

	for (auto& t : tris)
	{
		t.computeNormals();
		t.setColor(
			mrpt::img::TColorf(m_plane_R, m_plane_G, m_plane_B, m_plane_A));
	}
}

void CPlanarLaserScan::onUpdateBuffers_Points()
{
	auto& vbd = CRenderizableShaderPoints::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;
	std::unique_lock<std::shared_mutex> wfWriteLock(
		CRenderizableShaderPoints::m_pointsMtx.data);

	vbd.clear();

	size_t n;
	const float *x, *y, *z;
	m_cache_points.getPointsBuffer(n, x, y, z);

	for (size_t i = 0; i < n; i++)
		vbd.emplace_back(x[i], y[i], z[i]);

	cbd.assign(
		vbd.size(),
		mrpt::img::TColorf(m_points_R, m_points_G, m_points_B, m_points_A)
			.asTColor());
}

uint8_t CPlanarLaserScan::serializeGetVersion() const { return 3; }
void CPlanarLaserScan::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_scan;
	out << m_line_R << m_line_G << m_line_B << m_line_A << m_points_R
		<< m_points_G << m_points_B << m_points_A << m_plane_R << m_plane_G
		<< m_plane_B << m_plane_A << m_enable_points << m_enable_line
		<< m_enable_surface;  // new in v1
	CRenderizableShaderTriangles::params_serialize(out);  // v3
}

void CPlanarLaserScan::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			readFromStreamRender(in);
			in >> m_scan;

			if (version < 2)
			{  //  m_line_width
				float dummy;
				in >> dummy;
			}

			in >> m_line_R >> m_line_G >> m_line_B >> m_line_A;

			if (version < 2)
			{  // m_points_width
				float dummy;
				in >> dummy;
			}
			in >> m_points_R >> m_points_G >> m_points_B >> m_points_A >>
				m_plane_R >> m_plane_G >> m_plane_B >> m_plane_A;

			if (version >= 1)
			{
				in >> m_enable_points >> m_enable_line >>
					m_enable_surface;  // new in v1
			}
			else
			{
				m_enable_points = m_enable_line = m_enable_surface = true;
			}
			if (version >= 3)
				CRenderizableShaderTriangles::params_deserialize(in);
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

auto CPlanarLaserScan::internalBoundingBoxLocal() const
	-> mrpt::math::TBoundingBoxf
{
	// Load into cache:
	if (!m_cache_valid)
	{
		m_cache_valid = true;
		m_cache_points.clear();
		m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
		m_cache_points.insertionOptions.isPlanarMap = false;

		m_cache_points.insertObservation(m_scan);
	}

	if (m_cache_points.empty()) return {};

	return m_cache_points.boundingBox();
}

mrpt::math::TPoint3Df CPlanarLaserScan::getLocalRepresentativePoint() const
{
	return {
		d2f(m_scan.sensorPose.x()), d2f(m_scan.sensorPose.y()),
		d2f(m_scan.sensorPose.z())};
}
