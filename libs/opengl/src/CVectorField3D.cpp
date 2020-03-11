/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CVectorField3D.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CVectorField3D, CRenderizable, mrpt::opengl)

/** Constructor */
CVectorField3D::CVectorField3D()
	: x_vf(0, 0), y_vf(0, 0), z_vf(0, 0), x_p(0, 0), y_p(0, 0), z_p(0, 0)
{
	m_point_color = m_color;
	m_field_color = m_color;
	m_still_color = m_color;
	m_maxspeed_color = m_color;
	m_maxspeed = 1.f;
}

/** Constructor with a initial set of lines. */
CVectorField3D::CVectorField3D(
	CMatrixFloat x_vf_ini, CMatrixFloat y_vf_ini, CMatrixFloat z_vf_ini,
	CMatrixFloat x_p_ini, CMatrixFloat y_p_ini, CMatrixFloat z_p_ini)
	: m_colorFromModule(false), m_showPoints(true)
{
	x_vf = x_vf_ini;
	y_vf = y_vf_ini;
	z_vf = z_vf_ini;
	x_p = x_p_ini;
	y_p = y_p_ini;
	z_p = z_p_ini;
	m_point_color = m_color;
	m_field_color = m_color;
	m_still_color = m_color;
	m_maxspeed_color = m_color;
	m_maxspeed = 1.f;
}

void CVectorField3D::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::WIREFRAME:
			CRenderizableShaderWireFrame::render(rc);
			break;
		case DefaultShaderID::POINTS:
			if (m_showPoints) CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CVectorField3D::renderUpdateBuffers() const
{
	CRenderizableShaderPoints::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

void CVectorField3D::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();
	cbd.clear();

	vbd.reserve(x_vf.size());
	cbd.reserve(x_vf.size());

	for (int i = 0; i < x_vf.cols(); i++)
		for (int j = 0; j < x_vf.rows(); j++)
		{
			vbd.emplace_back(x_p(j, i), y_p(j, i), z_p(j, i));
			vbd.emplace_back(
				x_p(j, i) + x_vf(j, i), y_p(j, i) + y_vf(j, i),
				z_p(j, i) + z_vf(j, i));

			if (!m_colorFromModule)
			{
				cbd.emplace_back(m_field_color);
			}
			else
			{
				// Compute color
				mrpt::img::TColor col;
				const float module = sqrt(
					square(x_vf(j, i)) + square(y_vf(j, i)) +
					square(z_vf(j, i)));
				if (module > m_maxspeed)
					col = m_maxspeed_color;
				else
				{
					const float f = (m_maxspeed - module) / m_maxspeed;
					const float f2 = module / m_maxspeed;
					col = mrpt::img::TColorf(
							  f * m_still_color.R + f2 * m_maxspeed_color.R,
							  f * m_still_color.G + f2 * m_maxspeed_color.G,
							  f * m_still_color.B + f2 * m_maxspeed_color.B,
							  f * m_still_color.A + f2 * m_maxspeed_color.A)
							  .asTColor();
				}
				cbd.emplace_back(col);
			}
		}

	cbd.assign(vbd.size(), m_field_color);
}

void CVectorField3D::onUpdateBuffers_Points()
{
	auto& vbd = CRenderizableShaderPoints::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;

	vbd.clear();
	vbd.reserve(x_p.size());

	for (int i = 0; i < x_p.cols(); i++)
		for (int j = 0; j < x_p.rows(); j++)
			vbd.emplace_back(x_p(j, i), y_p(j, i), z_p(j, i));

	cbd.assign(vbd.size(), m_point_color);
}
uint8_t CVectorField3D::serializeGetVersion() const { return 0; }
void CVectorField3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);

	out << x_vf << y_vf << z_vf;
	out << x_p << y_p << z_p;
	out << m_lineWidth;
	out << m_pointSize;
	out << m_antiAliasing;
	out << m_point_color;
	out << m_field_color;
}
void CVectorField3D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
			readFromStreamRender(in);

			in >> x_vf >> y_vf >> z_vf;
			in >> x_p >> y_p >> z_p;
			in >> m_lineWidth;
			in >> m_pointSize;
			in >> m_antiAliasing;
			in >> m_point_color;
			in >> m_field_color;
			break;

		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
			break;
	};
	CRenderizable::notifyChange();
}

void CVectorField3D::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = 10e10;
	bb_min.y = 10e10;
	bb_min.z = 10e10;
	bb_max.x = -10e10;
	bb_max.y = -10e10;
	bb_max.z = -10e10;

	for (int i = 0; i < x_p.cols(); i++)
		for (int j = 0; j < x_p.rows(); j++)
		{
			// Minimum values
			if (x_p(j, i) < bb_min.x) bb_min.x = x_p(j, i);

			if (x_p(j, i) + x_vf(j, i) < bb_min.x)
				bb_min.x = x_p(j, i) + x_vf(j, i);

			if (y_p(j, i) < bb_min.y) bb_min.y = y_p(j, i);

			if (y_p(j, i) + y_vf(j, i) < bb_min.y)
				bb_min.y = y_p(j, i) + y_vf(j, i);

			if (z_p(j, i) < bb_min.z) bb_min.z = z_p(j, i);

			if (z_p(j, i) + z_vf(j, i) < bb_min.z)
				bb_min.z = z_p(j, i) + z_vf(j, i);

			// Maximum values
			if (x_p(j, i) > bb_max.x) bb_max.x = x_p(j, i);

			if (x_p(j, i) + x_vf(j, i) > bb_max.x)
				bb_max.x = x_p(j, i) + x_vf(j, i);

			if (y_p(j, i) > bb_max.y) bb_max.y = y_p(j, i);

			if (y_p(j, i) + y_vf(j, i) > bb_max.y)
				bb_max.y = y_p(j, i) + y_vf(j, i);

			if (z_p(j, i) > bb_max.z) bb_max.z = z_p(j, i);

			if (z_p(j, i) + z_vf(j, i) > bb_max.z)
				bb_max.z = z_p(j, i) + z_vf(j, i);
		}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
