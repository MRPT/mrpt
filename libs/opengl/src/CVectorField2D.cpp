/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/CVectorField2D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CVectorField2D, CRenderizable, mrpt::opengl)

/** Constructor */
CVectorField2D::CVectorField2D() : xcomp(0, 0), ycomp(0, 0)
{
	m_point_color = getColor_u8();
	m_field_color = getColor_u8();
}

/** Constructor with a initial set of lines. */
CVectorField2D::CVectorField2D(
	[[maybe_unused]] CMatrixFloat Matrix_x,
	[[maybe_unused]] CMatrixFloat Matrix_y, [[maybe_unused]] float xmin,
	[[maybe_unused]] float xmax, [[maybe_unused]] float ymin,
	[[maybe_unused]] float ymax)
{
	m_point_color = getColor_u8();
	m_field_color = getColor_u8();
}

void CVectorField2D::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::TRIANGLES_NO_LIGHT:
			CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			CRenderizableShaderWireFrame::render(rc);
			break;
		case DefaultShaderID::POINTS:
			CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CVectorField2D::renderUpdateBuffers() const
{
	CRenderizableShaderPoints::renderUpdateBuffers();
	CRenderizableShaderTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

void CVectorField2D::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	std::unique_lock<std::shared_mutex> wfWriteLock(
		CRenderizableShaderWireFrame::m_wireframeMtx.data);

	vbd.clear();

	vbd.reserve(xcomp.size() * 2);

	const float x_cell_size = (xMax - xMin) / (xcomp.cols() - 1);
	const float y_cell_size = (yMax - yMin) / (ycomp.rows() - 1);

	for (int i = 0; i < xcomp.cols(); i++)
		for (int j = 0; j < xcomp.rows(); j++)
		{
			vbd.emplace_back(xMin + i * x_cell_size, yMin + j * y_cell_size, 0);
			vbd.emplace_back(
				xMin + i * x_cell_size + xcomp(j, i),
				yMin + j * y_cell_size + ycomp(j, i), 0);
		}

	cbd.assign(vbd.size(), m_field_color);
}
void CVectorField2D::onUpdateBuffers_Triangles()
{
	using P3f = mrpt::math::TPoint3Df;

	std::unique_lock<std::shared_mutex> trisWriteLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;

	tris.clear();

	tris.reserve(xcomp.size());

	const float x_cell_size = (xMax - xMin) / (xcomp.cols() - 1);
	const float y_cell_size = (yMax - yMin) / (ycomp.rows() - 1);

	for (int i = 0; i < xcomp.cols(); i++)
		for (int j = 0; j < xcomp.rows(); j++)
		{
			const float tri_side = 0.25f *
				sqrt(xcomp(j, i) * xcomp(j, i) + ycomp(j, i) * ycomp(j, i));
			const float ang = ::atan2f(ycomp(j, i), xcomp(j, i)) - 1.5708f;
			tris.emplace_back(
				P3f(-sin(ang) * 0.866f * tri_side + xMin + i * x_cell_size +
						xcomp(j, i),
					cos(ang) * 0.866f * tri_side + yMin + j * y_cell_size +
						ycomp(j, i),
					0),
				P3f(cos(ang) * 0.5f * tri_side + xMin + i * x_cell_size +
						xcomp(j, i),
					sin(ang) * 0.5f * tri_side + yMin + j * y_cell_size +
						ycomp(j, i),
					0),
				P3f(-cos(ang) * 0.5f * tri_side + xMin + i * x_cell_size +
						xcomp(j, i),
					-sin(ang) * 0.5f * tri_side + yMin + j * y_cell_size +
						ycomp(j, i),
					0));
		}

	for (auto& t : tris)
	{
		t.computeNormals();
		t.setColor(m_field_color);
	}
}
void CVectorField2D::onUpdateBuffers_Points()
{
	auto& vbd = CRenderizableShaderPoints::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;
	std::unique_lock<std::shared_mutex> wfWriteLock(
		CRenderizableShaderPoints::m_pointsMtx.data);

	vbd.clear();
	vbd.reserve(xcomp.size());

	const float x_cell_size = (xMax - xMin) / (xcomp.cols() - 1);
	const float y_cell_size = (yMax - yMin) / (ycomp.rows() - 1);

	for (int i = 0; i < xcomp.cols(); i++)
		for (int j = 0; j < xcomp.rows(); j++)
			vbd.emplace_back(
				xMin + i * x_cell_size, yMin + j * y_cell_size, .0f);

	cbd.assign(vbd.size(), m_point_color);
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
	 CSerializable objects
  ---------------------------------------------------------------*/
uint8_t CVectorField2D::serializeGetVersion() const { return 1; }
void CVectorField2D::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);

	out << xcomp << ycomp;
	out << xMin << xMax << yMin << yMax;
	out << m_lineWidth;
	out << m_pointSize;
	out << m_antiAliasing;
	out << m_point_color;
	out << m_field_color;
	CRenderizableShaderTriangles::params_serialize(out);  // v1
}

void CVectorField2D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
			readFromStreamRender(in);

			in >> xcomp >> ycomp;
			in >> xMin >> xMax >> yMin >> yMax;
			in >> m_lineWidth;
			in >> m_pointSize;
			in >> m_antiAliasing;
			in >> m_point_color;
			in >> m_field_color;

			if (version >= 1)
				CRenderizableShaderTriangles::params_deserialize(in);
			break;

		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version); break;
	};
	CRenderizable::notifyChange();
}

auto CVectorField2D::internalBoundingBoxLocal() const
	-> mrpt::math::TBoundingBoxf
{
	return verticesBoundingBox();
}

void CVectorField2D::adjustVectorFieldToGrid()
{
	ASSERT_(xcomp.size() > 0);

	const float ratio_xp =
		xcomp.maxCoeff() * (xcomp.cols() - 1) / (xMax - xMin);
	const float ratio_xn =
		xcomp.minCoeff() * (xcomp.cols() - 1) / (xMax - xMin);
	const float ratio_yp =
		ycomp.maxCoeff() * (ycomp.rows() - 1) / (yMax - yMin);
	const float ratio_yn =
		ycomp.minCoeff() * (ycomp.rows() - 1) / (yMax - yMin);
	const float norm_factor = 0.85f /
		max(max(ratio_xp, std::abs(ratio_xn)),
			max(ratio_yp, std::abs(ratio_yn)));

	xcomp *= norm_factor;
	ycomp *= norm_factor;
	CRenderizable::notifyChange();
}
