/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CTexturedPlane, CRenderizable, mrpt::opengl)

CTexturedPlane::CTexturedPlane(
	float x_min, float x_max, float y_min, float y_max)
{
	CRenderizableShaderTriangles::enableLight(false);
	CRenderizableShaderTexturedTriangles::enableLight(false);

	setPlaneCorners(x_min, x_max, y_min, y_max);
}

void CTexturedPlane::render(const RenderContext& rc) const
{
	const bool hasTexture =
		textureImageHasBeenAssigned() && !getTextureImage().isEmpty();

	switch (rc.shader_id)
	{
		case DefaultShaderID::TRIANGLES:
			if (!hasTexture) CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::TEXTURED_TRIANGLES:
			if (hasTexture) CRenderizableShaderTexturedTriangles::render(rc);
			break;
	};
}

void CTexturedPlane::renderUpdateBuffers() const
{
	const bool hasTexture =
		textureImageHasBeenAssigned() && !getTextureImage().isEmpty();

	if (!hasTexture) CRenderizableShaderTriangles::renderUpdateBuffers();
	else
		CRenderizableShaderTexturedTriangles::renderUpdateBuffers();
}

void CTexturedPlane::onUpdateBuffers_TexturedTriangles()
{
	MRPT_START
	using P2f = mrpt::math::TPoint2Df;
	using P3f = mrpt::math::TPoint3Df;

	auto& tris = CRenderizableShaderTexturedTriangles::m_triangles;
	tris.clear();

	{
		TTriangle t;
		t.vertices[0].xyzrgba.pt = P3f(m_xMin, m_yMin, 0);
		t.vertices[1].xyzrgba.pt = P3f(m_xMax, m_yMin, 0);
		t.vertices[2].xyzrgba.pt = P3f(m_xMax, m_yMax, 0);

		t.vertices[0].uv = P2f(0, 0);
		t.vertices[1].uv = P2f(1, 0);
		t.vertices[2].uv = P2f(1, 1);

		tris.emplace_back(t);
	}
	{
		TTriangle t;
		t.vertices[0].xyzrgba.pt = P3f(m_xMin, m_yMin, 0);
		t.vertices[1].xyzrgba.pt = P3f(m_xMax, m_yMax, 0);
		t.vertices[2].xyzrgba.pt = P3f(m_xMin, m_yMax, 0);

		t.vertices[0].uv = P2f(0, 0);
		t.vertices[1].uv = P2f(1, 1);
		t.vertices[2].uv = P2f(0, 1);

		tris.emplace_back(t);
	}

	MRPT_END
}

void CTexturedPlane::onUpdateBuffers_Triangles()
{
	MRPT_START
	using P3f = mrpt::math::TPoint3Df;

	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	TTriangle t;
	for (int i = 0; i < 3; i++)
	{
		t.vertices[i].xyzrgba.r = this->m_color.R;
		t.vertices[i].xyzrgba.g = this->m_color.G;
		t.vertices[i].xyzrgba.b = this->m_color.B;
		t.vertices[i].xyzrgba.a = this->m_color.A;
	}

	t.vertices[0].xyzrgba.pt = P3f(m_xMin, m_yMin, 0);
	t.vertices[1].xyzrgba.pt = P3f(m_xMax, m_yMin, 0);
	t.vertices[2].xyzrgba.pt = P3f(m_xMax, m_yMax, 0);

	tris.emplace_back(t);

	t.vertices[0].xyzrgba.pt = P3f(m_xMin, m_yMin, 0);
	t.vertices[1].xyzrgba.pt = P3f(m_xMax, m_yMax, 0);
	t.vertices[2].xyzrgba.pt = P3f(m_xMin, m_yMax, 0);

	tris.emplace_back(t);

	MRPT_END
}

uint8_t CTexturedPlane::serializeGetVersion() const { return 2; }
void CTexturedPlane::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);

	out << m_xMin << m_xMax;
	out << m_yMin << m_yMax;

	writeToStreamTexturedObject(out);
}

void CTexturedPlane::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
			THROW_EXCEPTION("Deserialization of old formats not supported.");
			break;
		case 2:
		{
			readFromStreamRender(in);
			in >> m_xMin >> m_xMax;
			in >> m_yMin >> m_yMax;
			readFromStreamTexturedObject(in);
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

bool CTexturedPlane::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	if (!polygonUpToDate) updatePoly();
	return math::traceRay(tmpPoly, (o - this->m_pose).asTPose(), dist);
}

void CTexturedPlane::updatePoly() const
{
	TPolygon3D poly(4);
	poly[0].x = poly[1].x = m_xMin;
	poly[2].x = poly[3].x = m_xMax;
	poly[0].y = poly[3].y = m_yMin;
	poly[1].y = poly[2].y = m_yMax;
	for (size_t i = 0; i < 4; i++)
		poly[i].z = 0;
	tmpPoly.resize(1);
	tmpPoly[0] = poly;
	polygonUpToDate = true;
}

auto CTexturedPlane::getBoundingBox() const -> mrpt::math::TBoundingBox
{
	return mrpt::math::TBoundingBox(
			   mrpt::math::TPoint3D(m_xMin, m_yMin, 0),
			   mrpt::math::TPoint3D(m_xMax, m_yMax, 0))
		.compose(m_pose);
}
