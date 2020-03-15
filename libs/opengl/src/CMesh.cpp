/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/img/color_maps.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;
using mrpt::img::CImage;

IMPLEMENTS_SERIALIZABLE(CMesh, CRenderizable, mrpt::opengl)

CMesh::CMesh(
	bool enableTransparency, float m_xMin_p, float m_xMax_p, float m_yMin_p,
	float m_yMax_p)
	: m_enableTransparency(enableTransparency),
	  Z(0, 0),
	  mask(0, 0),
	  C(0, 0),
	  C_r(0, 0),
	  C_g(0, 0),
	  C_b(0, 0),
	  m_xMin(m_xMin_p),
	  m_xMax(m_xMax_p),
	  m_yMin(m_yMin_p),
	  m_yMax(m_yMax_p)
{
	enableTextureLinearInterpolation(true);
	enableLight(true);

	m_color.A = 255;
	m_color.R = 0;
	m_color.G = 0;
	m_color.B = 150;
}

CMesh::~CMesh() = default;

void CMesh::updateTriangles() const
{
	using mrpt::img::colormap;

	CRenderizable::notifyChange();

	// Remember:
	/** List of triangles in the mesh */
	// mutable
	// std::vector<std::pair<mrpt::opengl::TTriangle,TTriangleVertexIndices>
	// > actualMesh;

	/** The accumulated normals & counts for each vertex, so normals can be
	 * averaged. */
	// mutable std::vector<std::pair<mrpt::math::TPoint3D,size_t> >
	// vertex_normals;

	const auto cols = Z.cols();
	const auto rows = Z.rows();

	actualMesh.clear();
	if (cols == 0 && rows == 0) return;  // empty mesh

	ASSERT_(cols > 0 && rows > 0);
	ASSERT_ABOVE_(m_xMax, m_xMin);
	ASSERT_ABOVE_(m_yMax, m_yMin);

	// we have 1 more row & col of vertices than of triangles:
	vertex_normals.assign(
		(1 + cols) * (1 + rows),
		std::pair<TPoint3D, size_t>(TPoint3D(0, 0, 0), 0));

	if (m_colorFromZ || m_isImage) updateColorsMatrix();

	bool useMask = false;
	if (mask.cols() != 0 && mask.rows() != 0)
	{
		ASSERT_(mask.cols() == cols && mask.rows() == rows);
		useMask = true;
	}
	const float sCellX = (m_xMax - m_xMin) / (rows - 1);
	const float sCellY = (m_yMax - m_yMin) / (cols - 1);

	mrpt::opengl::TTriangle tri;

	for (int iX = 0; iX < rows - 1; iX++)
		for (int iY = 0; iY < cols - 1; iY++)
		{
			if (useMask && (!mask(iX, iY) || !mask(iX + 1, iY + 1))) continue;
			tri.x(0) = m_xMin + iX * sCellX;
			tri.y(0) = m_yMin + iY * sCellY;
			tri.z(0) = Z(iX, iY);
			tri.x(2) = tri.x(0) + sCellX;
			tri.y(2) = tri.y(0) + sCellY;
			tri.z(2) = Z(iX + 1, iY + 1);

			// Vertex indices:
			TTriangleVertexIndices tvi;
			tvi.vind[0] = iX + rows * iY;
			tvi.vind[2] = (iX + 1) + rows * (iY + 1);

			// Each quadrangle has up to 2 triangles:
			//  [0]
			//   |
			//   |
			//  [1]--[2]
			// Order: 0,1,2
			if (!useMask || mask(iX + 1, iY))
			{
				tri.x(1) = tri.x(2);
				tri.y(1) = tri.y(0);
				tri.z(1) = Z(iX + 1, iY);
				// Assign alpha channel
				for (int i = 0; i < 3; i++) tri.a(i) = m_color.A;

				if (m_colorFromZ)
				{
					mrpt::img::TColorf col(0, 0, 0, 1);
					colormap(m_colorMap, C(iX, iY), col.R, col.G, col.B);
					tri.r(0) = f2u8(col.R);
					tri.g(0) = f2u8(col.G);
					tri.b(0) = f2u8(col.B);
					colormap(m_colorMap, C(iX + 1, iY), col.R, col.G, col.B);
					tri.r(1) = f2u8(col.R);
					tri.g(1) = f2u8(col.G);
					tri.b(1) = f2u8(col.B);
					colormap(
						m_colorMap, C(iX + 1, iY + 1), col.R, col.G, col.B);
					tri.r(2) = f2u8(col.R);
					tri.g(2) = f2u8(col.G);
					tri.b(2) = f2u8(col.B);
				}
				else if (m_isImage)
				{
					if (getTextureImage().isColor())
					{
						tri.r(0) = tri.r(1) = tri.r(2) = C_r(iX, iY);
						tri.g(0) = tri.g(1) = tri.g(2) = C_g(iX, iY);
						tri.b(0) = tri.b(1) = tri.b(2) = C_b(iX, iY);
					}
					else
					{
						tri.r(0) = tri.r(1) = tri.r(2) = C(iX, iY);
						tri.g(0) = tri.g(1) = tri.g(2) = C(iX, iY);
						tri.b(0) = tri.b(1) = tri.b(2) = C(iX, iY);
					}
				}
				else
				{
					tri.r(0) = tri.r(1) = tri.r(2) = m_color.R / 255.f;
					tri.g(0) = tri.g(1) = tri.g(2) = m_color.G / 255.f;
					tri.b(0) = tri.b(1) = tri.b(2) = m_color.B / 255.f;
				}

				// Compute normal of this triangle, and add it up to the 3
				// neighboring vertices:
				// A = P1 - P0, B = P2 - P0
				float ax = tri.x(1) - tri.x(0);
				float bx = tri.x(2) - tri.x(0);
				float ay = tri.y(1) - tri.y(0);
				float by = tri.y(2) - tri.y(0);
				float az = tri.z(1) - tri.z(0);
				float bz = tri.z(2) - tri.z(0);
				const TPoint3D this_normal(
					ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);

				// Vertex indices:
				tvi.vind[1] = iX + 1 + rows * iY;

				// Add triangle:
				actualMesh.emplace_back(tri, tvi);

				// For averaging normals:
				for (unsigned long k : tvi.vind)
				{
					vertex_normals[k].first += this_normal;
					vertex_normals[k].second++;
				}
			}
			// 2:
			//  [0]--[1->2]
			//     \  |
			//       \|
			//       [2->1]
			// Order: 0,2,1
			if (!useMask || mask(iX, iY + 1))
			{
				tri.x(1) = tri.x(2);
				tri.y(1) = tri.y(2);
				tri.z(1) = tri.z(2);

				tri.x(2) = tri.x(0);
				// tri.y(2)=tri.y(1);
				tri.z(2) = Z(iX, iY + 1);
				if (m_colorFromZ)
				{
					mrpt::img::TColorf col(0, 0, 0, 1);

					colormap(m_colorMap, C(iX, iY), col.R, col.G, col.B);
					tri.r(0) = f2u8(col.R);
					tri.g(0) = f2u8(col.G);
					tri.b(0) = f2u8(col.B);
					colormap(
						m_colorMap, C(iX + 1, iY + 1), col.R, col.G, col.B);
					tri.r(1) = f2u8(col.R);
					tri.g(1) = f2u8(col.G);
					tri.b(1) = f2u8(col.B);
					colormap(m_colorMap, C(iX, iY + 1), col.R, col.G, col.B);
					tri.r(2) = f2u8(col.R);
					tri.g(2) = f2u8(col.G);
					tri.b(2) = f2u8(col.B);
				}
				else if (m_isImage)
				{
					if (getTextureImage().isColor())
					{
						tri.r(0) = tri.r(1) = tri.r(2) = C_r(iX, iY);
						tri.g(0) = tri.g(1) = tri.g(2) = C_g(iX, iY);
						tri.b(0) = tri.b(1) = tri.b(2) = C_b(iX, iY);
					}
					else
					{
						tri.r(0) = tri.r(1) = tri.r(2) = C(iX, iY);
						tri.g(0) = tri.g(1) = tri.g(2) = C(iX, iY);
						tri.b(0) = tri.b(1) = tri.b(2) = C(iX, iY);
					}
				}
				else
				{
					tri.r(0) = tri.r(1) = tri.r(2) = m_color.R / 255.f;
					tri.g(0) = tri.g(1) = tri.g(2) = m_color.G / 255.f;
					tri.b(0) = tri.b(1) = tri.b(2) = m_color.B / 255.f;
				}

				// Compute normal of this triangle, and add it up to the 3
				// neighboring vertices:
				// A = P1 - P0, B = P2 - P0
				float ax = tri.x(1) - tri.x(0);
				float bx = tri.x(2) - tri.x(0);
				float ay = tri.y(1) - tri.y(0);
				float by = tri.y(2) - tri.y(0);
				float az = tri.z(1) - tri.z(0);
				float bz = tri.z(2) - tri.z(0);
				const TPoint3D this_normal(
					ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);

				// Vertex indices:
				tvi.vind[1] = tvi.vind[2];
				tvi.vind[2] = iX + rows * (iY + 1);

				// Add triangle:
				actualMesh.emplace_back(tri, tvi);

				// For averaging normals:
				for (unsigned long k : tvi.vind)
				{
					vertex_normals[k].first += this_normal;
					vertex_normals[k].second++;
				}
			}
		}

	// Average normals:
	for (auto& vertex_normal : vertex_normals)
	{
		const size_t N = vertex_normal.second;
		if (N > 0)
		{
			vertex_normal.first *= 1.0 / N;
			vertex_normal.first = vertex_normal.first.unitarize();
		}
	}

	m_trianglesUpToDate = true;
	m_polygonsUpToDate = false;
}

void CMesh::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::TEXTURED_TRIANGLES:
			if (!m_isWireFrame)
				CRenderizableShaderTexturedTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			if (m_isWireFrame) CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CMesh::renderUpdateBuffers() const
{
	if (!m_trianglesUpToDate) updateTriangles();

	CRenderizableShaderTexturedTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

void CMesh::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();
	cbd.clear();

	for (auto& i : actualMesh)
	{
		const mrpt::opengl::TTriangle& t = i.first;

		for (int kk = 0; kk <= 3; kk++)
		{
			int k = kk % 3;
			int k1 = (kk + 1) % 3;

			vbd.emplace_back(t.x(k), t.y(k), t.z(k));
			cbd.emplace_back(t.r(k), t.g(k), t.b(k), t.a(k));

			vbd.emplace_back(t.x(k1), t.y(k1), t.z(k1));
			cbd.emplace_back(t.r(k1), t.g(k1), t.b(k1), t.a(k1));
		}
	}
}

void CMesh::onUpdateBuffers_TexturedTriangles()
{
	auto& tris = CRenderizableShaderTexturedTriangles::m_triangles;
	tris.clear();

	for (auto& i : actualMesh)
	{
		const mrpt::opengl::TTriangle& t = i.first;
		const TTriangleVertexIndices& tvi = i.second;

		const auto& n0 = vertex_normals.at(tvi.vind[0]).first;
		const auto& n1 = vertex_normals.at(tvi.vind[1]).first;
		const auto& n2 = vertex_normals.at(tvi.vind[2]).first;

		auto tri = t;

		tri.vertices[0].normal = n0;
		tri.vertices[1].normal = n1;
		tri.vertices[2].normal = n2;

		for (int k = 0; k < 3; k++)
		{
			tri.vertices[k].uv.x =
				(tri.vertices[k].xyzrgba.pt.x - m_xMin) / (m_xMax - m_xMin);
			tri.vertices[k].uv.y =
				(tri.vertices[k].xyzrgba.pt.y - m_yMin) / (m_yMax - m_yMin);
		}

		tris.emplace_back(std::move(tri));
	}
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void CMesh::assignImage(const CImage& img)
{
	MRPT_START

	// Make a copy:
	CRenderizableShaderTexturedTriangles::assignImage(img);

	// Delete content in Z
	Z.setZero(img.getHeight(), img.getWidth());

	m_modified_Image = true;
	m_enableTransparency = false;
	m_colorFromZ = false;
	m_isImage = true;
	m_trianglesUpToDate = false;

	CRenderizable::notifyChange();

	MRPT_END
}

void CMesh::assignImageAndZ(
	const CImage& img, const mrpt::math::CMatrixDynamic<float>& in_Z)
{
	MRPT_START

	ASSERT_(
		(img.getWidth() == static_cast<size_t>(in_Z.cols())) &&
		(img.getHeight() == static_cast<size_t>(in_Z.rows())));

	Z = in_Z;

	// Load the texture:
	CRenderizableShaderTexturedTriangles::assignImage(img);

	m_modified_Image = true;
	m_enableTransparency = false;
	m_colorFromZ = false;
	m_isImage = true;
	m_trianglesUpToDate = false;

	CRenderizable::notifyChange();

	MRPT_END
}

uint8_t CMesh::serializeGetVersion() const { return 1; }
void CMesh::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	writeToStreamTexturedObject(out);

	// Version 0:
	out << m_xMin << m_xMax << m_yMin << m_yMax;
	out << Z << mask;  // We don't need to serialize C, it's computed
	out << m_enableTransparency;
	out << m_colorFromZ;
	// new in v1
	out << m_isWireFrame;
	out << int16_t(m_colorMap);
}

void CMesh::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			readFromStreamRender(in);
			readFromStreamTexturedObject(in);

			in >> m_xMin;
			in >> m_xMax;
			in >> m_yMin;
			in >> m_yMax;

			in >> Z >> mask;
			in >> m_enableTransparency;
			in >> m_colorFromZ;

			if (version >= 1)
			{
				in >> m_isWireFrame;
				int16_t i;
				in >> i;
				m_colorMap = mrpt::img::TColormap(i);
			}
			else
				m_isWireFrame = false;

			m_modified_Z = true;
		}
			m_trianglesUpToDate = false;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	m_trianglesUpToDate = false;
	CRenderizable::notifyChange();
}

void CMesh::updateColorsMatrix() const
{
	if ((!m_modified_Z) && (!m_modified_Image)) return;

	CRenderizable::notifyChange();

	if (m_isImage)
	{
		const int cols = getTextureImage().getWidth();
		const int rows = getTextureImage().getHeight();

		if ((cols != Z.cols()) || (rows != Z.rows()))
			printf("\nTexture Image and Z sizes have to be equal");

		else if (getTextureImage().isColor())
		{
			C_r.setSize(rows, cols);
			C_g.setSize(rows, cols);
			C_b.setSize(rows, cols);
			getTextureImage().getAsRGBMatrices(C_r, C_g, C_b);
		}
		else
		{
			C.setSize(rows, cols);
			getTextureImage().getAsMatrix(C);
		}
	}
	else
	{
		const size_t cols = Z.cols();
		const size_t rows = Z.rows();
		C.setSize(rows, cols);

		// Color is proportional to height:
		C = Z;

		// If mask is empty -> Normalize the whole mesh
		if (mask.empty()) mrpt::math::normalize(C, 0.01f, 0.99f);

		// Else -> Normalize color ignoring masked-out cells:
		else
		{
			float val_max = -std::numeric_limits<float>::max(),
				  val_min = std::numeric_limits<float>::max();
			bool any_valid = false;

			for (size_t c = 0; c < cols; c++)
				for (size_t r = 0; r < rows; r++)
				{
					if (!mask(r, c)) continue;
					any_valid = true;
					const float val = C(r, c);
					mrpt::keep_max(val_max, val);
					mrpt::keep_min(val_min, val);
				}

			if (any_valid)
			{
				float minMaxDelta = val_max - val_min;
				if (minMaxDelta == 0) minMaxDelta = 1;
				const float minMaxDelta_ = 1.0f / minMaxDelta;
				C.array() = (C.array() - val_min) * minMaxDelta_;
			}
		}
	}

	m_modified_Image = false;
	m_modified_Z = false;
	m_trianglesUpToDate = false;
}

void CMesh::setZ(const mrpt::math::CMatrixDynamic<float>& in_Z)
{
	Z = in_Z;
	m_modified_Z = true;
	m_trianglesUpToDate = false;

	// Delete previously loaded images
	m_isImage = false;

	CRenderizable::notifyChange();
}

void CMesh::setMask(const mrpt::math::CMatrixDynamic<float>& in_mask)
{
	mask = in_mask;
	m_trianglesUpToDate = false;
	CRenderizable::notifyChange();
}

bool CMesh::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	if (!m_trianglesUpToDate || !m_polygonsUpToDate) updatePolygons();
	return mrpt::math::traceRay(tmpPolys, (o - this->m_pose).asTPose(), dist);
}

static math::TPolygon3D tmpPoly(3);
mrpt::math::TPolygonWithPlane createPolygonFromTriangle(
	const std::pair<mrpt::opengl::TTriangle, CMesh::TTriangleVertexIndices>& p)
{
	const mrpt::opengl::TTriangle& t = p.first;
	for (size_t i = 0; i < 3; i++) tmpPoly[i] = t.vertex(i);
	return mrpt::math::TPolygonWithPlane(tmpPoly);
}

void CMesh::updatePolygons() const
{
	if (!m_trianglesUpToDate) updateTriangles();
	size_t N = actualMesh.size();
	tmpPolys.resize(N);
	transform(
		actualMesh.begin(), actualMesh.end(), tmpPolys.begin(),
		createPolygonFromTriangle);
	m_polygonsUpToDate = true;
	CRenderizable::notifyChange();
}

void CMesh::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = m_xMin;
	bb_min.y = m_yMin;
	bb_min.z = Z.minCoeff();

	bb_max.x = m_xMax;
	bb_max.y = m_yMax;
	bb_max.z = Z.maxCoeff();

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}

void CMesh::adjustGridToImageAR()
{
	ASSERT_(m_isImage);
	const float ycenter = 0.5f * (m_yMin + m_yMax);
	const float xwidth = m_xMax - m_xMin;
	const float newratio = float(getTextureImage().getWidth()) /
						   float(getTextureImage().getHeight());
	m_yMax = ycenter + 0.5f * newratio * xwidth;
	m_yMin = ycenter - 0.5f * newratio * xwidth;
	CRenderizable::notifyChange();
}
