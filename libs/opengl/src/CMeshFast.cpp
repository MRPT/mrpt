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
#include <Eigen/Dense>	// First! to avoid conflicts with X.h
//
#include <mrpt/img/color_maps.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/opengl/CMeshFast.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::img;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CMeshFast, CRenderizable, mrpt::opengl)

void CMeshFast::updatePoints() const
{
	CRenderizable::notifyChange();

	const auto cols = Z.cols();
	const auto rows = Z.rows();

	if ((m_colorFromZ) || (m_isImage)) updateColorsMatrix();

	ASSERT_((cols > 0) && (rows > 0));
	ASSERT_((xMax > xMin) && (yMax > yMin));
	X.setSize(rows, cols);
	Y.setSize(rows, cols);
	const float sCellX = (xMax - xMin) / (rows - 1);
	const float sCellY = (yMax - yMin) / (cols - 1);

	for (int iX = 0; iX < rows; iX++)
		for (int iY = 0; iY < cols; iY++)
		{
			X(iX, iY) = xMin + iX * sCellX;
			Y(iX, iY) = yMin + iY * sCellY;
		}

	pointsUpToDate = true;
}

void CMeshFast::onUpdateBuffers_Points()
{
	using mrpt::img::TColor;
	using mrpt::img::TColorf;

	if (!pointsUpToDate) updatePoints();

	ASSERT_EQUAL_(X.size(), Y.size());
	ASSERT_EQUAL_(X.size(), Z.size());

	auto& vbd = CRenderizableShaderPoints::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;
	std::unique_lock<std::shared_mutex> wfWriteLock(
		CRenderizableShaderPoints::m_pointsMtx.data);

	vbd.clear();
	cbd.clear();

	for (int i = 0; i < X.rows(); i++)
	{
		for (int j = 0; j < X.cols(); j++)
		{
			TColor col;

			if (m_isImage && m_textureImage.isColor())
				col = TColor(C_r(i, j), C_g(i, j), C_b(i, j), m_color.A);
			else if (m_isImage)
				col = TColor(C(i, j), C(i, j), C(i, j), m_color.A);
			else if (m_colorFromZ)
			{
				float rz, gz, bz;
				colormap(m_colorMap, C(i, j) / 255.0f, rz, gz, bz);
				col = TColorf(rz, gz, bz, m_color.A / 255.f).asTColor();
			}
			else
				col = m_color;

			cbd.emplace_back(col);
			vbd.emplace_back(X(i, j), Y(i, j), Z(i, j));
		}
	}
}

void CMeshFast::assignImage(const CImage& img)
{
	MRPT_START

	// Make a copy:
	m_textureImage = img;

	// Delete content in Z
	Z.setZero(img.getHeight(), img.getWidth());

	// Update flags/states
	m_modified_Image = true;
	m_enableTransparency = false;
	m_colorFromZ = false;
	m_isImage = true;
	pointsUpToDate = false;

	CRenderizable::notifyChange();

	MRPT_END
}

void CMeshFast::assignImageAndZ(
	const CImage& img, const mrpt::math::CMatrixDynamic<float>& in_Z)
{
	MRPT_START

	ASSERT_(
		(img.getWidth() == static_cast<size_t>(in_Z.cols())) &&
		(img.getHeight() == static_cast<size_t>(in_Z.rows())));

	Z = in_Z;

	// Make a copy:
	m_textureImage = img;

	// Update flags/states
	m_modified_Image = true;
	m_enableTransparency = false;
	m_colorFromZ = false;
	m_isImage = true;
	pointsUpToDate = false;

	CRenderizable::notifyChange();

	MRPT_END
}

uint8_t CMeshFast::serializeGetVersion() const { return 0; }
void CMeshFast::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);

	out << m_textureImage;
	out << m_isImage;
	out << xMin << xMax << yMin << yMax;
	out << X << Y << Z;	 // We don't need to serialize C, it's computed
	out << m_enableTransparency;
	out << m_colorFromZ;
	out << int16_t(m_colorMap);
	out << m_pointSize;
}

void CMeshFast::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);

			in >> m_textureImage;
			in >> m_isImage;

			in >> xMin;
			in >> xMax;
			in >> yMin;
			in >> yMax;

			in >> X >> Y >> Z;
			in >> m_enableTransparency;
			in >> m_colorFromZ;

			int16_t i;
			in >> i;
			m_colorMap = TColormap(i);
			in >> m_pointSize;
			m_modified_Z = true;
		}

			pointsUpToDate = false;
			break;

		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CMeshFast::updateColorsMatrix() const
{
	if (!m_modified_Z && !m_modified_Image) return;

	CRenderizable::notifyChange();

	if (m_isImage)
	{
		const int cols = m_textureImage.getWidth();
		const int rows = m_textureImage.getHeight();

		ASSERT_EQUAL_(cols, Z.cols());
		ASSERT_EQUAL_(rows, Z.rows());

		if (m_textureImage.isColor())
		{
			C_r.setSize(rows, cols);
			C_g.setSize(rows, cols);
			C_b.setSize(rows, cols);
			m_textureImage.getAsRGBMatrices(C_r, C_g, C_b);
		}
		else
		{
			C.setSize(rows, cols);
			m_textureImage.getAsMatrix(C);
		}
	}
	else
	{
		const size_t cols = Z.cols();
		const size_t rows = Z.rows();

		C.setSize(rows, cols);

		// Color is proportional to difference between height of a cell and
		//  the mean of the nearby cells MEANS:
		Eigen::MatrixXf Zf = Z.asEigen().cast<float>();
		mrpt::math::normalize(Zf, 0.01f, 0.99f);

		Zf *= 255;

		C = Zf.cast<uint8_t>();
	}

	m_modified_Image = false;
	m_modified_Z = false;  // Done
	pointsUpToDate = false;
}

void CMeshFast::setZ(const mrpt::math::CMatrixDynamic<float>& in_Z)
{
	Z = in_Z;
	m_modified_Z = true;
	pointsUpToDate = false;

	// Delete previously loaded images
	m_isImage = false;

	CRenderizable::notifyChange();
}

auto CMeshFast::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
	return verticesBoundingBox();
}
