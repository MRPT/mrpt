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
#include <mrpt/opengl/CMeshFast.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <Eigen/Dense>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::img;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CMeshFast, CRenderizableDisplayList, mrpt::opengl)

void CMeshFast::updatePoints() const
{
	CRenderizableDisplayList::notifyChange();

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

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CMeshFast::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT

	if (!pointsUpToDate) updatePoints();

	ASSERT_(X.size() == Y.size());
	ASSERT_(X.size() == Z.size());

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
	for (int i = 0; i < X.rows(); i++)
		for (int j = 0; j < X.cols(); j++)
		{
			if (m_isImage && m_textureImage.isColor())
				glColor4f(C_r(i, j), C_g(i, j), C_b(i, j), m_color.A / 255.f);

			else if (m_isImage)
				glColor4f(C(i, j), C(i, j), C(i, j), m_color.A / 255.f);

			else if (m_colorFromZ)
			{
				float rz, gz, bz;
				colormap(m_colorMap, C(i, j), rz, gz, bz);
				glColor4f(rz, gz, bz, m_color.A / 255.f);
			}

			else
				glColor4f(
					m_color.R / 255.f, m_color.G / 255.f, m_color.B / 255.f,
					m_color.A / 255.f);

			glVertex3f(X(i, j), Y(i, j), Z(i, j));
		}

	glEnd();

	glEnable(GL_LIGHTING);

	// Undo flags:
	if (m_color.A != 255) glDisable(GL_BLEND);

	if (m_pointSmooth) glDisable(GL_POINT_SMOOTH);

	CHECK_OPENGL_ERROR();
#endif
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

	CRenderizableDisplayList::notifyChange();

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

	CRenderizableDisplayList::notifyChange();

	MRPT_END
}

uint8_t CMeshFast::serializeGetVersion() const { return 0; }
void CMeshFast::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);

	out << m_textureImage;
	out << m_isImage;
	out << xMin << xMax << yMin << yMax;
	out << X << Y << Z;  // We don't need to serialize C, it's computed
	out << m_enableTransparency;
	out << m_colorFromZ;
	out << int16_t(m_colorMap);
	out << m_pointSize;
	out << m_pointSmooth;
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
			in >> m_pointSmooth;
			m_modified_Z = true;
		}

			pointsUpToDate = false;
			break;

		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizableDisplayList::notifyChange();
}

void CMeshFast::updateColorsMatrix() const
{
	if ((!m_modified_Z) && (!m_modified_Image)) return;

	CRenderizableDisplayList::notifyChange();

	if (m_isImage)
	{
		const int cols = m_textureImage.getWidth();
		const int rows = m_textureImage.getHeight();

		if ((cols != Z.cols()) || (rows != Z.rows()))
		{
			printf("\nTexture Image and Z sizes have to be equal");
		}
		else if (m_textureImage.isColor())
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
		C = Z;

		mrpt::math::normalize(C, 0.01f, 0.99f);
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

	CRenderizableDisplayList::notifyChange();
}

void CMeshFast::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = xMin;
	bb_min.y = yMin;
	bb_min.z = Z.minCoeff();

	bb_max.x = xMax;
	bb_max.y = yMax;
	bb_max.z = Z.maxCoeff();

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
