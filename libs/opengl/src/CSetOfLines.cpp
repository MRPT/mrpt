/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSetOfLines, CRenderizable, mrpt::opengl)

/** Constructor */
CSetOfLines::CSetOfLines() : mSegments() {}
/** Constructor with a initial set of lines. */
CSetOfLines::CSetOfLines(const std::vector<TSegment3D>& sgms, bool antiAliasing)
	: mSegments(sgms),
	  mLineWidth(1.0),
	  m_antiAliasing(antiAliasing),
	  m_verticesPointSize(.0f)
{
}

/*---------------------------------------------------------------
							setLineByIndex
  ---------------------------------------------------------------*/
void CSetOfLines::setLineByIndex(
	size_t index, const mrpt::math::TSegment3D& segm)
{
	MRPT_START
	if (index >= mSegments.size()) THROW_EXCEPTION("Index out of bounds");
	CRenderizable::notifyChange();
	mSegments[index] = segm;
	MRPT_END
}

float CSetOfLines::getVerticesPointSize() const { return m_verticesPointSize; }
void CSetOfLines::setVerticesPointSize(const float size_points)
{
	m_verticesPointSize = size_points;
	CRenderizable::notifyChange();
}

void CSetOfLines::renderUpdateBuffers() const
{
	//
	MRPT_TODO("Implement me!");
}

void CSetOfLines::render() const
{
#if MRPT_HAS_OPENGL_GLUT

	// Enable antialiasing:
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LINE_BIT);
	if (m_antiAliasing || m_color.A != 255)
	{
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
	}
	if (m_antiAliasing) glEnable(GL_LINE_SMOOTH);
	glLineWidth(mLineWidth);
	CHECK_OPENGL_ERROR();
	;

	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glBegin(GL_LINES);
	glColor4ub(m_color.R, m_color.G, m_color.B, m_color.A);
	for (const auto& mSegment : mSegments)
	{
		glVertex3d(mSegment.point1.x, mSegment.point1.y, mSegment.point1.z);
		glVertex3d(mSegment.point2.x, mSegment.point2.y, mSegment.point2.z);
	}
	glEnd();
	CHECK_OPENGL_ERROR();
	;

	// Draw vertices?
	if (m_verticesPointSize > 0)
	{
		glPointSize(m_verticesPointSize);
		if (m_antiAliasing)
			glEnable(GL_POINT_SMOOTH);
		else
			glDisable(GL_POINT_SMOOTH);

		glBegin(GL_POINTS);
		glColor4ub(m_color.R, m_color.G, m_color.B, m_color.A);
		bool first = true;
		for (const auto& seg : mSegments)
		{
			if (first)
			{
				glVertex3d(seg.point1.x, seg.point1.y, seg.point1.z);
				first = false;
			}
			glVertex3d(seg.point2.x, seg.point2.y, seg.point2.z);
		}

		glEnd();
	}

	glEnable(GL_LIGHTING);  // Disable lights when drawing lines

	// End of antialiasing:
	glPopAttrib();

#endif
}

uint8_t CSetOfLines::serializeGetVersion() const { return 4; }
void CSetOfLines::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << mSegments << mLineWidth;
	out << m_antiAliasing;  // Added in v3
	out << m_verticesPointSize;  // v4
}

void CSetOfLines::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			readFromStreamRender(in);
			CVectorFloat x0, y0, z0, x1, y1, z1;
			in >> x0 >> y0 >> z0 >> x1 >> y1 >> z1;
			if (version >= 1)
				in >> mLineWidth;
			else
				mLineWidth = 1;
			size_t N = x0.size();
			mSegments.resize(N);
			for (size_t i = 0; i < N; i++)
			{
				mSegments[i][0][0] = x0[i];
				mSegments[i][0][1] = y0[i];
				mSegments[i][0][2] = z0[i];
				mSegments[i][1][0] = x1[i];
				mSegments[i][1][1] = y1[i];
				mSegments[i][1][2] = z1[i];
			}
		}
		break;
		case 2:
		case 3:
		case 4:
		{
			readFromStreamRender(in);
			in >> mSegments;
			in >> mLineWidth;
			if (version >= 3)
				in >> m_antiAliasing;
			else
				m_antiAliasing = true;
			if (version >= 4)
				in >> m_verticesPointSize;
			else
				m_verticesPointSize = .0f;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CSetOfLines::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min = mrpt::math::TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());

	for (const auto& s : mSegments)
	{
		for (size_t p = 0; p < 2; p++)
		{
			const TPoint3D& pt = s[p];
			for (size_t j = 0; j < 3; j++)
			{
				keep_min(bb_min[j], pt[j]);
				keep_max(bb_max[j], pt[j]);
			}
		}
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}

void CSetOfLines::getLineByIndex(
	size_t index, double& x0, double& y0, double& z0, double& x1, double& y1,
	double& z1) const
{
	ASSERT_(index < mSegments.size());
	const mrpt::math::TPoint3D& p0 = mSegments[index].point1;
	const mrpt::math::TPoint3D& p1 = mSegments[index].point2;
	x0 = p0.x;
	y0 = p0.y;
	z0 = p0.z;
	x1 = p1.x;
	y1 = p1.y;
	z1 = p1.z;
}
