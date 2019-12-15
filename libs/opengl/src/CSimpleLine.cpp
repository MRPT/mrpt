/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/serialization/CArchive.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;

using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSimpleLine, CRenderizableDisplayList, mrpt::opengl)

CSimpleLine::CSimpleLine(
	float x0, float y0, float z0, float x1, float y1, float z1, float lineWidth,
	bool antiAliasing)
	: m_x0(x0),
	  m_y0(y0),
	  m_z0(z0),
	  m_x1(x1),
	  m_y1(y1),
	  m_z1(z1),
	  m_lineWidth(lineWidth),
	  m_antiAliasing(antiAliasing)
{
}

/*---------------------------------------------------------------
							render_dl
  ---------------------------------------------------------------*/
void CSimpleLine::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	// Enable antialiasing:
	if (m_antiAliasing)
	{
		glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LINE_BIT);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
		glEnable(GL_LINE_SMOOTH);
	}
	glLineWidth(m_lineWidth);

	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glBegin(GL_LINES);

	glColor4ub(m_color.R, m_color.G, m_color.B, m_color.A);
	glVertex3f(m_x0, m_y0, m_z0);
	glVertex3f(m_x1, m_y1, m_z1);

	glEnd();
	CHECK_OPENGL_ERROR();
	glEnable(GL_LIGHTING);  // Disable lights when drawing lines

	// End antialiasing:
	if (m_antiAliasing)
	{
		glPopAttrib();
		CHECK_OPENGL_ERROR();
	}
#endif
}

uint8_t CSimpleLine::serializeGetVersion() const { return 1; }
void CSimpleLine::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_x0 << m_y0 << m_z0;
	out << m_x1 << m_y1 << m_z1 << m_lineWidth;
	out << m_antiAliasing;  // Added in v1
}

void CSimpleLine::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 1:
		{
			readFromStreamRender(in);
			in >> m_x0 >> m_y0 >> m_z0;
			in >> m_x1 >> m_y1 >> m_z1 >> m_lineWidth;
			if (version >= 1)
				in >> m_antiAliasing;
			else
				m_antiAliasing = true;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizableDisplayList::notifyChange();
}

void CSimpleLine::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = std::min(m_x0, m_x1);
	bb_min.y = std::min(m_y0, m_y1);
	bb_min.z = std::min(m_z0, m_z1);

	bb_max.x = std::max(m_x0, m_x1);
	bb_max.y = std::max(m_y0, m_y1);
	bb_max.z = std::max(m_z0, m_z1);

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
