/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header


#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/opengl/CSetOfTriangles.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CTexturedPlane, CTexturedObject, mrpt::opengl )

CTexturedPlanePtr CTexturedPlane::Create(
	float				x_min,
	float				x_max,
	float				y_min,
	float				y_max)
{
	return CTexturedPlanePtr( new CTexturedPlane(x_min, x_max, y_min, y_max) );
}
/*---------------------------------------------------------------
							CTexturedPlane
  ---------------------------------------------------------------*/
CTexturedPlane::CTexturedPlane(
	float				x_min,
	float				x_max,
	float				y_min,
	float				y_max
	) :
		polygonUpToDate(false)
{
	// Copy data:
	m_xMin = x_min;
	m_xMax = x_max;
	m_yMin = y_min;
	m_yMax = y_max;
}


/*---------------------------------------------------------------
							~CTexturedPlane
  ---------------------------------------------------------------*/
CTexturedPlane::~CTexturedPlane()
{
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CTexturedPlane::render_texturedobj() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	// Compute the exact texture coordinates:
	m_tex_x_min = 0;
	m_tex_x_max = 1.0f-((float)m_pad_x_right) / r_width;
	m_tex_y_min = 0;
	m_tex_y_max = 1.0f-((float)m_pad_y_bottom) / r_height;

	glBegin(GL_QUADS);

	glTexCoord2d(m_tex_x_min,m_tex_y_min);
	glVertex3f( m_xMin, m_yMin,0 );

	glTexCoord2d(m_tex_x_max,m_tex_y_min);
	glVertex3f( m_xMax, m_yMin,0 );

	glTexCoord2d(m_tex_x_max,m_tex_y_max);
	glVertex3f( m_xMax, m_yMax,0 );

	glTexCoord2d(m_tex_x_min,m_tex_y_max);
	glVertex3f( m_xMin, m_yMax,0 );

	glEnd();
	checkOpenGLError();

	MRPT_END
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CTexturedPlane::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		writeToStreamRender(out);

		out << m_xMin << m_xMax;
		out << m_yMin << m_yMax;

		writeToStreamTexturedObject(out);
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CTexturedPlane::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			in >> m_textureImage >> m_textureImageAlpha;
			in >> m_xMin >> m_xMax;
			in >> m_yMin >> m_yMax;

			assignImage( m_textureImage, m_textureImageAlpha );

		} break;
	case 1:
	case 2:
		{
			readFromStreamRender(in);

			in >> m_xMin >> m_xMax;
			in >> m_yMin >> m_yMax;

			if (version>=2)
			{
				readFromStreamTexturedObject(in);
			}
			else
			{	// Old version.
				in >> CTexturedObject::m_enableTransparency;
				in >> CTexturedObject::m_textureImage;
				if (CTexturedObject::m_enableTransparency)
				{
					in >> CTexturedObject::m_textureImageAlpha;
					assignImage( CTexturedObject::m_textureImage, CTexturedObject::m_textureImageAlpha );
				}
				else
					assignImage( CTexturedObject::m_textureImage );
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

bool CTexturedPlane::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	if (!polygonUpToDate) updatePoly();
	return math::traceRay(tmpPoly,o-this->m_pose,dist);
}

void CTexturedPlane::updatePoly() const	{
	TPolygon3D poly(4);
	poly[0].x=poly[1].x=m_xMin;
	poly[2].x=poly[3].x=m_xMax;
	poly[0].y=poly[3].y=m_yMin;
	poly[1].y=poly[2].y=m_yMax;
	for (size_t i=0;i<4;i++) poly[i].z=0;
	tmpPoly.resize(1);
	tmpPoly[0]=poly;
	polygonUpToDate=true;
}


void CTexturedPlane::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = std::min(m_xMin, m_xMax);
	bb_min.y = std::min(m_yMin, m_yMax);
	bb_min.z = 0;

	bb_max.x = std::max(m_xMin, m_xMax);
	bb_max.y = std::max(m_yMin, m_yMax);
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
