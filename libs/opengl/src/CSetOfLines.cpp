/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/stl_serialization.h>
#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CSetOfLines, CRenderizableDisplayList, mrpt::opengl )

CSetOfLinesPtr CSetOfLines::Create(const std::vector<TSegment3D> &sgms, const bool antiAliasing)
{
	return CSetOfLinesPtr(new CSetOfLines(sgms,antiAliasing));
}
/** Constructor */
CSetOfLines::CSetOfLines()
	: mSegments(),mLineWidth(1.0),m_antiAliasing(true)
{
}

/** Constructor with a initial set of lines. */
CSetOfLines::CSetOfLines(const std::vector<TSegment3D> &sgms,bool antiAliasing)
	: mSegments(sgms),mLineWidth(1.0),m_antiAliasing(antiAliasing)
{
}

/*---------------------------------------------------------------
							setLineByIndex
  ---------------------------------------------------------------*/
void CSetOfLines::setLineByIndex(size_t index,const mrpt::math::TSegment3D &segm)	{
	MRPT_START
	if (index>=mSegments.size()) THROW_EXCEPTION("Index out of bounds");
	CRenderizableDisplayList::notifyChange();
	mSegments[index]=segm;
	MRPT_END
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CSetOfLines::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT

	// Enable antialiasing:
	glPushAttrib( GL_COLOR_BUFFER_BIT | GL_LINE_BIT );
	if (m_antiAliasing || m_color.A != 255)
	{
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
	}
	if (m_antiAliasing)
		glEnable(GL_LINE_SMOOTH);
	glLineWidth(mLineWidth);
	checkOpenGLError();

	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glBegin(GL_LINES);
	glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
	for (std::vector<TSegment3D>::const_iterator it=mSegments.begin();it!=mSegments.end();++it)	{
		glVertex3d(it->point1.x,it->point1.y,it->point1.z);
		glVertex3d(it->point2.x,it->point2.y,it->point2.z);
	}
	glEnd();
	checkOpenGLError();
	glEnable(GL_LIGHTING);  // Disable lights when drawing lines

	// End of antialiasing:
    glPopAttrib();

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfLines::writeToStream(CStream &out,int *version) const
{
	if (version) *version=3;
	else	{
		writeToStreamRender(out);
		out<<mSegments<<mLineWidth;
		out << m_antiAliasing; // Added in v3
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfLines::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			readFromStreamRender(in);
			CVectorFloat x0,y0,z0,x1,y1,z1;
			in>>x0>>y0>>z0>>x1>>y1>>z1;
			if (version>=1) in>>mLineWidth;
			else mLineWidth=1;
			size_t N=x0.size();
			mSegments.resize(N);
			for (size_t i=0;i<N;i++)	{
				mSegments[i][0][0]=x0[i];
				mSegments[i][0][1]=y0[i];
				mSegments[i][0][2]=z0[i];
				mSegments[i][1][0]=x1[i];
				mSegments[i][1][1]=y1[i];
				mSegments[i][1][2]=z1[i];
			}
		}	break;
	case 2:
	case 3:
		{
			readFromStreamRender(in);
			in>>mSegments;
			in>>mLineWidth;
			if (version>=3)
				in >> m_antiAliasing;
			else m_antiAliasing = true;
		}
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

void CSetOfLines::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = mrpt::math::TPoint3D(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

	for (size_t i=0;i<mSegments.size();i++)
	{
		const TSegment3D &s=mSegments[i];
		for (size_t p=0;p<2;p++)
		{
			const TPoint3D &pt = s[p];
			for (size_t j=0;j<3;j++)
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
