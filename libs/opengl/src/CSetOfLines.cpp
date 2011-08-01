/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/opengl.h>  // Precompiled header


#include <mrpt/opengl/CSetOfLines.h>
#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CSetOfLines, CRenderizableDisplayList, mrpt::opengl )


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
	glLineWidth(mLineWidth);
	checkOpenGLError();

	if ( m_color.A != 255 )
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	glBegin(GL_LINES);
	glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
	for (std::vector<TSegment3D>::const_iterator it=mSegments.begin();it!=mSegments.end();++it)	{
		glVertex3d(it->point1.x,it->point1.y,it->point1.z);
		glVertex3d(it->point2.x,it->point2.y,it->point2.z);
	}
	glEnd();
	checkOpenGLError();

	if ( m_color.A != 255 )
		glDisable(GL_BLEND);

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfLines::writeToStream(CStream &out,int *version) const
{
	if (version) *version=2;
	else	{
		writeToStreamRender(out);
		out<<mSegments<<mLineWidth;
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
			vector_float x0,y0,z0,x1,y1,z1;
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
		readFromStreamRender(in);
		in>>mSegments;
		in>>mLineWidth;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}
