/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
