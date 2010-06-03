/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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


#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/math/utils.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CPointCloud, CRenderizable, mrpt::opengl )

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
CPointCloud::CPointCloud( ) :
    m_colorFromDepth(CPointCloud::None),
	m_xs(),m_ys(),m_zs(),
	m_pointSize(1),
	m_min(0),
	m_max(0),
	m_minmax_valid(false),
	m_colorFromDepth_min(0,0,0),
	m_colorFromDepth_max(0,0,1)
{
}


/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CPointCloud::render() const
{
#if MRPT_HAS_OPENGL_GLUT

    ASSERT_(m_xs.size() == m_ys.size());
    ASSERT_(m_xs.size() == m_zs.size());

	float A=0, A_1=0;

	if ( m_colorFromDepth )
	{
		if (!m_minmax_valid)
		{
			m_minmax_valid = true;
			math::minimum_maximum( m_colorFromDepth == CPointCloud::Z ? m_zs :
				(m_colorFromDepth == CPointCloud::Y ? m_ys : m_xs), m_min, m_max);

			A = m_max - m_min;
			if (std::abs(A)<1e-4)
					A=-1;
			else	m_min = m_max - A * 1.01f;
		}
		else
		{
			A = m_max - m_min;
		}

		A_1  = 1.0/A;
	}

    vector<float>::const_iterator itX,itY,itZ;
    vector<float>::const_iterator & itDepth = m_colorFromDepth == CPointCloud::Z ? itZ :
		(m_colorFromDepth == CPointCloud::Y ? itY : itX);


	if ( m_color_A != 1.0 )
	{
		//glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}


	// Slopes of color interpolation:
	const float AR = m_colorFromDepth_max.R - m_colorFromDepth_min.R;
	const float AG = m_colorFromDepth_max.G - m_colorFromDepth_min.G;
	const float AB = m_colorFromDepth_max.B - m_colorFromDepth_min.B;
	float AR_1,AG_1,AB_1;
	if (AR) AR_1 = 1.0/AR;
	if (AG) AG_1 = 1.0/AG;
	if (AB) AB_1 = 1.0/AB;

    glPointSize( m_pointSize );

    glBegin( GL_POINTS );

    glColor4f( m_color_R,m_color_G,m_color_B,m_color_A );

    for (itX=m_xs.begin(), itY=m_ys.begin(), itZ=m_zs.begin();
           itX!=m_xs.end();
         itX++,itY++,itZ++)
    {
		if ( m_colorFromDepth && A>0 )
		{
			float	f = (*itDepth - m_min) * A_1;
			f=max(0.0f,min(1.0f,f));

			glColor4f(
				m_colorFromDepth_min.R + f*AR,
				m_colorFromDepth_min.G + f*AG,
				m_colorFromDepth_min.B + f*AB,
				m_color_A );
		}
		glVertex3f( *itX,*itY,*itZ );
    }
    glEnd();

	if ( m_color_A != 1.0 )
	{
		//glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}

	checkOpenGLError();
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPointCloud::writeToStream(CStream &out,int *version) const
{

	if (version)
		*version = 3;
	else
	{
		writeToStreamRender(out);
		// Changed from bool to enum/int32_t in version 3.
		out << static_cast<int32_t>(m_colorFromDepth);
		out << m_xs << m_ys << m_zs;

		// Added in version 1.
		out << m_pointSize;

		// New in version 2:
		out << m_colorFromDepth_min.R << m_colorFromDepth_min.G << m_colorFromDepth_min.B;
		out << m_colorFromDepth_max.R << m_colorFromDepth_max.G << m_colorFromDepth_max.B;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPointCloud::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			m_minmax_valid = false;

			readFromStreamRender(in);
			if (version>=3)
			{
				int32_t axis;
				in >> axis;
				m_colorFromDepth = Axis(axis);
			}
			else
			{
				bool colorFromZ;
				in >> colorFromZ;
				m_colorFromDepth = colorFromZ ? CPointCloud::Z : CPointCloud::None;
			}
			in >> m_xs >> m_ys >> m_zs;

			if (version>=1)
				 in >> m_pointSize;
			else m_pointSize = 1;

			if (version>=2)
			{
				in >> m_colorFromDepth_min.R >> m_colorFromDepth_min.G >> m_colorFromDepth_min.B;
				in  >> m_colorFromDepth_max.R >> m_colorFromDepth_max.G >> m_colorFromDepth_max.B;
			}
			else
			{
				m_colorFromDepth_min = TColorf(0,0,0);
				m_colorFromDepth_max.R = m_color_R;
				m_colorFromDepth_max.G = m_color_G;
				m_colorFromDepth_max.B = m_color_B;
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
						clear
---------------------------------------------------------------*/
void CPointCloud::clear()
{
	m_xs.clear();
	m_ys.clear();
	m_zs.clear();
}

/*---------------------------------------------------------------
						insertPoint
---------------------------------------------------------------*/
void CPointCloud::insertPoint( float x,float y, float z )
{
	m_xs.push_back(x);
	m_ys.push_back(y);
	m_zs.push_back(z);
}

/*---------------------------------------------------------------
					setGradientColors
---------------------------------------------------------------*/
void  CPointCloud::setGradientColors( const mrpt::utils::TColorf &colorMin, const mrpt::utils::TColorf &colorMax )
{
	m_colorFromDepth_min = colorMin;
	m_colorFromDepth_max = colorMax;

}


