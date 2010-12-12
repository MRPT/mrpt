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
	m_pointSmooth(false),
	m_min(0),
	m_max(0),
	m_minmax_valid(false),
	m_colorFromDepth_min(0,0,0),
	m_colorFromDepth_max(0,0,1)
{
	markAllPointsAsNew();
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
			if (!m_zs.empty())
				mrpt::math::minimum_maximum(
					m_colorFromDepth == CPointCloud::Z ? m_zs : (m_colorFromDepth == CPointCloud::Y ? m_ys : m_xs),
					m_min, m_max);
			else m_max=m_min=0;

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

	if ( m_color_A != 1.0 )
	{
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
    if (m_pointSmooth)
			glEnable ( GL_POINT_SMOOTH );
	else 	glDisable( GL_POINT_SMOOTH );

    glBegin( GL_POINTS );

    glColor4f( m_color_R,m_color_G,m_color_B,m_color_A );

	const size_t N=m_xs.size();
    for (size_t i=0;i<N;i++)
    {
		if ( m_colorFromDepth!=None && A>0 )
		{
			const float depthCol = (m_colorFromDepth==X ? m_xs[i] : (m_colorFromDepth==Y ? m_ys[i] : m_zs[i]));

			float	f = (depthCol - m_min) * A_1;
			f=std::max(0.0f,min(1.0f,f));

			glColor4f(
				m_colorFromDepth_min.R + f*AR,
				m_colorFromDepth_min.G + f*AG,
				m_colorFromDepth_min.B + f*AB,
				m_color_A );
		}
		glVertex3f( m_xs[i],m_ys[i],m_zs[i] );
    }
    glEnd();

	if ( m_color_A != 1.0 )
		glDisable(GL_BLEND);

	if (m_pointSmooth)
		glDisable( GL_POINT_SMOOTH );

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
		*version = 4;
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

		// New in version 4:
		out << m_pointSmooth;
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
	case 4:
		{
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

			if (version>=4)
					in >> m_pointSmooth;
			else 	m_pointSmooth = false;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

	markAllPointsAsNew();
}

/*---------------------------------------------------------------
						clear
---------------------------------------------------------------*/
void CPointCloud::clear()
{
	m_xs.clear();
	m_ys.clear();
	m_zs.clear();
	markAllPointsAsNew();
}

/*---------------------------------------------------------------
						insertPoint
---------------------------------------------------------------*/
void CPointCloud::insertPoint( float x,float y, float z )
{
	m_xs.push_back(x);
	m_ys.push_back(y);
	m_zs.push_back(z);


	MRPT_TODO("octree update...")
}

/** Write an individual point (checks for "i" in the valid range only in Debug). */
void CPointCloud::setPoint(size_t i, const float x,const float y, const float z)
{
#ifdef _DEBUG
	ASSERT_BELOW_(i,size())
#endif
	m_xs[i] = x;
	m_ys[i] = y;
	m_zs[i] = z;

	MRPT_TODO("octree update...")
}

/*---------------------------------------------------------------
					setGradientColors
---------------------------------------------------------------*/
void  CPointCloud::setGradientColors( const mrpt::utils::TColorf &colorMin, const mrpt::utils::TColorf &colorMax )
{
	m_colorFromDepth_min = colorMin;
	m_colorFromDepth_max = colorMax;

}


// Do needed internal work if all points are new (octree rebuilt,...)
void CPointCloud::markAllPointsAsNew()
{
	m_minmax_valid = false;

	MRPT_TODO("octree update...")
}
