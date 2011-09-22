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

#include <mrpt/maps.h>  // Precompiled header


#include <mrpt/opengl/CPlanarLaserScan.h>


#if MRPT_HAS_OPENGL_GLUT
	#ifdef MRPT_OS_WINDOWS
		// Windows:
		#include <windows.h>
	#endif

	#ifdef MRPT_OS_APPLE
		#include <OpenGL/gl.h>
	#else
		#include <GL/gl.h>
	#endif
#endif

// Include libraries in linking:
#if MRPT_HAS_OPENGL_GLUT && defined(MRPT_OS_WINDOWS)
		// WINDOWS:
		#if defined(_MSC_VER) || defined(__BORLANDC__)
			#pragma comment (lib,"opengl32.lib")
			#pragma comment (lib,"GlU32.lib")
		#endif
#endif // MRPT_HAS_OPENGL_GLUT


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CPlanarLaserScan, CRenderizableDisplayList, mrpt::opengl )


/*---------------------------------------------------------------
				Constructor
  ---------------------------------------------------------------*/
CPlanarLaserScan::CPlanarLaserScan() :
	m_scan(),
	m_cache_points(),
	m_cache_valid(false),
	m_line_width(1),
	m_line_R(1),m_line_G(0),m_line_B(0),m_line_A(0.5),
	m_points_width(3),
	m_points_R(1),m_points_G(0),m_points_B(0),m_points_A(1),
	m_plane_R(0.01),m_plane_G(0.01),m_plane_B(0.6),m_plane_A(0.6),
	m_enable_points(true), m_enable_line(true), m_enable_surface(true)
{
}

/*---------------------------------------------------------------
							clear
  ---------------------------------------------------------------*/
void CPlanarLaserScan::clear()
{
	CRenderizableDisplayList::notifyChange();
	m_scan.scan.clear();
	m_scan.validRange.clear();
}


/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CPlanarLaserScan::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(m_scan.scan.size()==m_scan.validRange.size());

	// Load into cache:
	if (!m_cache_valid)
	{
		m_cache_valid=true;
		m_cache_points.clear();
		m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
		m_cache_points.insertionOptions.isPlanarMap=false;

		m_cache_points.insertObservation( &m_scan );
	}

	size_t	i,n;
	const float	*x,*y,*z;

	m_cache_points.getPointsBuffer(n,x,y,z);
	if (!n || !x) return;

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// LINES
	// ----------------------------
	if (n>1)
	{
		glLineWidth(m_line_width);	checkOpenGLError();

		glBegin( GL_LINES );
		glColor4f( m_line_R,m_line_G,m_line_B,m_line_A );

		for (i=0;i<n-1;i++)
		{
			glVertex3f( x[i],y[i],z[i] );
			glVertex3f( x[i+1],y[i+1],z[i+1] );
		}
		glEnd();
		checkOpenGLError();
	}

	// POINTS
	// ----------------------------
	if (n>0)
	{
		glPointSize(m_points_width);
		checkOpenGLError();

		glBegin( GL_POINTS );
		glColor4f( m_points_R,m_points_G,m_points_B,m_points_A );

		for (i=0;i<n;i++)
		{
			glVertex3f( x[i],y[i],z[i] );
		}
		glEnd();
		checkOpenGLError();
	}

	// SURFACE:
	// ------------------------------
	if (n>1)
	{
		glBegin( GL_TRIANGLES );

		glColor4f(m_plane_R,m_plane_G,m_plane_B,m_plane_A);

		for (i=0;i<n-1;i++)
		{
			glVertex3f( m_scan.sensorPose.x(), m_scan.sensorPose.y(), m_scan.sensorPose.z() );
			glVertex3f( x[i],y[i],z[i] );
			glVertex3f( x[i+1],y[i+1],z[i+1] );
		}
		glEnd();
		checkOpenGLError();
	}

	glDisable(GL_BLEND);

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPlanarLaserScan::writeToStream(CStream &out,int *version) const
{

	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);
		out << m_scan;
		out << m_line_width
			<< m_line_R << m_line_G << m_line_B << m_line_A
			<< m_points_width
			<< m_points_R << m_points_G << m_points_B << m_points_A
			<< m_plane_R << m_plane_G << m_plane_B << m_plane_A;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPlanarLaserScan::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			in >> m_scan;
			in >> m_line_width
				>> m_line_R >> m_line_G >> m_line_B >> m_line_A
				>> m_points_width
				>> m_points_R >> m_points_G >> m_points_B >> m_points_A
				>> m_plane_R >> m_plane_G >> m_plane_B >> m_plane_A;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

