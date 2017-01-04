/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header


#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/utils/CStream.h>


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
	m_line_R(1.f),m_line_G(0.f),m_line_B(0.f),m_line_A(0.5f),
	m_points_width(3),
	m_points_R(1.0f),m_points_G(0.0f),m_points_B(0.0f),m_points_A(1.0f),
	m_plane_R(0.01f),m_plane_G(0.01f),m_plane_B(0.6f),m_plane_A(0.6f),
	m_enable_points(true), m_enable_line(true), m_enable_surface(true)
{
}

/*---------------------------------------------------------------
							clear
  ---------------------------------------------------------------*/
void CPlanarLaserScan::clear()
{
	CRenderizableDisplayList::notifyChange();
	m_scan.resizeScan(0);
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
	if (n>1 && m_enable_line)
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
	if (n>0 && m_enable_points)
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
	if (n>1 && m_enable_surface)
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
void  CPlanarLaserScan::writeToStream(mrpt::utils::CStream &out,int *version) const
{

	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		out << m_scan;
		out << m_line_width
			<< m_line_R << m_line_G << m_line_B << m_line_A
			<< m_points_width
			<< m_points_R << m_points_G << m_points_B << m_points_A
			<< m_plane_R << m_plane_G << m_plane_B << m_plane_A
			<< m_enable_points << m_enable_line << m_enable_surface; // new in v1
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPlanarLaserScan::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			readFromStreamRender(in);
			in >> m_scan;
			in >> m_line_width
				>> m_line_R >> m_line_G >> m_line_B >> m_line_A
				>> m_points_width
				>> m_points_R >> m_points_G >> m_points_B >> m_points_A
				>> m_plane_R >> m_plane_G >> m_plane_B >> m_plane_A;

            if (version>=1)
            {
                in >> m_enable_points >> m_enable_line >> m_enable_surface; // new in v1
            }
            else
            {
                m_enable_points = m_enable_line = m_enable_surface = true;
            }
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


void CPlanarLaserScan::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	// Load into cache:
	if (!m_cache_valid)
	{
		m_cache_valid=true;
		m_cache_points.clear();
		m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
		m_cache_points.insertionOptions.isPlanarMap=false;

		m_cache_points.insertObservation( &m_scan );
	}

	size_t n;
	const float	*x,*y,*z;

	m_cache_points.getPointsBuffer(n,x,y,z);
	if (!n || !x) return;

	bb_min = mrpt::math::TPoint3D(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

	for (size_t i=0;i<n;i++)
	{
		keep_min(bb_min.x, x[i]);  keep_max(bb_max.x, x[i]);
		keep_min(bb_min.y, y[i]);  keep_max(bb_max.y, y[i]);
		keep_min(bb_min.z, z[i]);  keep_max(bb_max.z, z[i]);
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}

