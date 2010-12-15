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


#include <mrpt/opengl/CRenderizable.h>		// Include these before windows.h!!
#include <mrpt/synch.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/math/utils.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3D.h>

#include "opengl_internals.h"


using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::synch;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CRenderizable, CSerializable, mrpt::opengl )

extern CStartUpClassesRegister  mrpt_opengl_class_reg;
const int dumm = mrpt_opengl_class_reg.do_nothing(); // Avoid compiler removing this class in static linking


#define MAX_GL_TEXTURE_IDS       0x10000
#define MAX_GL_TEXTURE_IDS_MASK  0x0FFFF

struct TOpenGLNameBooker
{
private:
	TOpenGLNameBooker() :
		freeTextureNames(MAX_GL_TEXTURE_IDS,false),
		next_free_texture(1),   // 0 is a reserved number!!
		cs()
	{
	}

public:
	std::vector<bool>		freeTextureNames;
	unsigned int			next_free_texture;
	synch::CCriticalSection	cs;

	static TOpenGLNameBooker & instance()
	{
		static TOpenGLNameBooker dat;
		return dat;
	}
};

// Default constructor:
CRenderizable::CRenderizable() :
	m_name(),
	m_show_name(false),
	m_color_R(1),m_color_G(1),m_color_B(1),m_color_A(1),
	m_x(0),m_y(0),m_z(0),
	m_yaw(0),m_pitch(0),m_roll(0),
	m_scale_x(1), m_scale_y(1), m_scale_z(1),
	m_visible(true)
{
}

// Destructor:
CRenderizable::~CRenderizable()
{
}



/** Returns the lowest, free texture name.
  */
unsigned int CRenderizable::getNewTextureNumber()
{
	MRPT_START

	TOpenGLNameBooker &booker = TOpenGLNameBooker::instance();

	CCriticalSectionLocker lock ( &booker.cs );

	unsigned int ret = booker.next_free_texture;
	unsigned int tries = 0;
	while (ret!=0 && booker.freeTextureNames[ret])
	{
		ret++;
		ret = ret % MAX_GL_TEXTURE_IDS_MASK;

		if (++tries>=MAX_GL_TEXTURE_IDS) 
			THROW_EXCEPTION_CUSTOM_MSG1("Maximum number of textures (%u) excedeed! (are you deleting them?)", (unsigned int)MAX_GL_TEXTURE_IDS);
	}

	booker.freeTextureNames[ret] = true; // mark as used.
	booker.next_free_texture = ret+1;
	return ret;
	MRPT_END
}

void CRenderizable::releaseTextureName(unsigned int i)
{
	TOpenGLNameBooker &booker = TOpenGLNameBooker::instance();
	CCriticalSectionLocker lock ( &booker.cs );
	booker.freeTextureNames[i] = false;
}


void  CRenderizable::writeToStreamRender(CStream &out) const
{
	out << m_name << (float)m_color_R << (float)m_color_G << (float)m_color_B << (float)m_color_A;
	out << (float)m_x << (float)m_y << (float)m_z;

	// Version 2 (dummy=16.0f): Added scale vars
	// Version 3 (dummy=17.0f): Added "m_visible"
	if (m_scale_x==1.0f && m_scale_y==1.0f && m_scale_z==1.0f)
	{
		// Keep old format for compatibility:
		out << (float)m_yaw << (float)m_pitch << (float)m_roll;
		out << m_show_name;
	}
	else
	{
		const float dummy = 17.0f;
		out << (float)m_yaw << (float)m_pitch << dummy << (float)m_roll;
		out << m_show_name;
		out << m_scale_x << m_scale_y << m_scale_z;
		out << m_visible; // Added in v3
	}
}

void  CRenderizable::readFromStreamRender(CStream &in)
{
	in >> m_name;
	float f;

	in >> f; m_color_R=f;
	in >> f; m_color_G=f;
	in >> f; m_color_B=f;
	in >> f; m_color_A=f;
	in >> f; m_x=f;
	in >> f; m_y=f;
	in >> f; m_z=f;
	in >> f; m_yaw=f;
	in >> f; m_pitch=f;
	in >> f;
	// Version 2: Add scale vars:
	//  JL: Yes, this is a crappy hack since I forgot to enable versions here...what? :-P
	if (f!=16.0f && f!=17.0f)
	{
		// Old version:
		m_roll=f;
		in >> m_show_name;
		m_scale_x=m_scale_y=m_scale_z=1;	// Default values
	}
	else
	{
		// New version >=v2:
		in >> f; m_roll=f;
		in >> m_show_name;

		// Scale data:
		in >> m_scale_x >> m_scale_y >> m_scale_z;

		if (f==17.0f)  // version>=v3
			in >>m_visible;
		else
			m_visible = true; // Default
	}
}


void  CRenderizable::checkOpenGLError()
{
#if MRPT_HAS_OPENGL_GLUT
	int	 openglErr;
	if ( ( openglErr= glGetError()) != GL_NO_ERROR )
	{
		THROW_EXCEPTION( std::string("OpenGL error: ") + std::string( (char*)gluErrorString(openglErr) ) );
	}
#endif
}

/*--------------------------------------------------------------
					setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose( const mrpt::poses::CPose3D &o )
{
	m_x = o.x();
	m_y = o.y();
	m_z = o.z();
	m_yaw = RAD2DEG( o.yaw() );
	m_pitch = RAD2DEG( o.pitch() );
	m_roll = RAD2DEG( o.roll() );
	return *this;
}

/*--------------------------------------------------------------
					setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose( const mrpt::math::TPose3D &o )
{
	m_x = o.x;
	m_y = o.y;
	m_z = o.z;
	m_yaw = RAD2DEG( o.yaw );
	m_pitch = RAD2DEG( o.pitch );
	m_roll = RAD2DEG( o.roll );
	return *this;
}

/*--------------------------------------------------------------
					setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose( const mrpt::poses::CPoint3D &o )	//!< Set the 3D pose from a mrpt::poses::CPose3D object
{
	m_x = o.x();
	m_y = o.y();
	m_z = o.z();
	m_yaw = 0;
	m_pitch = 0;
	m_roll = 0;
	return *this;
}

/*--------------------------------------------------------------
					setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose( const mrpt::poses::CPoint2D &o )	//!< Set the 3D pose from a mrpt::poses::CPose3D object
{
	m_x = o.x();
	m_y = o.y();
	m_z = 0;
	m_yaw = 0;
	m_pitch = 0;
	m_roll = 0;
	return *this;
}


/*--------------------------------------------------------------
					getPose
  ---------------------------------------------------------------*/
mrpt::math::TPose3D CRenderizable::getPose() const
{
	return mrpt::math::TPose3D( m_x,m_y,m_z,DEG2RAD(m_yaw),DEG2RAD(m_pitch),DEG2RAD(m_roll) );
}


/*--------------------------------------------------------------
					traceRay
  ---------------------------------------------------------------*/
bool CRenderizable::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	return false;
}
/*--------------------------------------------------------------
					setColor
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setColor( double R, double G, double B, double A)
{
	m_color_R = R;
	m_color_G = G;
	m_color_B = B;
	m_color_A = A;
	return *this;
}

CRenderizablePtr &mrpt::opengl::operator<<(CRenderizablePtr &r,const CPose3D &p)	{
	r->setPose(p+r->getPose());
	return r;
}

void CRenderizable::renderTriangleWithNormal( const mrpt::math::TPoint3D &p1,const mrpt::math::TPoint3D &p2,const mrpt::math::TPoint3D &p3 )
{
#if MRPT_HAS_OPENGL_GLUT
	float	ax= p2.x - p1.x;
    float	ay= p2.y - p1.y;
	float	az= p2.z - p1.z;

	float	bx= p3.x - p1.x;
	float	by= p3.y - p1.y;
	float	bz= p3.z - p1.z;

	glNormal3f(ay*bz-az*by,-ax*bz+az*bx,ax*by-ay*bx);

	glVertex3f(p1.x,p1.y,p1.z);
	glVertex3f(p2.x,p2.y,p2.z);
	glVertex3f(p3.x,p3.y,p3.z);
#endif
}

CRenderizable& CRenderizable::setColor( const mrpt::utils::TColorf &c)
{
	m_color_R = c.R;
	m_color_G = c.G;
	m_color_B = c.B;
	m_color_A = c.A;
	return *this;
}


/** Gather useful information on the render parameters.
  *  It can be called from within the render() method of derived classes.
  */
void CRenderizable::getCurrentRenderingInfo(TRenderInfo &ri) const
{
#if MRPT_HAS_OPENGL_GLUT
	// Viewport geometry:
	GLint	win_dims[4];
	glGetIntegerv( GL_VIEWPORT, win_dims );
	ri.vp_x      = win_dims[0];
	ri.vp_y      = win_dims[1];
	ri.vp_width  = win_dims[2];
	ri.vp_height = win_dims[3];

	// Get the inverse camera position:
	GLfloat  mat_proj[16];
	glGetFloatv(GL_PROJECTION_MATRIX,mat_proj);
	ri.proj_matrix = Eigen::Matrix<float,4,4,Eigen::ColMajor>(mat_proj);

	// Extract the camera position:
	Eigen::Matrix<float,4,1> cam_pose_hm = ri.proj_matrix.inverse().col(3);
	if (cam_pose_hm[3]!=0)
	{
		ri.camera_position.x = cam_pose_hm[0]/cam_pose_hm[3];
		ri.camera_position.y = cam_pose_hm[1]/cam_pose_hm[3];
		ri.camera_position.z = cam_pose_hm[2]/cam_pose_hm[3];
	}
	else ri.camera_position= mrpt::math::TPoint3Df(0,0,0);

	// Get the model transformation:
	GLfloat  mat_mod[16];
	glGetFloatv(GL_MODELVIEW_MATRIX,mat_mod);
	ri.model_matrix = Eigen::Matrix<float,4,4,Eigen::ColMajor>(mat_mod);

	// PROJ * MODEL
	ri.full_matrix = ri.proj_matrix * ri.model_matrix;
#endif
}

/*---------------------------------------------------------------
					renderTextBitmap
  ---------------------------------------------------------------*/
void	CRenderizable::renderTextBitmap( const char *str, void *fontStyle )
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START;
	while ( *str ) glutBitmapCharacter( fontStyle ,*(str++) );
	MRPT_END;
#endif
}
