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


#include <mrpt/opengl/CRenderizable.h>		// Include these before windows.h!!
#include <mrpt/synch.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/math/utils.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3D.h>

#include "opengl_internals.h"

#include <mrpt/utils/CStartUpClassesRegister.h>

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
	m_color(255,255,255,255),
	m_pose(),
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
	if (i<booker.next_free_texture) booker.next_free_texture = i;  // try to reuse texture numbers.
	// "glDeleteTextures" seems not to be neeeded, since we do the reservation of texture names by our own.
}


void  CRenderizable::writeToStreamRender(CStream &out) const
{
	out << m_name << (float)(m_color.R*255.f) << (float)(m_color.G*255.f) << (float)(m_color.B*255.f) << (float)(m_color.A*255.f);
	out << (float)m_pose.x() << (float)m_pose.y() << (float)m_pose.z();

	// Version 2 (dummy=16.0f): Added scale vars
	// Version 3 (dummy=17.0f): Added "m_visible"
	if (m_scale_x==1.0f && m_scale_y==1.0f && m_scale_z==1.0f)
	{
		// Keep old format for compatibility:
		out << (float)RAD2DEG(m_pose.yaw())
		    << (float)RAD2DEG(m_pose.pitch())
			<< (float)RAD2DEG(m_pose.roll())
		    << m_show_name;
	}
	else
	{
		const float dummy = 17.0f;
		out << (float)RAD2DEG(m_pose.yaw())
		    << (float)RAD2DEG(m_pose.pitch())
		    << dummy
		    << (float)RAD2DEG(m_pose.roll())
		    << m_show_name
		    << m_scale_x << m_scale_y << m_scale_z
		    << m_visible; // Added in v3
	}
}

void  CRenderizable::readFromStreamRender(CStream &in)
{
	in >> m_name;
	float f;

	float yaw_deg,pitch_deg,roll_deg;

	mrpt::utils::TColorf col;
	in >> col.R >> col.G >> col.B >> col.A;
	m_color = mrpt::utils::TColor(col.R*255,col.G*255,col.B*255,col.A*255);

	in >> f; m_pose.x(f);
	in >> f; m_pose.y(f);
	in >> f; m_pose.z(f);
	in >> yaw_deg;
	in >> pitch_deg;
	in >> roll_deg;
	// Version 2: Add scale vars:
	//  JL: Yes, this is a crappy hack since I forgot to enable versions here...what? :-P
	if (f!=16.0f && f!=17.0f)
	{
		// Old version:
		// "roll_deg" is the actual roll.
		in >> m_show_name;
		m_scale_x=m_scale_y=m_scale_z=1;	// Default values
	}
	else
	{
		// New version >=v2:
		in >> roll_deg;
		in >> m_show_name;

		// Scale data:
		in >> m_scale_x >> m_scale_y >> m_scale_z;

		if (f==17.0f)  // version>=v3
			in >>m_visible;
		else
			m_visible = true; // Default
	}

	m_pose.setYawPitchRoll( DEG2RAD(yaw_deg),DEG2RAD(pitch_deg),DEG2RAD(roll_deg) );
}


void  CRenderizable::checkOpenGLError()
{
	mrpt::opengl::gl_utils::checkOpenGLError();
}

/*--------------------------------------------------------------
					setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose( const mrpt::poses::CPose3D &o )
{
	m_pose = o;
	return *this;
}

/*--------------------------------------------------------------
					setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose( const mrpt::math::TPose3D &o )
{
	m_pose = CPose3D(o);
	return *this;
}

/*--------------------------------------------------------------
					setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose( const mrpt::poses::CPoint3D &o )	//!< Set the 3D pose from a mrpt::poses::CPose3D object
{
	m_pose.setFromValues(o.x(), o.y(), o.z(),  0,0,0 );
	return *this;
}

/*--------------------------------------------------------------
					setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose( const mrpt::poses::CPoint2D &o )	//!< Set the 3D pose from a mrpt::poses::CPose3D object
{
	m_pose.setFromValues(o.x(), o.y(),0,  0,0,0 );
	return *this;
}


/*--------------------------------------------------------------
					getPose
  ---------------------------------------------------------------*/
mrpt::math::TPose3D CRenderizable::getPose() const
{
	return mrpt::math::TPose3D(m_pose);
}

/*--------------------------------------------------------------
					traceRay
  ---------------------------------------------------------------*/
bool CRenderizable::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	return false;
}

CRenderizablePtr &mrpt::opengl::operator<<(CRenderizablePtr &r,const CPose3D &p)	{
	r->setPose(p+r->getPose());
	return r;
}

CRenderizable& CRenderizable::setColor_u8( const mrpt::utils::TColor &c)
{
	m_color.R = c.R;
	m_color.G = c.G;
	m_color.B = c.B;
	m_color.A = c.A;
	return *this;
}



/** This method is safe for calling from within ::render() methods \sa renderTextBitmap */
void CRenderizable::renderTextBitmap( const char *str, void *fontStyle )
{
	gl_utils::renderTextBitmap(str,fontStyle);
}

/** Return the exact width in pixels for a given string, as will be rendered by renderTextBitmap().
  * \sa renderTextBitmap
  */
int CRenderizable::textBitmapWidth(
	const std::string &str,
	mrpt::opengl::TOpenGLFont font )
{
	return gl_utils::textBitmapWidth(str,font);
}
