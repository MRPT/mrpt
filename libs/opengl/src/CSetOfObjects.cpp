/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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


#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>

#include <mrpt/utils/CStringList.h>
#include <mrpt/poses/CPose3D.h>

#include "opengl_internals.h"
#include <algorithm>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

#include <mrpt/utils/metaprogramming.h>
using namespace mrpt::utils::metaprogramming;

IMPLEMENTS_SERIALIZABLE( CSetOfObjects, CRenderizable, mrpt::opengl )

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CSetOfObjects::clear()
{
	m_objects.clear(); // clear the list and delete objects (if there are no more copies out there!)
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CSetOfObjects::render() const
{
#if MRPT_HAS_OPENGL_GLUT
	CListOpenGLObjects::const_iterator	it;
	try
	{
		for (it=m_objects.begin();it!=m_objects.end();it++)
		{
			if (!it->present()) continue;
			if (!(*it)->isVisible()) continue;

			// 3D coordinates transformation:
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

			glPushAttrib(GL_ALL_ATTRIB_BITS);
			//CRenderizable::checkOpenGLError();

			// This is the right order so that the transformation results in the standard matrix.
			// The order seems to be wrong, but it's not.
			glTranslated((*it)->m_x, (*it)->m_y, (*it)->m_z);
			glRotated((*it)->m_yaw, 0.0, 0.0, 1.0);
			glRotated((*it)->m_pitch, 0.0, 1.0, 0.0);
			glRotated((*it)->m_roll, 1.0, 0.0, 0.0);

			// Do scaling after the other transformations!
			glScalef((*it)->m_scale_x,(*it)->m_scale_y,(*it)->m_scale_z);

			// Set color:
			glColor4f( (*it)->getColorR(),(*it)->getColorG(),(*it)->getColorB(),(*it)->getColorA());


			(*it)->render();


			if ((*it)->m_show_name)
			{
				glDisable(GL_DEPTH_TEST);
				glColor3f(1.f,1.f,1.f);  // Must be called BEFORE glRasterPos3f
				glRasterPos3f(0.0f,0.0f,0.0f);

				GLfloat		raster_pos[4];
				glGetFloatv( GL_CURRENT_RASTER_POSITION, raster_pos);
				float eye_distance= raster_pos[3];

				void *font=NULL;
				if (eye_distance<2)
						font = GLUT_BITMAP_TIMES_ROMAN_24;
				else if(eye_distance<200)
					font = GLUT_BITMAP_TIMES_ROMAN_10;

				if (font)
					CRenderizable::renderTextBitmap( (*it)->m_name.c_str(), font);

				glEnable(GL_DEPTH_TEST);
			}

			glPopMatrix();
			checkOpenGLError();

			glPopAttrib();
			//CRenderizable::checkOpenGLError();
		}
	}
	catch(exception &e)
	{
		char	str[1000];
		os::sprintf(str,1000,"Exception while rendering a class '%s'\n%s",
			(*it)->GetRuntimeClass()->className,
			e.what() );
		THROW_EXCEPTION(str);
	}
	catch(...)
	{
		THROW_EXCEPTION("Runtime error!");
	}
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfObjects::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);

		uint32_t	n;
		n = (uint32_t)m_objects.size();
		out << n;
		for (CListOpenGLObjects::const_iterator	it=m_objects.begin();it!=m_objects.end();++it)
			out << **it;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfObjects::readFromStream(CStream &in,int version)
{

	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);

			uint32_t	n;
			in >> n;
			clear();
			m_objects.resize(n);

			for_each(m_objects.begin(),m_objects.end(), ObjectReadFromStream(&in) );

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
					initializeAllTextures
  ---------------------------------------------------------------*/
void  CSetOfObjects::initializeAllTextures()
{
#if MRPT_HAS_OPENGL_GLUT
	CListOpenGLObjects::iterator it;
	//deque<mrpt::opengl::CRenderizable*>::iterator it;
	for (it=m_objects.begin();it!=m_objects.end();it++)
	{
		if ( IS_DERIVED(*it, CTexturedObject ))
			getAs<CTexturedObject>(*it)->loadTextureInOpenGL();
		else if ( IS_CLASS( *it, CSetOfObjects) )
			getAs<CSetOfObjects>(*it)->initializeAllTextures();
	}
#endif
}


CSetOfObjects::CSetOfObjects( )
{
}


CSetOfObjects::~CSetOfObjects()
{
	clear();
}


void CSetOfObjects::insert( const CRenderizablePtr &newObject )
{
	m_objects.push_back(newObject);
}




/*--------------------------------------------------------------
					dumpListOfObjects
  ---------------------------------------------------------------*/
void CSetOfObjects::dumpListOfObjects( utils::CStringList  &lst )
{
	for (CListOpenGLObjects::iterator	it=m_objects.begin();it!=m_objects.end();++it)
	{
		// Single obj:
		string  s( (*it)->GetRuntimeClass()->className );
		if ((*it)->m_name.size())
			s+= string(" (") +(*it)->m_name + string(")");
		lst.add( s );

		if ((*it)->GetRuntimeClass() == CLASS_ID_NAMESPACE(CSetOfObjects,mrpt::opengl))
		{
			CSetOfObjects *objs = getAs<CSetOfObjects>(*it);

			utils::CStringList  auxLst;
			objs->dumpListOfObjects(auxLst);
			for (size_t i=0;i<auxLst.size();i++)
				lst.add( string(" ")+auxLst(i) );
		}
	}
}

/*--------------------------------------------------------------
					removeObject
  ---------------------------------------------------------------*/
void CSetOfObjects::removeObject( const CRenderizablePtr &obj )
{
	for (CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it)
		if (it->pointer() == obj.pointer())
		{
			m_objects.erase(it);
			return;
		}
		else if ( (*it)->GetRuntimeClass()==CLASS_ID_NAMESPACE(CSetOfObjects,opengl) )
			getAs<CSetOfObjects>(*it)->removeObject(obj);
}

bool CSetOfObjects::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	CPose3D nueva=(CPose3D(0,0,0,0,0,0)-CPose3D(m_x,m_y,m_z,DEG2RAD(m_yaw),DEG2RAD(m_pitch),DEG2RAD(m_roll)))+o;
	bool found=false;
	double tmp;
	for (CListOpenGLObjects::const_iterator it=m_objects.begin();it!=m_objects.end();++it) if ((*it)->traceRay(nueva,tmp))	{
		if (!found)	{
			found=true;
			dist=tmp;
		}	else if (tmp<dist) dist=tmp;
	}
	return found;
}

class FSetColor	{
public:
	float r,g,b,a;
	void operator()(CRenderizablePtr &p)	{
		p->setColor(r,g,b,a);
	}
	FSetColor(float R,float G,float B,float A):r(R),g(G),b(B),a(A)	{}
	~FSetColor()	{}
};

CRenderizable& CSetOfObjects::setColor(const mrpt::utils::TColorf &c)	{
	for_each(m_objects.begin(),m_objects.end(),FSetColor(m_color_R=c.R,m_color_G=c.G,m_color_B=c.B,m_color_A=c.A));
	return *this;
}

CRenderizable& CSetOfObjects::setColor(double r,double g,double b,double a)	{
	for_each(m_objects.begin(),m_objects.end(),FSetColor(m_color_R=r,m_color_G=g,m_color_B=b,m_color_A=a));
	return *this;
}

bool CSetOfObjects::contains(const CRenderizablePtr &obj) const	{
	return find(m_objects.begin(),m_objects.end(),obj)!=m_objects.end();
}

CRenderizable& CSetOfObjects::setColorR(const double r)	{
	for(CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it) (*it)->setColorR(m_color_R=r);
	return *this;
}

CRenderizable& CSetOfObjects::setColorG(const double g)	{
	for(CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it) (*it)->setColorG(m_color_G=g);
	return *this;
}

CRenderizable& CSetOfObjects::setColorB(const double b)	{
	for(CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it) (*it)->setColorB(m_color_B=b);
	return *this;
}

CRenderizable& CSetOfObjects::setColorA(const double a)	{
	for(CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it) (*it)->setColorA(m_color_A=a);
	return *this;
}


/*---------------------------------------------------------------
							getByName
  ---------------------------------------------------------------*/
CRenderizablePtr CSetOfObjects::getByName( const string &str )
{
	for (CListOpenGLObjects::iterator	it=m_objects.begin();it!=m_objects.end();++it)
	{
		if ((*it)->m_name == str)
			return *it;
		else if ( (*it)->GetRuntimeClass() == CLASS_ID_NAMESPACE(CSetOfObjects,opengl))
		{
			CRenderizablePtr ret = getAs<CSetOfObjects>(*it)->getByName(str);
			if (ret.present())
				return ret;
		}
	}
	return CRenderizablePtr();
}
