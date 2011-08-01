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
	// Render all the objects:
	mrpt::opengl::gl_utils::renderSetOfObjects(m_objects);
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
	CPose3D nueva=(CPose3D()-this->m_pose)+o;
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
	uint8_t r,g,b,a;
	void operator()(CRenderizablePtr &p)	{
		p->setColor_u8(r,g,b,a);
	}
	FSetColor(uint8_t R,uint8_t G,uint8_t B,uint8_t A):r(R),g(G),b(B),a(A)	{}
	~FSetColor()	{}
};

CRenderizable& CSetOfObjects::setColor_u8(const mrpt::utils::TColor &c)	{
	for_each(m_objects.begin(),m_objects.end(),FSetColor(m_color.R=c.R,m_color.G=c.G,m_color.B=c.B,m_color.A=c.A));
	return *this;
}


bool CSetOfObjects::contains(const CRenderizablePtr &obj) const	{
	return find(m_objects.begin(),m_objects.end(),obj)!=m_objects.end();
}

CRenderizable& CSetOfObjects::setColorR_u8(const uint8_t r)	{
	for(CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it) (*it)->setColorR_u8(m_color.R=r);
	return *this;
}

CRenderizable& CSetOfObjects::setColorG_u8(const uint8_t  g)	{
	for(CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it) (*it)->setColorG_u8(m_color.G=g);
	return *this;
}

CRenderizable& CSetOfObjects::setColorB_u8(const uint8_t  b)	{
	for(CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it) (*it)->setColorB_u8(m_color.B=b);
	return *this;
}

CRenderizable& CSetOfObjects::setColorA_u8(const uint8_t  a)	{
	for(CListOpenGLObjects::iterator it=m_objects.begin();it!=m_objects.end();++it) (*it)->setColorA_u8(m_color.A=a);
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
