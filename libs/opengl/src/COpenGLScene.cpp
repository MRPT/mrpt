/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header


#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// Include libraries in linking:
#if MRPT_HAS_OPENGL_GLUT
	#ifdef MRPT_OS_WINDOWS
		// WINDOWS:
		#if defined(_MSC_VER) || defined(__BORLANDC__)
			#pragma comment (lib,"opengl32.lib")
			#pragma comment (lib,"GlU32.lib")
		#endif
	#endif	// MRPT_OS_WINDOWS
#endif // MRPT_HAS_OPENGL_GLUT


IMPLEMENTS_SERIALIZABLE( COpenGLScene, CRenderizableDisplayList, mrpt::opengl )


/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
COpenGLScene::COpenGLScene( ) :
	m_followCamera(false)
{
	createViewport("main");
}

/*--------------------------------------------------------------
					Copy constructor
  ---------------------------------------------------------------*/
COpenGLScene::COpenGLScene( const COpenGLScene &obj ) :
	CSerializable()
{
	(*this) = obj;
}

/*---------------------------------------------------------------
						Destructor:
 ---------------------------------------------------------------*/
COpenGLScene::~COpenGLScene()
{
	clear(false);
}

/*---------------------------------------------------------------
		Clear the scene.
 ---------------------------------------------------------------*/
void  COpenGLScene::clear( bool createMainViewport  )
{
	m_viewports.clear();

	if (createMainViewport)
		createViewport("main");
}

/*---------------------------------------------------------------
						  =
 ---------------------------------------------------------------*/
COpenGLScene& COpenGLScene::operator =( const COpenGLScene &obj )
{
	if (this != &obj)
	{
		m_followCamera = obj.m_followCamera;

		clear();
		m_viewports = obj.m_viewports;
		for_each(m_viewports.begin(),m_viewports.end(), metaprogramming::ObjectMakeUnique() );
	}
	return *this;
}

/*---------------------------------------------------------------
						render
 ---------------------------------------------------------------*/
void  COpenGLScene::render() const
{
	MRPT_START

#if MRPT_HAS_OPENGL_GLUT
	// We need the size of the viewport at the beginning: should be the whole window:
	GLint	win_dims[4];
	glGetIntegerv( GL_VIEWPORT, win_dims );

	for (TListViewports::const_iterator it=m_viewports.begin();it!=m_viewports.end();++it)
		(*it)->render( win_dims[2],win_dims[3] );

	// Assure we restore the original viewport:
	glViewport( win_dims[0],win_dims[1],win_dims[2],win_dims[3] );

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENGL_GLUT=0! OpenGL functions are not implemented");
#endif
	MRPT_END
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  COpenGLScene::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << m_followCamera;

		uint32_t	n;
		n = (uint32_t)m_viewports.size();
		out << n;
		for (TListViewports::const_iterator	it=m_viewports.begin();it!=m_viewports.end();++it)
			out << **it;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  COpenGLScene::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			// Old style: Just one viewport:
			clear(true);
			COpenGLViewportPtr view = m_viewports[0];

			// Load objects:
			uint32_t	n;
			in >> n;

			view->clear();
			view->m_objects.resize(n);
			for_each(view->m_objects.begin(), view->m_objects.end(), metaprogramming::ObjectReadFromStream(&in) );
		}
		break;
	case 1:
		{
			in >> m_followCamera;

			uint32_t	i,n;
			in >> n;
			clear(false);


			for (i=0;i<n;i++)
			{
				CSerializablePtr newObj;
				in >> newObj;

				COpenGLViewportPtr	newView = COpenGLViewportPtr(newObj);
				newView->m_parent = this;
				m_viewports.push_back( newView );
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
							insert
  ---------------------------------------------------------------*/
void COpenGLScene::insert( const CRenderizablePtr &newObject, const std::string &viewportName )
{
	MRPT_START
	for (TListViewports::iterator it = m_viewports.begin();it!= m_viewports.end();++it)
	{
		if ((*it)->m_name == viewportName )
		{
			(*it)->insert(newObject);
			return;
		}
	}
	THROW_EXCEPTION_CUSTOM_MSG1("Error: viewport '%s' not found.",viewportName.c_str());
	MRPT_END
}

/*---------------------------------------------------------------
							getByName
  ---------------------------------------------------------------*/
CRenderizablePtr	COpenGLScene::getByName( const string &str, const string &viewportName )
{
	MRPT_UNUSED_PARAM(viewportName);
	CRenderizablePtr obj;
	for (TListViewports::iterator it=m_viewports.begin();it!=m_viewports.end();++it)
		if ( (obj = (*it)->getByName(str) ).present() )
			break;
	return obj;
}

/*---------------------------------------------------------------
					initializeAllTextures
  ---------------------------------------------------------------*/
void  COpenGLScene::initializeAllTextures()
{
	for (TListViewports::iterator it=m_viewports.begin();it!=m_viewports.end();++it)
		(*it)->initializeAllTextures();
}

/*--------------------------------------------------------------
					dumpListOfObjects
  ---------------------------------------------------------------*/
void COpenGLScene::dumpListOfObjects( utils::CStringList  &lst )
{
	lst.clear();

	for (TListViewports::iterator	it=m_viewports.begin();it!=m_viewports.end();++it)
	{
		lst.add( string("VIEWPORT: ")+ (*it)->m_name );
		lst.add("============================================");
		(*it)->dumpListOfObjects(lst);
	}
}



/*--------------------------------------------------------------
					createViewport
  ---------------------------------------------------------------*/
COpenGLViewportPtr COpenGLScene::createViewport( const string &viewportName )
{
	MRPT_START

	COpenGLViewportPtr old = getViewport(viewportName);
	if (old)
		return old;

	COpenGLViewportPtr theNew = COpenGLViewportPtr( new COpenGLViewport(this, viewportName) );
	m_viewports.push_back(theNew);
	return  theNew;

	MRPT_END
}

/*--------------------------------------------------------------
					getViewport
  ---------------------------------------------------------------*/
COpenGLViewportPtr COpenGLScene::getViewport( const std::string &viewportName ) const
{
	MRPT_START
	for (TListViewports::const_iterator it = m_viewports.begin();it!=m_viewports.end();++it)
		if ( (*it)->m_name == viewportName)
			return *it;
	return COpenGLViewportPtr();
	MRPT_END
}


/*--------------------------------------------------------------
					removeObject
  ---------------------------------------------------------------*/
void COpenGLScene::removeObject( const CRenderizablePtr &obj, const std::string &viewportName )
{
	MRPT_START

	COpenGLViewportPtr view = getViewport(viewportName);
	ASSERT_(view.present());
	view->removeObject(obj);

	MRPT_END
}

bool COpenGLScene::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	bool found=false;
	double tmp;
	for (TListViewports::const_iterator it=m_viewports.begin();it!=m_viewports.end();++it)	{
		const COpenGLViewportPtr &vp=*it;
		for (CListOpenGLObjects::const_iterator it2=vp->m_objects.begin();it2!=vp->m_objects.end();++it2) if ((*it2)->traceRay(o,tmp))	{
			if (!found)	{
				found=true;
				dist=tmp;
			}	else if (tmp<dist) dist=tmp;
		}
	}
	return found;
}


bool COpenGLScene::saveToFile(const std::string &fil) const
{
	try {
		CFileGZOutputStream(fil) << *this;
		return true;
	}
	catch (...)	{ return false;	}
}

bool COpenGLScene::loadFromFile(const std::string &fil)
{
	try {
		CFileGZInputStream(fil) >> *this;
		return true;
	}
	catch (...)	{ return false;	}
}


/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
void COpenGLScene::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max,const std::string &vpn) const
{
	COpenGLViewportPtr vp = this->getViewport(vpn);
	ASSERTMSG_(vp.present(), "No opengl viewport exists with the given name")

	return vp->getBoundingBox(bb_min, bb_max);
}
