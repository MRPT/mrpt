/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::serialization::metaprogramming;
using namespace mrpt::math;
using namespace std;

// Include libraries in linking:
#if MRPT_HAS_OPENGL_GLUT
#ifdef _WIN32
// WINDOWS:
#if defined(_MSC_VER)
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "GlU32.lib")
#endif
#endif  // _WIN32
#endif  // MRPT_HAS_OPENGL_GLUT

IMPLEMENTS_SERIALIZABLE(COpenGLScene, CRenderizable, mrpt::opengl)

/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
COpenGLScene::COpenGLScene() { createViewport("main"); }
/*--------------------------------------------------------------
					Copy constructor
  ---------------------------------------------------------------*/
COpenGLScene::COpenGLScene(const COpenGLScene& obj) : CSerializable()
{
	(*this) = obj;
}

COpenGLScene::~COpenGLScene() { m_viewports.clear(); }

void COpenGLScene::clear(bool createMainViewport)
{
	m_viewports.clear();

	if (createMainViewport) createViewport("main");
}

/*---------------------------------------------------------------
						  =
 ---------------------------------------------------------------*/
COpenGLScene& COpenGLScene::operator=(const COpenGLScene& obj)
{
	if (this != &obj)
	{
		m_followCamera = obj.m_followCamera;

		clear();
		m_viewports = obj.m_viewports;
		for_each(m_viewports.begin(), m_viewports.end(), [](auto& ptr) {
			// make a unique copy of each object (copied as a shared ptr)
			ptr.reset(
				dynamic_cast<mrpt::opengl::COpenGLViewport*>(ptr->clone()));
		});
	}
	return *this;
}

/*---------------------------------------------------------------
						render
 ---------------------------------------------------------------*/
void COpenGLScene::render() const
{
	MRPT_START

#if MRPT_HAS_OPENGL_GLUT
	// We need the size of the viewport at the beginning: should be the whole
	// window:
	GLint win_dims[4];
	glGetIntegerv(GL_VIEWPORT, win_dims);
	CHECK_OPENGL_ERROR();

	for (const auto& m_viewport : m_viewports)
		m_viewport->render(win_dims[2], win_dims[3]);

	// Assure we restore the original viewport:
	glViewport(win_dims[0], win_dims[1], win_dims[2], win_dims[3]);
	CHECK_OPENGL_ERROR();

#else
	THROW_EXCEPTION(
		"The MRPT has been compiled with MRPT_HAS_OPENGL_GLUT=0! OpenGL "
		"functions are not implemented");
#endif
	MRPT_END
}

uint8_t COpenGLScene::serializeGetVersion() const { return 1; }
void COpenGLScene::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << m_followCamera;

	uint32_t n;
	n = (uint32_t)m_viewports.size();
	out << n;
	for (const auto& m_viewport : m_viewports) out << *m_viewport;
}

void COpenGLScene::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			// Old style: Just one viewport:
			clear(true);
			COpenGLViewport::Ptr view = m_viewports[0];

			// Load objects:
			uint32_t n;
			in >> n;

			view->clear();
			view->m_objects.resize(n);
			for_each(
				view->m_objects.begin(), view->m_objects.end(),
				ObjectReadFromStream(&in));
		}
		break;
		case 1:
		{
			in >> m_followCamera;

			uint32_t i, n;
			in >> n;
			clear(false);

			for (i = 0; i < n; i++)
			{
				CSerializable::Ptr newObj;
				in >> newObj;

				COpenGLViewport::Ptr newView =
					std::dynamic_pointer_cast<COpenGLViewport>(newObj);
				newView->m_parent = this;
				m_viewports.push_back(newView);
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
							insert
  ---------------------------------------------------------------*/
void COpenGLScene::insert(
	const CRenderizable::Ptr& newObject, const std::string& viewportName)
{
	MRPT_START
	for (auto& m_viewport : m_viewports)
	{
		if (m_viewport->m_name == viewportName)
		{
			m_viewport->insert(newObject);
			return;
		}
	}
	THROW_EXCEPTION_FMT(
		"Error: viewport '%s' not found.", viewportName.c_str());
	MRPT_END
}

/*---------------------------------------------------------------
							getByName
  ---------------------------------------------------------------*/
CRenderizable::Ptr COpenGLScene::getByName(
	const string& str, const string& viewportName)
{
	MRPT_UNUSED_PARAM(viewportName);
	CRenderizable::Ptr obj;
	for (auto& m_viewport : m_viewports)
		if ((obj = m_viewport->getByName(str))) break;
	return obj;
}

/*---------------------------------------------------------------
					initializeAllTextures
  ---------------------------------------------------------------*/
void COpenGLScene::initializeAllTextures()
{
	for (auto& m_viewport : m_viewports) m_viewport->initializeAllTextures();
}

/*--------------------------------------------------------------
					dumpListOfObjects
  ---------------------------------------------------------------*/
void COpenGLScene::dumpListOfObjects(std::vector<std::string>& lst)
{
	lst.clear();

	for (auto& v : m_viewports)
	{
		lst.emplace_back(string("VIEWPORT: ") + v->m_name);
		lst.emplace_back("============================================");
		v->dumpListOfObjects(lst);
	}
}

/*--------------------------------------------------------------
					createViewport
  ---------------------------------------------------------------*/
COpenGLViewport::Ptr COpenGLScene::createViewport(const string& viewportName)
{
	MRPT_START

	COpenGLViewport::Ptr old = getViewport(viewportName);
	if (old) return old;

	auto theNew = std::make_shared<COpenGLViewport>(this, viewportName);
	m_viewports.push_back(theNew);
	return theNew;

	MRPT_END
}

/*--------------------------------------------------------------
					getViewport
  ---------------------------------------------------------------*/
COpenGLViewport::Ptr COpenGLScene::getViewport(
	const std::string& viewportName) const
{
	MRPT_START
	for (const auto& m_viewport : m_viewports)
		if (m_viewport->m_name == viewportName) return m_viewport;
	return COpenGLViewport::Ptr();
	MRPT_END
}

/*--------------------------------------------------------------
					removeObject
  ---------------------------------------------------------------*/
void COpenGLScene::removeObject(
	const CRenderizable::Ptr& obj, const std::string& viewportName)
{
	MRPT_START

	COpenGLViewport::Ptr view = getViewport(viewportName);
	ASSERT_(view);
	view->removeObject(obj);

	MRPT_END
}

bool COpenGLScene::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	bool found = false;
	double tmp;
	for (const auto& vp : m_viewports)
	{
		for (auto it2 = vp->m_objects.begin(); it2 != vp->m_objects.end();
			 ++it2)
			if ((*it2)->traceRay(o, tmp))
			{
				if (!found)
				{
					found = true;
					dist = tmp;
				}
				else if (tmp < dist)
					dist = tmp;
			}
	}
	return found;
}

bool COpenGLScene::saveToFile(const std::string& fil) const
{
	try
	{
		mrpt::io::CFileGZOutputStream f(fil);
		mrpt::serialization::archiveFrom(f) << *this;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

bool COpenGLScene::loadFromFile(const std::string& fil)
{
	try
	{
		mrpt::io::CFileGZInputStream f(fil);
		mrpt::serialization::archiveFrom(f) >> *this;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

/** Evaluates the bounding box of this object (including possible children) in
 * the coordinate frame of the object parent. */
void COpenGLScene::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max,
	const std::string& vpn) const
{
	COpenGLViewport::Ptr vp = this->getViewport(vpn);
	ASSERTMSG_(vp, "No opengl viewport exists with the given name");

	return vp->getBoundingBox(bb_min, bb_max);
}
