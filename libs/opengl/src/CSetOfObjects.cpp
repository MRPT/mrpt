/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/opengl/gl_utils.h>

#include "opengl_internals.h"
#include <algorithm>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;

using namespace mrpt::math;
using namespace std;

#include <mrpt/serialization/metaprogramming_serialization.h>
using namespace mrpt::serialization::metaprogramming;

IMPLEMENTS_SERIALIZABLE(CSetOfObjects, CRenderizable, mrpt::opengl)

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CSetOfObjects::clear()
{
	m_objects.clear();  // clear the list and delete objects (if there are no
	// more copies out there!)
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CSetOfObjects::render() const
{
	// Render all the objects:
	mrpt::opengl::gl_utils::renderSetOfObjects(m_objects);
}

uint8_t CSetOfObjects::serializeGetVersion() const { return 0; }
void CSetOfObjects::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);

	out.WriteAs<uint32_t>(m_objects.size());
	for (CListOpenGLObjects::const_iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
		out << **it;
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void CSetOfObjects::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);

			uint32_t n;
			in >> n;
			clear();
			m_objects.resize(n);

			for_each(
				m_objects.begin(), m_objects.end(), ObjectReadFromStream(&in));
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
					initializeAllTextures
  ---------------------------------------------------------------*/
void CSetOfObjects::initializeAllTextures()
{
#if MRPT_HAS_OPENGL_GLUT
	CListOpenGLObjects::iterator it;
	for (it = m_objects.begin(); it != m_objects.end(); ++it++)
	{
		if (IS_DERIVED(*it, CTexturedObject))
			dynamic_cast<CTexturedObject*>(it->get())->loadTextureInOpenGL();
		else if (IS_CLASS(*it, CSetOfObjects))
			dynamic_cast<CSetOfObjects*>(it->get())->initializeAllTextures();
	}
#endif
}

CSetOfObjects::CSetOfObjects() {}
CSetOfObjects::~CSetOfObjects() { clear(); }
void CSetOfObjects::insert(const CRenderizable::Ptr& newObject)
{
	ASSERTMSG_(
		newObject.get() != this,
		"Error: Trying to insert container into itself!");
	m_objects.push_back(newObject);
}

/*--------------------------------------------------------------
					dumpListOfObjects
  ---------------------------------------------------------------*/
void CSetOfObjects::dumpListOfObjects(std::vector<std::string>& lst)
{
	for (CListOpenGLObjects::iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
	{
		// Single obj:
		string s((*it)->GetRuntimeClass()->className);
		if ((*it)->m_name.size())
			s += string(" (") + (*it)->m_name + string(")");
		lst.emplace_back(s);

		if ((*it)->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::opengl))
		{
			CSetOfObjects* objs = dynamic_cast<CSetOfObjects*>(it->get());

			std::vector<std::string> auxLst;
			objs->dumpListOfObjects(auxLst);
			for (size_t i = 0; i < auxLst.size(); i++)
				lst.emplace_back(string(" ") + auxLst[i]);
		}
	}
}

/*--------------------------------------------------------------
					removeObject
  ---------------------------------------------------------------*/
void CSetOfObjects::removeObject(const CRenderizable::Ptr& obj)
{
	for (CListOpenGLObjects::iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
		if (*it == obj)
		{
			m_objects.erase(it);
			return;
		}
		else if (
			(*it)->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, opengl))
			dynamic_cast<CSetOfObjects*>(it->get())->removeObject(obj);
}

bool CSetOfObjects::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	CPose3D nueva = (CPose3D() - this->m_pose) + o;
	bool found = false;
	double tmp;
	for (CListOpenGLObjects::const_iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
		if ((*it)->traceRay(nueva, tmp))
		{
			if (!found)
			{
				found = true;
				dist = tmp;
			}
			else if (tmp < dist)
				dist = tmp;
		}
	return found;
}

class FSetColor
{
   public:
	uint8_t r, g, b, a;
	void operator()(CRenderizable::Ptr& p) { p->setColor_u8(r, g, b, a); }
	FSetColor(uint8_t R, uint8_t G, uint8_t B, uint8_t A)
		: r(R), g(G), b(B), a(A)
	{
	}
	~FSetColor() {}
};

CRenderizable& CSetOfObjects::setColor_u8(const mrpt::img::TColor& c)
{
	for_each(
		m_objects.begin(), m_objects.end(),
		FSetColor(
			m_color.R = c.R, m_color.G = c.G, m_color.B = c.B,
			m_color.A = c.A));
	return *this;
}

bool CSetOfObjects::contains(const CRenderizable::Ptr& obj) const
{
	return find(m_objects.begin(), m_objects.end(), obj) != m_objects.end();
}

CRenderizable& CSetOfObjects::setColorR_u8(const uint8_t r)
{
	for (CListOpenGLObjects::iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
		(*it)->setColorR_u8(m_color.R = r);
	return *this;
}

CRenderizable& CSetOfObjects::setColorG_u8(const uint8_t g)
{
	for (CListOpenGLObjects::iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
		(*it)->setColorG_u8(m_color.G = g);
	return *this;
}

CRenderizable& CSetOfObjects::setColorB_u8(const uint8_t b)
{
	for (CListOpenGLObjects::iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
		(*it)->setColorB_u8(m_color.B = b);
	return *this;
}

CRenderizable& CSetOfObjects::setColorA_u8(const uint8_t a)
{
	for (CListOpenGLObjects::iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
		(*it)->setColorA_u8(m_color.A = a);
	return *this;
}

/*---------------------------------------------------------------
							getByName
  ---------------------------------------------------------------*/
CRenderizable::Ptr CSetOfObjects::getByName(const string& str)
{
	for (CListOpenGLObjects::iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
	{
		if ((*it)->m_name == str)
			return *it;
		else if (
			(*it)->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, opengl))
		{
			CRenderizable::Ptr ret =
				dynamic_cast<CSetOfObjects*>(it->get())->getByName(str);
			if (ret) return ret;
		}
	}
	return CRenderizable::Ptr();
}

/** Evaluates the bounding box of this object (including possible children) in
 * the coordinate frame of the object parent. */
void CSetOfObjects::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min = TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb_max = TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());

	for (CListOpenGLObjects::const_iterator it = m_objects.begin();
		 it != m_objects.end(); ++it)
	{
		TPoint3D child_bbmin(
			std::numeric_limits<double>::max(),
			std::numeric_limits<double>::max(),
			std::numeric_limits<double>::max());
		TPoint3D child_bbmax(
			-std::numeric_limits<double>::max(),
			-std::numeric_limits<double>::max(),
			-std::numeric_limits<double>::max());
		(*it)->getBoundingBox(child_bbmin, child_bbmax);

		keep_min(bb_min.x, child_bbmin.x);
		keep_min(bb_min.y, child_bbmin.y);
		keep_min(bb_min.z, child_bbmin.z);

		keep_max(bb_max.x, child_bbmax.x);
		keep_max(bb_max.y, child_bbmax.y);
		keep_max(bb_max.z, child_bbmax.z);
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
