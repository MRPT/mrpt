/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>

#include <algorithm>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

#include <mrpt/serialization/metaprogramming_serialization.h>
using namespace mrpt::serialization::metaprogramming;

IMPLEMENTS_SERIALIZABLE(CSetOfObjects, CRenderizable, mrpt::opengl)

void CSetOfObjects::clear() { m_objects.clear(); }

void CSetOfObjects::renderUpdateBuffers() const
{
	// Do nothing:
}

void CSetOfObjects::render(const RenderContext& rc) const
{
	// Do nothing: the enqueueForRenderRecursive() does the actual job.
}

void CSetOfObjects::enqueueForRenderRecursive(
	const mrpt::opengl::TRenderMatrices& state, RenderQueue& rq,
	bool wholeInView) const
{
	mrpt::opengl::enqueueForRendering(m_objects, state, rq, wholeInView);
}

uint8_t CSetOfObjects::serializeGetVersion() const { return 0; }
void CSetOfObjects::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);

	out.WriteAs<uint32_t>(m_objects.size());
	for (const auto& m_object : m_objects)
		out << *m_object;
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
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CSetOfObjects::insert(const CRenderizable::Ptr& newObject)
{
	ASSERTMSG_(
		newObject.get() != this,
		"Error: Trying to insert container into itself!");
	m_objects.push_back(newObject);
}

void CSetOfObjects::dumpListOfObjects(std::vector<std::string>& lst) const
{
	for (auto& obj : m_objects)
	{
		// Single obj:
		string s(obj->GetRuntimeClass()->className);
		if (obj->m_name.size()) s += string(" (") + obj->m_name + string(")");
		lst.emplace_back(s);

		if (obj->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::opengl))
		{
			auto* objs = dynamic_cast<CSetOfObjects*>(obj.get());

			std::vector<std::string> auxLst;
			objs->dumpListOfObjects(auxLst);
			for (const auto& i : auxLst)
				lst.emplace_back(string(" ") + i);
		}
	}
}

mrpt::containers::yaml CSetOfObjects::asYAML() const
{
	mrpt::containers::yaml d = mrpt::containers::yaml::Sequence();

	d.asSequence().resize(m_objects.size());

	for (uint32_t i = 0; i < m_objects.size(); i++)
	{
		const auto obj = m_objects.at(i);
		mrpt::containers::yaml de = mrpt::containers::yaml::Map();

		// class-specific properties:
		obj->toYAMLMap(de);

		de["index"] = i;  // type for "i" must be a stdint type
		if (!obj)
		{
			de["class"] = "nullptr";
			continue;
		}
		de["class"] = obj->GetRuntimeClass()->className;

		// Single obj:
		if (obj->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::opengl))
		{
			de["obj_children"] =
				dynamic_cast<CSetOfObjects*>(obj.get())->asYAML();
		}
		d.asSequence().at(i) = std::move(de);
	}
	return d;
}

/*--------------------------------------------------------------
					removeObject
  ---------------------------------------------------------------*/
void CSetOfObjects::removeObject(const CRenderizable::Ptr& obj)
{
	for (auto it = m_objects.begin(); it != m_objects.end(); ++it)
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
	for (const auto& m_object : m_objects)
		if (m_object->traceRay(nueva, tmp))
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
	~FSetColor() = default;
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
	for (auto& m_object : m_objects)
		m_object->setColorR_u8(m_color.R = r);
	return *this;
}

CRenderizable& CSetOfObjects::setColorG_u8(const uint8_t g)
{
	for (auto& m_object : m_objects)
		m_object->setColorG_u8(m_color.G = g);
	return *this;
}

CRenderizable& CSetOfObjects::setColorB_u8(const uint8_t b)
{
	for (auto& m_object : m_objects)
		m_object->setColorB_u8(m_color.B = b);
	return *this;
}

CRenderizable& CSetOfObjects::setColorA_u8(const uint8_t a)
{
	for (auto& m_object : m_objects)
		m_object->setColorA_u8(m_color.A = a);
	return *this;
}

/*---------------------------------------------------------------
							getByName
  ---------------------------------------------------------------*/
CRenderizable::Ptr CSetOfObjects::getByName(const string& str)
{
	for (auto& m_object : m_objects)
	{
		if (m_object->m_name == str) return m_object;
		else if (
			m_object->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, opengl))
		{
			CRenderizable::Ptr ret =
				dynamic_cast<CSetOfObjects*>(m_object.get())->getByName(str);
			if (ret) return ret;
		}
	}
	return CRenderizable::Ptr();
}

auto CSetOfObjects::internalBoundingBoxLocal() const
	-> mrpt::math::TBoundingBoxf
{
	mrpt::math::TBoundingBoxf bb;
	bool first = true;

	for (const auto& o : m_objects)
	{
		if (!o) continue;
		if (first)
		{
			bb = o->getBoundingBoxLocalf();
			first = false;
		}
		else
			bb.unionWith(o->getBoundingBoxLocalf());
	}

	return bb;
}
