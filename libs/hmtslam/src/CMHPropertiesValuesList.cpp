/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precompiled headers

#include <mrpt/hmtslam/CMHPropertiesValuesList.h>
#include <mrpt/system/os.h>
#include <cstdio>

using namespace mrpt::hmtslam;
using namespace mrpt::system;
using namespace mrpt::serialization;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMHPropertiesValuesList, CSerializable, mrpt::hmtslam)

uint8_t CMHPropertiesValuesList::serializeGetVersion() const { return 0; }
void CMHPropertiesValuesList::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	uint32_t i, n = (uint32_t)m_properties.size();
	uint8_t isNull;
	out << n;

	for (i = 0; i < n; i++)
	{
		// Name:
		out << m_properties[i].name.c_str();

		// Object:
		isNull = !m_properties[i].value;
		out << isNull;

		if (!isNull) out << *m_properties[i].value;

		// Hypot. ID:
		out << m_properties[i].ID;
	}
}

void CMHPropertiesValuesList::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint32_t i, n;
			uint8_t isNull;

			// Erase previous contents:
			clear();

			in >> n;

			m_properties.resize(n);
			for (i = 0; i < n; i++)
			{
				// Name:
				in >> m_properties[i].name;

				// Object:
				in >> isNull;

				if (isNull)
					m_properties[i].value.reset();
				else
					in >> m_properties[i].value;

				// Hypot. ID:
				in >> m_properties[i].ID;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CMHPropertiesValuesList::CMHPropertiesValuesList() = default;
/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CMHPropertiesValuesList::~CMHPropertiesValuesList() { clear(); }
/*---------------------------------------------------------------
						clear
 ---------------------------------------------------------------*/
void CMHPropertiesValuesList::clear()
{
	MRPT_START
	m_properties.clear();
	MRPT_END
}

/*---------------------------------------------------------------
						get
 ---------------------------------------------------------------*/
CSerializable::Ptr CMHPropertiesValuesList::get(
	const char* propertyName, const int64_t& hypothesis_ID) const
{
	std::vector<TPropertyValueIDTriplet>::const_iterator it;
	for (it = m_properties.begin(); it != m_properties.end(); ++it)
		if (!os::_strcmpi(propertyName, it->name.c_str()) &&
			it->ID == hypothesis_ID)
			return it->value;

	for (it = m_properties.begin(); it != m_properties.end(); ++it)
		if (!os::_strcmpi(propertyName, it->name.c_str()) && it->ID == 0)
			return it->value;

	// Not found:
	return CSerializable::Ptr();
}

/*---------------------------------------------------------------
						getAnyHypothesis
 ---------------------------------------------------------------*/
CSerializable::Ptr CMHPropertiesValuesList::getAnyHypothesis(
	const char* propertyName) const
{
	for (const auto& m_propertie : m_properties)
	{
		if (!os::_strcmpi(propertyName, m_propertie.name.c_str()))
			return m_propertie.value;
	}
	// Not found:
	return CSerializable::Ptr();
}

/*---------------------------------------------------------------
						set
 ---------------------------------------------------------------*/
void CMHPropertiesValuesList::set(
	const char* propertyName, const CSerializable::Ptr& obj,
	const int64_t& hypothesis_ID)
{
	MRPT_START

	for (auto& m_propertie : m_properties)
	{
		if (m_propertie.ID == hypothesis_ID &&
			!os::_strcmpi(propertyName, m_propertie.name.c_str()))
		{
			// Delete current contents:
			// Copy new value:
			m_propertie.value.reset(dynamic_cast<CSerializable*>(obj->clone()));

			// if (!obj)	it->value.clear();
			// else		it->value = obj; //->clone();
			return;
		}
	}

	// Insert:
	TPropertyValueIDTriplet newPair;
	newPair.name = std::string(propertyName);
	newPair.value = obj;
	newPair.ID = hypothesis_ID;
	m_properties.push_back(newPair);

	MRPT_END_WITH_CLEAN_UP(
		printf("Exception while setting annotation '%s'", propertyName););
}

/*---------------------------------------------------------------
						setMemoryReference
 ---------------------------------------------------------------*/
void CMHPropertiesValuesList::setMemoryReference(
	const char* propertyName, const CSerializable::Ptr& obj,
	const int64_t& hypothesis_ID)
{
	MRPT_START

	for (auto& m_propertie : m_properties)
	{
		if (m_propertie.ID == hypothesis_ID &&
			!os::_strcmpi(propertyName, m_propertie.name.c_str()))
		{
			// Delete current contents & set a copy of the same smart pointer:
			m_propertie.value = obj;
			return;
		}
	}

	// Insert:
	TPropertyValueIDTriplet newPair;
	newPair.name = std::string(propertyName);
	newPair.value = obj;
	newPair.ID = hypothesis_ID;
	m_properties.push_back(newPair);

	MRPT_END_WITH_CLEAN_UP(
		printf("Exception while setting annotation '%s'", propertyName););
}

/*---------------------------------------------------------------
						getPropertyNames
 ---------------------------------------------------------------*/
std::vector<std::string> CMHPropertiesValuesList::getPropertyNames() const
{
	std::vector<std::string> ret;

	for (const auto& m_propertie : m_properties)
	{
		bool isNew = true;
		for (auto& itS : ret)
		{
			if (itS == m_propertie.name)
			{
				isNew = false;
				break;
			}
		}
		if (isNew) ret.push_back(m_propertie.name);  // Yes, it is new:
	}

	return ret;
}

/*---------------------------------------------------------------
						remove
 ---------------------------------------------------------------*/
void CMHPropertiesValuesList::remove(
	const char* propertyName, const int64_t& hypothesis_ID)
{
	for (auto it = m_properties.begin(); it != m_properties.end();)
		if (!os::_strcmpi(propertyName, it->name.c_str()) &&
			it->ID == hypothesis_ID)
			it = m_properties.erase(it);
		else
			++it;
}

/*---------------------------------------------------------------
						removeAll
 ---------------------------------------------------------------*/
void CMHPropertiesValuesList::removeAll(const int64_t& hypothesis_ID)
{
	for (auto it = m_properties.begin(); it != m_properties.end();)
		if (it->ID == hypothesis_ID)
			it = m_properties.erase(it);
		else
			++it;
}

/*---------------------------------------------------------------
						Copy
 ---------------------------------------------------------------*/
CMHPropertiesValuesList::CMHPropertiesValuesList(
	const CMHPropertiesValuesList& o)
	: m_properties(o.m_properties)
{
	for (auto& m_propertie : m_properties)
		m_propertie.value.reset(
			dynamic_cast<CSerializable*>(m_propertie.value->clone()));
}

/*---------------------------------------------------------------
						Copy
 ---------------------------------------------------------------*/
CMHPropertiesValuesList& CMHPropertiesValuesList::operator=(
	const CMHPropertiesValuesList& o)
{
	if (this == &o) return *this;

	m_properties = o.m_properties;

	for (auto& m_propertie : m_properties)
		m_propertie.value.reset(
			dynamic_cast<CSerializable*>(m_propertie.value->clone()));
	return *this;
}
