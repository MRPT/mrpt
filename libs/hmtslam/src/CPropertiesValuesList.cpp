/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precompiled headers

#include <mrpt/hmtslam/CPropertiesValuesList.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>
#include <cstdio>
#include <iostream>

using namespace mrpt::system;
using namespace mrpt::hmtslam;
using namespace mrpt::serialization;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CPropertiesValuesList, CSerializable, mrpt::hmtslam)

uint8_t CPropertiesValuesList::serializeGetVersion() const { return 0; }
void CPropertiesValuesList::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	uint32_t i, n = (uint32_t)size();
	uint8_t isNull;
	out << n;

	for (i = 0; i < n; i++)
	{
		// Name:
		out << m_properties[i].name.c_str();

		// Object:
		isNull = m_properties[i].value ? 1 : 0;
		out << isNull;

		if (m_properties[i].value) out << *m_properties[i].value;
	}
}

void CPropertiesValuesList::serializeFrom(
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
					m_properties[i].value.reset(
						static_cast<CSerializable*>(nullptr));
				else
					in >> m_properties[i].value;
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
CPropertiesValuesList::CPropertiesValuesList() = default;
/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CPropertiesValuesList::~CPropertiesValuesList() { clear(); }
/*---------------------------------------------------------------
					Copy Constructor
 ---------------------------------------------------------------*/
CPropertiesValuesList::CPropertiesValuesList(const CPropertiesValuesList& o)
	: m_properties(o.m_properties)
{
	for (auto& m_propertie : m_properties)
		m_propertie.value.reset(
			dynamic_cast<CSerializable*>(m_propertie.value->clone()));
}

/*---------------------------------------------------------------
					Copy
 ---------------------------------------------------------------*/
CPropertiesValuesList& CPropertiesValuesList::operator=(
	const CPropertiesValuesList& o)
{
	if (this != &o) return *this;

	m_properties = o.m_properties;
	for (auto& m_propertie : m_properties)
		m_propertie.value.reset(
			dynamic_cast<CSerializable*>(m_propertie.value->clone()));
	return *this;
}

/*---------------------------------------------------------------
						clear
 ---------------------------------------------------------------*/
void CPropertiesValuesList::clear()
{
	MRPT_START
	m_properties.clear();
	MRPT_END
}

/*---------------------------------------------------------------
						get
 ---------------------------------------------------------------*/
CSerializable::Ptr CPropertiesValuesList::get(
	const std::string& propertyName) const
{
	for (const auto& m_propertie : m_properties)
	{
		if (!os::_strcmpi(propertyName.c_str(), m_propertie.name.c_str()))
			return m_propertie.value;
	}
	// Not found:
	return CSerializable::Ptr();
}

/*---------------------------------------------------------------
						set
 ---------------------------------------------------------------*/
void CPropertiesValuesList::set(
	const std::string& propertyName, const CSerializable::Ptr& obj)
{
	MRPT_START

	for (auto& m_propertie : m_properties)
	{
		if (!os::_strcmpi(propertyName.c_str(), m_propertie.name.c_str()))
		{
			// Delete current contents:
			// Copy new value:
			if (!obj)
				m_propertie.value.reset();
			else
				m_propertie.value = obj;  //->clone();
			return;
		}
	}

	// Insert:
	TPropertyValuePair newPair;
	newPair.name = std::string(propertyName);
	newPair.value = obj;
	m_properties.push_back(newPair);

	MRPT_END_WITH_CLEAN_UP(
		printf(
			"Exception while setting annotation '%s'", propertyName.c_str()););
}

/*---------------------------------------------------------------
						size
 ---------------------------------------------------------------*/
size_t CPropertiesValuesList::size() const { return m_properties.size(); }
/*---------------------------------------------------------------
						getPropertyNames
 ---------------------------------------------------------------*/
std::vector<std::string> CPropertiesValuesList::getPropertyNames() const
{
	std::vector<std::string> ret;

	for (const auto& m_propertie : m_properties)
		ret.push_back(m_propertie.name);

	return ret;
}
