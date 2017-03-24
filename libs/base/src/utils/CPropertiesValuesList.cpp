/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CPropertiesValuesList.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>
#include <stdio.h>
#include <iostream>

using namespace mrpt::utils;
using namespace mrpt::system;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CPropertiesValuesList, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CPropertiesValuesList::writeToStream(mrpt::utils::CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		uint32_t	i,n = (uint32_t)size();
		uint8_t		isNull;
		out << n;

		for (i=0;i<n;i++)
		{
			// Name:
			out << m_properties[i].name.c_str();

			// Object:
			isNull = m_properties[i].value.present() ? 1:0;
			out << isNull;

			if (m_properties[i].value.present())
				out << *m_properties[i].value;
		}
	}
}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CPropertiesValuesList::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	i,n;
			uint8_t		isNull;

			// Erase previous contents:
			clear();

			in >> n;

			m_properties.resize(n);
			for (i=0;i<n;i++)
			{
				char	nameBuf[1024];
				// Name:
				in >> nameBuf;
				m_properties[i].name = nameBuf;

				// Object:
				in >> isNull;

				if (isNull)
					m_properties[i].value.set(  static_cast<CSerializable*>(NULL) );
				else
					in >> m_properties[i].value;
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CPropertiesValuesList::CPropertiesValuesList()
{
}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CPropertiesValuesList::~CPropertiesValuesList()
{
	clear();
}

/*---------------------------------------------------------------
					Copy Constructor
 ---------------------------------------------------------------*/
CPropertiesValuesList::CPropertiesValuesList(const CPropertiesValuesList &o) :
	m_properties	( o.m_properties )
{
	for (std::vector<TPropertyValuePair>::iterator it=m_properties.begin();it!=m_properties.end();++it)
		it->value.make_unique();
}

/*---------------------------------------------------------------
					Copy
 ---------------------------------------------------------------*/
CPropertiesValuesList & CPropertiesValuesList::operator = (const CPropertiesValuesList &o)
{
	if (this!=&o) return *this;

	m_properties = o.m_properties;
	for (std::vector<TPropertyValuePair>::iterator it=m_properties.begin();it!=m_properties.end();++it)
		it->value.make_unique();
	return *this;
}


/*---------------------------------------------------------------
						clear
 ---------------------------------------------------------------*/
void  CPropertiesValuesList::clear()
{
	MRPT_START
	m_properties.clear();
	MRPT_END
}

/*---------------------------------------------------------------
						get
 ---------------------------------------------------------------*/
CSerializablePtr  CPropertiesValuesList::get(const std::string &propertyName)const
{
	for (std::vector<TPropertyValuePair>::const_iterator it=m_properties.begin();it!=m_properties.end();++it)
	{
		if (!os::_strcmpi(propertyName.c_str(),it->name.c_str()))
			return it->value;
	}
	// Not found:
	return CSerializablePtr();
}


/*---------------------------------------------------------------
						set
 ---------------------------------------------------------------*/
void  CPropertiesValuesList::set(const std::string &propertyName, const CSerializablePtr &obj)
{
	MRPT_START

	for (std::vector<TPropertyValuePair>::iterator it=m_properties.begin();it!=m_properties.end();++it)
	{
		if (!os::_strcmpi(propertyName.c_str(),it->name.c_str()))
		{
			// Delete current contents:
			// Copy new value:
			if (!obj)	it->value.clear_unique();
			else		it->value = obj; //->duplicate();
			return;
		}
	}

	// Insert:
	TPropertyValuePair	newPair;
	newPair.name = std::string(propertyName);
	newPair.value = obj;
	m_properties.push_back(newPair);

	MRPT_END_WITH_CLEAN_UP( \
		printf("Exception while setting annotation '%s'",propertyName.c_str()); \
		);
}

/*---------------------------------------------------------------
						size
 ---------------------------------------------------------------*/
size_t  CPropertiesValuesList::size()const
{
	return m_properties.size();
}

/*---------------------------------------------------------------
						getPropertyNames
 ---------------------------------------------------------------*/
std::vector<std::string>  CPropertiesValuesList::getPropertyNames()const
{
	std::vector<std::string>	ret;

	for (std::vector<TPropertyValuePair>::const_iterator it=m_properties.begin();it!=m_properties.end();++it)
		ret.push_back(it->name);

	return ret;
}
