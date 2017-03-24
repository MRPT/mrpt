/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CMHPropertiesValuesList.h>
#include <mrpt/system/os.h>
#include <stdio.h>

using namespace mrpt::utils;
using namespace mrpt::system;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMHPropertiesValuesList, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::writeToStream(mrpt::utils::CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		uint32_t	i,n = (uint32_t)m_properties.size();
		uint8_t		isNull;
		out << n;

		for (i=0;i<n;i++)
		{
			// Name:
			out << m_properties[i].name.c_str();

			// Object:
			isNull = !m_properties[i].value;
			out << isNull;

			if (!isNull)
				out << *m_properties[i].value;

			// Hypot. ID:
			out << m_properties[i].ID;
		}

	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::readFromStream(mrpt::utils::CStream &in, int version)
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
					m_properties[i].value.clear();
				else
					in >> m_properties[i].value;

				// Hypot. ID:
				in >> m_properties[i].ID;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CMHPropertiesValuesList::CMHPropertiesValuesList()
{

}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CMHPropertiesValuesList::~CMHPropertiesValuesList()
{
	clear();
}

/*---------------------------------------------------------------
						clear
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::clear()
{
	MRPT_START
	m_properties.clear();
	MRPT_END
}

/*---------------------------------------------------------------
						get
 ---------------------------------------------------------------*/
CSerializablePtr  CMHPropertiesValuesList::get(
	const char    *propertyName,
	const int64_t &hypothesis_ID) const
{
	std::vector<TPropertyValueIDTriplet>::const_iterator it;
	for (it=m_properties.begin();it!=m_properties.end();++it)
		if (!os::_strcmpi(propertyName,it->name.c_str()) && it->ID == hypothesis_ID )
			return it->value;

	for (it=m_properties.begin();it!=m_properties.end();++it)
		if (!os::_strcmpi(propertyName,it->name.c_str()) && it->ID == 0 )
			return it->value;

	// Not found:
	return CSerializablePtr();
}

/*---------------------------------------------------------------
						getAnyHypothesis
 ---------------------------------------------------------------*/
CSerializablePtr  CMHPropertiesValuesList::getAnyHypothesis(const char *propertyName) const
{
	for (std::vector<TPropertyValueIDTriplet>::const_iterator it=m_properties.begin();it!=m_properties.end();++it)
	{
		if (!os::_strcmpi(propertyName,it->name.c_str()) )
			return it->value;
	}
	// Not found:
	return CSerializablePtr();
}

/*---------------------------------------------------------------
						set
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::set(
	const char *propertyName,
	const CSerializablePtr &obj,
	const int64_t &hypothesis_ID)
{
	MRPT_START

	for (std::vector<TPropertyValueIDTriplet>::iterator it=m_properties.begin();it!=m_properties.end();++it)
	{
		if ( it->ID == hypothesis_ID && !os::_strcmpi(propertyName,it->name.c_str()) )
		{
			// Delete current contents:
			// Copy new value:
			it->value = obj;
			it->value.make_unique();

			//if (!obj)	it->value.clear();
			//else		it->value = obj; //->duplicate();
			return;
		}
	}

	// Insert:
	TPropertyValueIDTriplet	newPair;
	newPair.name = std::string(propertyName);
	newPair.value = obj;
	newPair.ID    = hypothesis_ID;
	m_properties.push_back(newPair);

	MRPT_END_WITH_CLEAN_UP( \
		printf("Exception while setting annotation '%s'",propertyName); \
		);
}

/*---------------------------------------------------------------
						setMemoryReference
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::setMemoryReference(
	const char *propertyName,
	const CSerializablePtr &obj,
	const int64_t & hypothesis_ID)
{
	MRPT_START

	for (std::vector<TPropertyValueIDTriplet>::iterator it=m_properties.begin();it!=m_properties.end();++it)
	{
		if ( it->ID == hypothesis_ID && !os::_strcmpi(propertyName,it->name.c_str()) )
		{
			// Delete current contents & set a copy of the same smart pointer:
			it->value = obj;
			return;
		}
	}

	// Insert:
	TPropertyValueIDTriplet	newPair;
	newPair.name = std::string(propertyName);
	newPair.value = obj;
	newPair.ID    = hypothesis_ID;
	m_properties.push_back(newPair);

	MRPT_END_WITH_CLEAN_UP( \
		printf("Exception while setting annotation '%s'",propertyName); \
		);
}



/*---------------------------------------------------------------
						getPropertyNames
 ---------------------------------------------------------------*/
std::vector<std::string>  CMHPropertiesValuesList::getPropertyNames() const
{
	std::vector<std::string>	ret;

	for (std::vector<TPropertyValueIDTriplet>::const_iterator it=m_properties.begin();it!=m_properties.end();++it)
	{
		bool isNew = true;
		for (std::vector<std::string>::iterator itS = ret.begin(); itS!=ret.end(); ++itS)
		{
			if ( (*itS) == it->name )
			{
				isNew = false;
				break;
			}
		}
		if (isNew)
			ret.push_back(it->name);	// Yes, it is new:
	}

	return ret;
}

/*---------------------------------------------------------------
						remove
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::remove(
	const char *propertyName,
	const int64_t & hypothesis_ID)
{
	for (std::vector<TPropertyValueIDTriplet>::iterator it=m_properties.begin();it!=m_properties.end(); )
		if (!os::_strcmpi(propertyName,it->name.c_str()) && it->ID == hypothesis_ID )
				it = m_properties.erase(it);
		else	++it;
}

/*---------------------------------------------------------------
						removeAll
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::removeAll( const int64_t & hypothesis_ID)
{
	for (std::vector<TPropertyValueIDTriplet>::iterator it=m_properties.begin();it!=m_properties.end();)
		if (it->ID == hypothesis_ID )
				it = m_properties.erase(it);
		else	++it;
}



/*---------------------------------------------------------------
						Copy
 ---------------------------------------------------------------*/
CMHPropertiesValuesList::CMHPropertiesValuesList( const CMHPropertiesValuesList& o ) :
	m_properties ( o.m_properties )
{
	for (std::vector<TPropertyValueIDTriplet>::iterator it=m_properties.begin();it!=m_properties.end();++it)
		it->value.make_unique();
}

/*---------------------------------------------------------------
						Copy
 ---------------------------------------------------------------*/
CMHPropertiesValuesList & CMHPropertiesValuesList::operator =( const CMHPropertiesValuesList& o )
{
	if (this==&o) return *this;

	m_properties = o.m_properties;

	for (std::vector<TPropertyValueIDTriplet>::iterator it=m_properties.begin();it!=m_properties.end();++it)
		it->value.make_unique();
	return *this;
}

