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

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/CMHPropertiesValuesList.h>
using namespace mrpt::utils;
using namespace mrpt::system;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMHPropertiesValuesList, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::writeToStream(CStream &out, int *out_Version) const
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
void  CMHPropertiesValuesList::readFromStream(CStream &in, int version)
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
	for (it=m_properties.begin();it!=m_properties.end();it++)
		if (!os::_strcmpi(propertyName,it->name.c_str()) && it->ID == hypothesis_ID )
			return it->value;

	for (it=m_properties.begin();it!=m_properties.end();it++)
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
		else	it++;
}

/*---------------------------------------------------------------
						removeAll
 ---------------------------------------------------------------*/
void  CMHPropertiesValuesList::removeAll( const int64_t & hypothesis_ID)
{
	for (std::vector<TPropertyValueIDTriplet>::iterator it=m_properties.begin();it!=m_properties.end();)
		if (it->ID == hypothesis_ID )
				it = m_properties.erase(it);
		else	it++;
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

