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


#include <mrpt/utils/CPropertiesValuesList.h>
using namespace mrpt::utils;
using namespace mrpt::system;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CPropertiesValuesList, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CPropertiesValuesList::writeToStream(CStream &out, int *out_Version) const
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
void  CPropertiesValuesList::readFromStream(CStream &in, int version)
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
