/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CConfigFilePrefixer.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

CConfigFilePrefixer::CConfigFilePrefixer() : 
	m_bound_object(NULL)
{
}
CConfigFilePrefixer::CConfigFilePrefixer(const CConfigFileBase &o, const std::string &prefix_sections, const std::string &prefix_keys) :
	m_bound_object( const_cast<CConfigFileBase*>(&o) ),
	m_prefix_sections(prefix_sections), 
	m_prefix_keys(prefix_keys)
{
}
CConfigFilePrefixer::~CConfigFilePrefixer()
{
	// Nothing to free manually
}

void CConfigFilePrefixer::bind(const CConfigFileBase &o)
{
	m_bound_object = const_cast<CConfigFileBase*>(&o);
}

void CConfigFilePrefixer::setPrefixes(const std::string &prefix_sections, const std::string &prefix_keys)
{
	m_prefix_sections = prefix_sections;
	m_prefix_keys = prefix_keys;
}

std::string CConfigFilePrefixer::getSectionPrefix() const {
	return m_prefix_sections;
}
std::string CConfigFilePrefixer::getKeyPrefix() const {
	return m_prefix_keys;
}
CConfigFileBase *CConfigFilePrefixer::getBoundConfigFileBase() const {
	return m_bound_object;
}

void CConfigFilePrefixer::getAllSections( vector_string	&sections ) const
{
	ASSERTMSG_(m_bound_object, "You must first bind CConfigFilePrefixer to an existing object!")
	m_bound_object->getAllSections(sections);
	for (size_t i=0;i<sections.size();i++)
		sections[i] = m_prefix_sections + sections[i];
}

void CConfigFilePrefixer::getAllKeys( const std::string &section, vector_string	&keys ) const
{
	ASSERTMSG_(m_bound_object, "You must first bind CConfigFilePrefixer to an existing object!")
	m_bound_object->getAllKeys(section,keys);
	for (size_t i=0;i<keys.size();i++)
		keys[i] = m_prefix_keys + keys[i];
}

void  CConfigFilePrefixer::writeString(const std::string &section,const std::string &name, const std::string &str)
{
	ASSERTMSG_(m_bound_object, "You must first bind CConfigFilePrefixer to an existing object!")
	m_bound_object->writeString( m_prefix_sections + section, m_prefix_keys + name, str);
}

std::string  CConfigFilePrefixer::readString(const std::string &section,const std::string &name,const std::string &defaultStr,bool failIfNotFound ) const
{
	ASSERTMSG_(m_bound_object, "You must first bind CConfigFilePrefixer to an existing object!")
	return m_bound_object->readString( m_prefix_sections + section, m_prefix_keys + name, defaultStr,failIfNotFound);
}
