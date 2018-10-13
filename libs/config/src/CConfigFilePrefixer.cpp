/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "config-precomp.h"  // Precompiled headers

#include <mrpt/config/CConfigFilePrefixer.h>

using namespace mrpt::config;
using namespace std;

CConfigFilePrefixer::CConfigFilePrefixer() = default;
CConfigFilePrefixer::CConfigFilePrefixer(
	const CConfigFileBase& o, const std::string& prefix_sections,
	const std::string& prefix_keys)
	: m_bound_object(const_cast<CConfigFileBase*>(&o)),
	  m_prefix_sections(prefix_sections),
	  m_prefix_keys(prefix_keys)
{
}
CConfigFilePrefixer::~CConfigFilePrefixer()
{
	// Nothing to free manually
}

void CConfigFilePrefixer::bind(const CConfigFileBase& o)
{
	m_bound_object = const_cast<CConfigFileBase*>(&o);
}

void CConfigFilePrefixer::setPrefixes(
	const std::string& prefix_sections, const std::string& prefix_keys)
{
	m_prefix_sections = prefix_sections;
	m_prefix_keys = prefix_keys;
}

std::string CConfigFilePrefixer::getSectionPrefix() const
{
	return m_prefix_sections;
}
std::string CConfigFilePrefixer::getKeyPrefix() const { return m_prefix_keys; }
CConfigFileBase* CConfigFilePrefixer::getBoundConfigFileBase() const
{
	return m_bound_object;
}

void CConfigFilePrefixer::getAllSections(
	std::vector<std::string>& sections) const
{
	ASSERTMSG_(
		m_bound_object,
		"You must first bind CConfigFilePrefixer to an existing object!");
	m_bound_object->getAllSections(sections);
	for (auto& section : sections) section = m_prefix_sections + section;
}

void CConfigFilePrefixer::getAllKeys(
	const std::string& section, std::vector<std::string>& keys) const
{
	ASSERTMSG_(
		m_bound_object,
		"You must first bind CConfigFilePrefixer to an existing object!");
	m_bound_object->getAllKeys(section, keys);
	for (auto& key : keys) key = m_prefix_keys + key;
}

void CConfigFilePrefixer::writeString(
	const std::string& section, const std::string& name, const std::string& str)
{
	ASSERTMSG_(
		m_bound_object,
		"You must first bind CConfigFilePrefixer to an existing object!");
	m_bound_object->writeString(
		m_prefix_sections + section, m_prefix_keys + name, str);
}

std::string CConfigFilePrefixer::readString(
	const std::string& section, const std::string& name,
	const std::string& defaultStr, bool failIfNotFound) const
{
	ASSERTMSG_(
		m_bound_object,
		"You must first bind CConfigFilePrefixer to an existing object!");
	return m_bound_object->readString(
		m_prefix_sections + section, m_prefix_keys + name, defaultStr,
		failIfNotFound);
}
