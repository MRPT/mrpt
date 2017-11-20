/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "config-precomp.h"  // Precompiled headers

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/system/string_utils.h>

#include "simpleini/SimpleIni.h"

using namespace mrpt;
using namespace mrpt::config;
using namespace mrpt::config::simpleini;
using namespace std;

#define THE_INI static_cast<MRPT_CSimpleIni*>(m_ini.get())

CConfigFileMemory::CConfigFileMemory(const std::vector<std::string>& stringList)
{
	// Create the object:
	m_ini = (void*)new MRPT_CSimpleIni();

	// Load the strings:
	std::string aux;
	mrpt::system::stringListAsString(stringList, aux);
	THE_INI->Load(aux.c_str(), aux.size());
}

CConfigFileMemory::CConfigFileMemory(const std::string& str)
{
	// Create the object:
	m_ini = (void*)new MRPT_CSimpleIni();

	// Load the strings:
	THE_INI->Load(str.c_str(), str.size());
}

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFileMemory::CConfigFileMemory()
{
	// Create the empty object:
	m_ini = (void*)new MRPT_CSimpleIni();
}

/** Copy constructor */
CConfigFileMemory::CConfigFileMemory(const CConfigFileMemory& o)
{
	// Create the empty object:
	m_ini = (void*)new MRPT_CSimpleIni();
	(*this) = o;
}

/** Copy operator */
CConfigFileMemory& CConfigFileMemory::operator=(const CConfigFileMemory& o)
{
	std::string str;
	static_cast<const MRPT_CSimpleIni*>(o.m_ini.get())->Save(str);
	THE_INI->Load(str.c_str(), str.size());
	return *this;
}

void CConfigFileMemory::setContent(const std::vector<std::string>& stringList)
{
	// Load the strings:
	std::string aux;
	mrpt::system::stringListAsString(stringList, aux);
	THE_INI->Load(aux.c_str(), aux.size());
}

void CConfigFileMemory::setContent(const std::string& str)
{
	THE_INI->Load(str.c_str(), str.size());
}

void CConfigFileMemory::getContent(std::string& str) const
{
	((MRPT_CSimpleIni*)(m_ini.get()))->Save(str);
}

CConfigFileMemory::~CConfigFileMemory() { delete THE_INI; }

void CConfigFileMemory::writeString(
	const std::string& section, const std::string& name, const std::string& str)
{
	MRPT_START

	SI_Error ret =
		THE_INI->SetValue(section.c_str(), name.c_str(), str.c_str(), nullptr);
	if (ret < 0) THROW_EXCEPTION("Error changing value in INI-style file!");

	MRPT_END
}

std::string CConfigFileMemory::readString(
	const std::string& section, const std::string& name,
	const std::string& defaultStr, bool failIfNotFound) const
{
	MRPT_START
	const char* defVal = failIfNotFound ? nullptr : defaultStr.c_str();

	const char* aux =
		static_cast<const MRPT_CSimpleIni*>(m_ini.get())
			->GetValue(
				section.c_str(), name.c_str(), defVal,
				nullptr);  // The memory is managed by the SimpleIni object

	if (failIfNotFound && !aux)
	{
		string tmpStr(
			format(
				"Value '%s' not found in section '%s' of memory configuration "
				"string list and failIfNotFound=true.",
				name.c_str(), section.c_str()));
		THROW_EXCEPTION(tmpStr);
	}

	// Remove possible comments: "//"
	std::string ret = aux;
	size_t pos;
	if ((pos = ret.find("//")) != string::npos && pos > 0 &&
		isspace(ret[pos - 1]))
		ret = ret.substr(0, pos);
	return ret;
	MRPT_END
}

void CConfigFileMemory::getAllSections(std::vector<std::string>& sections) const
{
	MRPT_CSimpleIni::TNamesDepend names;
	static_cast<const MRPT_CSimpleIni*>(m_ini.get())->GetAllSections(names);

	MRPT_CSimpleIni::TNamesDepend::iterator n;
	std::vector<std::string>::iterator s;
	sections.resize(names.size());
	for (n = names.begin(), s = sections.begin(); n != names.end(); ++n, ++s)
		*s = n->pItem;
}

void CConfigFileMemory::getAllKeys(
	const string& section, std::vector<std::string>& keys) const
{
	MRPT_CSimpleIni::TNamesDepend names;
	static_cast<const MRPT_CSimpleIni*>(m_ini.get())
		->GetAllKeys(section.c_str(), names);

	MRPT_CSimpleIni::TNamesDepend::iterator n;
	std::vector<std::string>::iterator s;
	keys.resize(names.size());
	for (n = names.begin(), s = keys.begin(); n != names.end(); ++n, ++s)
		*s = n->pItem;
}
