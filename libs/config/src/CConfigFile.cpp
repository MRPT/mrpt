/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "config-precomp.h"  // Precompiled headers

#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/os.h>
#include "simpleini/SimpleIni.h"

using namespace mrpt;
using namespace mrpt::config;
using namespace mrpt::config::simpleini;
using namespace std;

struct CConfigFile::Impl
{
	MRPT_CSimpleIni m_ini;
};

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFile::CConfigFile(const std::string& fileName)
	: m_impl(mrpt::make_impl<CConfigFile::Impl>())
{
	MRPT_START

	m_file = fileName;
	m_modified = false;
	m_impl->m_ini.LoadFile(fileName.c_str());

	MRPT_END
}

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFile::CConfigFile() : m_impl(mrpt::make_impl<CConfigFile::Impl>())
{
	MRPT_START

	m_file = "";
	m_modified = false;

	MRPT_END
}

/*---------------------------------------------------------------
					setFileName
 ---------------------------------------------------------------*/
void CConfigFile::setFileName(const std::string& fil_path)
{
	MRPT_START

	m_file = fil_path;
	m_modified = false;

	m_impl->m_ini.LoadFile(fil_path.c_str());
	MRPT_END
}

/*---------------------------------------------------------------
					writeNow
 ---------------------------------------------------------------*/
void CConfigFile::writeNow()
{
	MRPT_START
	if (m_modified && !m_file.empty())
	{
		m_impl->m_ini.SaveFile(m_file.c_str());
		m_modified = false;
	}
	MRPT_END
}

void CConfigFile::discardSavingChanges() { m_modified = false; }
/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CConfigFile::~CConfigFile() { writeNow(); }
/*---------------------------------------------------------------
					writeString
 ---------------------------------------------------------------*/
void CConfigFile::writeString(
	const std::string& section, const std::string& name, const std::string& str)
{
	MRPT_START

	m_modified = true;

	if (0 > m_impl->m_ini.SetValue(
				section.c_str(), name.c_str(), str.c_str(), nullptr))
		THROW_EXCEPTION("Error changing value in INI-style file!");

	MRPT_END
}

/*---------------------------------------------------------------
					readString
 ---------------------------------------------------------------*/
std::string CConfigFile::readString(
	const std::string& section, const std::string& name,
	const std::string& defaultStr, bool failIfNotFound) const
{
	MRPT_START
	const char* defVal = failIfNotFound ? nullptr : defaultStr.c_str();

	const char* aux = m_impl->m_ini.GetValue(
		section.c_str(), name.c_str(), defVal,
		nullptr);  // The memory is managed by the SimpleIni object

	if (failIfNotFound && !aux)
	{
		string tmpStr(format(
			"Value '%s' not found in section '%s' of file '%s' and "
			"failIfNotFound=true.",
			name.c_str(), section.c_str(), m_file.c_str()));
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

/*---------------------------------------------------------------
					 getAllSections
 ---------------------------------------------------------------*/
void CConfigFile::getAllSections(std::vector<std::string>& sections) const
{
	MRPT_CSimpleIni::TNamesDepend names;
	m_impl->m_ini.GetAllSections(names);

	MRPT_CSimpleIni::TNamesDepend::iterator n;
	std::vector<std::string>::iterator s;
	sections.resize(names.size());
	for (n = names.begin(), s = sections.begin(); n != names.end(); ++n, ++s)
		*s = n->pItem;
}

/*---------------------------------------------------------------
					  getAllKeys
 ---------------------------------------------------------------*/
void CConfigFile::getAllKeys(
	const string& section, std::vector<std::string>& keys) const
{
	MRPT_CSimpleIni::TNamesDepend names;
	m_impl->m_ini.GetAllKeys(section.c_str(), names);

	MRPT_CSimpleIni::TNamesDepend::iterator n;
	std::vector<std::string>::iterator s;
	keys.resize(names.size());
	for (n = names.begin(), s = keys.begin(); n != names.end(); ++n, ++s)
		*s = n->pItem;
}
