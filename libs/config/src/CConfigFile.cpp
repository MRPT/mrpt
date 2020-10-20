/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "config-precomp.h"  // Precompiled headers

// Fix to SimpleIni bug: not able to build with C++17
#include <functional>
#ifdef _MSC_VER
namespace std
{
template <typename T1, typename T2, typename RET>
using binary_function = std::function<RET(T1, T2)>;
}
#endif

#include <mrpt/config.h>
#if MRPT_HAS_SIMPLEINI_SYSTEM
// Enforce using libuci instead of copyrighted ConvertUTF.h
#define SI_CONVERT_ICU 1
#endif
#include <SimpleIni.h>

#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/config_parser.h>
#include <mrpt/system/os.h>
#include <fstream>
#include <iostream>

using namespace mrpt;
using namespace mrpt::config;
using namespace std;

struct CConfigFile::Impl
{
	std::shared_ptr<CSimpleIniA> ini = std::make_shared<CSimpleIniA>();
};

// copied from mrpt-io to avoid lib dependency:
static std::string local_file_get_contents(const std::string& fileName)
{
	// Credits: https://stackoverflow.com/a/2602258/1631514
	// Note: Add "binary" to make sure the "tellg" file size matches the actual
	// number of read bytes afterwards:
	std::ifstream t(fileName, ios::binary);
	if (!t.is_open())
		THROW_EXCEPTION_FMT(
			"file_get_contents(): Error opening for read file `%s`",
			fileName.c_str());

	t.seekg(0, std::ios::end);
	std::size_t size = t.tellg();
	std::string buffer(size, ' ');
	t.seekg(0);
	t.read(&buffer[0], size);
	return buffer;
}

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFile::CConfigFile(const std::string& fileName)
	: m_impl(mrpt::make_impl<CConfigFile::Impl>())
{
	MRPT_START

	m_file = fileName;
	m_modified = false;

	const auto sIn = local_file_get_contents(fileName);
	const auto sOut = mrpt::config::config_parser(sIn);
	m_impl->ini->LoadData(sOut);

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

	m_impl->ini->LoadFile(fil_path.c_str());
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
		m_impl->ini->SaveFile(m_file.c_str());
		m_modified = false;
	}
	MRPT_END
}

void CConfigFile::discardSavingChanges() { m_modified = false; }

CConfigFile::~CConfigFile()
{
	try
	{
		writeNow();
	}
	catch (const std::exception& e)
	{
		std::cerr << "[~CConfigFile] Exception:\n" << mrpt::exception_to_str(e);
	}
}

void CConfigFile::writeString(
	const std::string& section, const std::string& name, const std::string& str)
{
	MRPT_START

	m_modified = true;

	if (0 > m_impl->ini->SetValue(
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

	const char* aux = m_impl->ini->GetValue(
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
	CSimpleIniA::TNamesDepend names;
	m_impl->ini->GetAllSections(names);

	CSimpleIniA::TNamesDepend::iterator n;
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
	CSimpleIniA::TNamesDepend names;
	m_impl->ini->GetAllKeys(section.c_str(), names);

	CSimpleIniA::TNamesDepend::iterator n;
	std::vector<std::string>::iterator s;
	keys.resize(names.size());
	for (n = names.begin(), s = keys.begin(); n != names.end(); ++n, ++s)
		*s = n->pItem;
}

void CConfigFile::clear() { m_impl->ini->Reset(); }
