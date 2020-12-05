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

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/config_parser.h>
#include <mrpt/system/string_utils.h>

using namespace mrpt;
using namespace mrpt::config;
using namespace std;

struct CConfigFileMemory::Impl
{
	std::shared_ptr<CSimpleIniA> ini = std::make_shared<CSimpleIniA>();
};

CConfigFileMemory::CConfigFileMemory(const std::vector<std::string>& stringList)
	: m_impl(mrpt::make_impl<CConfigFileMemory::Impl>())
{
	// Load the strings:
	setContent(stringList);
}

CConfigFileMemory::CConfigFileMemory(const std::string& str)
	: m_impl(mrpt::make_impl<CConfigFileMemory::Impl>())
{
	setContent(str);
}

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CConfigFileMemory::CConfigFileMemory()
	: m_impl(mrpt::make_impl<CConfigFileMemory::Impl>())
{
}

void CConfigFileMemory::setContent(const std::vector<std::string>& stringList)
{
	// Load the strings:
	std::string aux;
	mrpt::system::stringListAsString(stringList, aux);
	const auto sOut = mrpt::config::config_parser(aux);
	m_impl->ini->LoadData(sOut);
}

void CConfigFileMemory::setContent(const std::string& str)
{
	const auto sOut = mrpt::config::config_parser(str);
	m_impl->ini->LoadData(sOut);
}

void CConfigFileMemory::getContent(std::string& str) const
{
	m_impl->ini->Save(str);
}

CConfigFileMemory::~CConfigFileMemory() = default;
void CConfigFileMemory::writeString(
	const std::string& section, const std::string& name, const std::string& str)
{
	MRPT_START

	SI_Error ret = m_impl->ini->SetValue(
		section.c_str(), name.c_str(), str.c_str(), nullptr);
	if (ret < 0) THROW_EXCEPTION("Error changing value in INI-style file!");

	MRPT_END
}

std::string CConfigFileMemory::readString(
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
	CSimpleIniA::TNamesDepend names;
	m_impl->ini->GetAllSections(names);

	CSimpleIniA::TNamesDepend::iterator n;
	std::vector<std::string>::iterator s;
	sections.resize(names.size());
	for (n = names.begin(), s = sections.begin(); n != names.end(); ++n, ++s)
		*s = n->pItem;
}

void CConfigFileMemory::getAllKeys(
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

void CConfigFileMemory::clear() { m_impl->ini->Reset(); }
