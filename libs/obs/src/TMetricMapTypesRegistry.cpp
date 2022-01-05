/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precomp header
//
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/TMetricMapTypesRegistry.h>
#include <mrpt/system/string_utils.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::maps::internal;

TMetricMapTypesRegistry& TMetricMapTypesRegistry::Instance()
{
	static TMetricMapTypesRegistry reg;
	return reg;
}

static std::string stripNamespace(const std::string& n)
{
	std::string ret = n;
	const auto pos = ret.rfind("::");
	if (pos != std::string::npos) ret = ret.substr(pos + 2);

	return ret;
}

size_t TMetricMapTypesRegistry::doRegister(
	const std::string& names, MapDefCtorFunctor func1,
	MapCtorFromDefFunctor func2)
{
	std::vector<std::string> lstNames;
	mrpt::system::tokenize(names, " \t\r\n,", lstNames);
	for (const auto& lstName : lstNames)
	{
		const auto p = std::make_pair(func1, func2);
		m_registry[lstName] = p;
		// register also the version without the "mrpt::NS::" prefix, for
		// backwards compatibility:
		m_registry[stripNamespace(lstName)] = p;
	}
	return m_registry.size();
}

mrpt::maps::TMetricMapInitializer*
	TMetricMapTypesRegistry::factoryMapDefinition(
		const std::string& className) const
{
	// 1st attempt: full qualified name:
	auto it = m_registry.find(className);
	// 2nd attempt: without namespace prefix:
	if (it == m_registry.end()) it = m_registry.find(stripNamespace(className));

	if (it == m_registry.end()) return nullptr;
	ASSERT_(it->second.first);
	return (it->second.first)();
}

mrpt::maps::CMetricMap* TMetricMapTypesRegistry::factoryMapObjectFromDefinition(
	const mrpt::maps::TMetricMapInitializer& mi) const
{
	auto it = m_registry.find(mi.getMetricMapClassType()->className);
	if (it == m_registry.end())
	{
		THROW_EXCEPTION_FMT(
			"[TMetricMapTypesRegistry] Error: Cannot create map of "
			"unregistered map type '%s'",
			mi.getMetricMapClassType()->className);
	}

	ASSERT_(it->second.second);
	mrpt::maps::CMetricMap* theMap = (it->second.second)(mi);

	// Common params for all maps:
	theMap->genericMapParams = mi.genericMapParams;

	return theMap;
}
