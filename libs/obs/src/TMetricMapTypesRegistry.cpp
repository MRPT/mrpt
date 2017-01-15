/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h" // Precomp header

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/TMetricMapTypesRegistry.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/system/string_utils.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::maps::internal;

TMetricMapTypesRegistry & TMetricMapTypesRegistry::Instance() 
{
	static TMetricMapTypesRegistry reg;
	return reg;
}

size_t TMetricMapTypesRegistry::doRegister(const std::string &names,MapDefCtorFunctor func1,MapCtorFromDefFunctor func2)
{
	std::vector<std::string> lstNames;
	mrpt::system::tokenize(names," \t\r\n,",lstNames);
	for (size_t i=0;i<lstNames.size();i++)
		m_registry[lstNames[i]] = std::make_pair(func1,func2);
	return m_registry.size();
}

mrpt::maps::TMetricMapInitializer* TMetricMapTypesRegistry::factoryMapDefinition(const std::string &className) const 
{
	TListRegisteredMaps::const_iterator it=m_registry.find(className);
	if (it==m_registry.end()) return NULL;
	ASSERT_(it->second.first!=NULL)
	return (*it->second.first)();
}

mrpt::maps::CMetricMap* TMetricMapTypesRegistry::factoryMapObjectFromDefinition(
	const mrpt::maps::TMetricMapInitializer&mi) const
{
	TListRegisteredMaps::const_iterator it=m_registry.find( mi.getMetricMapClassType()->className );
	if (it==m_registry.end()) {
		THROW_EXCEPTION_CUSTOM_MSG1("[TMetricMapTypesRegistry] Error: Cannot create map of unregistered map type '%s'",mi.getMetricMapClassType()->className); 
	}

	ASSERT_(it->second.second!=NULL)
	mrpt::maps::CMetricMap* theMap = (*it->second.second)(mi);

	// Common params for all maps:
	theMap->genericMapParams = mi.genericMapParams;

	return theMap;
}
