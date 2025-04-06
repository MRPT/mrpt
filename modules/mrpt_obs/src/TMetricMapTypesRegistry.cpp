/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
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

namespace
{
std::string stripNamespace(const std::string& n)
{
  std::string ret = n;
  const auto pos = ret.rfind("::");
  if (pos != std::string::npos) ret = ret.substr(pos + 2);

  return ret;
}
}  // namespace

size_t TMetricMapTypesRegistry::doRegister(
    const std::string& names, MapDefCtorFunctor func1, MapCtorFromDefFunctor func2)
{
  std::vector<std::string> lstNames;
  mrpt::system::tokenize(names, " \t\r\n,", lstNames);
  for (const auto& lstName : lstNames)
  {
    m_registry[lstName] = {func1, func2};
    // register also the version without the "mrpt::NS::" prefix, for
    // backwards compatibility:
    m_registry[stripNamespace(lstName)] = {func1, func2};
  }
  return m_registry.size();
}

mrpt::maps::TMetricMapInitializer::Ptr TMetricMapTypesRegistry::factoryMapDefinition(
    const std::string& className) const
{
  // 1st attempt: full qualified name:
  auto it = m_registry.find(className);
  // 2nd attempt: without namespace prefix:
  if (it == m_registry.end()) it = m_registry.find(stripNamespace(className));

  if (it == m_registry.end()) return nullptr;
  ASSERT_(it->second.defCtor);
  return (it->second.defCtor)();
}

mrpt::maps::CMetricMap::Ptr TMetricMapTypesRegistry::factoryMapObjectFromDefinition(
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

  ASSERT_(it->second.mapCtor);
  mrpt::maps::CMetricMap::Ptr theMap = (it->second.mapCtor)(mi);

  // Common params for all maps:
  theMap->genericMapParams = mi.genericMapParams;

  return theMap;
}
