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
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/serialization/CArchive.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;

/** Looks up in the registry of known map types and call the corresponding
 * `<metric_map_class>::MapDefinition()`. */
TMetricMapInitializer::Ptr TMetricMapInitializer::factory(const std::string& mapClassName)
{
  using mrpt::maps::internal::TMetricMapTypesRegistry;
  TMetricMapTypesRegistry& mmr = TMetricMapTypesRegistry::Instance();
  return mmr.factoryMapDefinition(mapClassName);
}

TMetricMapInitializer::TMetricMapInitializer(const mrpt::rtti::TRuntimeClassId* classID) :
    metricMapClassType(classID)
{
}

/** Load all params from a config file/source. For examples and format, read the
 * docs of mrpt::maps::CMultiMetricMap */
void TMetricMapInitializer::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& source, const std::string& sectionNamePrefix)
{
  // Common:
  const std::string sctCreat = sectionNamePrefix + std::string("_creationOpts");
  this->genericMapParams.loadFromConfigFile(source, sctCreat);

  // Class-specific:
  this->loadFromConfigFile_map_specific(source, sectionNamePrefix);
}

void TMetricMapInitializer::saveToConfigFile(
    mrpt::config::CConfigFileBase& target, const std::string& section) const
{
  auto s = section + std::string("_") + std::string(this->metricMapClassType->className);
  this->genericMapParams.saveToConfigFile(target, s);
}

void TMetricMapInitializer::dumpToTextStream(std::ostream& out) const
{
  out << "-----------------TMetricMapInitializer --------------------\n";
  out << "================ C++ Class: '" << this->metricMapClassType->className << "'\n";
  this->genericMapParams.dumpToTextStream(out);
  // Class-specific:
  this->dumpToTextStream_map_specific(out);
}

void TSetOfMetricMapInitializers::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& ini, const std::string& sectionName)
{
  MRPT_START

  using mrpt::maps::internal::TMetricMapTypesRegistry;
  using namespace std::string_literals;

  // Delete previous contents:
  clear();

  TMetricMapTypesRegistry& mmr = TMetricMapTypesRegistry::Instance();

  const auto& allMapKinds = mmr.getAllRegistered();
  for (const auto& allMapKind : allMapKinds)
  {
    //  ; Creation of maps:
    //  occupancyGrid_count=<Number of mrpt::maps::COccupancyGridMap2D maps>
    const std::string sMapName = allMapKind.first;

    unsigned int n = ini.read_uint64_t(sectionName, sMapName + "_count"s, 0);
    for (unsigned int i = 0; i < n; i++)
    {
      TMetricMapInitializer::Ptr mi = mmr.factoryMapDefinition(sMapName);
      ASSERT_(mi);

      // Load from sections formatted like this:
      // [<sectionName>+"_occupancyGrid_##_creationOpts"]
      // [<sectionName>+"_occupancyGrid_##_insertOpts"]
      // [<sectionName>+"_occupancyGrid_##_likelihoodOpts"]
      // ...
      // ==> Section prefix:
      const string sMapSectionsPrefix =
          mrpt::format("%s_%s_%02u", sectionName.c_str(), sMapName.c_str(), i);
      mi->loadFromConfigFile(ini, sMapSectionsPrefix);

      // Add the params to the list:
      this->push_back(mi);
    }

  }  // end for each map kind

  // Check for unknown map types and throw an error:
  for (const auto& s : ini.keys(sectionName))
  {
    auto p = s.find("_count");
    if (p == std::string::npos) continue;
    const auto className = s.substr(0, p);
    if (allMapKinds.count(className) != 0) continue;  // ok, it exists
    THROW_EXCEPTION_FMT(
        "Error: found INI section '%s' while parsing "
        "TSetOfMetricMapInitializers, but there is no such registered "
        "CMetricMap class '%s'",
        s.c_str(), className.c_str());
  }

  MRPT_END
}

void TSetOfMetricMapInitializers::saveToConfigFile(
    mrpt::config::CConfigFileBase& target, const std::string& section) const
{
  for (auto& mi : *this) mi->saveToConfigFile(target, section);
}

void TSetOfMetricMapInitializers::dumpToTextStream(std::ostream& out) const
{
  MRPT_START
  out << "===============================================================\n\n"
         "      Set of internal maps for 'CMultiMetricMap' object\n\n"
         "=================================================================\n"
         "Showing next the "
      << this->size() << " internal maps:\n\n";

  int i = 0;
  for (auto it = begin(); it != end(); ++it, i++)
  {
    out << mrpt::format("------- Internal map %u out of %u:\n", i + 1, (int)size());
    (*it)->dumpToTextStream(out);
  }  // for "it"
  MRPT_END
}
