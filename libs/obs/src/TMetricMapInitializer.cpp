/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h" // Precomp header

#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;


/** Looks up in the registry of known map types and call the corresponding `<metric_map_class>::MapDefinition()`. */
TMetricMapInitializer* TMetricMapInitializer::factory(const std::string &mapClassName)
{
	using internal::TMetricMapTypesRegistry;
	TMetricMapTypesRegistry & mmr = TMetricMapTypesRegistry::Instance();
	return mmr.factoryMapDefinition(mapClassName);
}

TMetricMapInitializer::TMetricMapInitializer(const mrpt::utils::TRuntimeClassId* classID ) : 
	metricMapClassType(classID)
{
}

/** Load all params from a config file/source. For examples and format, read the docs of mrpt::maps::CMultiMetricMap */
void  TMetricMapInitializer::loadFromConfigFile(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	// Common:
	const std::string sctCreat = sectionNamePrefix + std::string("_creationOpts");
	this->genericMapParams.loadFromConfigFile(source,sctCreat);
	
	// Class-specific:
	this->loadFromConfigFile_map_specific(source,sectionNamePrefix);
}

/** Dump the options of the metric map in human-readable format */
void  TMetricMapInitializer::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("-------------------------TMetricMapInitializer --------------------------\n");
	out.printf("================ C++ Class: '%s'\n", this->metricMapClassType->className);
	this->genericMapParams.dumpToTextStream(out);
	
	// Class-specific:
	this->dumpToTextStream_map_specific(out);
}

/*---------------------------------------------------------------
		TSetOfMetricMapInitializers::loadFromConfigFile
 ---------------------------------------------------------------*/
void  TSetOfMetricMapInitializers::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &ini,
	const std::string &sectionName )
{
	MRPT_START
	using internal::TMetricMapTypesRegistry;

	// Delete previous contents:
	clear();

	TMetricMapTypesRegistry & mmr = TMetricMapTypesRegistry::Instance();

	const TMetricMapTypesRegistry::TListRegisteredMaps & allMapKinds = mmr.getAllRegistered();
	for (TMetricMapTypesRegistry::TListRegisteredMaps::const_iterator itMapKind=allMapKinds.begin();itMapKind!=allMapKinds.end();++itMapKind)
	{
		//  ; Creation of maps:
		//  occupancyGrid_count=<Number of mrpt::maps::COccupancyGridMap2D maps>
		const std::string sMapName = itMapKind->first;

		unsigned int n = ini.read_uint64_t(sectionName,sMapName+string("_count"),0);
		for (unsigned int i=0;i<n;i++)
		{
			TMetricMapInitializer *mi = mmr.factoryMapDefinition(sMapName);
			ASSERT_(mi);

			// Load from sections formatted like this:
			// [<sectionName>+"_occupancyGrid_##_creationOpts"]
			// [<sectionName>+"_occupancyGrid_##_insertOpts"]
			// [<sectionName>+"_occupancyGrid_##_likelihoodOpts"]
			// ...
			// ==> Section prefix:
			const string sMapSectionsPrefix = mrpt::format("%s_%s_%02u",sectionName.c_str(),sMapName.c_str(),i);
			mi->loadFromConfigFile(ini,sMapSectionsPrefix);

			// Add the params to the list:
			this->push_back( TMetricMapInitializerPtr(mi) );
		}

	}// end for each map kind
	
	MRPT_END
}

/*---------------------------------------------------------------
		TSetOfMetricMapInitializers::dumpToTextStream
 ---------------------------------------------------------------*/
void  TSetOfMetricMapInitializers::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	MRPT_START

	out.printf("====================================================================\n\n");
	out.printf("      Set of internal maps for 'CMultiMetricMap' object\n\n");
	out.printf("====================================================================\n");

	// Show each map:
	out.printf("Showing next the %u internal maps:\n\n", (int)size());

	int i=0;
	for (const_iterator it=begin();it!=end();++it,i++)
	{
		out.printf("------------------------- Internal map %u out of %u --------------------------\n",i+1,(int)size());
		(*it)->dumpToTextStream(out);
	} // for "it"

	MRPT_END
}






