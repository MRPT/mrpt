/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h" // Precomp header

#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/utils/CConfigFileBase.h>

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
	metricMapClassType(classID),
	enableSaveAs3DObject(true),
	enableObservationLikelihood(true),
	enableObservationInsertion(true)
{
}

/** Load all params from a config file/source. For examples and format, read the docs of mrpt::maps::CMultiMetricMap */
void  TMetricMapInitializer::loadFromConfigFile(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	// Common:
	const std::string sctCreat = sectionNamePrefix + std::string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(enableSaveAs3DObject          , bool,   source,sctCreat);
	MRPT_LOAD_CONFIG_VAR(enableObservationLikelihood   , bool,   source,sctCreat);
	MRPT_LOAD_CONFIG_VAR(enableObservationInsertion    , bool,   source,sctCreat);
	
	// Class-specific:
	this->loadFromConfigFile_map_specific(source,sectionNamePrefix);
}

/** Dump the options of the metric map in human-readable format */
void  TMetricMapInitializer::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("-------------------------TMetricMapInitializer --------------------------\n");
	out.printf("================ C++ Class: '%s'\n", this->metricMapClassType->className);
	// Common:
	LOADABLEOPTS_DUMP_VAR(enableSaveAs3DObject         , bool);
	LOADABLEOPTS_DUMP_VAR(enableObservationLikelihood  , bool);
	LOADABLEOPTS_DUMP_VAR(enableObservationInsertion   , bool);
	
	// Class-specific:
	this->dumpToTextStream(out);
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
			const string sMapSectionsPrefix = mrpt::format("%s_%s_%02u_",sectionName.c_str(),sMapName.c_str(),i);
			mi->loadFromConfigFile(ini,sMapSectionsPrefix);

			// Create map itself:
			mrpt::maps::CMetricMap *theMap = mmr.factoryMapObjectFromDefinition(sMapName, *mi);
			ASSERT_(theMap);
		}

	}// end for each map kind
	
#if 0
	n = ini.read_int(sectionName,"octoMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( COctoMap );

		// [<sectionName>+"_octoMap_##_creationOpts"]
		std::string subSectName = format("%s_octoMap_%02u_creationOpts",sectionName.c_str(),i);

		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.octoMap_options.resolution = ini.read_float(subSectName,"resolution",init.octoMap_options.resolution);

		// [<sectionName>+"_octoMap_##_insertOpts"]
		init.octoMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_octoMap_%02u_insertOpts",sectionName.c_str(),i));

		// [<sectionName>+"_octoMap_##_likelihoodOpts"]
		init.octoMap_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_octoMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i


	n = ini.read_int(sectionName,"colourOctoMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( COctoMap );

		// [<sectionName>+"_colourOctoMap_##_creationOpts"]
		std::string subSectName = format("%s_colourOctoMap_%02u_creationOpts",sectionName.c_str(),i);

		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.colourOctoMap_options.resolution = ini.read_float(subSectName,"resolution",init.octoMap_options.resolution);

		// [<sectionName>+"_colourOctoMap_##_insertOpts"]
		init.colourOctoMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_colourOctoMap_%02u_insertOpts",sectionName.c_str(),i));

		// [<sectionName>+"_colourOctoMap_##_likelihoodOpts"]
		init.colourOctoMap_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_colourOctoMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i


	n = ini.read_int(sectionName,"pointsMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CSimplePointsMap );

		// [<sectionName>+"_pointsMap_##_creationOpts"]
		std::string subSectName = format("%s_pointsMap_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);

		// [<sectionName>+"_pointsMap_##_insertOpts"]
		init.pointsMapOptions_options.insertionOpts.loadFromConfigFile(ini,format("%s_pointsMap_%02u_insertOpts",sectionName.c_str(),i));

		// [<sectionName>+"_pointsMap_##_likelihoodOpts"]
		init.pointsMapOptions_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_pointsMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"gasGrid_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CGasConcentrationGridMap2D );

		// [<sectionName>+"_gasGrid_##_creationOpts"]
		std::string subSectName = format("%s_gasGrid_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.gasGridMap_options.mapType = ini.read_enum<CGasConcentrationGridMap2D::TMapRepresentation>(subSectName,"mapType",init.gasGridMap_options.mapType);
		init.gasGridMap_options.min_x	= ini.read_float(subSectName,"min_x",init.occupancyGridMap2D_options.min_x);
		init.gasGridMap_options.max_x	= ini.read_float(subSectName,"max_x",init.occupancyGridMap2D_options.max_x);
		init.gasGridMap_options.min_y	= ini.read_float(subSectName,"min_y",init.occupancyGridMap2D_options.min_y);
		init.gasGridMap_options.max_y	= ini.read_float(subSectName,"max_y",init.occupancyGridMap2D_options.max_y);
		init.gasGridMap_options.resolution = ini.read_float(subSectName,"resolution",init.occupancyGridMap2D_options.resolution);

		// [<sectionName>+"_gasGrid_##_insertOpts"]
		init.gasGridMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_gasGrid_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"wifiGrid_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CWirelessPowerGridMap2D );

		// [<sectionName>+"_wifiGrid_##_creationOpts"]
		std::string subSectName = format("%s_wifiGrid_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.wifiGridMap_options.mapType = ini.read_enum<CWirelessPowerGridMap2D::TMapRepresentation>(subSectName,"mapType",init.wifiGridMap_options.mapType);
		init.wifiGridMap_options.min_x	= ini.read_float(subSectName,"min_x",init.occupancyGridMap2D_options.min_x);
		init.wifiGridMap_options.max_x	= ini.read_float(subSectName,"max_x",init.occupancyGridMap2D_options.max_x);
		init.wifiGridMap_options.min_y	= ini.read_float(subSectName,"min_y",init.occupancyGridMap2D_options.min_y);
		init.wifiGridMap_options.max_y	= ini.read_float(subSectName,"max_y",init.occupancyGridMap2D_options.max_y);
		init.wifiGridMap_options.resolution = ini.read_float(subSectName,"resolution",init.occupancyGridMap2D_options.resolution);

		// [<sectionName>+"_wifiGrid_##_insertOpts"]
		init.wifiGridMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_wifiGrid_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"landmarksMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer				init;
		unsigned int						nBeacons;
		std::pair<CPoint3D,unsigned int>	newPair;

		init.metricMapClassType					= CLASS_ID( CLandmarksMap );

		// [<sectionName>+"_landmarksMap_##_creationOpts"]
		std::string subSectName = format("%s_landmarksMap_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);

		init.landmarksMap_options.initialBeacons.clear();
		nBeacons = ini.read_int(subSectName,"nBeacons",0);
		for (unsigned int q=1;q<=nBeacons;q++)
		{
			newPair.second = ini.read_int(subSectName,format("beacon_%03u_ID",q),0);

			newPair.first.x( ini.read_float(subSectName,format("beacon_%03u_x",q),0) );
			newPair.first.y( ini.read_float(subSectName,format("beacon_%03u_y",q),0) );
			newPair.first.z( ini.read_float(subSectName,format("beacon_%03u_z",q),0) );

			init.landmarksMap_options.initialBeacons.push_back(newPair);
		}

		// [<sectionName>+"_landmarksMap_##_insertOpts"]
		init.landmarksMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_landmarksMap_%02u_insertOpts",sectionName.c_str(),i));

		// [<sectionName>+"_landmarksMap_##_likelihoodOpts"]
		init.landmarksMap_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_landmarksMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"beaconMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer				init;

		init.metricMapClassType				= CLASS_ID( CBeaconMap );

		// [<sectionName>+"_beaconMap_##_creationOpts"]
		std::string subSectName = format("%s_beaconMap_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);

		// [<sectionName>+"_beaconMap_##_likelihoodOpts"]
		init.beaconMap_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_beaconMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// [<sectionName>+"_beaconMap_##_insertOpts"]
		init.beaconMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_beaconMap_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i


	// Height grid maps:
	n = ini.read_int(sectionName,"heightMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CHeightGridMap2D );

		// [<sectionName>+"_heightGrid_##_creationOpts"]
		std::string subSectName = format("%s_heightGrid_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.heightMap_options.mapType = ini.read_enum<CHeightGridMap2D::TMapRepresentation>(subSectName,"mapType",init.heightMap_options.mapType);
		init.heightMap_options.min_x	= ini.read_float(subSectName,"min_x",init.heightMap_options.min_x);
		init.heightMap_options.max_x	= ini.read_float(subSectName,"max_x",init.heightMap_options.max_x);
		init.heightMap_options.min_y	= ini.read_float(subSectName,"min_y",init.heightMap_options.min_y);
		init.heightMap_options.max_y	= ini.read_float(subSectName,"max_y",init.heightMap_options.max_y);
		init.heightMap_options.resolution = ini.read_float(subSectName,"resolution",init.heightMap_options.resolution);

		// [<sectionName>+"_heightGrid_##_insertOpts"]
		init.heightMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_heightGrid_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	// reflectivity grid maps:
	n = ini.read_int(sectionName,"reflectivityMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CHeightGridMap2D );

		// [<sectionName>+"_reflectivityGrid_##_creationOpts"]
		std::string subSectName = format("%s_reflectivityGrid_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.reflectivityMap_options.min_x	= ini.read_float(subSectName,"min_x",init.heightMap_options.min_x);
		init.reflectivityMap_options.max_x	= ini.read_float(subSectName,"max_x",init.heightMap_options.max_x);
		init.reflectivityMap_options.min_y	= ini.read_float(subSectName,"min_y",init.heightMap_options.min_y);
		init.reflectivityMap_options.max_y	= ini.read_float(subSectName,"max_y",init.heightMap_options.max_y);
		init.reflectivityMap_options.resolution = ini.read_float(subSectName,"resolution",init.heightMap_options.resolution);

		// [<sectionName>+"_reflectivityGrid_##_insertOpts"]
		init.reflectivityMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_reflectivityGrid_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	// Colour points map:
	n = ini.read_int(sectionName,"colourPointsMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CColouredPointsMap );

		// [<sectionName>+"_colourPointsMap_##_creationOpts"]
		init.m_disableSaveAs3DObject = ini.read_bool(format("%s_colourPointsMap_%02u_creationOpts",sectionName.c_str(),i),"disableSaveAs3DObject",false);

		// [<sectionName>+"_colourPointsMap_##_insertOpts"]
		init.colouredPointsMapOptions_options.insertionOpts.loadFromConfigFile(ini, format("%s_colourPointsMap_%02u_insertOpts",sectionName.c_str(),i) );

		// [<sectionName>+"_colourPointsMap_##_colorOpts"]
		init.colouredPointsMapOptions_options.colourOpts.loadFromConfigFile(ini, format("%s_colourPointsMap_%02u_colorOpts",sectionName.c_str(),i) );

		// [<sectionName>+"_pointsMap_##_likelihoodOpts"]
		init.colouredPointsMapOptions_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_colourPointsMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	// Weighted points map:
	n = ini.read_int(sectionName,"weightedPointsMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CWeightedPointsMap );

		// [<sectionName>+"_weightedPointsMap_##_creationOpts"]
		init.m_disableSaveAs3DObject = ini.read_bool(format("%s_weightedPointsMap_%02u_creationOpts",sectionName.c_str(),i),"disableSaveAs3DObject",false);

		// [<sectionName>+"_weightedPointsMap_##_insertOpts"]
		init.weightedPointsMapOptions_options.insertionOpts.loadFromConfigFile(ini, format("%s_weightedPointsMap_%02u_insertOpts",sectionName.c_str(),i) );

		// [<sectionName>+"_weightedPointsMap_##_likelihoodOpts"]
		init.weightedPointsMapOptions_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_weightedPointsMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i


/*
		  *  ; Selection of map for likelihood: (occGrid=0, points=1,landmarks=2,gasGrid=3)
		  *  likelihoodMapSelection=<0-3>
*/
	MRPT_LOAD_HERE_CONFIG_VAR_CAST(likelihoodMapSelection,int,CMultiMetricMap::TOptions::TMapSelectionForLikelihood,options.likelihoodMapSelection, ini,sectionName );

/*
		  *  ; Enables (1) / Disables (0) insertion into specific maps:
		  *  enableInsertion_pointsMap=<0/1>
		  *  enableInsertion_landmarksMap=<0/1>
		  *  enableInsertion_gridMaps=<0/1>
		  *  enableInsertion_gasGridMaps=<0/1>
		  *  enableInsertion_wifiGridMaps=<0/1>
		  *  enableInsertion_beaconMap=<0/1>
*/
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_pointsMap,bool,	options.enableInsertion_pointsMap,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_landmarksMap,bool, options.enableInsertion_landmarksMap,	ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_gridMaps,bool,		options.enableInsertion_gridMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_octoMaps,bool,		options.enableInsertion_octoMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_colourOctoMaps,bool,		options.enableInsertion_colourOctoMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_gasGridMaps,bool,	options.enableInsertion_gasGridMaps,	ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_wifiGridMaps,bool,	options.enableInsertion_wifiGridMaps,	ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_beaconMap,bool,	options.enableInsertion_beaconMap,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_heightMaps,bool,	options.enableInsertion_heightMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_reflectivityMaps,bool,	options.enableInsertion_reflectivityMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_colourPointsMaps,bool,	options.enableInsertion_colourPointsMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_weightedPointsMaps,bool,	options.enableInsertion_weightedPointsMaps,		ini,sectionName);
#endif

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

#if 0
		if (it->metricMapClassType==CLASS_ID(CLandmarksMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("number of initial beacons               = %u\n",(int)it->landmarksMap_options.initialBeacons.size());

			out.printf("      ID         (X,Y,Z)\n");
			out.printf("--------------------------------------------------------\n");
			for (std::deque<CMultiMetricMap::TPairIdBeacon>::const_iterator p=it->landmarksMap_options.initialBeacons.begin();p!=it->landmarksMap_options.initialBeacons.end();++p)
				out.printf("      %03u         (%8.03f,%8.03f,%8.03f)\n", p->second,p->first.x(),p->first.y(),p->first.z());

			it->landmarksMap_options.insertionOpts.dumpToTextStream(out);
			it->landmarksMap_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CBeaconMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->beaconMap_options.likelihoodOpts.dumpToTextStream(out);
			it->beaconMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CGasConcentrationGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                   = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CGasConcentrationGridMap2D::TMapRepresentation>::value2name(it->gasGridMap_options.mapType).c_str() );
			out.printf("min_x                                     = %f\n", it->gasGridMap_options.min_x );
			out.printf("max_x                                     = %f\n", it->gasGridMap_options.max_x );
			out.printf("min_y                                     = %f\n", it->gasGridMap_options.min_y );
			out.printf("max_y                                     = %f\n", it->gasGridMap_options.max_y );
			out.printf("resolution                                = %f\n", it->gasGridMap_options.resolution );

			it->gasGridMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CWirelessPowerGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                   = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CWirelessPowerGridMap2D::TMapRepresentation>::value2name(it->wifiGridMap_options.mapType).c_str() );
			out.printf("min_x                                     = %f\n", it->wifiGridMap_options.min_x );
			out.printf("max_x                                     = %f\n", it->wifiGridMap_options.max_x );
			out.printf("min_y                                     = %f\n", it->wifiGridMap_options.min_y );
			out.printf("max_y                                     = %f\n", it->wifiGridMap_options.max_y );
			out.printf("resolution                                = %f\n", it->wifiGridMap_options.resolution );

			it->wifiGridMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CHeightGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                   = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CHeightGridMap2D::TMapRepresentation>::value2name(it->heightMap_options.mapType).c_str() );
			out.printf("min_x                                     = %f\n", it->heightMap_options.min_x );
			out.printf("max_x                                     = %f\n", it->heightMap_options.max_x );
			out.printf("min_y                                     = %f\n", it->heightMap_options.min_y );
			out.printf("max_y                                     = %f\n", it->heightMap_options.max_y );
			out.printf("resolution                                = %f\n", it->heightMap_options.resolution );

			it->heightMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CReflectivityGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                   = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("min_x                                     = %f\n", it->reflectivityMap_options.min_x );
			out.printf("max_x                                     = %f\n", it->reflectivityMap_options.max_x );
			out.printf("min_y                                     = %f\n", it->reflectivityMap_options.min_y );
			out.printf("max_y                                     = %f\n", it->reflectivityMap_options.max_y );
			out.printf("resolution                                = %f\n", it->reflectivityMap_options.resolution );

			it->reflectivityMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CSimplePointsMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->pointsMapOptions_options.insertionOpts.dumpToTextStream(out);
			it->pointsMapOptions_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
			if (it->metricMapClassType==CLASS_ID(CColouredPointsMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->colouredPointsMapOptions_options.insertionOpts.dumpToTextStream(out);
			it->colouredPointsMapOptions_options.likelihoodOpts.dumpToTextStream(out);
			it->colouredPointsMapOptions_options.colourOpts.dumpToTextStream(out);
		}
		else
			if (it->metricMapClassType==CLASS_ID(CWeightedPointsMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->weightedPointsMapOptions_options.insertionOpts.dumpToTextStream(out);
			it->weightedPointsMapOptions_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
			if (it->metricMapClassType==CLASS_ID(COctoMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->octoMap_options.insertionOpts.dumpToTextStream(out);
			it->octoMap_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
			if (it->metricMapClassType==CLASS_ID(CColouredOctoMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->colourOctoMap_options.insertionOpts.dumpToTextStream(out);
			it->colourOctoMap_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
		{
			THROW_EXCEPTION_CUSTOM_MSG1("Unknown class!: '%s'",it->metricMapClassType->className);
		}
#endif
	} // for "it"

	MRPT_END
}






