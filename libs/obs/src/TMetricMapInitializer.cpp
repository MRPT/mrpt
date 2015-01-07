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
	// Common:
	LOADABLEOPTS_DUMP_VAR(enableSaveAs3DObject         , bool);
	LOADABLEOPTS_DUMP_VAR(enableObservationLikelihood  , bool);
	LOADABLEOPTS_DUMP_VAR(enableObservationInsertion   , bool);
	
	// Class-specific:
	this->dumpToTextStream(out);
}




