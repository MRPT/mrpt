/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
#include <mrpt/maps/CHeightGridMap2D_MRF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace std;
using namespace mrpt::math;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"CHeightGridMap2D_MRF,dem_mrf", mrpt::maps::CHeightGridMap2D_MRF)

CHeightGridMap2D_MRF::TMapDefinition::TMapDefinition()
	: run_map_estimation_at_ctor(true),
	  min_x(-2),
	  max_x(2),
	  min_y(-2),
	  max_y(2),
	  resolution(0.10f),
	  mapType(mrGMRF_SD)
{
}

void CHeightGridMap2D_MRF::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation =
		sectionNamePrefix + string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(
		run_map_estimation_at_ctor, bool, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_x, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_x, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_y, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_y, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(resolution, double, source, sSectCreation);
	mapType = source.read_enum<CHeightGridMap2D_MRF::TMapRepresentation>(
		sSectCreation, "mapType", mapType);

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
}

void CHeightGridMap2D_MRF::TMapDefinition::dumpToTextStream_map_specific(
	mrpt::utils::CStream& out) const
{
	out.printf(
		"MAP TYPE                                  = %s\n",
		mrpt::utils::TEnumType<
			CHeightGridMap2D_MRF::TMapRepresentation>::value2name(mapType)
			.c_str());
	LOADABLEOPTS_DUMP_VAR(run_map_estimation_at_ctor, bool);
	LOADABLEOPTS_DUMP_VAR(min_x, double);
	LOADABLEOPTS_DUMP_VAR(max_x, double);
	LOADABLEOPTS_DUMP_VAR(min_y, double);
	LOADABLEOPTS_DUMP_VAR(max_y, double);
	LOADABLEOPTS_DUMP_VAR(resolution, double);

	this->insertionOpts.dumpToTextStream(std::ostream& out) const
{
	out.printf(
		"\n----------- [CHeightGridMap2D_MRF::TInsertionOptions] ------------ "
		"\n\n");
	out.printf("[TInsertionOptions.Common] ------------ \n\n");
	internal_dumpToTextStream_common(
		out);  // Common params to all random fields maps:

	//	out.printf("[TInsertionOptions.CHeightGridMap2D_MRF] ------------
	//\n\n");
	//	out.printf("std_windNoise_phi						= %f\n",
	// std_windNoise_phi);

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CHeightGridMap2D_MRF::TInsertionOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	// Common data fields for all random fields maps:
	internal_loadFromConfigFile_common(iniFile, section);

	// Specific data fields for this class:
	// ...
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void CHeightGridMap2D_MRF::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	MRPT_START
	if (!genericMapParams.enableSaveAs3DObject) return;
	CRandomFieldGridMap2D::getAs3DObject(outObj);
	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void CHeightGridMap2D_MRF::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& meanObj,
	mrpt::opengl::CSetOfObjects::Ptr& varObj) const
{
	MRPT_START
	if (!genericMapParams.enableSaveAs3DObject) return;
	CRandomFieldGridMap2D::getAs3DObject(meanObj, varObj);
	MRPT_END
}
