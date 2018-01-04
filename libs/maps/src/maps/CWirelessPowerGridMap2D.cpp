/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CWirelessPowerGridMap2D.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/system/os.h>
#include <mrpt/core/round.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"CWirelessPowerGridMap2D,wifiGrid", mrpt::maps::CWirelessPowerGridMap2D)

CWirelessPowerGridMap2D::TMapDefinition::TMapDefinition()
	: min_x(-2),
	  max_x(2),
	  min_y(-2),
	  max_y(2),
	  resolution(0.10f),
	  mapType(CWirelessPowerGridMap2D::mrKernelDM)
{
}

void CWirelessPowerGridMap2D::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation =
		sectionNamePrefix + string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(min_x, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_x, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_y, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_y, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(resolution, double, source, sSectCreation);
	mapType = source.read_enum<CWirelessPowerGridMap2D::TMapRepresentation>(
		sSectCreation, "mapType", mapType);

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
}

void CWirelessPowerGridMap2D::TMapDefinition::dumpToTextStream_map_specific(
	mrpt::utils::CStream& out) const
{
	out << mrpt::format(
		"MAP TYPE                                  = %s\n",
		mrpt::utils::TEnumType<
			CWirelessPowerGridMap2D::TMapRepresentation>::value2name(mapType)
			.c_str());
	LOADABLEOPTS_DUMP_VAR(min_x, double);
	LOADABLEOPTS_DUMP_VAR(max_x, double);
	LOADABLEOPTS_DUMP_VAR(min_y, double);
	LOADABLEOPTS_DUMP_VAR(max_y, double);
	LOADABLEOPTS_DUMP_VAR(resolution, double);

	this->insertionOpts.dumpToTextStream(std::ostream& out) const
{
	out << mrpt::format(
		"\n----------- [CWirelessPowerGridMap2D::TInsertionOptions] "
		"------------ \n\n");
	internal_dumpToTextStream_common(
		out);  // Common params to all random fields maps:

	out << mrpt::format("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CWirelessPowerGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	// Common data fields for all random fields maps:
	internal_loadFromConfigFile_common(iniFile, section);
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void CWirelessPowerGridMap2D::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	MRPT_START
	if (!genericMapParams.enableSaveAs3DObject) return;
	CRandomFieldGridMap2D::getAs3DObject(outObj);
	MRPT_END
}
