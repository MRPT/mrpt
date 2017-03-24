/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/maps/metric_map_types.h>

using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(TMapGenericParams, CSerializable, mrpt::maps )

TMapGenericParams::TMapGenericParams() : 
	enableSaveAs3DObject(true), 
	enableObservationLikelihood(true), 
	enableObservationInsertion(true) 
{ 
}

void TMapGenericParams::loadFromConfigFile(const mrpt::utils::CConfigFileBase  &source, const std::string &sct)
{
	MRPT_LOAD_CONFIG_VAR(enableSaveAs3DObject          , bool,   source,sct);
	MRPT_LOAD_CONFIG_VAR(enableObservationLikelihood   , bool,   source,sct);
	MRPT_LOAD_CONFIG_VAR(enableObservationInsertion    , bool,   source,sct);
}
void TMapGenericParams::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	// Common:
	LOADABLEOPTS_DUMP_VAR(enableSaveAs3DObject         , bool);
	LOADABLEOPTS_DUMP_VAR(enableObservationLikelihood  , bool);
	LOADABLEOPTS_DUMP_VAR(enableObservationInsertion   , bool);
}

void TMapGenericParams::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << enableSaveAs3DObject << enableObservationLikelihood << enableObservationInsertion;
	}
}

void TMapGenericParams::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in >> enableSaveAs3DObject >> enableObservationLikelihood >> enableObservationInsertion;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

