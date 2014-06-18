/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/graphslam.h>
#include <mrpt/utils/CConfigFileBase.h>

using namespace mrpt::graphslam;
using namespace mrpt::graphslam::deciders;

// Default ctor
GS_GenericDecider::GS_GenericDecider()
{
}

// TParams:
GS_GenericDecider::TParams::TParams() :
	new_kf_min_distance_xy (1.0),
	new_kf_min_angle (DEG2RAD(30.0)),
	verbose(false)
{
}

void GS_GenericDecider::TParams::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR(new_kf_min_distance_xy, double, source,section)
	MRPT_LOAD_CONFIG_VAR_DEGREES(new_kf_min_angle, source,section)
	MRPT_LOAD_CONFIG_VAR(verbose,bool, source,section)

}

void GS_GenericDecider::TParams::dumpToTextStream(mrpt::utils::CStream &out) const
{
	out.printf("\n----------- [GS_GenericDecider::TParams] ------------ \n\n");
	LOADABLEOPTS_DUMP_VAR(new_kf_min_distance_xy, double)
	LOADABLEOPTS_DUMP_VAR_DEG(new_kf_min_angle)
	LOADABLEOPTS_DUMP_VAR_DEG(verbose)
}

