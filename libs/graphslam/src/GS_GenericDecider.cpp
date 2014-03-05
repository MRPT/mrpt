/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/graphslam.h>

using namespace mrpt::graphslam;
using namespace mrpt::graphslam::deciders;

// Default ctor
GS_GenericDecider::GS_GenericDecider()
{
}

// TParams:
GS_GenericDecider::TParams::TParams() :
	new_kf_min_distance_xy (1.0),
	new_kf_min_angle (DEG2RAD(30.0))
{
}

void GS_GenericDecider::TParams::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string &section)
{
}

void GS_GenericDecider::TParams::dumpToTextStream(mrpt::utils::CStream &out) const
{
}

		