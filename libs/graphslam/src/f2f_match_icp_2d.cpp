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
using namespace mrpt::graphslam::f2f_match;

// Default ctor
GS_F2F_ICP_2D::GS_F2F_ICP_2D()
{
}

// TParams:
GS_F2F_ICP_2D::TParams::TParams() :
	kf2kf_max_search_radius (6.0)
{
}

void GS_F2F_ICP_2D::TParams::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string &section)
{
}

void GS_F2F_ICP_2D::TParams::dumpToTextStream(mrpt::utils::CStream &out) const
{
}

		