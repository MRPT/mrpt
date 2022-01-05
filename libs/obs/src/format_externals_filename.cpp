/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/format_externals_filename.h>

#include <regex>

std::string mrpt::obs::format_externals_filename(
	const mrpt::obs::CObservation& obs, const std::string& fmt)
{
	std::string sType = "other";
	if (IS_CLASS(obs, mrpt::obs::CObservationImage)) sType = "img";
	else if (IS_CLASS(obs, mrpt::obs::CObservationStereoImages))
		sType = "stereo";
	else if (IS_CLASS(obs, mrpt::obs::CObservation3DRangeScan))
		sType = "3dcam";

	std::string ret = fmt;

	ret = std::regex_replace(ret, std::regex("\\$\\{type\\}"), sType);
	ret =
		std::regex_replace(ret, std::regex("\\$\\{label\\}"), obs.sensorLabel);

	ret = mrpt::format(ret.c_str(), mrpt::Clock::toDouble(obs.getTimeStamp()));

	return ret;
}