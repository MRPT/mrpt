/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <string>

namespace mrpt::obs
{
/** Replaces format placeholders in a string according to an observation:
 *  
 *  For example, the default format string used in 
 *  `rawlog-edit --rename-externals` is `"${type}_${label}_%.06%f"`.
 *  
 * \ingroup mrpt_obs_grp
 * \note (new in MRPT 2.4.1)
 */
std::string format_externals_filename(
	const mrpt::obs::CObservation& obs, const std::string& fmt);
 
};	// End of class def.
