/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/obs/CObservation.h>

#include <string>

namespace mrpt::obs
{
/** Replaces format placeholders in a string according to an observation:
 *  - `${type}` is replaced by: `img`, `stereo`, `3dcam` for single images,
 *     stereo images and depth camera observations, respectively, or `other`
 *     otherwise.
 *  - `${label}` is replaced by the observation `sensorLabel` field.
 *  - `%f` with any standard `printf()` format modifiers will be replaced by
 *    UNIX timestamp (with fractions of seconds) of the observation.
 *  - Anything else will be left unmodified.
 *
 *  For example, the default format string used in
 *  `rawlog-edit --rename-externals` is `"${type}_${label}_%.06%f"`.
 *
 * \ingroup mrpt_obs_grp
 * \note (new in MRPT 2.4.1)
 */
std::string format_externals_filename(const mrpt::obs::CObservation& obs, const std::string& fmt);

};  // namespace mrpt::obs
