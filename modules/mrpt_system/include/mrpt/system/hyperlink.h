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

#include <cstddef>
#include <string>

namespace mrpt::system
{
/** Build an text with a hyperlink, if printed to a terminal that supports it
 * (e.g. Ubuntu >=21.04), or the plain text otherwise.
 *
 * \ingroup mrpt_system_grp
 * \note (New in MRPT 2.11.8)
 * \sa See discussion in, for example,
 * https://gist.github.com/egmontkob/eb114294efbcd5adb1944c9f3cb5feda
 */
std::string hyperlink(
    const std::string& text,
    const std::string& uri,
    bool force_use_format = false,
    bool show_uri_anyway = false);

}  // namespace mrpt::system
