/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
