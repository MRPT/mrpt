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
/** Build an text incremental progress bar with
 * [UNICODE block elements](https://en.wikipedia.org/wiki/Block_Elements),
 * for example:
 * \code
 * 50%  => "[████    ]"
 * \endcode
 *
 * See example: \ref system_progress_bar/test.cpp
 * \snippet system_progress_bar/test.cpp example-system-progress-bar
 *
 * \ingroup mrpt_system_grp
 * \note (New in MRPT 2.3.0)
 */
std::string progress(
    const double progressRatio0to1,
    const std::size_t barLength,
    bool encloseInSquareBrackets = true);

}  // namespace mrpt::system
