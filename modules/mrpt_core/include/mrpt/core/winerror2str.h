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

#include <string>

namespace mrpt
{
/** [Windows only] Returns an error string "[<caller>] Error: <error msg>" with
 * "<caller>" the given user-provided string, and "<error msg>" the combination
 * of calling GetLastError() + FormatMessageA(). If no "<caller>" string is
 * provided (nullptr), it just returns the "<error msg>" part.
 *
 * Returns an empty string in non Windows OSes, or if GetLastError() gives no
 * error.
 * \note (New in MRPT 2.1.5)
 * \ingroup mrpt_core_grp
 */
std::string winerror2str(const char* caller = nullptr, const char* errorPrefix = " Error: ");

}  // namespace mrpt
