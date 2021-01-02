/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
std::string winerror2str(
	const char* caller = nullptr, const char* errorPrefix = " Error: ");

}  // namespace mrpt
