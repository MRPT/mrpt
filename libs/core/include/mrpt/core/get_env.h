/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/from_string.h>
#include <cstdlib>

namespace mrpt
{
/** Reads an environment variable, with a default value if not present.
 * \ingroup mrpt_core_grp
 */
template <class T>
inline T get_env(const std::string& varname, const T& defValue = T())
{
	auto s = ::getenv(varname.c_str());
	if (!s) return defValue;
	return mrpt::from_string<T>(s, defValue, false /*dont throw*/);
}

}  // namespace mrpt
