/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/from_string.h>

#include <cstdlib>
#include <string_view>

namespace mrpt
{
/** Reads an environment variable, with a default value if not present.
 * \ingroup mrpt_core_grp
 */
template <class T>
inline T get_env(const std::string_view& varname, const T& defValue = T())
{
	const std::string v(varname.data(), varname.size());
	auto s = ::getenv(v.c_str());
	if (!s) return defValue;
	return mrpt::from_string<T>(s, defValue, false /*dont throw*/);
}

/** Specialization for bool: understands "true", "True", number!=0 as `true` */
template <>
inline bool get_env(const std::string_view& varname, const bool& defValue)
{
	const std::string v(varname.data(), varname.size());
	auto s = ::getenv(v.c_str());
	if (!s) return defValue;
	const std::string str(s);
	if (str == "true" || str == "TRUE" || str == "True") return true;
	if (0 != mrpt::from_string<int>(s, 0, false /*dont throw*/)) return true;
	return false;
}

}  // namespace mrpt
