/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <sstream>
#include <string>
#include <type_traits>

namespace mrpt
{
namespace internal
{
inline std::istringstream& get_istringstream()
{
	static thread_local std::istringstream stream;
	stream.str("");
	return stream;
}
}  // namespace internal

/** Converts from string to any data type that supports reading (>>) from a text
 * stream. In case of error, the given default value is returned, or an
 * exception raised. \ingroup mrpt_core_grp
 */
template <typename T>
inline T from_string(
	const std::string& s, const T& defValue = T{}, bool throw_on_error = true)
{
	// shortcut that also fixes problems with strings with whitespaces:
	if constexpr (std::is_same_v<std::string, T>) { return s; }

	auto& iss(internal::get_istringstream());
	iss.clear();  // Reset "bad" bit
	iss.str(s);
	iss.seekg(0);
	T result = defValue;
	if (!(iss >> result))
	{
		result = defValue;
		if (throw_on_error)
			throw std::runtime_error(
				std::string("[from_string()] Cannot parse string: ") + s);
	}
	return result;
}

}  // namespace mrpt
