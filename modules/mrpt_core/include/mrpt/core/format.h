/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#pragma once

#include <mrpt/core/common.h>

#include <string>

/** This is the global namespace for all Mobile Robot Programming Toolkit (MRPT)
 * libraries. */
namespace mrpt
{
/** A std::string version of C sprintf.
 *  You can call this to obtain a std::string using printf-like syntax.
 * \ingroup mrpt_core_grp
 */
std::string format(const char* fmt, ...) MRPT_printf_format_check(1, 2);

/** Just like std::to_string(), but with an overloaded version
 * for std::string arguments.
 * \ingroup mrpt_core_grp
 */
template <typename T>
std::string to_string(T v)
{
  return std::to_string(v);
}
template <>
inline std::string to_string<>(std::string v)
{
  return v;
}
template <>
inline std::string to_string<>(bool v)
{
  return v ? "true" : "false";
}
template <>
inline std::string to_string<>(const char* s)
{
  return std::string(s);
}
}  // namespace mrpt
