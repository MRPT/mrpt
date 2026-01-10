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

#include <mrpt/core/from_string.h>

#include <string>
#include <string_view>

namespace mrpt
{
namespace detail
{
// Thread-safe environment variable retrieval
std::string get_env_string_threadsafe(const std::string_view& varname);
}  // namespace detail

/** Reads an environment variable, with a default value if not present.
 * Thread-safe implementation using mutex protection and secure_getenv() on Linux.
 * \ingroup mrpt_core_grp
 */
template <class T>
inline T get_env(const std::string_view& varname, const T& defValue = T())
{
  const std::string env_value = detail::get_env_string_threadsafe(varname);

  if (env_value.empty())
  {
    return defValue;
  }

  return mrpt::from_string<T>(env_value, defValue, false /*dont throw*/);
}

/** Specialization for bool: understands "true", "True", number!=0 as `true` */
template <>
inline bool get_env(const std::string_view& varname, const bool& defValue)
{
  const std::string env_value = detail::get_env_string_threadsafe(varname);

  if (env_value.empty())
  {
    return defValue;
  }

  if (env_value == "true" || env_value == "TRUE" || env_value == "True")
  {
    return true;
  }

  if (0 != mrpt::from_string<int>(env_value, 0, false /*dont throw*/))
  {
    return true;
  }

  return false;
}

}  // namespace mrpt