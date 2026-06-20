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

#include <mrpt/core/get_env.h>

#include <cstdlib>
#include <mutex>
#include <string>
#include <string_view>

// Platform detection for secure environment variable access
#if defined(__linux__) || defined(__gnu_linux__)
#define MRPT_HAS_SECURE_GETENV 1
#elif defined(_WIN32)
#define MRPT_HAS_SECURE_GETENV 0
#elif defined(__APPLE__) && defined(__MACH__)
#define MRPT_HAS_SECURE_GETENV 0
#else
#define MRPT_HAS_SECURE_GETENV 0
#endif

namespace mrpt::detail
{
std::string get_env_string_threadsafe(const std::string_view& varname)
{
  static std::mutex env_mutex;
  std::lock_guard<std::mutex> lock(env_mutex);

  const std::string v(varname.data(), varname.size());

#if MRPT_HAS_SECURE_GETENV
  // Linux: use secure_getenv which is also thread-safe in practice
  const char* s = ::secure_getenv(v.c_str());
#else
  // Other platforms: use standard getenv with mutex protection
  const char* s = ::getenv(v.c_str());
#endif

  if (s == nullptr)
  {
    return {};
  }

  // Copy the string while holding the lock to avoid potential race conditions
  return {s};
}
}  // namespace mrpt::detail

#undef MRPT_HAS_SECURE_GETENV