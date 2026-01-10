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

#include <mrpt/core/winerror2str.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

std::string mrpt::winerror2str(
    [[maybe_unused]] const char* caller, [[maybe_unused]] const char* errorPrefix)
{
#ifdef _WIN32
  DWORD e = GetLastError();
  if (!e) return {};  // no error

  char s[2048];
  FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, 0, e, 0, s, sizeof(s), nullptr);
  if (caller != nullptr)
  {
    std::string ret;
    ret = "[";
    ret += caller;
    ret += "]";
    if (errorPrefix) ret += errorPrefix;
    ret += s;
    return ret;
  }
  else
    return s;
#else
  return {};
#endif
}
