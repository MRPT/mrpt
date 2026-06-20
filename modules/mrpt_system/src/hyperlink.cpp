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

#include <mrpt/core/format.h>
#include <mrpt/system/hyperlink.h>

#ifndef _WIN32
#include <unistd.h>

#include <cstdio>
#endif

std::string mrpt::system::hyperlink(
    const std::string& text, const std::string& uri, bool force_use_format, bool show_uri_anyway)
{
  using namespace std::string_literals;

  bool supportsLinks = false;

#ifndef _WIN32
  thread_local bool coutIsAtty = isatty(fileno(stdout));
  supportsLinks = coutIsAtty;
#endif
  // See: https://gist.github.com/egmontkob/eb114294efbcd5adb1944c9f3cb5feda
  if (supportsLinks || force_use_format)
  {
    return mrpt::format("\033]8;;%s\033\\%s\033]8;;\033\\", uri.c_str(), text.c_str());
  }
  return show_uri_anyway ?  //
             (text + " ("s + uri + ")"s)
                         :  //
             text;
}
