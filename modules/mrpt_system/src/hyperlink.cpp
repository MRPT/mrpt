/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers
//
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
    return mrpt::format("\e]8;;%s\e\\%s\e]8;;\e\\", uri.c_str(), text.c_str());
  else
    return show_uri_anyway ?  //
               (text + " ("s + uri + ")"s)
                           :  //
               text;
}
