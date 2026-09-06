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

#include <mrpt/gui/config.h>

#include <cstdlib>

namespace mrpt::test
{
/** Empty if a window can actually be created in this environment; otherwise a
 * human-readable reason why not.
 *
 * On X11 systems a display is required; CI runs these tests under a virtual
 * one (`xvfb-run`), and they self-skip when none is available. Windows and
 * macOS always have a window server, but MRPT there may have been built with
 * `-DDISABLE_WXWIDGETS=ON`, in which case there is no window class to test.
 */
[[nodiscard]] inline const char* guiUnavailableReason()
{
  // Escape hatch for environments where creating windows is undesirable
  // (remote sessions, sandboxes, ...):
  if (const char* off = ::getenv("MRPT_SKIP_GUI_TESTS"); off != nullptr && off[0] == '1')
  {
    return "MRPT_SKIP_GUI_TESTS=1 is set.";
  }

#if !MRPT_HAS_WXWIDGETS
  return "MRPT was built without wxWidgets (-DDISABLE_WXWIDGETS=ON).";
#elif defined(_WIN32) || defined(__APPLE__)
  return "";
#else
  if (const char* d = ::getenv("DISPLAY"); d != nullptr && d[0] != '\0')
  {
    return "";
  }
  if (const char* w = ::getenv("WAYLAND_DISPLAY"); w != nullptr && w[0] != '\0')
  {
    return "";
  }
  return "No window server available (set DISPLAY, or run the tests under "
         "`xvfb-run -a`).";
#endif
}

/** True if a window can actually be created in this environment. */
[[nodiscard]] inline bool guiIsAvailable() { return guiUnavailableReason()[0] == '\0'; }
}  // namespace mrpt::test

/** Skips the current test if no window can be created, saying why. */
#define SKIP_IF_NO_GUI()                                                      \
  do                                                                          \
  {                                                                           \
    if (const char* why = mrpt::test::guiUnavailableReason(); why[0] != '\0') \
    {                                                                         \
      GTEST_SKIP() << why;                                                    \
    }                                                                         \
  } while (0)
