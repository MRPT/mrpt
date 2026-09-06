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
/** True if a window can actually be created in this environment.
 *
 * On X11 systems a display is required; CI runs these tests under a virtual
 * one (`xvfb-run`), and they self-skip when none is available. Windows and
 * macOS always have a window server.
 */
[[nodiscard]] inline bool guiIsAvailable()
{
  // Escape hatch for environments where creating windows is undesirable
  // (remote sessions, sandboxes, ...):
  if (const char* off = ::getenv("MRPT_SKIP_GUI_TESTS"); off != nullptr && off[0] == '1')
  {
    return false;
  }

#if !MRPT_HAS_WXWIDGETS
  return false;
#elif defined(_WIN32) || defined(__APPLE__)
  return true;
#else
  const char* d = ::getenv("DISPLAY");
  if (d != nullptr && d[0] != '\0')
  {
    return true;
  }
  const char* w = ::getenv("WAYLAND_DISPLAY");
  return w != nullptr && w[0] != '\0';
#endif
}
}  // namespace mrpt::test

/** Skips the current test if no window server is reachable. */
#define SKIP_IF_NO_GUI()                                                     \
  do                                                                         \
  {                                                                          \
    if (!mrpt::test::guiIsAvailable())                                       \
    {                                                                        \
      GTEST_SKIP() << "No window server available (set DISPLAY, or run the " \
                      "tests under `xvfb-run -a`).";                         \
    }                                                                        \
  } while (0)
