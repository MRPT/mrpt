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

#include <cstddef>

namespace mrpt::apps
{
/** The C++ class behind the rawlog-edit CLI tool.
 *
 *  Refer to the online documentation for rawlog-edit.
 * \ingroup mrpt_apps_grp
 */
class RawlogEditApp
{
 public:
  RawlogEditApp() = default;

  /** @name Main API
   * @{ */

  /** Initializes and runs the application from CLI parameters. Refer to the
   * manpage of rawlog-edit. Throws on errors.
   */
  void run(int argc, const char** argv);

  void run(int argc, char** argv) { run(argc, const_cast<const char**>(argv)); }

  /** @} */
};

}  // namespace mrpt::apps
