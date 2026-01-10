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

#include <string>

namespace mrpt::apps
{
/** Virtual interface for applications that initialize from CLI parameters.
 *
 * \ingroup mrpt_apps_grp
 */
class BaseAppInitializableCLI
{
 public:
  BaseAppInitializableCLI() = default;
  virtual ~BaseAppInitializableCLI() = default;

 protected:
  virtual void impl_initialize(int argc, const char** argv) = 0;
  virtual std::string impl_get_usage() const = 0;
};

}  // namespace mrpt::apps
