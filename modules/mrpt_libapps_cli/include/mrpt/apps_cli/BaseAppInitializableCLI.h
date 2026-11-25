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

#include <iostream>
#include <optional>
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

  /** To be called from main() before initialize(): if the arguments contain
   * "-h" or "--help", prints the usage to stdout and returns 0; if the number
   * of arguments is wrong, prints the usage to stderr and returns 1. The
   * returned value is the program exit code. Otherwise, returns std::nullopt
   * and the program should go on.
   */
  [[nodiscard]] std::optional<int> checkUsage(int argc, const char** argv) const
  {
    for (int i = 1; i < argc; i++)
    {
      const std::string arg = argv[i];
      if (arg == "-h" || arg == "--help")
      {
        std::cout << "Usage: " << impl_get_usage() << "\n";
        return 0;
      }
    }
    if (argc < impl_min_argc() || argc > impl_max_argc())
    {
      std::cerr << "Usage: " << impl_get_usage() << "\n";
      return 1;
    }
    return std::nullopt;
  }

  [[nodiscard]] std::optional<int> checkUsage(int argc, char** argv) const
  {
    return checkUsage(argc, const_cast<const char**>(argv));
  }

 protected:
  virtual void impl_initialize(int argc, const char** argv) = 0;
  virtual std::string impl_get_usage() const = 0;

  /** Valid range of argc, including the program name. */
  virtual int impl_min_argc() const { return 2; }
  virtual int impl_max_argc() const { return 3; }
};

}  // namespace mrpt::apps
