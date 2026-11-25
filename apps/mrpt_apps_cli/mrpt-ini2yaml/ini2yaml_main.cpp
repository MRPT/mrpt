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

#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>

#include <iostream>
#include <string_view>

namespace
{
void printUsage(std::ostream& o)
{
  o << "Usage: mrpt-ini2yaml <input.ini>\n"
       "Converts an MRPT INI configuration file into YAML, printed to stdout.\n"
       "Options:\n"
       "  -h, --help  Show this help and exit.\n";
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    // Parse arguments:
    if (argc == 2 && (std::string_view(argv[1]) == "-h" || std::string_view(argv[1]) == "--help"))
    {
      printUsage(std::cout);
      return 0;
    }
    if (argc != 2)
    {
      printUsage(std::cerr);
      return 1;
    }

    const std::string iniFile = std::string(argv[1]);
    ASSERT_FILE_EXISTS_(iniFile);

    mrpt::config::CConfigFile cfg(iniFile);
    const std::string s = cfg.getContentAsYAML();

    std::cout << s << "\n";

    // successful end of program.
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
