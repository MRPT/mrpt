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

#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <clocale>
#include <iostream>
#include <optional>

#include "RawLogViewerApp.h"

// Forces linking mrpt-maps, so the classes of its observations and maps are
// registered and can be deserialized:
const mrpt::maps::CGenericPointsMap dummy_map;

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli("RawLogViewer: viewer and editor of MRPT datasets (rawlog files)");
    std::string file;
    std::string plugins;
    cli.add_option("file", file, "Rawlog file to open");
    std::optional<size_t> entry;
    cli.add_option("-l,--load", plugins, "Load plugin libraries (comma-separated)");
    cli.add_option("-e,--entry", entry, "Entry to select once the rawlog is loaded");
    CLI11_PARSE(cli, argc, argv);

    // Consistent number formats in the generated files, in all locales:
    std::setlocale(LC_NUMERIC, "C");

    if (!plugins.empty())
    {
      std::cout << "Loading plugin libraries: " << plugins << "...\n";
      mrpt::system::loadPluginModules(plugins);
    }

    RawLogViewerApp app;
    app.initialEntry = entry;
    return app.run(file);
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
