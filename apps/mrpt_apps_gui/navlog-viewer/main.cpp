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
#include <mrpt/system/os.h>
#include <mrpt/version.h>

#include <CLI/CLI.hpp>
#include <iostream>

#include "navlog-viewer-ui.h"

int main(int argc, char** argv)
{
  try
  {
    // Declare the supported command line switches ===========
    CLI::App app("navlog-viewer");
    app.set_version_flag("--version", MRPT_version_str);

    std::vector<std::string> inputFiles;
    app.add_option("LogFiles", inputFiles, "Input navlog files (*.reactivenavlog)");

    std::string extraModuleToLoad;
    app.add_option("-l,--load", extraModuleToLoad, "Load an additional DLL/SO module (*.so/*.dll)");

    CLI11_PARSE(app, argc, argv);

    if (!extraModuleToLoad.empty())
    {
      std::cout << "Loading plugin modules: " << extraModuleToLoad << "...\n";
      mrpt::system::loadPluginModules(extraModuleToLoad);
    }

    // UI setup:
    nanogui::init();

    NavlogViewerApp viewer_app;

    if (!inputFiles.empty()) viewer_app.loadLogfile(inputFiles.at(0));

    nanogui::mainloop();

    nanogui::shutdown();

    return 0;
  }
  catch (const std::exception& e)
  {
    if (strlen(e.what()) > 0) std::cerr << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
}
