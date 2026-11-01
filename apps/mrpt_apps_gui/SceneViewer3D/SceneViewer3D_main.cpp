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
#include <mrpt/img/CImage.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <iostream>

#include "SceneViewerApp.h"

// Forces linking mrpt-maps, so its visual object classes (e.g. laser scans)
// are registered and can be deserialized from .3Dscene files:
const mrpt::maps::CGenericPointsMap dummy_map;

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli("SceneViewer3D: viewer of 3D scenes (.3Dscene), 3D models and point clouds");
    std::string file;
    std::string plugins;
    std::string imgDir;
    cli.add_option("file", file, "File to open: a .3Dscene file, or any 3D model (Assimp)");
    cli.add_option("-l,--load", plugins, "Load plugin libraries (comma-separated)");
    cli.add_option("-d,--imgdir", imgDir, "Base directory of lazy-load (external) images");
    CLI11_PARSE(cli, argc, argv);

    if (!plugins.empty())
    {
      std::cout << "Loading plugin libraries: " << plugins << "...\n";
      mrpt::system::loadPluginModules(plugins);
    }
    if (!imgDir.empty())
    {
      std::cout << "Using lazy-load images base directory: " << imgDir << "\n";
      mrpt::img::CImage::setImagesPathBase(imgDir);
    }

    SceneViewerApp app;
    return app.run(file);
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
