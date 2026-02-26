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

#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/viz/Scene.h>

#include <CLI/CLI.hpp>

int main(int argc, char** argv)
{
  try
  {
    // Declare the supported command line switches ===========
    CLI::App app("ros-map-yaml2mrpt");
    app.set_version_flag("--version", mrpt::system::MRPT_getVersion());

    std::string inputFile;
    app.add_option("-i,--input", inputFile, "Input map yaml file (required) (*.yaml)")->required();

    std::string outputDirectory;
    app.add_option(
        "-d,--output-directory", outputDirectory,
        "If provided, output files will be written to the specified directory, "
        "instead of the same directory of the input file, which is the default "
        "behavior. The output directory must exist, it will not be created.");

    bool overwrite = false;
    app.add_flag("-w,--overwrite", overwrite, "Force overwrite target file without prompting.");

    bool quiet = false;
    app.add_flag("-q,--quiet", quiet, "Do not print info messages to cout, only errors to cerr");

    bool generate_3d = false;
    app.add_flag(
        "--generate-3d", generate_3d,
        "Create a .3Dscene view of the gridmap, suitable for quick visualization in the "
        "SceneViewer3D program.");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    if (!quiet)
    {
      printf(  //
          " ros-map-yaml2mrpt - Part of %s\n"
          "------------------------------------------------------------"
          "\n",
          mrpt::system::MRPT_getVersion().c_str());
    }

    const auto grid = mrpt::maps::COccupancyGridMap2D::FromROSMapServerYAML(inputFile);

    const std::string outDir =
        !outputDirectory.empty() ? outputDirectory : mrpt::system::extractFileDirectory(inputFile);

    const std::string outGridFil = mrpt::system::pathJoin(
        {outDir, mrpt::system::fileNameChangeExtension(
                     mrpt::system::extractFileName(inputFile), "gridmap.gz")});

    std::string out3D;
    if (generate_3d)
    {
      out3D = mrpt::system::pathJoin(
          {outDir, mrpt::system::fileNameChangeExtension(
                       mrpt::system::extractFileName(inputFile), "3Dscene")});
    }

    if (!quiet)
    {
      std::cout << "Input file        : " << inputFile << "\n";
      std::cout << "Output gridmap    : " << outGridFil << "\n";
      if (!out3D.empty()) std::cout << "Output 3D view    : " << out3D << "\n";
    }

    if (mrpt::system::fileExists(outGridFil) && !overwrite)
    {
      std::cerr << "Output gridmap file already exists, aborting. Use "
                   "`-w` flag "
                   "to overwrite."
                << "\n";
      return 1;
    }
    if (mrpt::system::fileExists(out3D) && !overwrite)
    {
      std::cerr << "Output 3D file already exists, aborting. Use `-w` flag "
                   "to overwrite."
                << "\n";
      return 1;
    }

    {
      mrpt::io::CCompressedInputStream f(outGridFil);
      mrpt::serialization::archiveFrom(f) << grid;
    }

    if (!out3D.empty())
    {
      mrpt::viz::Scene scene;
      scene.insert(grid.getVisualization());
      scene.saveToFile(out3D);
    }

    std::cout << "All done.\n";

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
