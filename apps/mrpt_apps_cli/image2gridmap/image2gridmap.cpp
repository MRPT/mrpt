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

#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>

int main(int argc, char** argv)
{
  try
  {
    CLI::App app("image2gridmap");
    app.set_version_flag("--version", mrpt::system::MRPT_getVersion());

    std::string input_file;
    app.add_option("-i,--input", input_file, "Input image file (required) (*.png,*.jpg,...)")
        ->required();

    std::string output_file;
    app.add_option("-o,--output", output_file, "Name of the output file (*.gridmap, *.gridmap.gz)");

    double res = 0.1;
    app.add_option(
           "-r,--res", res, "Resolution: size (in meters) of one pixel in the image (required)")
        ->required();

    double cx = 0.0;
    auto opt_cx = app.add_option(
        "--cx", cx,
        "(Use either --cx or --px) X coordinate of the image central pixel (Default:0)");

    double cy = 0.0;
    auto opt_cy = app.add_option(
        "--cy", cy,
        "(Use either --cy or --py) Y coordinate of the image central pixel (Default:0)");

    double px = 0.0;
    auto opt_px = app.add_option(
        "--px", px,
        "(Use either --cx or --px) Pixel horizontal coordinate of the origin of coordinates in the "
        "image");

    double py = 0.0;
    auto opt_py = app.add_option(
        "--py", py,
        "(Use either --cx or --px) Pixel verticl coordinate of the origin of coordinates in the "
        "image");

    bool overwrite = false;
    app.add_flag("-w,--overwrite", overwrite, "Force overwrite target file without prompting.");

    printf(" image2gridmap - Part of the MRPT\n");
    printf(
        " MRPT C++ Library: %s - Sources timestamp: %s\n", mrpt::system::MRPT_getVersion().c_str(),
        mrpt::system::MRPT_getCompilationDate().c_str());
    printf(
        "------------------------------------------------------------------"
        "-\n");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    mrpt::img::CImage img;
    if (!img.loadFromFile(input_file))
      throw std::runtime_error(
          mrpt::format("Cannot load the map image file `%s`!", input_file.c_str()));

    // Check if px/py or cx/cy were set
    bool px_set = opt_px->count() > 0;
    bool py_set = opt_py->count() > 0;
    bool cx_set = opt_cx->count() > 0;
    bool cy_set = opt_cy->count() > 0;

    if ((px_set && !py_set) || (!px_set && py_set))
      throw std::runtime_error("You cannot set only one of --px & --py arguments!");

    if ((cx_set && !cy_set) || (!cx_set && cy_set))
      throw std::runtime_error("You cannot set only one of --cx & --cy arguments!");

    if (cx_set && px_set) throw std::runtime_error("You cannot set BOTH --cx & --px arguments!");

    if (!px_set)
    {
      // Use cx, cy instead
      px = -cx / res + img.getWidth() / 2;
      py = -cy / res + img.getHeight() / 2;
    }

    mrpt::maps::COccupancyGridMap2D grid;
    grid.loadFromBitmap(img, res, {px, py});

    const std::string sOutFile =
        !output_file.empty() ? output_file
                             : mrpt::system::fileNameChangeExtension(input_file, "gridmap.gz");
    std::cout << "Output map file: " << sOutFile << "\n";

    if (mrpt::system::fileExists(sOutFile) && !overwrite)
    {
      std::cerr << "Output file already exists, aborting. Use `-w` flag "
                   "to overwrite.\n";
      return 1;
    }

    {
      mrpt::io::CCompressedOutputStream f(sOutFile);
      mrpt::serialization::archiveFrom(f) << grid;
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
