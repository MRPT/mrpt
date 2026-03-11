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

// ===========================================================================
//  Program: carmen2simplemap
//
//  Intention: A converter from CARMEN log text files (WITH ground
//  truth/corrected poses)
//    to "simplemaps" files.
//  See the "--help" output for list of supported operations and further
//   instructions.
//
//  About integration with bash/.BAT scripts: The program will return 0 upon
//   successful execution. Upon error, it will return -1.
//
//  Started: JLBC @ Aug-2010
// ===========================================================================

#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/carmen_log_tools.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <fstream>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

// Declarations:
#define VERBOSE_COUT \
  if (verbose) std::cout << "[carmen2simplemap] "

// -----------------------------------------------
//				MAIN
// -----------------------------------------------
int main(int argc, char** argv)
{
  try
  {
    // Declare the supported command line switches ===========
    CLI::App app("carmen2simplemap");
    app.set_version_flag("--version", MRPT_getVersion());

    bool arg_overwrite = false;
    app.add_flag("-w,--overwrite", arg_overwrite, "Force overwrite target file without prompting.");

    bool arg_quiet = false;
    app.add_flag("-q,--quiet", arg_quiet, "Terse output");

    std::string arg_output_file;
    app.add_option("-o,--output", arg_output_file, "Output file (*.simplemap)")->required();

    std::string arg_input_file;
    app.add_option("-i,--input", arg_input_file, "Input dataset (required) (*.log)")->required();

    int arg_gz_level = 5;
    app.add_option(
        "-z,--compress-level", arg_gz_level, "Output GZ-compress level (0: none, 1-9: min-max)");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    const string input_log = arg_input_file;
    const string output_file = arg_output_file;
    const bool verbose = !arg_quiet;
    const bool overwrite = arg_overwrite;
    const int compress_level = arg_gz_level;

    // Check files:
    if (!mrpt::system::fileExists(input_log))
      throw runtime_error(format("Input file doesn't exist: '%s'", input_log.c_str()));

    if (mrpt::system::fileExists(output_file) && !overwrite)
      throw runtime_error(format(
          "Output file already exist: '%s' (Use --overwrite to "
          "override)",
          output_file.c_str()));

    VERBOSE_COUT << "Input log        : " << input_log << "\n";
    VERBOSE_COUT << "Output map file  : " << output_file
                 << " (Compression level: " << compress_level << ")\n";

    // Open I/O streams:
    std::ifstream input_stream(input_log.c_str());
    if (!input_stream.is_open())
      throw runtime_error(format("Error opening for read: '%s'", input_log.c_str()));

    // --------------------------------
    // The main loop
    // --------------------------------
    vector<CObservation::Ptr> importedObservations;
    mrpt::maps::CSimpleMap theSimpleMap;
    const mrpt::system::TTimeStamp base_timestamp = mrpt::Clock::now();

    const uint64_t totalInFileSize = mrpt::system::getFileSize(input_log);
    int decimateUpdateConsole = 0;

    while (carmen_log_parse_line(input_stream, importedObservations, base_timestamp))
    {
      CPose2D gt_pose;
      bool has_gt_pose = false;

      for (size_t i = 0; i < importedObservations.size(); i++)
      {
        // If we have an "odometry" observation but it's not alone, it's
        // probably
        //  a "corrected" odometry from some SLAM program, so save it as
        //  ground truth:
        if (importedObservations.size() > 1 &&
            IS_CLASS(*importedObservations[i], CObservationOdometry))
        {
          CObservationOdometry::Ptr odo =
              std::dynamic_pointer_cast<CObservationOdometry>(importedObservations[i]);
          gt_pose = odo->odometry;
          has_gt_pose = true;
          break;
        }
      }

      // Only if we have a valid pose, save it to the simple map:
      if (has_gt_pose)
      {
        CSensoryFrame::Ptr SF = std::make_shared<CSensoryFrame>();

        for (const auto& importedObservation : importedObservations)
        {
          if (!IS_CLASS(*importedObservation,
                        CObservationOdometry))  // Odometry was already used
          // as positioning...
          {
            SF->insert(importedObservation);
          }
        }

        // Insert (observations, pose) pair:
        auto pos = CPose3DPDFGaussian::Create();
        pos->mean = mrpt::poses::CPose3D(gt_pose);
        theSimpleMap.insert(pos, SF);
      }

      // Update progress in the console:
      // ----------------------------------
      if (verbose && ++decimateUpdateConsole > 10)
      {
        decimateUpdateConsole = 0;

        const std::streampos curPos = input_stream.tellg();
        const double progress_ratio = double(curPos) / double(totalInFileSize);
        static const int nBlocksTotal = 50;
        const int nBlocks = progress_ratio * nBlocksTotal;
        std::cout << "\rProgress: [" << string(nBlocks, '#') << string(nBlocksTotal - nBlocks, ' ')
                  << format(
                         "] %6.02f%% (%u frames)", progress_ratio * 100,
                         static_cast<unsigned int>(theSimpleMap.size()));
        cout.flush();
      }
    };
    std::cout << "\n";

    // Save final map object:
    {
      mrpt::io::CCompressedOutputStream out_map;
      if (!out_map.open(output_file, {mrpt::io::CompressionType::Zstd, compress_level}))
        throw runtime_error(format("Error opening for write: '%s'", output_file.c_str()));

      std::cout << "Dumping simplemap object to file...";
      cout.flush();
      mrpt::serialization::archiveFrom(out_map) << theSimpleMap;
      std::cout << "Done\n";
      cout.flush();
    }

    // successful end of program.
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
