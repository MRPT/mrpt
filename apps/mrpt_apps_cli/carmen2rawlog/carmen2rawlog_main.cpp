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
//  Program: carmen2rawlog
//
//  Intention: A converter from CARMEN log text files to binary Rawlog files
//  See the "--help" output for list of supported operations and further
//   instructions.
//
//  About integration with bash/.BAT scripts: The program will return 0 upon
//   successful execution. Upon error, it will return -1.
//
//  Started: JLBC @ Aug-2010
// ===========================================================================

#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/carmen_log_tools.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <fstream>
#include <map>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

// Declarations:
#define VERBOSE_COUT \
  if (verbose) cout << "[carmen2rawlog] "

// -----------------------------------------------
//				MAIN
// -----------------------------------------------
int main(int argc, char** argv)
{
  try
  {
    // Declare the supported command line switches ===========
    CLI::App app("carmen2rawlog");

    app.set_version_flag("--version", MRPT_getVersion());

    bool arg_overwrite = false;
    app.add_flag("-w,--overwrite", arg_overwrite, "Force overwrite target file without prompting.");

    bool arg_quiet = false;
    app.add_flag("-q,--quiet", arg_quiet, "Terse output");

    std::string arg_output_file;
    app.add_option("-o,--output", arg_output_file, "Output dataset (*.rawlog)")->required();

    std::string arg_input_file;
    app.add_option("-i,--input", arg_input_file, "Input dataset (required) (*.log)")->required();

    int arg_gz_level = 5;
    app.add_option(
        "-z,--compress-level", arg_gz_level, "Output GZ-compress level (0: none, 1-9: min-max)");

    double arg_obs_period = 0.1;
    app.add_option(
        "-p,--period", arg_obs_period, "Observation recording period in seconds (optional)");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    const string input_log = arg_input_file;
    const string output_rawlog = arg_output_file;
    const bool verbose = !arg_quiet;
    const bool overwrite = arg_overwrite;
    const int compress_level = arg_gz_level;

    // Check files:
    if (!mrpt::system::fileExists(input_log))
    {
      throw runtime_error(format("Input file doesn't exist: '%s'", input_log.c_str()));
    }

    if (mrpt::system::fileExists(output_rawlog) && !overwrite)
    {
      throw runtime_error(format(
          "Output file already exist: '%s' (Use --overwrite to "
          "override)",
          output_rawlog.c_str()));
    }

    VERBOSE_COUT << "Input log        : " << input_log << "\n";
    VERBOSE_COUT << "Output rawlog    : " << output_rawlog
                 << " (Compression level: " << compress_level << ")\n";

    // Open I/O streams:
    std::ifstream input_stream(input_log.c_str());
    if (!input_stream.is_open())
    {
      throw runtime_error(format("Error opening for read: '%s'", input_log.c_str()));
    }

    mrpt::io::CCompressedOutputStream out_rawlog;
    if (!out_rawlog.open(output_rawlog, {mrpt::io::CompressionType::Zstd, compress_level}))
    {
      throw runtime_error(format("Error opening for write: '%s'", output_rawlog.c_str()));
    }

    // --------------------------------
    // The main loop
    // --------------------------------
    vector<CObservation::Ptr> importedObservations;
    map<TTimeStamp, TPose2D> groundTruthPoses;  // If found...
    unsigned int nSavedObs = 0;
    const double dt = arg_obs_period;
    double tim = mrpt::Clock::nowDouble();

    const mrpt::system::TTimeStamp base_timestamp = mrpt::Clock::now();

    const uint64_t totalInFileSize = mrpt::system::getFileSize(input_log);
    int decimateUpdateConsole = 0;

    while (carmen_log_parse_line(input_stream, importedObservations, base_timestamp))
    {
      // fix timestamps. Carmen logs did not store timing information:
      for (auto& o : importedObservations)
      {
        o->timestamp = mrpt::Clock::fromDouble(tim);
      }

      tim += dt;  // for the next carmen line

      // save them:
      for (size_t i = 0; i < importedObservations.size(); i++)
      {
        mrpt::serialization::archiveFrom(out_rawlog) << *importedObservations[i];
        nSavedObs++;

        // by the way: if we have an "odometry" observation but it's not
        // alone, it's probably
        //  a "corrected" odometry from some SLAM program, so save it as
        //  ground truth:
        if (importedObservations.size() > 1 &&
            IS_CLASS(*importedObservations[i], CObservationOdometry))
        {
          CObservationOdometry::Ptr odo =
              std::dynamic_pointer_cast<CObservationOdometry>(importedObservations[i]);
          groundTruthPoses[odo->timestamp] = odo->odometry.asTPose();
        }
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
        cout << "\rProgress: [" << string(nBlocks, '#') << string(nBlocksTotal - nBlocks, ' ')
             << format("] %6.02f%% (%u objects)", progress_ratio * 100, nSavedObs);
        cout.flush();
      }
    };
    cout << "\n";

    // If we had ground-truth robot poses, save to file:
    if (!groundTruthPoses.empty())
    {
      const string gt_filename = mrpt::system::fileNameChangeExtension(output_rawlog, "gt.txt");
      cout << "Note: Saving ground truth pose information to '" << gt_filename << "'\n";

      std::ofstream gt_file;
      gt_file.open(gt_filename.c_str());
      if (!gt_file.is_open())
      {
        throw std::runtime_error(
            format("Couldn't open output file for ground truth: '%s'", gt_filename.c_str()));
      }

      gt_file << "%          Ground truth positioning data \n"
                 "%  Timestamp (sec)       x (m)    y (m)    phi (rad)  \n"
                 "% ----------------------------------------------------\n";

      for (auto it = groundTruthPoses.begin(); it != groundTruthPoses.end(); ++it)
      {
        gt_file << format(
            "   %12.06f %9.03f %9.03f %9.04f\n", mrpt::Clock::toDouble(it->first), it->second.x,
            it->second.y, it->second.phi);
      }
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
