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
//  Program: gps2rawlog
//  Intention: Parse binary/text GPS logs, recognice all messages and save
//             as a RawLog file, easily readable by MRPT C++ programs.
//
//  Started: JLBC @ Feb-2016
// ===========================================================================

#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::system;
using namespace std;

int main(int argc, char** argv)
{
  try
  {
    printf(" gps2rawlog - Part of the MRPT\n");
    printf(
        " MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(),
        MRPT_getCompilationDate().c_str());

    // Declare the supported command line switches ===========
    CLI::App app("gps2rawlog");
    app.set_version_flag("--version", MRPT_getVersion());

    std::string input_file;
    app.add_option("-i,--input", input_file, "Input raw file (required) (*.raw,*.gps,...)")
        ->required();

    std::string output_file;
    app.add_option("-o,--output", output_file, "Output dataset (*.rawlog)");

    bool overwrite = false;
    app.add_flag("-w,--overwrite", overwrite, "Force overwrite target file without prompting.");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    string output_rawlog_file = output_file;
    if (output_rawlog_file.empty())
      output_rawlog_file = mrpt::system::fileNameChangeExtension(input_file, "rawlog");

    ASSERT_FILE_EXISTS_(input_file);

    // Open input rawlog:
    auto fil_input = std::make_shared<CCompressedInputStream>();
    std::cout << "Opening for reading: '" << input_file << "'...\n";
    fil_input->open(input_file);
    std::cout << "Open OK.\n";

    // Open output:
    if (mrpt::system::fileExists(output_rawlog_file) && !overwrite)
    {
      std::cout << "Output file already exists: `" << output_rawlog_file
                << "`, aborting. Use `-w` flag to overwrite.\n";
      return 1;
    }

    CCompressedOutputStream fil_out;
    std::cout << "Opening for writing: '" << output_rawlog_file << "'...\n";
    if (!fil_out.open(output_rawlog_file)) throw std::runtime_error("Error writing file!");

    // GPS object:
    CGPSInterface gps_if;
    gps_if.bindStream(fil_input);

    auto arch = archiveFrom(fil_out);

    // ------------------------------------
    //  Parse:
    // ------------------------------------
    while (!fil_input->checkEOF())
    {
      gps_if.doProcess();

      const auto lst_obs = gps_if.getObservations();

      printf(
          "%u bytes parsed, %u new observations identified...\n",
          (unsigned)fil_input->getPosition(), (unsigned)lst_obs.size());

      for (const auto& kv : lst_obs) arch << *kv.second;
    }

    // successful end of program.
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}  // end of main()
