/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

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

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/obs/carmen_log_tools.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/serialization/CArchive.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("carmen2simplemap", ' ', MRPT_getVersion().c_str());

TCLAP::SwitchArg arg_overwrite(
	"w", "overwrite", "Force overwrite target file without prompting.", cmd,
	false);
TCLAP::SwitchArg arg_quiet("q", "quiet", "Terse output", cmd, false);
TCLAP::ValueArg<std::string> arg_output_file(
	"o", "output", "Output file (*.simplemap)", true, "", "map.simplemap", cmd);
TCLAP::ValueArg<std::string> arg_input_file(
	"i", "input", "Input dataset (required) (*.log)", true, "", "carmen.log",
	cmd);

TCLAP::ValueArg<int> arg_gz_level(
	"z", "compress-level", "Output GZ-compress level (optional)", false, 5,
	"0: none, 1-9: min-max", cmd);

// Declarations:
#define VERBOSE_COUT \
	if (verbose) cout << "[carmen2simplemap] "

// -----------------------------------------------
//				MAIN
// -----------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		// Parse arguments:
		if (!cmd.parse(argc, argv))
			throw std::runtime_error("");  // should exit.

		const string input_log = arg_input_file.getValue();
		const string output_file = arg_output_file.getValue();
		const bool verbose = !arg_quiet.getValue();
		const bool overwrite = arg_overwrite.getValue();
		const int compress_level = arg_gz_level.getValue();

		// Check files:
		if (!mrpt::system::fileExists(input_log))
			throw runtime_error(
				format("Input file doesn't exist: '%s'", input_log.c_str()));

		if (mrpt::system::fileExists(output_file) && !overwrite)
			throw runtime_error(format(
				"Output file already exist: '%s' (Use --overwrite to "
				"override)",
				output_file.c_str()));

		VERBOSE_COUT << "Input log        : " << input_log << endl;
		VERBOSE_COUT << "Output map file  : " << output_file
					 << " (Compression level: " << compress_level << ")\n";

		// Open I/O streams:
		std::ifstream input_stream(input_log.c_str());
		if (!input_stream.is_open())
			throw runtime_error(
				format("Error opening for read: '%s'", input_log.c_str()));

		// --------------------------------
		// The main loop
		// --------------------------------
		vector<CObservation::Ptr> importedObservations;
		mrpt::maps::CSimpleMap theSimpleMap;
		const mrpt::system::TTimeStamp base_timestamp = mrpt::system::now();

		const uint64_t totalInFileSize = mrpt::system::getFileSize(input_log);
		int decimateUpdateConsole = 0;

		while (carmen_log_parse_line(
			input_stream, importedObservations, base_timestamp))
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
					IS_CLASS(importedObservations[i], CObservationOdometry))
				{
					CObservationOdometry::Ptr odo =
						std::dynamic_pointer_cast<CObservationOdometry>(
							importedObservations[i]);
					gt_pose = odo->odometry;
					has_gt_pose = true;
					break;
				}
			}

			// Only if we have a valid pose, save it to the simple map:
			if (has_gt_pose)
			{
				CSensoryFrame::Ptr SF =
					mrpt::make_aligned_shared<CSensoryFrame>();

				for (const auto& importedObservation : importedObservations)
				{
					if (!IS_CLASS(
							importedObservation,
							CObservationOdometry))  // Odometry was already used
					// as positioning...
					{
						SF->insert(importedObservation);
					}
				}

				// Insert (observations, pose) pair:
				CPosePDFGaussian::Ptr pos =
					mrpt::make_aligned_shared<CPosePDFGaussian>();
				pos->mean = gt_pose;
				theSimpleMap.insert(pos, SF);
			}

			// Update progress in the console:
			// ----------------------------------
			if (verbose && ++decimateUpdateConsole > 10)
			{
				decimateUpdateConsole = 0;

				const std::streampos curPos = input_stream.tellg();
				const double progress_ratio =
					double(curPos) / double(totalInFileSize);
				static const int nBlocksTotal = 50;
				const int nBlocks = progress_ratio * nBlocksTotal;
				cout << "\rProgress: [" << string(nBlocks, '#')
					 << string(nBlocksTotal - nBlocks, ' ')
					 << format(
							"] %6.02f%% (%u frames)", progress_ratio * 100,
							static_cast<unsigned int>(theSimpleMap.size()));
				cout.flush();
			}
		};
		cout << "\n";

		// Save final map object:
		{
			mrpt::io::CFileGZOutputStream out_map;
			if (!out_map.open(output_file, compress_level))
				throw runtime_error(format(
					"Error opening for write: '%s'", output_file.c_str()));

			cout << "Dumping simplemap object to file...";
			cout.flush();
			mrpt::serialization::archiveFrom(out_map) << theSimpleMap;
			cout << "Done\n";
			cout.flush();
		}

		// successful end of program.
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
