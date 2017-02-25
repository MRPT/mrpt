/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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

#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/obs/carmen_log_tools.h>
#include <mrpt/obs/CObservationOdometry.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>

#include <fstream>
#include <map>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("carmen2rawlog", ' ', MRPT_getVersion().c_str());

TCLAP::SwitchArg arg_overwrite("w","overwrite","Force overwrite target file without prompting.",cmd, false);
TCLAP::SwitchArg arg_quiet("q","quiet","Terse output",cmd, false);
TCLAP::ValueArg<std::string> arg_output_file("o","output","Output dataset (*.rawlog)",true,"","dataset_out.rawlog",cmd);
TCLAP::ValueArg<std::string> arg_input_file ("i","input","Input dataset (required) (*.log)",true,"","carmen.log",cmd);

TCLAP::ValueArg<int> arg_gz_level("z","compress-level","Output GZ-compress level (optional)",false,5,"0: none, 1-9: min-max",cmd);

// Declarations:
#define VERBOSE_COUT	if (verbose) cout << "[carmen2rawlog] "


// -----------------------------------------------
//				MAIN
// -----------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			throw std::runtime_error(""); // should exit.

		const string input_log  		= arg_input_file.getValue();
		const string output_rawlog  	= arg_output_file.getValue();
		const bool verbose 				= !arg_quiet.getValue();
		const bool overwrite 			= arg_overwrite.getValue();
		const int compress_level		= arg_gz_level.getValue();

		// Check files:
		if (!mrpt::system::fileExists(input_log))
			throw runtime_error(format("Input file doesn't exist: '%s'",input_log.c_str()));

		if (mrpt::system::fileExists(output_rawlog) && !overwrite)
			throw runtime_error(format("Output file already exist: '%s' (Use --overwrite to override)",output_rawlog.c_str()));


		VERBOSE_COUT << "Input log        : " << input_log << endl;
		VERBOSE_COUT << "Output rawlog    : " << output_rawlog << " (Compression level: " << compress_level << ")\n";

		// Open I/O streams:
		std::ifstream   input_stream(input_log.c_str());
		if (!input_stream.is_open())
			throw runtime_error(format("Error opening for read: '%s'",input_log.c_str()));

		mrpt::utils::CFileGZOutputStream out_rawlog;
		if (!out_rawlog.open(output_rawlog,compress_level))
			throw runtime_error(format("Error opening for write: '%s'",output_rawlog.c_str()));



		// --------------------------------
		// The main loop
		// --------------------------------
		vector<CObservationPtr>  importedObservations;
		map<TTimeStamp,TPose2D>  groundTruthPoses;  // If found...
		unsigned int  nSavedObs = 0;

		const mrpt::system::TTimeStamp  base_timestamp = mrpt::system::now();

		const uint64_t totalInFileSize = mrpt::system::getFileSize(input_log);
		int decimateUpdateConsole = 0;

		while ( carmen_log_parse_line(input_stream,importedObservations, base_timestamp) )
		{
			for (size_t i=0;i<importedObservations.size();i++)
			{
				out_rawlog << *importedObservations[i];
				nSavedObs++;

				// by the way: if we have an "odometry" observation but it's not alone, it's probably
				//  a "corrected" odometry from some SLAM program, so save it as ground truth:
				if (importedObservations.size()>1 && IS_CLASS(importedObservations[i], CObservationOdometry) )
				{
					CObservationOdometryPtr odo = CObservationOdometryPtr(importedObservations[i]);
					groundTruthPoses[odo->timestamp] = TPose2D(odo->odometry);
				}
			}

			// Update progress in the console:
			// ----------------------------------
			if (verbose && ++decimateUpdateConsole>10)
			{
				decimateUpdateConsole = 0;

				const std::streampos curPos = input_stream.tellg();
				const double progress_ratio =  double(curPos)/double(totalInFileSize);
				static const int nBlocksTotal = 50;
				const int nBlocks = progress_ratio * nBlocksTotal;
				cout << "\rProgress: [" << string(nBlocks,'#') << string(nBlocksTotal-nBlocks,' ') << format("] %6.02f%% (%u objects)",progress_ratio*100, nSavedObs  );
				cout.flush();
			}

		};
		cout << "\n";

		// If we had ground-truth robot poses, save to file:
		if (!groundTruthPoses.empty())
		{
			const string gt_filename = mrpt::system::fileNameChangeExtension(output_rawlog,"gt.txt");
			cout << "Note: Saving ground truth pose information to '"<< gt_filename <<"'\n";

			std::ofstream gt_file;
			gt_file.open(gt_filename.c_str());
			if (!gt_file.is_open())
				throw std::runtime_error(format("Couldn't open output file for ground truth: '%s'",gt_filename.c_str()));
			gt_file << "%          Ground truth positioning data \n"
					   "%  Timestamp (sec)       x (m)    y (m)    phi (rad)  \n"
			           "% ----------------------------------------------------\n";
			for (map<TTimeStamp,TPose2D>::const_iterator it=groundTruthPoses.begin();it!=groundTruthPoses.end();++it)
				gt_file << format("   %12.06f %9.03f %9.03f %9.04f\n",
									mrpt::system::timestampToDouble(it->first),
									it->second.x,
									it->second.y,
									it->second.phi);
		}


		// successful end of program.
		return 0;
	}
	catch(std::exception &e)
	{
		if (strlen(e.what())) std::cerr << e.what() << std::endl;
		return -1;
	}
}

