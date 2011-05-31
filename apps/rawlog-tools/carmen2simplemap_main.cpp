/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |

   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

// ===========================================================================
//  Program: carmen2simplemap
//
//  Intention: A converter from CARMEN log text files (WITH ground truth/corrected poses)
//    to "simplemaps" files.
//  See the "--help" output for list of supported operations and further
//   instructions.
//
//  About integration with bash/.BAT scripts: The program will return 0 upon
//   successful execution. Upon error, it will return -1.
//
//  Started: JLBC @ Aug-2010
// ===========================================================================

#include <mrpt/obs.h>
#include <mrpt/base.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("carmen2simplemap", ' ', MRPT_getVersion().c_str());

TCLAP::SwitchArg arg_overwrite("w","overwrite","Force overwrite target file without prompting.",cmd, false);
TCLAP::SwitchArg arg_quiet("q","quiet","Terse output",cmd, false);
TCLAP::ValueArg<std::string> arg_output_file("o","output","Output file (*.simplemap)",true,"","map.simplemap",cmd);
TCLAP::ValueArg<std::string> arg_input_file ("i","input","Input dataset (required) (*.log)",true,"","carmen.log",cmd);

TCLAP::ValueArg<int> arg_gz_level("z","compress-level","Output GZ-compress level (optional)",false,5,"0: none, 1-9: min-max",cmd);

// Declarations:
#define VERBOSE_COUT	if (verbose) cout << "[carmen2simplemap] "


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
		const string output_file 	 	= arg_output_file.getValue();
		const bool verbose 				= !arg_quiet.getValue();
		const bool overwrite 			= arg_overwrite.getValue();
		const int compress_level		= arg_gz_level.getValue();

		// Check files:
		if (!mrpt::system::fileExists(input_log))
			throw runtime_error(format("Input file doesn't exist: '%s'",input_log.c_str()));

		if (mrpt::system::fileExists(output_file) && !overwrite)
			throw runtime_error(format("Output file already exist: '%s' (Use --overwrite to override)",output_file.c_str()));


		VERBOSE_COUT << "Input log        : " << input_log << endl;
		VERBOSE_COUT << "Output map file  : " << output_file << " (Compression level: " << compress_level << ")\n";

		// Open I/O streams:
		std::ifstream   input_stream(input_log.c_str());
		if (!input_stream.is_open())
			throw runtime_error(format("Error opening for read: '%s'",input_log.c_str()));


		// --------------------------------
		// The main loop
		// --------------------------------
		vector<CObservationPtr>  importedObservations;
		mrpt::slam::CSimpleMap  theSimpleMap;
		const mrpt::system::TTimeStamp  base_timestamp = mrpt::system::now();

		const uint64_t totalInFileSize = mrpt::system::getFileSize(input_log);
		int decimateUpdateConsole = 0;

		while ( carmen_log_parse_line(input_stream,importedObservations, base_timestamp) )
		{
			CPose2D  gt_pose;
			bool  has_gt_pose = false;

			for (size_t i=0;i<importedObservations.size();i++)
			{
				// If we have an "odometry" observation but it's not alone, it's probably
				//  a "corrected" odometry from some SLAM program, so save it as ground truth:
				if (importedObservations.size()>1 && IS_CLASS(importedObservations[i], CObservationOdometry) )
				{
					CObservationOdometryPtr odo = CObservationOdometryPtr(importedObservations[i]);
					gt_pose = TPose2D(odo->odometry);
					has_gt_pose = true;
					break;
				}
			}

			// Only if we have a valid pose, save it to the simple map:
			if (has_gt_pose)
			{
				CSensoryFramePtr  SF = CSensoryFrame::Create();

				for (size_t i=0;i<importedObservations.size();i++)
				{
					if (!IS_CLASS(importedObservations[i], CObservationOdometry) )  // Odometry was already used as positioning...
					{
						SF->insert(importedObservations[i]);
					}
				}

				// Insert (observations, pose) pair:
				CPosePDFGaussianPtr pos = CPosePDFGaussian::Create();
				pos->mean = gt_pose;
				theSimpleMap.insert(pos, SF);
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
				cout << "\rProgress: [" << string(nBlocks,'#') << string(nBlocksTotal-nBlocks,' ') << format("] %6.02f%% (%u frames)",progress_ratio*100, static_cast<unsigned int>(theSimpleMap.size()) );
				cout.flush();
			}

		};
		cout << "\n";

		// Save final map object:
		{
			mrpt::utils::CFileGZOutputStream out_map;
			if (!out_map.open(output_file,compress_level))
				throw runtime_error(format("Error opening for write: '%s'",output_file.c_str()));

			cout << "Dumping simplemap object to file..."; cout.flush();
			out_map << theSimpleMap;
			cout << "Done\n";cout.flush();
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

