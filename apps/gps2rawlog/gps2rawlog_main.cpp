/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// ===========================================================================
//  Program: gps2rawlog
//  Intention: Parse binary/text GPS logs, recognice all messages and save 
//             as a RawLog file, easily readable by MRPT C++ programs.
//
//  Started: JLBC @ Feb-2016
// ===========================================================================

#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CFileGZInputStream.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace std;

// Declare the supported command line switches ===========
TCLAP::CmdLine cmd("gps2rawlog", ' ', MRPT_getVersion().c_str());

TCLAP::ValueArg<std::string> arg_input_file ("i","input","Input raw file (required) (*.raw,*.gps,...)",true,"","log.gps",cmd);
TCLAP::ValueArg<std::string> arg_output_file("o","output","Output dataset (*.rawlog)",false,"","dataset_out.rawlog",cmd);

TCLAP::SwitchArg arg_overwrite("w","overwrite","Force overwrite target file without prompting.",cmd, false);


int main(int argc, char **argv)
{
	try
	{
		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			throw std::runtime_error(""); // should exit.

		const string input_gps_file  = arg_input_file.getValue();
		const string output_rawlog_file  = arg_output_file.getValue();

		ASSERT_FILE_EXISTS_(input_gps_file)

		// Open input rawlog:
		CFileGZInputStream  fil_input;
		cout << "Opening '" << input_gps_file << "'...\n";
		fil_input.open(input_gps_file);
		cout << "Open OK.\n";

		// GPS object:
		CGPSInterface  gps_if;
		gps_if.bindStream(&fil_input);
		
		// ------------------------------------
		//  Parse:
		// ------------------------------------
		while (1)
		{
			gps_if.doProcess();

			CGenericSensor::TListObservations lst_obs;
			gps_if.getObservations(lst_obs);

			cout << "Read obs: " << lst_obs.size() << endl;
		}

		// successful end of program.
		return 0;
	}
	catch(std::exception &e)
	{
		if (strlen(e.what())) std::cerr << e.what() << std::endl;
		return 1;
	}
} // end of main()

