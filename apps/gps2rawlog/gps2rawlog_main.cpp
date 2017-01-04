/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>

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
		printf(" gps2rawlog - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());

		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			throw std::runtime_error(""); // should exit.

		const string input_gps_file  = arg_input_file.getValue();
		string output_rawlog_file = arg_output_file.getValue();
		if (output_rawlog_file.empty())
			output_rawlog_file = mrpt::system::fileNameChangeExtension(input_gps_file,"rawlog");

		ASSERT_FILE_EXISTS_(input_gps_file)

		// Open input rawlog:
		CFileGZInputStream  fil_input;
		cout << "Opening for reading: '" << input_gps_file << "'...\n";
		fil_input.open(input_gps_file);
		cout << "Open OK.\n";

		// Open output:
		if (mrpt::system::fileExists(output_rawlog_file) && !arg_overwrite.isSet())
		{
			cout << "Output file already exists: `"<<output_rawlog_file<<"`, aborting. Use `-w` flag to overwrite.\n";
			return 1;
		}

		CFileGZOutputStream fil_out;
		cout << "Opening for writing: '" << output_rawlog_file << "'...\n";
		if (!fil_out.open(output_rawlog_file))
			throw std::runtime_error("Error writing file!");

		// GPS object:
		CGPSInterface  gps_if;
		gps_if.bindStream(&fil_input);
		
		// ------------------------------------
		//  Parse:
		// ------------------------------------
		while ( !fil_input.checkEOF() )
		{
			gps_if.doProcess();

			CGenericSensor::TListObservations lst_obs;
			gps_if.getObservations(lst_obs);

			printf("%u bytes parsed, %u new observations identified...\n",(unsigned)fil_input.getPosition(),(unsigned)lst_obs.size());
			for (CGenericSensor::TListObservations::const_iterator it=lst_obs.begin();it!=lst_obs.end();++it) {
				fil_out << *it->second;
			}
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

