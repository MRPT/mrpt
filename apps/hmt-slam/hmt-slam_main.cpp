/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


/*---------------------------------------------------------------
    APPLICATION: Hybrid Metric-Topological SLAM implementation
    FILE: hmt-slam_main.cpp
    AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	See README.txt for instructions.
  ---------------------------------------------------------------*/

#include <mrpt/hmtslam/CHMTSLAM.h>
#include <mrpt/utils/CConsoleRedirector.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::hmtslam;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

std::string		configFile;

#define  STEPS_BETWEEN_WAITING_FOR_QUEUE_EMPTY   10

void Run_HMT_SLAM();

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		CConsoleRedirector	dbg_out_file("./DEBUG_log_streaming.txt",true, true,false,0 );

		printf(" HMT-SLAM version 0.2 - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str() );
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2)
		{
			printf("Use: hmt-slam <config_file>\n\nPush any key to exit...\n");
			os::getch();
			return -1;
		}

		configFile = std::string( argv[1] );

		Run_HMT_SLAM();

		return 0;
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl << "Program finished for an exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		std::cerr << "Untyped exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
}

// ------------------------------------------------------
//				TestMapping
// ------------------------------------------------------
void Run_HMT_SLAM()
{
	CHMTSLAM			mapping;

	CConfigFile			cfgFile( configFile );
	std::string			rawlogFileName;

	// wait for threads init. (Just to do not mix debug strings on console)
	sleep(100);

	// The rawlog file:
	// ----------------------------------------
	rawlogFileName = cfgFile.read_string("HMT-SLAM","rawlog_file",std::string("log.rawlog"));
	unsigned int	rawlog_offset = cfgFile.read_int("HMT-SLAM","rawlog_offset",0);

	mapping.logFmt(mrpt::utils::LVL_INFO, "RAWLOG FILE: \n%s\n",rawlogFileName.c_str());

	const std::string OUT_DIR =  cfgFile.read_string("HMT-SLAM","LOG_OUTPUT_DIR", "HMT_SLAM_OUTPUT");


	ASSERT_FILE_EXISTS_( rawlogFileName )
	CFileGZInputStream	 rawlogFile( rawlogFileName);

	mapping.logFmt(mrpt::utils::LVL_INFO, "---------------------------------------------------\n\n");


	// Set relative path for externally-stored images in rawlogs:
	string	rawlog_images_path = extractFileDirectory( rawlogFileName );
	rawlog_images_path+="/Images";
	CImage::IMAGES_PATH_BASE = rawlog_images_path;		// Set it.


	// Load the config options for mapping:
	// ----------------------------------------
	mapping.loadOptions( configFile );



	//			INITIALIZATION
	// ----------------------------------------
	//utils::CRandomGenerator::Randomize( );	// Not necesary (called inside the classes)
	mapping.initializeEmptyMap();

	// The main loop:
	// ---------------------------------------
	unsigned int			rawlogEntry = 0, step = 0;
	bool					finish=false;

	while  (!finish && !mapping.abortedDueToErrors() )
	{
		if (os::kbhit())
        {
			char	pushKey = os::getch();
			finish = 27 == pushKey;
        }

		// Load next object from the rawlog:
		// ----------------------------------------
		CSerializablePtr objFromRawlog;
		try
		{
			rawlogFile >> objFromRawlog;
			rawlogEntry++;
		}
		catch(std::exception &) { break; }
		catch(...) { printf("Untyped exception reading rawlog file!!\n");break;}

		if (rawlogEntry>=rawlog_offset)
		{

			// Process the action and observations:
			// --------------------------------------------
			if (IS_CLASS(objFromRawlog,CActionCollection))
			{
				mapping.pushAction( CActionCollectionPtr( objFromRawlog) ); // Memory will be freed in mapping class
			}
			else if (IS_CLASS(objFromRawlog,CSensoryFrame))
			{
				mapping.pushObservations( CSensoryFramePtr( objFromRawlog) ); // Memory will be freed in mapping class
			}
			else if (IS_CLASS(objFromRawlog,CObservation))
			{
				mapping.pushObservation( CObservationPtr( objFromRawlog) ); // Memory will be freed in mapping class
			}
			else THROW_EXCEPTION("Invalid object class from rawlog!!")


			// Wait for the mapping framework processed the data
			// ---------------------------------------------------
			if ((rawlogEntry % STEPS_BETWEEN_WAITING_FOR_QUEUE_EMPTY)==0)
				while (!mapping.isInputQueueEmpty() && !os::kbhit() && !mapping.abortedDueToErrors() )
				{
					sleep(2);
				}
		} // (rawlogEntry>=rawlog_offset)

		mapping.logFmt(mrpt::utils::LVL_INFO, "======== Rawlog entries processed: %i ========\n", rawlogEntry);

        step++;

	};	// end "while(1)"

	mapping.logFmt(mrpt::utils::LVL_INFO, "********* Application finished!! 3 seconds to exit... **********\n");
	sleep(1000);
	mapping.logFmt(mrpt::utils::LVL_INFO, "********* Application finished!! 2 seconds to exit... **********\n");
	sleep(1000);
	mapping.logFmt(mrpt::utils::LVL_INFO, "********* Application finished!! 1 second to exit... **********\n");
	sleep(1000);

	{
		string final_file = OUT_DIR+string("/final_map.hmtslam");
		mapping.logFmt(mrpt::utils::LVL_WARN, "\n Saving FINAL HMT-MAP to file: %s\n",final_file.c_str());
		CFileGZOutputStream	fil(final_file);
		mapping.saveState( fil );
	}
}


