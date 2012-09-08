/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */


/*---------------------------------------------------------------
    APPLICATION: Hybrid Metric-Topological SLAM implementation
    FILE: hmt-slam_main.cpp
    AUTHOR: Jose Luis Blanco Claraco <jlblanco@ctima.uma.es>

	See README.txt for instructions.
  ---------------------------------------------------------------*/

#include <mrpt/slam.h>
#include <mrpt/base.h>
#include <mrpt/hmtslam/CHMTSLAM.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
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
		printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str() );
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

	mapping.printf_debug("RAWLOG FILE: \n%s\n",rawlogFileName.c_str());

	const std::string OUT_DIR =  cfgFile.read_string("HMT-SLAM","LOG_OUTPUT_DIR", "HMT_SLAM_OUTPUT");


	ASSERT_FILE_EXISTS_( rawlogFileName )
	CFileGZInputStream	 rawlogFile( rawlogFileName);

	mapping.printf_debug("---------------------------------------------------\n\n");


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

		mapping.printf_debug("======== Rawlog entries processed: %i ========\n", rawlogEntry);

        step++;

	};	// end "while(1)"

	mapping.printf_debug("********* Application finished!! 3 seconds to exit... **********\n");
	sleep(1000);
	mapping.printf_debug("********* Application finished!! 2 seconds to exit... **********\n");
	sleep(1000);
	mapping.printf_debug("********* Application finished!! 1 second to exit... **********\n");
	sleep(1000);

	{
		string final_file = OUT_DIR+string("/final_map.hmtslam");
		mapping.printf_debug("\n Saving FINAL HMT-MAP to file: %s\n",final_file.c_str());
		CFileGZOutputStream	fil(final_file);
		mapping.saveState( fil );
	}
}


