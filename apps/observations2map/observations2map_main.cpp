/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
    APPLICATION: observations2map
    FILE: observations2map_main.cpp
    AUTHOR: Jose Luis Blanco Claraco <jlblanco@ctima.uma.es>

	See README.txt for instructions.
  ---------------------------------------------------------------*/

#include <mrpt/base.h>
#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;


string METRIC_MAP_CONFIG_SECTION  =  "MappingApplication";

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" observations2map - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if ((argc!=4 && argc!=6) || (argc==6 && 0!=mrpt::system::os::_strcmp(argv[4],"-s") ) )
		{
			cout << "Use: observations2map <config_file.ini> <observations.simplemap> <outputmap_prefix> [-s INI_FILE_SECTION_NAME] " << endl;
			cout << "  Default: INI_FILE_SECTION_NAME = MappingApplication" << endl;
			cout << "Push any key to exit..." << endl;
			os::getch();
			return -1;
		}

		string configFile = std::string( argv[1] );
		string inputFile = std::string( argv[2] );
		string outprefix = std::string( argv[3] );

		if (argc>4)
		{
			METRIC_MAP_CONFIG_SECTION = string(argv[5]);
		}


		// Load simplemap:
		cout << "Loading simplemap...";
		mrpt::slam::CSimpleMap	simplemap;
		CFileGZInputStream f( inputFile.c_str() );
		f >> simplemap;
		cout <<"done: " << simplemap.size() << " observations." << endl;

		// Create metric maps:
		TSetOfMetricMapInitializers		mapCfg;
		mapCfg.loadFromConfigFile( CConfigFile( configFile ), METRIC_MAP_CONFIG_SECTION );

		CMultiMetricMap		metricMap;
		metricMap.setListOfMaps( &mapCfg );

		// Build metric maps:
		cout << "Building metric maps...";

		metricMap.loadFromProbabilisticPosesAndObservations( simplemap );

		cout << "done." << endl;


		// Save metric maps:
		// ---------------------------
		metricMap.saveMetricMapRepresentationToFile( outprefix );

		// grid maps:
		size_t i;
		for (i=0;i<metricMap.m_gridMaps.size();i++)
		{
			string str = format( "%s_gridmap_no%02u.gridmap", outprefix.c_str(), (unsigned)i );
			cout << "Saving gridmap #" << i << " to " << str << endl;

			CFileGZOutputStream f(str);
			f << *metricMap.m_gridMaps[i];

			cout << "done." << endl;
		}

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

