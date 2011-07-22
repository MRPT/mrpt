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

