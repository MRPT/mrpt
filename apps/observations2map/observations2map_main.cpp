/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: observations2map
	FILE: observations2map_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	See README.txt for instructions.
  ---------------------------------------------------------------*/

#include <mrpt/system/os.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::io;
using namespace mrpt::system;
using namespace mrpt::config;
using namespace mrpt::math;
using namespace std;

string METRIC_MAP_CONFIG_SECTION = "MappingApplication";

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		printf(" observations2map - Part of the MRPT\n");
		printf(
			" MRPT C++ Library: %s - Sources timestamp: %s\n",
			MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf(
			"------------------------------------------------------------------"
			"-\n");

		// Process arguments:
		if ((argc != 4 && argc != 6) ||
			(argc == 6 && 0 != mrpt::system::os::_strcmp(argv[4], "-s")))
		{
			cout << "Use: observations2map <config_file.ini> "
					"<observations.simplemap> <outputmap_prefix> [-s "
					"INI_FILE_SECTION_NAME] "
				 << endl;
			cout << "  Default: INI_FILE_SECTION_NAME = MappingApplication"
				 << endl;
			cout << "Push any key to exit..." << endl;
			os::getch();
			return -1;
		}

		string configFile = std::string(argv[1]);
		string inputFile = std::string(argv[2]);
		string outprefix = std::string(argv[3]);

		if (argc > 4)
		{
			METRIC_MAP_CONFIG_SECTION = string(argv[5]);
		}

		// Load simplemap:
		cout << "Loading simplemap...";
		mrpt::maps::CSimpleMap simplemap;
		mrpt::io::CFileGZInputStream f(inputFile.c_str());
		mrpt::serialization::archiveFrom(f) >> simplemap;
		cout << "done: " << simplemap.size() << " observations." << endl;

		// Create metric maps:
		TSetOfMetricMapInitializers mapCfg;
		mapCfg.loadFromConfigFile(
			CConfigFile(configFile), METRIC_MAP_CONFIG_SECTION);

		CMultiMetricMap metricMap;
		metricMap.setListOfMaps(mapCfg);

		// Build metric maps:
		cout << "Building metric maps...";

		metricMap.loadFromProbabilisticPosesAndObservations(simplemap);

		cout << "done." << endl;

		// Save metric maps:
		// ---------------------------
		metricMap.saveMetricMapRepresentationToFile(outprefix);

		// grid maps:
		for (unsigned int i = 0;
			 i < metricMap.countMapsByClass<COccupancyGridMap2D>(); i++)
		{
			using namespace std::string_literals;
			const auto str = outprefix + "_gridmap_no"s +
							 mrpt::format("%02u", i) + ".gridmap"s;
			cout << "Saving gridmap #" << i << " to " << str << endl;

			CFileGZOutputStream fo(str);
			mrpt::serialization::archiveFrom(fo)
				<< *metricMap.mapByClass<COccupancyGridMap2D>(i);

			cout << "done." << endl;
		}

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl
				  << "Program finished for an exception!!" << std::endl;
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
