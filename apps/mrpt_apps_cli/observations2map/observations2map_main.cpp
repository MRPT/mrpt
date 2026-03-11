/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

/*---------------------------------------------------------------
  APPLICATION: observations2map
  FILE: observations2map_main.cpp
  AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

  See README.txt for instructions.
  ---------------------------------------------------------------*/

#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::maps;
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
        " MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(),
        MRPT_getCompilationDate().c_str());
    printf(
        "------------------------------------------------------------------"
        "-\n");

    // Process arguments:
    if ((argc != 4 && argc != 6) || (argc == 6 && 0 != mrpt::system::os::_strcmp(argv[4], "-s")))
    {
      std::cout << "Use: observations2map <config_file.ini> "
                   "<observations.simplemap> <outputmap_prefix> [-s "
                   "INI_FILE_SECTION_NAME] "
                << "\n";
      std::cout << "  Default: INI_FILE_SECTION_NAME = MappingApplication"
                << "\n";
      std::cout << "Push any key to exit..."
                << "\n";
      return 1;
    }

    string configFile = std::string(argv[1]);
    string inputFile = std::string(argv[2]);
    string outprefix = std::string(argv[3]);

    if (argc > 4)
    {
      METRIC_MAP_CONFIG_SECTION = string(argv[5]);
    }

    // Load simplemap:
    std::cout << "Loading simplemap...";
    mrpt::maps::CSimpleMap simplemap;
    mrpt::io::CCompressedInputStream f(inputFile.c_str());
    mrpt::serialization::archiveFrom(f) >> simplemap;
    std::cout << "done: " << simplemap.size() << " observations."
              << "\n";

    // Create metric maps:
    CConfigFile cfg(configFile);

    ASSERT_(cfg.sectionExists(METRIC_MAP_CONFIG_SECTION));

    TSetOfMetricMapInitializers mapCfg;
    mapCfg.loadFromConfigFile(cfg, METRIC_MAP_CONFIG_SECTION);

    CMultiMetricMap metricMap;
    metricMap.setListOfMaps(mapCfg);

    // Build metric maps:
    std::cout << "Building metric maps..."
              << "\n";

    metricMap.loadFromSimpleMap(simplemap);

    std::cout << "done."
              << "\n";

    // Save metric maps:
    // ---------------------------
    metricMap.saveMetricMapRepresentationToFile(outprefix);

    // And as binary serialized files:
    // -------------------------------------
    for (unsigned int i = 0; i < metricMap.maps.size(); i++)
    {
      using namespace std::string_literals;

      const auto& m = metricMap.maps.at(i);

      const auto str = outprefix + mrpt::format("_%02u_", i) +
                       mrpt::system::fileNameStripInvalidChars(m->GetRuntimeClass()->className) +
                       ".bin"s;

      std::cout << "Saving map #" << i << " to " << str << "\n";

      CCompressedOutputStream fo(str);
      mrpt::serialization::archiveFrom(fo) << m;

      std::cout << "Done."
                << "\n";
    }

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n"
              << "Program finished for an exception!!"
              << "\n";
    mrpt::system::pause();
    return 1;
  }
  catch (...)
  {
    std::cerr << "Untyped exception!!"
              << "\n";
    mrpt::system::pause();
    return 1;
  }
}
