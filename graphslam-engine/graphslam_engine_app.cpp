/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/obs/CRawlog.h>
#include <mrpt/graphslam.h>
#include <mrpt/graphs.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/opengl/CPlanarLaserScan.h> 
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

#include <string>
#include <cerrno>

#include "GraphSlamEngine.h"

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;

using namespace std;


int main(int argc, char **argv)
{
  
  try {
    const string laser_scans_fname = "../dataset_malaga20060121/20060121-Teleco_Faculty_Malaga_laser_only.rawlog";
    const string config_fname = "../default_config.ini";
    GraphSlamEngine_t<CNetworkOfPoses2DInf> g_engine;
    g_engine.parseLaserScansFile(laser_scans_fname);

    g_engine.readConfigFile(config_fname);
    cout << "Config file read." << endl;

    g_engine.printProblemParams();
    cout << "Config file printed." << endl;
    


  }
  catch (exception& e) {
    setConsoleColor(CONCOL_RED, true);
    cerr << "Program finished with an exception!" << endl;
    setConsoleColor(CONCOL_NORMAL, true);

    cerr << e.what() << endl;

    mrpt::system::pause();
    return -1;
  }
  catch (...) {
    setConsoleColor(CONCOL_RED, true);
    cerr << "Program finished for an untyped exception!!" << endl;
    setConsoleColor(CONCOL_NORMAL, true);

    mrpt::system::pause();
    return -1;
  }

  return 0;
}
