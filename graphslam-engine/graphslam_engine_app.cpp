/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: Graphsam Engine Application
	FILE: graphslam_engine.cpp

  ---------------------------------------------------------------*/


#include <mrpt/obs.h>
#include <mrpt/graphslam.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // This class is in mrpt-maps

#include "sup_funs.h"

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;

using namespace std;


void runGraphslam(void) {
  MRPT_START

  cout << "In runGraphslam fun: " << endl;


  MRPT_END
}

int main(int argc, char **argv)
{

  const string config_fname = "default_config.ini";

  CConfigFile config_file(config_fname);
  CParams problem_params(config_file);

  try {
    runGraphslam();
    
  }
  catch (exception &e)
  {
    setConsoleColor(CONCOL_RED,true);
    cerr << "Program finished with an exception!" << endl;
    setConsoleColor(CONCOL_NORMAL,true);

    cerr << e.what() << endl;

    mrpt::system::pause();
    return -1;
  }
  catch (...)
  {
    setConsoleColor(CONCOL_RED,true);
    cerr << "Program finished for an untyped exception!!" << endl;
    setConsoleColor(CONCOL_NORMAL,true);

    mrpt::system::pause();
    return -1;
  }
}

