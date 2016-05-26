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

#include <mrpt/otherlibs/tclap/CmdLine.h>

#include <string>
#include <cerrno>

#include "GraphSlamEngine.h"

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::utils;

using namespace std;

#define VERBOSE_COUT  if (verbose) std::cout << "[graphslam_engine] "

/** 
 * Forward declaration
 * display_graph function template provided in display_graph.cpp
 */
template <class GRAPHTYPE> void display_graph(const GRAPHTYPE & g);


// /**
//  * Command line options initialization
//  * http://reference.mrpt.org/devel/class_t_c_l_a_p_1_1_cmd_line.html 
//  */
// // TODO - implement this, have the user input either the .ini file or have the
// // demo .ini file used instead
// TCLAP::CmdLine cmd(/*message = */ "GraphslamEngine",
//     /* delimeter = */ ' ', 
//     /* version =  */ MRPT_getVersion().c_str(),
//     /* helpAndVersion = */ true);
// 


/**
 * main
 */
int main(int argc, char **argv)
{
  
  try {
    // Initialize the visualization objects
    CDisplayWindow3D	win("Graphslam building procedure",800, 600);
    win.setCameraElevationDeg(75);

    const string config_fname = "../default_config.ini";

    // Initialize the class
    GraphSlamEngine_t<CNetworkOfPoses2DInf> g_engine(config_fname, 
        &win);
    
    //g_engine.testEdgeCounterObject();

    g_engine.parseLaserScansFile();

    // saving the graph to external file
    g_engine.saveGraph();
    //g_engine.saveGraph("kalimera.txt");

    //display_graph(g_engine.graph);

    // TODO
    // add "keylogger" function to know when to exit..
    while (win.isOpen()) {
      ;;
    }


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
