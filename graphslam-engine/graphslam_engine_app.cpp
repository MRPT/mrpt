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

#define VERBOSE_COUT  if (verbose) std::cout << "[graphslam_engine] "

int main(int argc, char **argv)
{
  
  try {
    // Initialize the class
    GraphSlamEngine_t<CNetworkOfPoses2DInf> g_engine;

    // introductory actions
    const string config_fname = "../default_config.ini";
    g_engine.readConfigFile(config_fname);
    g_engine.printProblemParams();
    g_engine.initOutputDir();

    // read the scans file - build the fixed-intervals CNetworkOfPoses graph
    // TODO
    g_engine.parseLaserScansFile();


    //// make sure that we have added the nodes from odometry
    //// and print the nodes
    //std::set <TNodeID> lst_node_ids;
    //g_engine.graph.getAllNodes(lst_node_ids);
    //for (set<TNodeID>::iterator node_it = lst_node_ids.begin();
        //node_it != lst_node_ids.end(); ++node_it) {
      //cout << "id: " <<  *node_it << endl;
    //}
    VERBOSE_COUT << "Total num of nodes: " << g_engine.graph.nodeCount() << endl;

    // saving the graph to external file
    g_engine.saveGraph();
    //g_engine.saveGraph("kalimera.txt");



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
