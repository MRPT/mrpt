/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	EXAMPLE: Experimentation file
	FILE: experimental.cpp

  ---------------------------------------------------------------*/


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

#define VERBOSE_COUT  if (verbose) std::cout << "[graphslam_engine] "

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;

using namespace std;

// Use compact representation for storing parameters
/**
 * Configuration parameters
 * Use a compact form of storage CConfigFile? for storing these
 */
bool verbose = true;


CPoses2DSequence seq;

int main(int argc, char **argv)
{

  try {
    // Goal 1: 
    // Open the rawlog file and read out its measurements
    // Goal 2:
    // After some threshold values insert poses into the graph
    
    string rawlog_fname;
    //rawlog_fname = "localization_demo.rawlog";
    rawlog_fname = "dataset_malaga20060121/20060121-Teleco_Faculty_Malaga_laser_only.rawlog";

    CFileGZInputStream rawlog_file(rawlog_fname);
    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr observation;
    size_t curr_rawlog_entry = 0;
    bool end = false;


    // Read from the rawlog
    while (CRawlog::getActionObservationPairOrObservation(
          rawlog_file,
          action,               // Possible out var: Action of a a pair action / obs
          observations,         // Possible out var: obs's of a pair actin     / obs
          observation,           // Possible out var
          curr_rawlog_entry ) ) // IO counter
    {
      // process action & observations
      if (observation)
      {
        // Read a single observation from the rawlog (Format #2 rawlog file)
        VERBOSE_COUT << "Foramt #2: Found observation." << endl;
        cout << "--------------------------------------------------" << endl;
        //TODO Implement 2nd format
      }
      else 
      {
        // action, observations should contain a pair of valid data 
        // (Format #1 rawlog file)
        VERBOSE_COUT << "Format #1: Found pair of action & observation" << endl;
        cout << "-----------------------------------------------------------------" << endl;

        CActionRobotMovement2DPtr robotMovement2D = action->getBestMovementEstimation();
        CPose2D a = robotMovement2D->poseChange->getMeanVal();
        cout << "a = " << a << endl;

      }
    }
    //cout << "Final robot pose by odometry: " << seq.absolutePoseAfterAll() << endl;

  }
  catch (exception &e)
  {
    setConsoleColor(CONCOL_RED, true);
    cerr << "Program finished with an exception!" << endl;
    setConsoleColor(CONCOL_NORMAL, true);

    cerr << e.what() << endl;

    mrpt::system::pause();
    return -1;
  }
  catch (...)
  {
    setConsoleColor(CONCOL_RED, true);
    cerr << "Program finished for an untyped exception!!" << endl;
    setConsoleColor(CONCOL_NORMAL, true);

    mrpt::system::pause();
    return -1;
  }
}

