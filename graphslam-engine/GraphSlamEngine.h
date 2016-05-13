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
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/CConfigFile.h>

#include <string>
#include <sstream>
#include <map>
#include <cerrno>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;

using namespace std;

#define VERBOSE_COUT  if (verbose) std::cout << "[graphslam_engine] "
bool verbose = true;

template <class GRAPH_t>
class GraphSlamEngine_t {
  public:
    // Ctors, Dtors, init fun.
    //////////////////////////////////////////////////////////////
    GraphSlamEngine_t() {
      initGraphSlamEngine();
    };
    ~GraphSlamEngine_t() {};

    void initGraphSlamEngine() {
      m_node_max = 0;

    }

    // IO funs
    //////////////////////////////////////////////////////////////
    /**
     * Wrapper fun around the GRAPH_t method
     */
    void saveGraphToTextFile(const std::string& fname) const {
      m_graph.saveToTextFile(fname);
      VERBOSE_COUT << "Saved graph to text file." << endl;
    }
    void dumpGraphToConsole() const {}
    /** 
     * Read the configuration file specified and fill in the corresponding
     * class members
     */
   void readConfigFile(const std::string& fname);
    /**
     * Print the problem parameters (usually fetched from a configuration file)
     * to the console for verification
     */
    void printProblemParams();
    /**
     * TODO - Make this a function template so that it can handle camera
     * images, laser scan files, etc.
     * Reads the file provided and builds the initial graph prior to loop
     * closure searches
     */
    void parseLaserScansFile(const std::string& fname);


    // GRAPH_t manipulation methods
    //////////////////////////////////////////////////////////////

    /**
     * TODO: Make this a template so that you mark HOW was this node created
     * (ICP, odometry, camera etc.)
     * Adds a nodes to graph
     */
    //addNode
    //adEdge

  private:
    GRAPH_t m_graph;
    size_t m_node_max;

    /**
     * Problem parameters.
     * Most are imported from a .ini config file
     * See "readConfigFile" fun.
     */
    string m_config_fname;

    string m_rawlog_fname;
    string m_output_dir_fname;
    bool m_do_debug;
    string m_debug_fname;
    string m_robot_poses_fname;

    bool m_do_pose_graph_only;

    string m_loop_closing_alg;

    string m_decider_alg;
    double m_distance_threshold;
    double m_angle_threshold;

};


template<class GRAPH_T>
void GraphSlamEngine_t<GRAPH_T>::parseLaserScansFile(const std::string& fname) {
  // test whether the given fname is a valid file name. Otherwise throw
  // exception to the user
  if (!fname.empty() && fileExists(fname)) {
    CFileGZInputStream rawlog_file(fname);
    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr observation;
    size_t curr_rawlog_entry = 0;

    bool end = false;
    CPose2D odom_pose2d_tot(0, 0, 0);



    // Read from the rawlog
    while (CRawlog::getActionObservationPairOrObservation(
          rawlog_file,
          action,               // Possible out var: Action of a a pair action / obs
          observations,         // Possible out var: obs's of a pair actin     / obs
          observation,          // Possible out var
          curr_rawlog_entry ) ) // IO counter
    {
      // process action & observations
      if (observation.present()) {
        // Read a single observation from the rawlog (Format #2 rawlog file)
        VERBOSE_COUT << "Foramt #2: Found observation." << endl;
        cout << "--------------------------------------------------" << endl;
        //TODO Implement 2nd format
      }
      else {
        // action, observations should contain a pair of valid data 
        // (Format #1 rawlog file)
        VERBOSE_COUT << "Format #1: Found pair of action & observation" << endl;
        cout << "-----------------------------------------------------------------" << endl;

        CActionRobotMovement2DPtr robotMovement2D = action->getBestMovementEstimation();
        CPose2D a = robotMovement2D->poseChange->getMeanVal();
        cout << "a = " << a << endl;
        odom_pose2d_tot += a;

      }
    }
    VERBOSE_COUT << "Final Pose - Odometry: " << odom_pose2d_tot << endl;
  }
  else {
    THROW_EXCEPTION("parseLaserScansFile: Inputted Rawlog file ( " << fname <<
        " ) not found");
  }
}


template<class GRAPH_T>
void GraphSlamEngine_t<GRAPH_T>::readConfigFile(const std::string& fname) {
  CConfigFile cfg_file(fname);
  m_config_fname = fname;

  // Section: GeneralConfiguration 
  // ////////////////////////////////
  m_rawlog_fname = cfg_file.read_string(/*section_name = */ "GeneralConfiguration", 
                                       /*var_name = */ "rawlog_fname",
                                       /*default_value = */ "", 
                                       /*failIfNotFound = */ true);
  m_output_dir_fname = cfg_file.read_string("GeneralConfiguration", "output_dir_fname",
      "graphslam_engine_results", false);
  m_do_debug =  cfg_file.read_bool("GeneralConfiguration", "do_debug", 
      true, false);
  m_debug_fname = cfg_file.read_string("GeneralConfiguration", "debug_fname", 
      "debug.log", false);
  m_robot_poses_fname = cfg_file.read_string("GeneralConfiguration", "robot_poses_fname",
      "poses.log", false);

  // Section: GraphSLAMParameters 
  // ////////////////////////////////
  m_do_pose_graph_only = cfg_file.read_bool("GraphSLAMParameters", "do_pose_graph_only",
      true, false);
  
  // Section: LoopClosingParameters 
  // ////////////////////////////////
  m_loop_closing_alg = cfg_file.read_string("LoopClosingParameters", "loop_closing_alg",
      "", true);

  // Section: DecidersConfiguration 
  // ////////////////////////////////
  m_decider_alg = cfg_file.read_string("DecidersConfiguration", "decider_alg",
      "", true);
  m_distance_threshold = cfg_file.read_double("DecidersConfiguration", "distance_threshold",
      1 /* meter */, false);
  m_angle_threshold = cfg_file.read_double("DecidersConfiguration", "angle_threshold",
      60 /* degrees */, false);
  m_angle_threshold = DEG2RAD(m_angle_threshold);

}

template<class GRAPH_T>
void GraphSlamEngine_t<GRAPH_T>::printProblemParams() {
  stringstream ss_out;

  ss_out << "--------------------------------------------------------------------------" << endl;
  ss_out << "Graphslam_engine: Problem Parameters "  << endl;
  ss_out << " \t Config fname:       " << m_config_fname << endl;
  ss_out << " \t Output dir:         " << m_output_dir_fname<< endl;
  ss_out << " \t Debug mode:         " << m_do_debug << endl;
  ss_out << " \t robot_poses_fname:  " << m_robot_poses_fname << endl;
  ss_out << " \t do_pose_graph_only: " <<  m_do_pose_graph_only << endl;
  ss_out << " \t Loop closing alg:   " << m_loop_closing_alg << endl;
  ss_out << " \t Decider alg:        " << m_decider_alg << endl;
  ss_out << " \t Distance Threshold: " << m_distance_threshold << endl;
  ss_out << " \t Angle Threshold:    " << m_angle_threshold << endl;
  ss_out << "--------------------------------------------------------------------------" << endl;

  cout << ss_out.str();
}

