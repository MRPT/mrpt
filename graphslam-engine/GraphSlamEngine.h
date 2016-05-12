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


#include <string>
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
    GraphSlamEngine_t() {};
    ~GraphSlamEngine_t() {};

    void initGraphSlamEngine() {
      // TODO - read these from a configuration file on initialization of
      // instance
      m_distance_thresh = 1;
      m_angle_thresh = DEG2RAD(60);
      m_verbose = true;
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
     * variables
     */
    void readConfigFile(const std::string& fname) {}
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

    string  m_rawlog_file;
    bool m_verbose;

    // fixed thresholds values for adding poses to the graph
    size_t m_distance_thresh;
    size_t m_angle_thresh;

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
void GraphSlamEngine_t<GRAPH_T>::printProblemParams() {
}
