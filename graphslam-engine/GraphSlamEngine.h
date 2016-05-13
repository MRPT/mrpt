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
#include <mrpt/system/datetime.h>
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

typedef std::map<string, CFileOutputStream*> fstreams;
typedef std::map<string, CFileOutputStream*>::iterator fstreams_it;
typedef std::map<string, CFileOutputStream*>::const_iterator fstreams_cit;

template <class GRAPH_t>
class GraphSlamEngine_t {
  public:
    // Ctors, Dtors, init fun.
    //////////////////////////////////////////////////////////////
    GraphSlamEngine_t() {
      initGraphSlamEngine();
    };
    ~GraphSlamEngine_t();

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
      VERBOSE_COUT << "Saved graph to text file: " << fname <<
        " successfully" << endl;
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
     *
     * \sa GraphSlamEngine_t::parseLaserScansFile
     */
    void printProblemParams();
    /**
     * TODO - Make this a function template so that it can handle camera
     * images, laser scan files, etc.
     * Reads the file provided and builds the initial graph prior to loop
     * closure searches
     */
    void parseLaserScansFile(std::string fname);
    /**
     * Initialize (clean up and create new files) the output directory
     * Also provides cmd line arguements for the user to choose the desired
     * action.
     * \sa GraphSlamEngine_t::initResultsFile
     */
    void initOutputDir();
    /**
     * Method to automate the creation of the output result files
     * Open and write an introductory message using the provided fname
     */
    void initResultsFile(const string& fname);


    // GRAPH_t manipulation methods
    //////////////////////////////////////////////////////////////

    /**
     * TODO: Make this a template so that you mark HOW was this node created
     * (ICP, odometry, camera etc.)
     * Adds a nodes to graph
     */
    //TODO Add the actual classes
    //addNode
    //adEdge

  private:
    GRAPH_t m_graph;
    size_t m_node_max;

    /**
     * Problem parameters.
     * Most are imported from a .ini config file
     * \sa GraphSlamEngine_t::readConfigFile
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

    bool m_has_read_config;

    /** 
     * FileStreams
     * Class keeps track of these so that they can be closed (if still open) 
     * in the class Dtor.
     */
    fstreams m_out_streams;

};


template<class GRAPH_t>
GraphSlamEngine_t<GRAPH_t>::~GraphSlamEngine_t() {
  VERBOSE_COUT << "Deleting GraphSlamEngine_t instance..." << endl;

  // close all open files
  for (fstreams_it it  = m_out_streams.begin(); it != m_out_streams.end(); ++it) {
    if ((it->second)->fileOpenCorrectly()) {
      VERBOSE_COUT << "Closing file: " << (it->first).c_str() << endl;
      (it->second)->close();
    }
  }
}


template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::parseLaserScansFile(std::string fname = "") {
  MRPT_START

  if (!m_has_read_config) {
    THROW_EXCEPTION("Config file has not been provided yet.");
  }
  // if no fname is provided, use your own (standard behavior)
  if (fname.empty()) {
    fname = m_rawlog_fname;
  }

  // test whether the given fname is a valid file name. Otherwise throw
  // exception to the user
  if (fileExists(fname)) {
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

  MRPT_END
}


template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::readConfigFile(const std::string& fname) {
  MRPT_START

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

  // mark this, so that we know if we have valid input data to work with.
  m_has_read_config = true;

  MRPT_END
}

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::printProblemParams() {
  MRPT_START

  stringstream ss_out;

  ss_out << "--------------------------------------------------------------------------" << endl;
  ss_out << "Graphslam_engine: Problem Parameters "  << endl;
  ss_out << " \t Config fname:       " << m_config_fname << endl;
  ss_out << " \t Rawlog fname:       " << m_rawlog_fname << endl;
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

  MRPT_END
}

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::initOutputDir() {
  MRPT_START

  // current time vars - handy in the rest of the function.
  TTimeStamp cur_date(getCurrentTime());
  string cur_date_str(dateTimeToString(cur_date));
  string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));


  if (!m_has_read_config) {
    THROW_EXCEPTION("Cannot initialize output directory. " <<
        "Make sure you have parsed the configuration file first");
  }
  else {
    // Determine what to do with existing results if previous output directory
    // exists
    if (directoryExists(m_output_dir_fname)) {
      /**
       * Give the user 3 choices.
       * - Remove the current directory contents
       * - Rename (and keep) the current directory contents
       */
      stringstream question;
      string answer;

      question << "Directory exists. Choose between the following options" << endl;
      question << "\t 1: Rename current folder and start new output directory (default)" << endl;
      question << "\t 2: Remove existing contents and continue execution" << endl;
      question << "\t 3: Handle potential conflict manually (Halts program execution)" << endl;
      question << "\t [ 1 | 2 | 3 ] --> ";
      cout << question.str();

      getline(cin, answer);
      answer = mrpt::system::trim(answer);
      int answer_int = atoi(&answer[0]);
      switch (answer_int) {
        case 2: {
          VERBOSE_COUT << "Deleting existing files..." << endl;
          // purge directory
          deleteFilesInDirectory(m_output_dir_fname, 
              /*deleteDirectoryAsWell = */ true);
          break;
        }
        case 3: {
          // Exit gracefully - call Dtor implicitly
          return;
        }
        case 1: 
        default: {
          // rename the whole directory to DATE_TIME_${OUTPUT_DIR_NAME}
          string dst_fname = m_output_dir_fname + cur_date_validstr;
          VERBOSE_COUT << "Renaming directory to: " << dst_fname << endl;
          string* error_msg = NULL;
          bool did_rename = renameFile(m_output_dir_fname,
              dst_fname, 
              error_msg);
          if (!did_rename) {
            THROW_EXCEPTION("Error while trying to rename the output directory:" <<
                *error_msg)
          }
          break;
        }
      }
    }
    // Now rebuild the directory from scratch
    VERBOSE_COUT << "Creating the new directory structure..." << endl;
    string cur_fname;
    
    // debug_fname
    createDirectory(m_output_dir_fname);
    if (m_do_debug) {
      cur_fname = m_output_dir_fname + "/" + m_debug_fname;
      this->initResultsFile(cur_fname);
    }

    // robot_poses_fname
    cur_fname = m_output_dir_fname + "/" + m_robot_poses_fname;
    this->initResultsFile(cur_fname);

    VERBOSE_COUT << "Finished initializing output directory." << endl;
  }
  
  MRPT_END
}

template <class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::initResultsFile(const string& fname) {
  MRPT_START

  // current time vars - handy in the rest of the function.
  TTimeStamp cur_date(getCurrentTime());
  string cur_date_str(dateTimeToString(cur_date));
  string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));

  m_out_streams[fname] = new CFileOutputStream(fname);
  if (m_out_streams[fname]->fileOpenCorrectly()) {
    m_out_streams[fname]->printf("GraphSlamEngine Application\n");
    m_out_streams[fname]->printf("%s\n", cur_date_str.c_str());
    m_out_streams[fname]->printf("---------------------------------------------");
  }
  else {
    THROW_EXCEPTION("Error while trying to open " <<  fname)
  }

  MRPT_END
}
