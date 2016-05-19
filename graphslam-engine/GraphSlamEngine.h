/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef GRAPHSLAMENGINE_H
#define GRAPHSLAMENGINE_H


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
#include <cmath> // fabs function




using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;

using namespace std;


bool verbose = true;
#define VERBOSE_COUT  if (verbose) std::cout << "[graphslam_engine] "

typedef std::map<string, CFileOutputStream*> fstreams;
typedef std::map<string, CFileOutputStream*>::iterator fstreams_it;
typedef std::map<string, CFileOutputStream*>::const_iterator fstreams_cit;

// TODO - have CPOSE as a template parameter as well
template <class GRAPH_t>
class GraphSlamEngine_t {
  public:
    // Ctors, Dtors, init fun.
    //////////////////////////////////////////////////////////////
    GraphSlamEngine_t(CDisplayWindow3D* win) {
      // TODO - Have flag to enable/disable visualization (in .ini)
      m_win = win;

      initGraphSlamEngine();
    };
    ~GraphSlamEngine_t();

    // IO funs
    //////////////////////////////////////////////////////////////
    /**
     * Wrapper fun around the GRAPH_t corresponding method
     */
    void saveGraph() const {
      if (!m_has_read_config) {
        THROW_EXCEPTION("Config file has not been provided yet.\nExiting...");
      }
      string fname = m_output_dir_fname + "/" + m_save_graph_fname;
      this->saveGraph(fname);

    }
    void saveGraph(std::string fname) const {
      graph.saveToTextFile(fname);
      VERBOSE_COUT << "Saved graph to text file: " << fname <<
        " successfully." << endl;
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
     * Have the graph as a public variable so that you don't have to write all
     * the setters and getters for it.
     */
    GRAPH_t graph;

  private:
    // METHODS
    //////////////////////////////////////////////////////////////

    /**
     * General initialization method to call from all the Ctors
     */
    void initGraphSlamEngine() {
      m_nodeID_max = 0;

    }


    /**
     * Method to automate the creation of the output result files
     * Open and write an introductory message using the provided fname
     */
    void initResultsFile(const string& fname);




    /**
     * GraphSlamEngine_t::updateGraphVizualization
     *
     * Called internally for updating the vizualization scene for the graph
     * building procedure
     */
    void updateGraphVizualization();

    // VARIABLES
    //////////////////////////////////////////////////////////////

    // visualization objects
    CDisplayWindow3D* m_win;
    TParametersDouble m_graph_viz_params;

    // max node number already in the graph
    TNodeID m_nodeID_max;

    /**
     * Problem parameters.
     * Most are imported from a .ini config file
     * \sa GraphSlamEngine_t::readConfigFile
     */
    string   m_config_fname;

    string   m_rawlog_fname;
    string   m_output_dir_fname;
    bool     m_user_decides_about_output_dir;
    bool     m_do_debug;
    string   m_debug_fname;
    string   m_save_graph_fname;

    bool     m_do_pose_graph_only;

    string   m_loop_closing_alg;

    string   m_decider_alg;
    double   m_distance_threshold;
    double   m_angle_threshold;

    bool     m_has_read_config;

    /** 
     * FileStreams
     * variable that keeps track of the out streams so that they can be closed 
     * (if still open) in the class Dtor.
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
    THROW_EXCEPTION("Config file has not been provided yet.\nExiting...");
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
    CPose2D odom_pose_tot(0, 0, 0); // current pose - given by odometry
    CPose2D last_pose_inserted(0, 0, 0);
    TNodeID from = TNodeID(0); // first node shall be the root - 0



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
        VERBOSE_COUT << "Format #2: Found observation." << endl;
        VERBOSE_COUT << "--------------------------------------------------" << endl;
        //TODO Implement 2nd format
      }
      else {
        // action, observations should contain a pair of valid data 
        // (Format #1 rawlog file)
        VERBOSE_COUT << endl;
        VERBOSE_COUT << "Format #1: Found pair of action & observation" << endl;
        VERBOSE_COUT << "-----------------------------------------------------------------" << endl;

        /**
         * ACTION PART - Handle the odometry information of the rawlog file
         */

        CActionRobotMovement2DPtr robotMovement2D = action->getBestMovementEstimation();
        CPose2D pose_incr = robotMovement2D->poseChange->getMeanVal();
        odom_pose_tot += pose_incr;

        /** 
         * Fixed intervals node insertion
         * Determine whether to insert a new pose in the graph given the
         * distance and angle thresholds
         */
        // TODO add a time constraint as well - use the time information of the
        // rawlog file
        if ( (last_pose_inserted.distanceTo(odom_pose_tot) > m_distance_threshold) ||
            fabs(last_pose_inserted.phi() - odom_pose_tot.phi()) > m_angle_threshold )
        {
          //TODO - remove these
          double distance = (last_pose_inserted.distanceTo(odom_pose_tot));
          double angle = fabs(last_pose_inserted.phi() - odom_pose_tot.phi());

          VERBOSE_COUT << "Adding new edge to the graph (# " << m_nodeID_max + 1<< " )" << endl;
          VERBOSE_COUT << "prev_pose = " << last_pose_inserted << endl;
          VERBOSE_COUT << "new_pose  = " << odom_pose_tot << endl;
          VERBOSE_COUT << "distance: " << distance << "m | " 
            << "angle: " << RAD2DEG(angle) << " deg" << endl;


          CPose2D rel_pose = odom_pose_tot - last_pose_inserted;
          from = m_nodeID_max;
          TNodeID to = ++m_nodeID_max;

          graph.nodes[to] = odom_pose_tot;
          // TODO consider calling insertEdgeAtEnd method?
          graph.insertEdge(from, to, rel_pose);

          last_pose_inserted = odom_pose_tot;

          // update the visualization window
          this->updateGraphVizualization();

        } // IF ODOMETRY_CRITERIUM
      } // ELSE FORMAT #1
    } // WHILE CRAWLOG FILE
  }  // IF FILE_EXISTS
  else {
    THROW_EXCEPTION("parseLaserScansFile: Inputted Rawlog file ( " << fname <<
        " ) not found");
  }

  MRPT_END
} // END OF FUNCTION

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::updateGraphVizualization() {

  // update the graph (clear and rewrite..)
  COpenGLScenePtr& m_scene = m_win->get3DSceneAndLock();
  m_scene->clear(); 

  // TODO - consider adding a custom extension to graph_visualize 
  // (in case it is not fast or customisable enough)
  CSetOfObjectsPtr m_graph_obj = graph_tools::graph_visualize(graph,
      m_graph_viz_params);
  m_scene->insert(m_graph_obj);
  m_win->unlockAccess3DScene();

  // set the view correctly
  m_win->setCameraElevationDeg(75);
  CGridPlaneXYPtr obj_grid = m_graph_obj->CSetOfObjects::getByClass<CGridPlaneXY>();
  if (obj_grid)
  {
    float x_min,x_max, y_min,y_max;
    obj_grid->getPlaneLimits(x_min,x_max, y_min,y_max);
    const float z_min = obj_grid->getPlaneZcoord();
    m_win->setCameraPointingToPoint( 0.5*(x_min+x_max), 0.5*(y_min+y_max), z_min );
    m_win->setCameraZoom( 2.0f * std::max(10.0f, std::max(x_max-x_min, y_max-y_min) ) );
  }

  // push the changes
  m_win->repaint();
}

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::readConfigFile(const std::string& fname) {
  MRPT_START

  CConfigFile cfg_file(fname);
  m_config_fname = fname;

  // Section: GeneralConfiguration 
  // ////////////////////////////////
  m_rawlog_fname = cfg_file.read_string(
      /*section_name = */ "GeneralConfiguration", 
      /*var_name = */ "rawlog_fname",
      /*default_value = */ "", /*failIfNotFound = */ true);
  m_output_dir_fname = cfg_file.read_string(
      "GeneralConfiguration", 
      "output_dir_fname",
      "graphslam_engine_results", false);
  m_user_decides_about_output_dir = cfg_file.read_bool(
      "GeneralConfiguration", 
      "user_decides_about_output_dir",
      true, false);
  m_do_debug =  cfg_file.read_bool(
      "GeneralConfiguration", 
      "do_debug", 
      true, false);
  m_debug_fname = cfg_file.read_string(
      "GeneralConfiguration", 
      "debug_fname", 
      "debug.log", false);
  m_save_graph_fname = cfg_file.read_string(
      "GeneralConfiguration",
      "save_graph_fname",
      "poses.log", false);

  // Section: GraphSLAMParameters 
  // ////////////////////////////////
  m_do_pose_graph_only = cfg_file.read_bool(
      "GraphSLAMParameters",
      "do_pose_graph_only",
      true, false);

  // Section: LoopClosingParameters 
  // ////////////////////////////////
  m_loop_closing_alg = cfg_file.read_string(
      "LoopClosingParameters",
      "loop_closing_alg",
      "", true);

  // Section: DecidersConfiguration  - When to insert new nodes?
  // ////////////////////////////////
  m_decider_alg = cfg_file.read_string(
      "DecidersConfiguration",
      "decider_alg",
      "", true);
  m_distance_threshold = cfg_file.read_double(
      "DecidersConfiguration",
      "distance_threshold",
      1 /* meter */, false);
  m_angle_threshold = cfg_file.read_double(
      "DecidersConfiguration",
      "angle_threshold",
      60 /* degrees */, false);
  m_angle_threshold = DEG2RAD(m_angle_threshold);

  // Section: GraphVisualizationParameters
  // ////////////////////////////////
  // http://reference.mrpt.org/devel/group__mrpt__opengl__grp.html#ga30efc9f6fcb49801e989d174e0f65a61
  
	m_graph_viz_params["show_ID_labels"] = cfg_file.read_bool(
      "GraphVisualizationParameters",
      "show_ID_labels",
      0, false);
	m_graph_viz_params["show_ground_grid"] = cfg_file.read_bool(
      "GraphVisualizationParameters",
      "show_ground_grid",
      1, false);
	m_graph_viz_params["show_edges"] = cfg_file.read_bool(
      "GraphVisualizationParameters",
      "show_edges",
      1, false);
	m_graph_viz_params["edge_color"] = cfg_file.read_int(
      "GraphVisualizationParameters",
      "edge_color",
      4286611456, false);
	m_graph_viz_params["edge_width"] = cfg_file.read_double(
      "GraphVisualizationParameters",
      "edge_width",
      1.5, false);
	m_graph_viz_params["show_node_corners"] = cfg_file.read_bool(
      "GraphVisualizationParameters",
      "show_node_corners",
      1, false);
	m_graph_viz_params["show_edge_rel_poses"] = cfg_file.read_bool(
      "GraphVisualizationParameters",
      "show_edge_rel_poses",
      1, false);
	m_graph_viz_params["edge_rel_poses_color"] = cfg_file.read_int(
      "GraphVisualizationParameters",
      "edge_rel_poses_color",
      1090486272, false);
	m_graph_viz_params["nodes_edges_corner_scale"] = cfg_file.read_double(
      "GraphVisualizationParameters",
      "nodes_edges_corner_scale",
      0.4, false);
	m_graph_viz_params["nodes_corner_scale"] = cfg_file.read_double(
      "GraphVisualizationParameters",
      "nodes_corner_scale",
      0.7, false);
	m_graph_viz_params["nodes_point_color"] = cfg_file.read_int(
      "GraphVisualizationParameters",
      "nodes_point_color",
      10526880, false);

	m_has_read_config = true;
	MRPT_END
}

// TODO - add the new values from visualization part
template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::printProblemParams() {
  MRPT_START

    stringstream ss_out;

  ss_out << "--------------------------------------------------------------------------" << endl;
  ss_out << " Graphslam_engine: Problem Parameters " << endl;
  ss_out << " \t Config fname:                     " << m_config_fname << endl;
  ss_out << " \t Rawlog fname:                     " << m_rawlog_fname << endl;
  ss_out << " \t Output dir:                       " << m_output_dir_fname << endl;
  ss_out << " \t User decides about output dir? :  " << m_user_decides_about_output_dir << endl;
  ss_out << " \t Debug mode:                       " << m_do_debug << endl;
  ss_out << " \t save_graph_fname:                 " << m_save_graph_fname << endl;
  ss_out << " \t do_pose_graph_only:               " <<  m_do_pose_graph_only << endl;
  ss_out << " \t Loop closing alg:                 " << m_loop_closing_alg << endl;
  ss_out << " \t Decider alg:                      " << m_decider_alg << endl;
  ss_out << " \t Distance Threshold:               " << m_distance_threshold << " m" << endl;
  ss_out << " \t Angle Threshold:                  " << RAD2DEG(m_angle_threshold) << " deg" << endl;
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
      int answer_int;
      if (m_user_decides_about_output_dir) {
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
        answer_int = atoi(&answer[0]);
      }
      else {
        answer_int = 2;
      }
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

#endif /* end of include guard: GRAPHSLAMENGINE_H */
