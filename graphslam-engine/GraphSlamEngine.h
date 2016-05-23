/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Sun May 22 12:48:25 EEST 2016, nickkouk
// General TODO list:
// TODO: Make class generic - so that it handles 3D datasets
// TODO: Add functionality to be able to use different optimizers
// TODO: Plot x^2 ,x^2/s, time, iteration in the viz. window

#ifndef GRAPHSLAMENGINE_H
#define GRAPHSLAMENGINE_H

#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/mrpt_stdint.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/graphslam.h>
#include <mrpt/graphs.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // It's in the lib mrpt-maps now

#include <string>
#include <sstream>
#include <map>
#include <cerrno>
#include <cmath> // fabs function




using namespace mrpt;
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

// TODO - have CPOSE as a template parameter as well
template <class GRAPH_t>
class GraphSlamEngine_t {
  public:

    typedef std::map<string, CFileOutputStream*> fstreams;
    typedef std::map<string, CFileOutputStream*>::iterator fstreams_it;
    typedef std::map<string, CFileOutputStream*>::const_iterator fstreams_cit;

    typedef typename GRAPH_t::constraint_t constraint_t;
    typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
    typedef CMatrixFixedNumeric<double,
            constraint_t::state_length, constraint_t::state_length> InfMat;


    // Ctors, Dtors, init fun.
    //////////////////////////////////////////////////////////////
    GraphSlamEngine_t(const string& config_file,
        CDisplayWindow3D* win = NULL ):
      kOffsetYStep(20), // textMessage vertical text position
      kIndexTextStep(1), // textMessage index
      kFontName("mono"), // font used in the textMessage 
      kFontSize(11) // font size used in the textMessage 
    {
      m_win = win;

      m_config_fname = config_file;
      this->initGraphSlamEngine();
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
      saveGraph(fname);

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
    void parseLaserScansFile();
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
     * Method to automate the creation of the output result files
     * Open and write an introductory message using the provided fname
     */
    void initResultsFile(const string& fname);

    /**
     * GraphSlamEngine_t::visualizeGraph
     *
     * Called internally for updating the vizualization scene for the graph
     * building procedure
     */
    void visualizeGraph(const GRAPH_t& gr);
    /**
     * assignTextMessageParameters
     *
     * Assign the next available offset_y and text_index for the textMessage under
     * construction
     */
    void assignTextMessageParameters(int* offset_y, int* text_index) {
      *offset_y = m_curr_offset_y;
      m_curr_offset_y += kOffsetYStep;

      *text_index = m_curr_text_index;
      m_curr_text_index += kIndexTextStep;
    }

    /**
     * General initialization method to call from all the Ctors
     */
    void initGraphSlamEngine() {

      // Initialization of member variables
      //////////////////////////////////////////////////////////////

      m_is3D = constraint_t::is_3D_val;

      // max node number already in the graph
      m_nodeID_max = 0;

      graph.root = TNodeID(0);

      // register the available optimizers
      optimizer_codes["levmarq"] = 0;
      optimizer_codes["isam"] = 1;

      // initialize the necessary maps for graph information
      graph_to_name[&graph] = "optimized_graph";
      graph_to_viz_params[&graph] = &m_optimized_graph_viz_params;

      // Current Text Position
      m_curr_offset_y = 30;
      m_curr_text_index = 1;

      // Calling of initalization-relevant functions
      this->readConfigFile(m_config_fname);
      this->initOutputDir();
      this->printProblemParams();

      // timestamp
      this->assignTextMessageParameters(&m_offset_y_timestamp, 
                                        &m_text_index_timestamp);


      // optimized graph
      assert(m_has_read_config);
      if (m_visualize_optimized_graph) {
        this->assignTextMessageParameters( /* offset_y*   = */ &m_offset_y_graph,
                                     /* text_index* = */ &m_text_index_graph );
      }


      // odometry visualization
      assert(m_has_read_config);
      if (m_visualize_odom_poses) {
        this->assignTextMessageParameters( /* offset_y*   = */ &m_offset_y_odometry,
                                     /* text_index* = */ &m_text_index_odometry);

        COpenGLScenePtr scene = m_win->get3DSceneAndLock();

        CPointCloudPtr odom_poses_cloud = CPointCloud::Create();
        scene->insert(odom_poses_cloud);
        odom_poses_cloud->setPointSize(2.0);
        odom_poses_cloud->enablePointSmooth();
        odom_poses_cloud->enableColorFromY();
        odom_poses_cloud->setName("odom_poses_cloud");

        m_win->unlockAccess3DScene();

        m_win->addTextMessage(5,-m_offset_y_odometry, 
                              format("Odometry path"),
                              TColorf(0.0, 0.0, 1.0),
                              kFontName, kFontSize, // font name & size
                              mrpt::opengl::NICE,
                              /* unique_index = */ m_text_index_odometry );

        m_win->forceRepaint(); 
      }

      // axis
      {
        COpenGLScenePtr scene = m_win->get3DSceneAndLock();

        CAxisPtr obj = CAxis::Create();
        obj->setFrequency(5);
        obj->enableTickMarks();
        obj->setAxisLimits(-10,-10,-10, 10,10,10);
        obj->setName("axis");
        scene->insert(obj);

        m_win->unlockAccess3DScene();
        m_win->forceRepaint();
      }


    }


    // VARIABLES
    //////////////////////////////////////////////////////////////

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
    string   m_optimizer;

    string   m_loop_closing_alg;

    string   m_decider_alg;
    double   m_distance_threshold;
    double   m_angle_threshold;

    bool     m_has_read_config;
    map<string, int> optimizer_codes; // use lookpu table to hold the optimizer codes

    /** 
     * FileStreams
     * variable that keeps track of the out fstreams so that they can be closed 
     * (if still open) in the class Dtor.
     */
    fstreams m_out_streams;

    // visualization objects
    CDisplayWindow3D* m_win;

    TParametersDouble m_optimized_graph_viz_params;
    bool m_visualize_optimized_graph;
    bool m_visualize_odom_poses;

    /**
     * textMessage Parameters 
     *
     */
    const string kFontName;
    const int kFontSize;

    // textMessage vertical text position
    const int kOffsetYStep;
    int m_curr_offset_y;
    int m_offset_y_graph;
    int m_offset_y_odometry;
    int m_offset_y_timestamp;

    // textMessage index
    const int kIndexTextStep;
    int m_curr_text_index;
    int m_text_index_graph;
    int m_text_index_odometry;
    int m_text_index_timestamp;


    /** 
     * std::maps to store information about the graphs
     */
    map<const GRAPH_t*, string> graph_to_name;
    map<const GRAPH_t*, TParametersDouble*> graph_to_viz_params;

    // odometry visualization
    vector<pose_t*> m_odom_poses;

    bool m_is3D;
    TNodeID m_nodeID_max;

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
void GraphSlamEngine_t<GRAPH_t>::parseLaserScansFile() {
  MRPT_START

  if (!m_has_read_config) {
    THROW_EXCEPTION("Config file has not been provided yet.\nExiting...");
  }

  // test whether the given m_rawlog_fname is a valid file name. Otherwise throw
  // exception to the user
  if (fileExists(m_rawlog_fname)) {
    CFileGZInputStream rawlog_file(m_rawlog_fname);
    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr observation;
    size_t curr_rawlog_entry = 0;

    bool end = false;

    // Tracking the PDF of the current position of the robot - use a
    // constraint_t. Its information matrix is the relative uncertainty between
    // the current and the previous registered graph node.
    
    // I am sure of the initial position, set to identity matrix
    double tmp[] = {1.0, 0.0, 0.0,
                    0.0, 1.0 ,0.0,
                    0.0, 0.0, 1.0 };
    InfMat init_path_uncertainty(tmp);
    pose_t last_pose_inserted;
    constraint_t cur_pathPDF(last_pose_inserted, init_path_uncertainty); 

    TNodeID from = graph.root; // first node shall be the root - 0

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
        //VERBOSE_COUT << endl;
        //VERBOSE_COUT << "Format #1: Found pair of action & observation" << endl;
        //VERBOSE_COUT << "-----------------------------------------------------------------" << endl;

        /**
         * ACTION PART - Handle the odometry information of the rawlog file
         */

        CActionRobotMovement2DPtr robot_move = action->getBestMovementEstimation();
        CPosePDFPtr increment = robot_move->poseChange;
        
        // timestamp textMessage
        // use the dataset timestamp otherwise fallback to mrpt::system::now()
        TTimeStamp  timestamp = robot_move->timestamp;
        if (timestamp != INVALID_TIMESTAMP) {
          m_win->addTextMessage(5,-m_offset_y_timestamp, 
              format("Simulated time: %s", timeLocalToString(timestamp).c_str()),
              TColorf(1.0, 1.0, 1.0),
              kFontName, kFontSize, // font name & size
              mrpt::opengl::NICE,
              /* unique_index = */ m_text_index_timestamp );
        }
        else {
          m_win->addTextMessage(5,-m_offset_y_timestamp, 
              format("Wall time: %s", timeLocalToString(system::now()).c_str()),
              TColorf(1.0, 1.0, 1.0),
              kFontName, kFontSize, // font name & size
              mrpt::opengl::NICE,
              /* unique_index = */ m_text_index_timestamp );
        }



        // current pose of the robot - w/o PDF
        pose_t odom_pose_tot = cur_pathPDF.getMeanVal();

        // update the cur_pathPDF
        // add the PDF of the incremental odometry update
        pose_t pose_increment = increment->getMeanVal();
        InfMat inf_increment; increment->getInformationMatrix(inf_increment);
        constraint_t incremental_constraint(pose_increment, inf_increment);
        cur_pathPDF += incremental_constraint;

        // add to the odometry PointCloud and visualize it
        {
          pose_t* odom_pose = new pose_t;
          *odom_pose = odom_pose_tot;
          m_odom_poses.push_back(odom_pose);

          if (m_visualize_odom_poses) {
            COpenGLScenePtr scene = m_win->get3DSceneAndLock();

            CRenderizablePtr obj = scene->getByName("odom_poses_cloud");
            CPointCloudPtr odom_poses_cloud = static_cast<CPointCloudPtr>(obj);
            odom_poses_cloud->insertPoint(m_odom_poses.back()->x(),
                                          m_odom_poses.back()->y(),
                                          0 );

            m_win->unlockAccess3DScene();

            m_win->forceRepaint();
          }
          
        }

        /** 
         * Fixed intervals odometry edge insertion
         * Determine whether to insert a new pose in the graph given the
         * distance and angle thresholds
         */
        // TODO - manipulate the timestamp
        // TODO - add different color to edges based on where they come from
        if ( (last_pose_inserted.distanceTo(odom_pose_tot) > m_distance_threshold) ||
            fabs(wrapToPi(last_pose_inserted.phi() - odom_pose_tot.phi())) > m_angle_threshold )
        {

          from = m_nodeID_max;
          TNodeID to = ++m_nodeID_max;

          // build the relative edge and insert it
          {
            constraint_t rel_edge;
            rel_edge.mean = cur_pathPDF.getMeanVal() - last_pose_inserted;
            rel_edge.cov_inv = cur_pathPDF.cov_inv;

            graph.nodes[to] = odom_pose_tot;
            graph.insertEdgeAtEnd(from, to, rel_edge);
            //graph.insertEdgeAtEnd(from, to, odom_pose_tot - last_pose_inserted);
            
          }

          // optimize the graph - LM
          

          // update the visualization window
          if (m_visualize_optimized_graph) {
            visualizeGraph(graph);
          }

          
          ////TODO - remove these
          //double distance = (last_pose_inserted.distanceTo(odom_pose_tot));
          //double angle = fabs(last_pose_inserted.phi() - odom_pose_tot.phi());
          //VERBOSE_COUT << "Added new odometry edge to the graph (# " << m_nodeID_max + 1<< " )" << endl;
          //VERBOSE_COUT << "prev_pose = " << last_pose_inserted << endl;
          //VERBOSE_COUT << "new_pose  = " << odom_pose_tot << endl;
          //VERBOSE_COUT << "distance: " << distance << "m | " 
                       //<< "angle: " << RAD2DEG(wrapToPi(angle)) << " deg" << endl;
          //VERBOSE_COUT << "Relative uncertainty between nodes: " << endl
                       //<< cur_pathPDF.cov_inv << endl;


          // Change the current uncertainty of cur_pathPDF so that it measures
          // the *relative* uncertainty from last_inserted_pose
          cur_pathPDF.cov_inv = init_path_uncertainty;

          last_pose_inserted = odom_pose_tot;

        

        } // IF ODOMETRY_CRITERIUM
      } // ELSE FORMAT #1
    } // WHILE CRAWLOG FILE
  }  // IF FILE_EXISTS
  else {
    THROW_EXCEPTION("parseLaserScansFile: Inputted rawlog file ( " 
        << m_rawlog_fname << " ) not found");
  }

  MRPT_END
} // END OF FUNCTION

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::visualizeGraph(const GRAPH_t& gr) {
  // TODO - add preprocessor flags for compilation of visualization part 
  
  string gr_name = graph_to_name[&gr];
  const TParametersDouble* viz_params = graph_to_viz_params[&gr];

  //TODO: Delete this
  //cout << "Visualization parameters: " << endl;
  //for (map<string, double>::const_iterator it = viz_params->begin();
      //it != viz_params->end(); ++it) {
    //cout << it->first << " : " << it->second << endl;
  //}
  
  // update the graph (clear and rewrite..)
  COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

  // remove previous graph
  CRenderizablePtr prev_object = scene->getByName(gr_name);
  scene->removeObject(prev_object);

  // Insert the new instance of the graph
  CSetOfObjectsPtr graph_obj = graph_tools::graph_visualize(gr, *viz_params);
  graph_obj->setName(gr_name);
  scene->insert(graph_obj);

  m_win->unlockAccess3DScene();
  m_win->addTextMessage(5,-m_offset_y_graph, 
      format("Optimized Graph (#%d)", static_cast<int>(gr.nodeCount())),
      TColorf(0.0, 0.0, 0.0),
      kFontName, kFontSize, // font name & size
      mrpt::opengl::NICE,
      /* unique_index = */ m_text_index_graph);

  m_win->forceRepaint();

  // set the view correctly
  m_win->setCameraElevationDeg(75);
  CGridPlaneXYPtr obj_grid = graph_obj->CSetOfObjects::getByClass<CGridPlaneXY>();
  if (obj_grid) {
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
  m_optimizer = cfg_file.read_string(
      "GraphSLAMParameters",
      "optimizer",
      "levmarq", false);

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

  // Section: VisualizationParameters
  // ////////////////////////////////
  // http://reference.mrpt.org/devel/group__mrpt__opengl__grp.html#ga30efc9f6fcb49801e989d174e0f65a61
  
  // Optimized graph
  // TODO - have some of the parameters as non-configurable

  m_visualize_optimized_graph = cfg_file.read_bool(
      "VisualizationParameters",
      "visualize_optimized_graph",
      1, false);
 
	m_optimized_graph_viz_params["show_ID_labels"] = cfg_file.read_bool(
      "VisualizationParameters",
      "optimized_show_ID_labels",
      0, false);
	m_optimized_graph_viz_params["show_ground_grid"] = cfg_file.read_bool(
      "VisualizationParameters",
      "optimized_show_ground_grid",
      1, false);
	m_optimized_graph_viz_params["show_edges"] = cfg_file.read_bool(
      "VisualizationParameters",
      "optimized_show_edges",
      1, false);
	m_optimized_graph_viz_params["edge_color"] = cfg_file.read_int(
      "VisualizationParameters",
      "optimized_edge_color",
      4286611456, false);
	m_optimized_graph_viz_params["edge_width"] = cfg_file.read_double(
      "VisualizationParameters",
      "optimized_edge_width",
      1.5, false);
	m_optimized_graph_viz_params["show_node_corners"] = cfg_file.read_bool(
      "VisualizationParameters",
      "optimized_show_node_corners",
      1, false);
	m_optimized_graph_viz_params["show_edge_rel_poses"] = cfg_file.read_bool(
      "VisualizationParameters",
      "optimized_show_edge_rel_poses",
      1, false);
	m_optimized_graph_viz_params["edge_rel_poses_color"] = cfg_file.read_int(
      "VisualizationParameters",
      "optimized_edge_rel_poses_color",
      1090486272, false);
	m_optimized_graph_viz_params["nodes_edges_corner_scale"] = cfg_file.read_double(
      "VisualizationParameters",
      "optimized_nodes_edges_corner_scale",
      0.4, false);
	m_optimized_graph_viz_params["nodes_corner_scale"] = cfg_file.read_double(
      "VisualizationParameters",
      "optimized_nodes_corner_scale",
      0.7, false);
	m_optimized_graph_viz_params["point_size"] = cfg_file.read_int(
      "VisualizationParameters",
      "optimized_point_size",
      0, false);
	m_optimized_graph_viz_params["point_color"] = cfg_file.read_int(
      "VisualizationParameters",
      "optimized_point_color",
      10526880, false);

  // odometry-only visualization
  
  m_visualize_odom_poses = cfg_file.read_bool(
      "VisualizationParameters",
      "visualize_odom_poses",
      1, false);


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
  ss_out << " \t optimizer:                        " << m_optimizer << endl;
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
