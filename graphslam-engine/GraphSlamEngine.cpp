#include "GraphSlamEngine.h"

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
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/graphs.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // It's in the lib mrpt-maps now

#include <string>
#include <sstream>
#include <map>
#include <set>
#include <cerrno>
#include <cmath> // fabs function

#include "supplementary_funs.h"

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::graphslam;

using namespace std;

// Ctors, Dtors implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
GraphSlamEngine_t<GRAPH_t>::GraphSlamEngine_t(const string& config_file,
    CDisplayWindow3D* win /* = NULL */):
  kOffsetYStep(20.0), // textMessage vertical text position
  kIndexTextStep(1) // textMessage index
{
  m_win = win;

  m_config_fname = config_file;
  this->initGraphSlamEngine();
};

template<class GRAPH_t>
GraphSlamEngine_t<GRAPH_t>::~GraphSlamEngine_t() {
  VERBOSE_COUT << "In Destructor: Deleting GraphSlamEngine_t instance..." << endl;

  // close all open files
  for (fstreams_it it  = m_out_streams.begin(); it != m_out_streams.end(); ++it) {
    if ((it->second)->fileOpenCorrectly()) {
      VERBOSE_COUT << "Closing file: " << (it->first).c_str() << endl;
      (it->second)->close();
    }
  }
}


// Member functions implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::initGraphSlamEngine() {
  MRPT_START

  /**
   * Optimization related parameters 
   */

  // register the available optimizers
  optimizer_codes["levmarq"] = 0;
  optimizer_codes["isam"] = 1;

  m_log_sq_err_evolution.clear();

  /**
   * Initialization of various member variables
   */

  // initialize the necessary maps for graph information
  graph_to_name[&m_graph] = "optimized_graph";
  graph_to_viz_params[&m_graph] = &m_optimized_graph_viz_params;


  m_is3D = constraint_t::is_3D_val;

  // max node number already in the graph
  m_nodeID_max = 0;

  m_graph.root = TNodeID(0);

  // Calling of initalization-relevant functions
  this->readConfigFile(m_config_fname);
  this->initOutputDir();
  this->printProblemParams();

  /**
   * Visualization-related parameters initialization
   */

 
  // Current Text Position
  m_curr_offset_y = 30.0;
  m_curr_text_index = 1;

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
  if (m_visualize_odometry_poses) {
    this->assignTextMessageParameters( /* offset_y*   = */ &m_offset_y_odometry,
        /* text_index* = */ &m_text_index_odometry);

    COpenGLScenePtr scene = m_win->get3DSceneAndLock();

    CPointCloudPtr odometry_poses_cloud = CPointCloud::Create();
    scene->insert(odometry_poses_cloud);
    odometry_poses_cloud->setPointSize(2.0);
    odometry_poses_cloud->enablePointSmooth();
    odometry_poses_cloud->enableColorFromY();
    odometry_poses_cloud->setName("odometry_poses_cloud");

    m_win->unlockAccess3DScene();

    m_win->addTextMessage(5,-m_offset_y_odometry, 
        format("Odometry path"),
        TColorf(0.0, 0.0, 1.0),
        m_font_name, m_font_size, // font name & size
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

  m_edge_counter.setVisualizationWindow(m_win);

  // register the types of edges that are going to be displayed in the
  // visualization window
  {
    // total edges / loop closures
    double offset_y_total_edges, offset_y_loop_closures;
    int text_index_total_edges, text_index_loop_closures;

    this->assignTextMessageParameters(&offset_y_total_edges, &text_index_total_edges);
    //cout << "in GraphSlamEngine:  " << endl 
    //<< "offset_y_total_edges: " << offset_y_total_edges << endl
    //<< "text_index_total_edges: " << text_index_total_edges << endl;

    // constraint types..

    const char* strings[] = {"odometry", "ICP", "Visual"};
    //vector<string> vec_strings(strings, strings + 3);

    // register all the edge types
    vector<string> vec_edge_types;
    vec_edge_types.push_back("Odometry"); m_edge_counter.addEdgeType("Odometry");
    vec_edge_types.push_back("ICP");      m_edge_counter.addEdgeType("ICP");
    vec_edge_types.push_back("Visual");   m_edge_counter.addEdgeType("Visual");

    // build each one of these
    map<string, double> name_to_offset_y;
    map<string, int> name_to_text_index;
    for (vector<string>::const_iterator it = vec_edge_types.begin(); it != vec_edge_types.end();
        ++it) {
      this->assignTextMessageParameters(&name_to_offset_y[*it], &name_to_text_index[*it]);
      //cout << "in initGraphSlamEngine: " << endl;
      //cout << "name: " << *it << " | offset_y: " 
      //<< name_to_offset_y[*it] << " | text_index: " 
      //<< name_to_text_index[*it] << endl;
    }

    this->assignTextMessageParameters(&offset_y_loop_closures, &text_index_loop_closures);
    //cout << "in GraphSlamEngine:  " << endl 
    //<< "offset_y_loop_closures: " << offset_y_loop_closures << endl
    //<< "text_index_loop_closures: " << text_index_loop_closures <<endl;

    // add all the parameters to the EdgeCounter_t object
    m_edge_counter.setTextMessageParams(name_to_offset_y, name_to_text_index, 
        offset_y_total_edges, text_index_total_edges,
        offset_y_loop_closures, text_index_loop_closures,
        m_font_name, m_font_size);
    //m_edge_counter.setTextMessageParams(name_to_offset_y, name_to_text_index,
    //m_font_name, m_font_size);

  }


  MRPT_END
}

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::parseRawlogFile() {
  MRPT_START

  if (!m_has_read_config)
    THROW_EXCEPTION("Config file has not been provided yet.\nExiting...");
  if (!fileExists(m_rawlog_fname))
    THROW_EXCEPTION("parseRawlogFile: Inputted rawlog file ( " 
        << m_rawlog_fname << " ) not found");

 // good to go.. 

  CFileGZInputStream rawlog_file(m_rawlog_fname);
  CActionCollectionPtr action;
  CSensoryFramePtr observations;
  CObservationPtr observation;
  size_t curr_rawlog_entry = 0;

  // TODO unused - delete it?
  bool end = false;

  // Tracking the PDF of the current position of the robot with regards to the
  // PREVIOUS registered node
  // I am sure of the initial position, set to identity matrix
  double tmp[] = {
    1.0, 0.0, 0.0,
    0.0, 1.0 ,0.0,
    0.0, 0.0, 1.0 };
  InfMat init_path_uncertainty(tmp);
  pose_t last_pose_inserted; // defaults to all 0s
  constraint_t since_prev_node_PDF(last_pose_inserted, init_path_uncertainty); 
  pose_t curr_odometry_only_pose; // defaults to all 0s

  TNodeID from = m_graph.root; // first node shall be the root - 0


  // Read the first rawlog pair / observation explicitly to registre the laser
  // scan of the root node
  {
    CRawlog::getActionObservationPairOrObservation(
        rawlog_file,
        action,               // Possible out var: Action of a a pair action / obs
        observations,         // Possible out var: obs's of a pair actin     / obs
        observation,          // Possible out var
        curr_rawlog_entry );
    if (observation.present()) {
      // TODO - add here..
    }
    else {
      CObservation2DRangeScanPtr curr_laser_scan =
        observations->getObservationByClass<CObservation2DRangeScan>();
      m_nodes_to_laser_scans[m_graph.root] = curr_laser_scan;
    }
  }

  /** 
   * Read the rest of the rawlog file
   */
  while (CRawlog::getActionObservationPairOrObservation(
        rawlog_file,
        action,               // Possible out var: Action of a a pair action / obs
        observations,         // Possible out var: obs's of a pair actin     / obs
        observation,          // Possible out var
        curr_rawlog_entry ) ) {

    // process action & observations
    if (observation.present()) {
      // Read a single observation from the rawlog 
      // (Format #2 rawlog file)
      //TODO Implement 2nd format
    }
    else {
      // action, observations should contain a pair of valid data 
      // (Format #1 rawlog file)

      /**
       * ACTION PART - Handle the odometry information of the rawlog file
       */

      // parse the current action  
      CActionRobotMovement2DPtr robot_move = action->getBestMovementEstimation();
      CPosePDFPtr increment = robot_move->poseChange;
      pose_t increment_pose = increment->getMeanVal();
      InfMat increment_inf_mat; 
      increment->getInformationMatrix(increment_inf_mat);

      curr_odometry_only_pose += increment_pose;

      // update the relative uncertainty of the path since the LAST node was inserted
      constraint_t incremental_constraint(increment_pose, increment_inf_mat);
      since_prev_node_PDF += incremental_constraint;


      /**
       * Timestamp textMessage
       */
      // use the dataset timestamp otherwise fallback to mrpt::system::now()
      TTimeStamp  timestamp = robot_move->timestamp;
      if (timestamp != INVALID_TIMESTAMP) {
        m_win->addTextMessage(5,-m_offset_y_timestamp, 
            format("Simulated time: %s", timeLocalToString(timestamp).c_str()),
            TColorf(1.0, 1.0, 1.0),
            m_font_name, m_font_size, // font name & size
            mrpt::opengl::NICE,
            /* unique_index = */ m_text_index_timestamp );
      }
      else {
        m_win->addTextMessage(5,-m_offset_y_timestamp, 
            format("Wall time: %s", timeLocalToString(system::now()).c_str()),
            TColorf(1.0, 1.0, 1.0),
            m_font_name, m_font_size, // font name & size
            mrpt::opengl::NICE,
            /* unique_index = */ m_text_index_timestamp );
      }

      // add to the odometry PointCloud and visualize it
      {
        pose_t* odometry_poses = new pose_t;
        *odometry_poses = curr_odometry_only_pose;
        m_odometry_poses.push_back(odometry_poses);

        if (m_visualize_odometry_poses) {
          COpenGLScenePtr scene = m_win->get3DSceneAndLock();

          CRenderizablePtr obj = scene->getByName("odometry_poses_cloud");
          CPointCloudPtr odometry_poses_cloud = static_cast<CPointCloudPtr>(obj);
          odometry_poses_cloud->insertPoint(
              m_odometry_poses.back()->x(),
              m_odometry_poses.back()->y(),
              0 );

          m_win->unlockAccess3DScene();

          m_win->forceRepaint();
        }

        delete odometry_poses;
      }

      /** 
       * Fixed intervals odometry edge insertion
       * Determine whether to insert a new pose in the graph given the
       * distance and angle thresholds
       */
      //if ( (last_pose_inserted.distanceTo(curr_odometry_only_pose) > m_distance_threshold) ||
          //fabs(wrapToPi(last_pose_inserted.phi() - curr_odometry_only_pose.phi())) > m_angle_threshold ) {

      // update the current estimated pose of the robot
      m_curr_estimated_pose = m_graph.nodes[from] + since_prev_node_PDF.getMeanVal();
      if ( (last_pose_inserted.distanceTo(m_curr_estimated_pose) > m_distance_threshold) ||
          fabs(wrapToPi(last_pose_inserted.phi() - m_curr_estimated_pose.phi())) > m_angle_threshold ) {
          
        from = m_nodeID_max;
        TNodeID to = ++m_nodeID_max;

        // build the relative edge and insert it
        {
          //constraint_t rel_edge;

          //m_graph.nodes[to] = curr_odometry_only_pose;
          m_graph.nodes[to] = m_graph.nodes[from] + since_prev_node_PDF.getMeanVal();
          m_graph.insertEdgeAtEnd(from, to, since_prev_node_PDF);
          m_edge_counter.addEdge("Odometry");

          //TODO - remove these
          //double distance = (last_pose_inserted.distanceTo(curr_odometry_only_pose));
          //double angle = fabs(last_pose_inserted.phi() - curr_odometry_only_pose.phi());
          //VERBOSE_COUT << "Added new odometry edge to the graph (# " << m_nodeID_max + 1<< " )" << endl;
          //VERBOSE_COUT << "prev_pose = " << last_pose_inserted << endl;
          //VERBOSE_COUT << "new_pose  = " << curr_odometry_only_pose << endl;
          //VERBOSE_COUT << "distance: " << distance << "m | " 
          //<< "angle: " << RAD2DEG(wrapToPi(angle)) << " deg" << endl;
          //VERBOSE_COUT << "Relative uncertainty between nodes: " << endl
          //<< since_prev_node_PDF.cov_inv << endl;

        }

        /**
         * add ICP constraint with "some" of the previous nodes
         */
        cout << endl;
        cout << "Testing ICP for node: " << m_nodeID_max << endl;
 
        CObservation2DRangeScanPtr curr_laser_scan =
          observations->getObservationByClass<CObservation2DRangeScan>();
        m_nodes_to_laser_scans[m_nodeID_max] = curr_laser_scan;
       
        //If m_prev_nodes_for_ICP = -1 try adding with all of the other nodes
        int nodes_to_check = 0; // how many nodes to check ICP against
        switch ( m_prev_nodes_for_ICP ) {
          case -1: {
            nodes_to_check = m_graph.nodeCount() - 1;
            cout << "Checking ICP against all other nodes in the graph." << endl;
            break;
          }
          default: {
            nodes_to_check = m_prev_nodes_for_ICP;
            cout << "Checking ICP against " << nodes_to_check << " nodes in the graph." << endl;
            break;
          }
        }

        for (TNodeID prev_node = m_nodeID_max-1; 
            prev_node != (m_nodeID_max - nodes_to_check - 1) ; --prev_node) {

          // make sure you have enough nodes to start adding edges
          if (m_nodeID_max < nodes_to_check)
            break;

          CObservation2DRangeScanPtr prev_laser_scan = m_nodes_to_laser_scans[prev_node]; 
          constraint_t rel_edge;
          double ICP_goodness = this->getICPEdge(prev_node, to, &rel_edge);
          if (ICP_goodness > m_ICP_goodness_thres) {
            VERBOSE_COUT << "Adding ICP constraints betwen nodes " 
                        << to << ", " << prev_node << "." << endl;
            VERBOSE_COUT << "Adding edge: " << endl << rel_edge << endl;
            m_graph.insertEdge(prev_node, to, rel_edge);
            VERBOSE_COUT << "ICP goodness: " << ICP_goodness << endl;

            // register a loop closing constraint if distance of nodes larger
            // than the predefined m_loop_closing_min_nodeid_diff 
            bool is_loop_closure = (to - prev_node >= m_loop_closing_min_nodeid_diff) ? 1 : 0;
            m_edge_counter.addEdge("ICP", is_loop_closure);
          }
        }

        // optimize the graph - LM
        this->optimizeGraph(m_graph);


        // update the visualization window
        if (m_visualize_optimized_graph) {
          visualizeGraph(m_graph);
          updateCurPosViewport(m_graph);
        }

        // reset the relative PDF
        since_prev_node_PDF.cov_inv = init_path_uncertainty;
        since_prev_node_PDF.mean = pose_t(0, 0, 0);
        last_pose_inserted = m_graph.nodes[to];



      } // IF ODOMETRY_CRITERIUM
    } // ELSE FORMAT #1
  } // WHILE CRAWLOG FILE
MRPT_END
} // END OF FUNCTION

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::visualizeGraph(const GRAPH_t& gr) {
  
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
      format("Optimized Graph: #nodes %d", 
        static_cast<int>(gr.nodeCount())),
      TColorf(0.0, 0.0, 0.0),
      m_font_name, m_font_size, // font name & size
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
void GraphSlamEngine_t<GRAPH_t>::readConfigFile(const string& fname) {
  MRPT_START

  CConfigFile cfg_file(fname);

  // Section: GeneralConfiguration 
  // ////////////////////////////////
  m_rawlog_fname = cfg_file.read_string(
      /*section_name = */ "MappingApplication", 
      /*var_name = */ "rawlog_file",
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
  m_loop_closing_min_nodeid_diff = cfg_file.read_double(
      "LoopClosingParameters",
      "loop_closing_min_nodeid_diff",
      5, true);

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
  
  //// Section: MappingApplication
  //// ////////////////////////////////
	//const unsigned int rawlog_offset		 = iniFile.read_int("MappingApplication","rawlog_offset",0,  [>Force existence:<] true);
	//const string OUT_DIR_STD			 = iniFile.read_string("MappingApplication","logOutput_dir","log_out",  [>Force existence:<] true);
	//const int LOG_FREQUENCY		 = iniFile.read_int("MappingApplication","LOG_FREQUENCY",5,  [>Force existence:<] true);
	//const bool  SAVE_POSE_LOG		 = iniFile.read_bool("MappingApplication","SAVE_POSE_LOG", false,  [>Force existence:<] true);
	//const bool  SAVE_3D_SCENE        = iniFile.read_bool("MappingApplication","SAVE_3D_SCENE", false,  [>Force existence:<] true);
	//const bool  CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication","CAMERA_3DSCENE_FOLLOWS_ROBOT", true,  [>Force existence:<] true);
  //mapBuilder.ICP_options.loadFromConfigFile( iniFile, "MappingApplication");

  //// Section: ICP
  //// ////////////////////////////////
  //mapBuilder.ICP_params.loadFromConfigFile ( iniFile, "ICP");
  m_ICP.options.loadFromConfigFile(cfg_file, "ICP");

  m_prev_nodes_for_ICP = cfg_file.read_int(
      "ICP",
      "prev_nodes_for_ICP",
      3, false);
  m_ICP_goodness_thres = cfg_file.read_double(
      "ICP",
      "goodness_to_accept",
      0.80, false);



  // Section: VisualizationParameters
  // ////////////////////////////////
  // http://reference.mrpt.org/devel/group__mrpt__opengl__grp.html#ga30efc9f6fcb49801e989d174e0f65a61

  m_font_name = cfg_file.read_string(
      "VisualizationParameters",
      "font_name",
      "sans", false);
  m_font_size = cfg_file.read_int(
      "VisualizationParameters",
      "font_size",
      12, false);


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
	m_optimized_graph_viz_params["show_ground_grid"] = cfg_file.read_double(
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
  
  m_visualize_odometry_poses = cfg_file.read_bool(
      "VisualizationParameters",
      "visualize_odometry_poses",
      1, false);


	m_has_read_config = true;
	MRPT_END
}

// TODO - add the new values from visualization part
template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::printProblemParams() const {
  MRPT_START

  assert(m_has_read_config);

  stringstream ss_out;

  ss_out << "--------------------------------------------------------------------------" << endl;
  ss_out << " Graphslam_engine: Problem Parameters " << endl;
  ss_out << " Config fname                    = " << m_config_fname << endl;
  ss_out << " Rawlog fname                    = " << m_rawlog_fname << endl;
  ss_out << " Output dir                      = " << m_output_dir_fname << endl;
  ss_out << " User decides about output dir?  = " << m_user_decides_about_output_dir << endl;
  ss_out << " Debug mode                      = " << m_do_debug << endl;
  ss_out << " save_graph_fname                = " << m_save_graph_fname << endl;
  ss_out << " do_pose_graph_only              = " <<  m_do_pose_graph_only << endl;
  ss_out << " optimizer                       = " << m_optimizer << endl;
  ss_out << " Loop closing alg                = " << m_loop_closing_alg << endl;
  ss_out << " Loop Closing min. node distance = " << m_loop_closing_alg << endl;
  ss_out << " Decider alg                     = " << m_decider_alg << endl;
  ss_out << " Distance threshold              = " << m_distance_threshold << " m" << endl;
  ss_out << " Angle threshold                 = " << RAD2DEG(m_angle_threshold) << " deg" << endl;
  ss_out << "ICP Goodness threshold           = " << m_ICP_goodness_thres << endl;
  ss_out << "-------------------------------------------------------------------------" << endl;
  ss_out << endl;

  cout << ss_out.str(); ss_out.str("");

  //m_optimized_graph_viz_params.dumpToConsole();

  m_ICP.options.dumpToConsole();

  ss_out << "-------------------------------------------------------------------------" << endl;
  cout << ss_out.str(); ss_out.str("");

	//map_builder.ICP_params.dumpToConsole();
	//map_builder.ICP_options.dumpToConsole();


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
      } // SWITCH (ANSWER_INT)
    } // IF DIRECTORY EXISTS.. 

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
} // end of initOutputDir

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

template <class GRAPH_t>
double GraphSlamEngine_t<GRAPH_t>::getICPEdge(const TNodeID& from,
    const TNodeID& to,
    constraint_t *rel_edge ) {
  MRPT_START

    assert(m_has_read_config);

  // get the laser scan measurements
  CObservation2DRangeScanPtr prev_laser_scan = m_nodes_to_laser_scans[from];
  CObservation2DRangeScanPtr curr_laser_scan = m_nodes_to_laser_scans[to];

  // Uses m_ICP member variable
  CSimplePointsMap m1,m2;
  float running_time;
  CICP::TReturnInfo info;

  // use the difference of the node positions as an initial alignment
  // estimation
  m_graph.dijkstra_nodes_estimate();
  pose_t initial_pose = m_graph.nodes[to] - m_graph.nodes[from];

  //CPose2D initial_pose(0.0f,0.0f,(float)DEG2RAD(0.0f));


  m1.insertObservation(&(*prev_laser_scan));
  m2.insertObservation(&(*curr_laser_scan));

  CPosePDFPtr pdf = m_ICP.Align(
      &m1,
      &m2,
      initial_pose, 
      &running_time,
      (void*)&info);

  //VERBOSE_COUT << "getICPEdge: goodness: " 
  //<< info.goodness << endl;
  //VERBOSE_COUT << "initial estimate: " << endl 
  //<< initial_pose << endl;
  //VERBOSE_COUT << "relative edge: " << endl
  //<< rel_edge->getMeanVal() << endl;
  rel_edge->copyFrom(*pdf);  // return the edge regardless of the goodness of the alignment
  return info.goodness;

  MRPT_END
}

/**
 * updateCurPosViewport
 *
 * Udpate the viewport responsible for displaying the graph-building procedure
 * in the estimated position of the robot
 */
template <class GRAPH_t>
inline void GraphSlamEngine_t<GRAPH_t>::updateCurPosViewport(const GRAPH_t& gr) {
  MRPT_START

  pose_t curr_robot_pose = gr.nodes.find(gr.nodeCount()-1)->second; // get the last added pose
  
  COpenGLScenePtr &scene = m_win->get3DSceneAndLock();

  COpenGLViewportPtr viewp= scene->createViewport("current_pos");
  // Add a clone viewport, using [0,1] factor X,Y,Width,Height coordinates:
  viewp->setViewportPosition(0.78,0.78,0.20,0.20);
  viewp->setCloneView("main");
  viewp->getCamera().setPointingAt(CPose3D(curr_robot_pose));
  viewp->getCamera().setAzimuthDegrees(90);
  viewp->getCamera().setElevationDegrees(90);
  viewp->setTransparent(false);
  viewp->setCustomBackgroundColor(TColorf(TColor(205, 193, 197)));
  viewp->getCamera().setZoomDistance(40);

  m_win->unlockAccess3DScene();
  m_win->forceRepaint();
  //VERBOSE_COUT << "Updated the \"current_pos\" viewport" << endl;

  MRPT_END
}

