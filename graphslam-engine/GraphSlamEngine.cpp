#include "GraphSlamEngine.h"

#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/mrpt_stdint.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam.h>
#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // It's in the lib mrpt-maps now
#include <mrpt/opengl/graph_tools.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>

#include <string>
#include <sstream>
#include <map>
#include <set>
#include <algorithm>
#include <cerrno>
#include <cmath> // fabs function
#include <cstdlib>

#include "supplementary_funs.h"
#include "CWindowObserver.h"

using namespace mrpt;
using namespace mrpt::synch;
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

using namespace supplementary_funs;

using namespace std;

// Ctors, Dtors implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
GraphSlamEngine_t<GRAPH_t>::GraphSlamEngine_t(const string& config_file,
		CDisplayWindow3D* win /* = NULL */,
		CWindowObserver* win_observer /* = NULL */,
		string rawlog_fname /* = "" */ ):
	m_config_fname(config_file),
	m_win(win),
	m_win_observer(win_observer),
	m_rawlog_fname(rawlog_fname),
	kOffsetYStep(20.0), // textMessage vertical text position
	kIndexTextStep(1), // textMessage index
	m_odometry_color(0, 0, 1),
	m_GT_color(0, 1, 0)
{
	//m_win = win;
	//m_rawlog_fname = rawlog_fname;
	//m_config_fname = config_file;
	
	this->initGraphSlamEngine();
};

template<class GRAPH_t>
GraphSlamEngine_t<GRAPH_t>::~GraphSlamEngine_t() {
	VERBOSE_COUT << "In Destructor: Deleting GraphSlamEngine_t instance..." << endl;

	// close all open files
	for (fstreams_out_it it  = m_out_streams.begin(); it != m_out_streams.end(); 
			++it) {
		if ((it->second)->fileOpenCorrectly()) {
			VERBOSE_COUT << "Closing file: " << (it->first).c_str() << endl;
			(it->second)->close();
		}
	}
	for (fstreams_in_it it  = m_in_streams.begin(); it != m_in_streams.end(); 
			++it) {
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
	 * Parameters validation
	 */
	assert(!(!m_win && m_win_observer) && 
			"CObsever was provided even though no CDisplayWindow3D was not");

	// Calling of initalization-relevant functions
	this->readConfigFile(m_config_fname);
	this->initOutputDir();
	this->printProblemParams();
	
	/**
	 * Initialization of various member variables
	 */

	// check for duplicated edges every..
	m_num_of_edges_for_collapse = 100;

	m_is3D = constraint_t::is_3D_val;


	// initialize the necessary maps for graph information
	graph_to_name[&m_graph] = "optimized_graph";
	graph_to_viz_params[&m_graph] = &m_optimized_graph_viz_params;


	// max node number already in the graph
	m_nodeID_max = 0;
	m_graph.root = TNodeID(0);


	/**
	 * Visualization-related parameters initialization
	 */

	if (!m_win) {
		m_visualize_optimized_graph = 0;
		m_visualize_odometry_poses = 0;
		m_visualize_GT = 0;
	}

	// Current Text Position
	m_curr_offset_y = 30.0;
	m_curr_text_index = 1;

	// timestamp
	this->assignTextMessageParameters(&m_offset_y_timestamp,
			&m_text_index_timestamp);

	// optimized graph
	assert(m_has_read_config);
	if (m_visualize_optimized_graph) {
		this->assignTextMessageParameters( /* offset_y*		= */ &m_offset_y_graph,
				/* text_index* = */ &m_text_index_graph );
	}


	// odometry visualization
	assert(m_has_read_config);
	if (m_visualize_odometry_poses) {
		assert(m_win && 
				"Visualization of data was requested but no CDisplayWindow3D pointer was given");

		this->assignTextMessageParameters( /* offset_y*		= */ &m_offset_y_odometry,
				/* text_index* = */ &m_text_index_odometry);

		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CPointCloudPtr odometry_poses_cloud = CPointCloud::Create();
		scene->insert(odometry_poses_cloud);

		odometry_poses_cloud->setPointSize(2.0);
		odometry_poses_cloud->enablePointSmooth();
		odometry_poses_cloud->enableColorFromX(false);
		odometry_poses_cloud->enableColorFromY(false);
		odometry_poses_cloud->enableColorFromZ(false);
		odometry_poses_cloud->setColor(m_odometry_color);
		odometry_poses_cloud->setName("odometry_poses_cloud");

		m_win->unlockAccess3DScene();

		m_win->addTextMessage(5,-m_offset_y_odometry,
				format("Odometry path"),
				m_odometry_color,
				m_font_name, m_font_size, // font name & size
				mrpt::opengl::NICE,
				/* unique_index = */ m_text_index_odometry );

		m_win->forceRepaint();
	}

	// GT visualization
	assert(m_has_read_config);
	if (m_visualize_GT) {
		assert(m_win && 
				"Visualization of data was requested but no CDisplayWindow3D pointer was given");

		if (m_fname_GT.empty()) {
			THROW_EXCEPTION("Visualization of Ground Truth is TRUE, but no ground"
					<< " truth file was specified") 
		}
		this->BuildGroundTruthMap(m_fname_GT);

		this->assignTextMessageParameters( /* offset_y*		= */ &m_offset_y_GT,
				/* text_index* = */ &m_text_index_GT);

		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CPointCloudPtr GT_cloud = CPointCloud::Create();
		scene->insert(GT_cloud);

		GT_cloud->setPointSize(2.0);
		GT_cloud->enablePointSmooth();
		GT_cloud->enableColorFromX(false);
		GT_cloud->enableColorFromY(false);
		GT_cloud->enableColorFromZ(false);
		GT_cloud->setColor(m_GT_color);
		GT_cloud->setName("GT_cloud");

		m_win->unlockAccess3DScene();

		m_win->addTextMessage(5,-m_offset_y_GT,
				format("Ground truth path"),
				m_GT_color,
				m_font_name, m_font_size, // font name & size
				mrpt::opengl::NICE,
				/* unique_index = */ m_text_index_GT );

		m_win->forceRepaint();
	}

	// second viewport
	if (m_win) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();
		COpenGLViewportPtr viewp= scene->createViewport("curr_robot_pose_viewport");
		// Add a clone viewport, using [0,1] factor X,Y,Width,Height coordinates:
		viewp->setCloneView("main");
		viewp->setViewportPosition(0.78,0.78,0.20,0.20);
		viewp->setTransparent(false);
		viewp->getCamera().setAzimuthDegrees(90);
		viewp->getCamera().setElevationDegrees(90);
		viewp->setCustomBackgroundColor(TColorf(205, 193, 197, /*alpha = */ 255));
		viewp->getCamera().setZoomDistance(30);
		viewp->getCamera().setOrthogonal();

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();

	}
	
	// axis
	if (m_win) {
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

	// robot model
	if (m_win) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		opengl::CSetOfObjectsPtr obj = stock_objects::RobotPioneer();
		pose_t initial_pose;
		obj->setPose(initial_pose);
		obj->setName("robot_model");
		obj->setColor_u8(TColor(142, 142, 56));
		obj->setScale(5);
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	// ICP_max_distance disk
	assert(m_has_read_config);
	if (m_win && m_ICP_use_distance_criterion && m_ICP_max_distance > 0) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CDiskPtr obj = CDisk::Create();
		pose_t initial_pose;
		obj->setPose(initial_pose);
		obj->setName("ICP_max_distance");
		obj->setColor_u8(TColor(142, 142, 56));
		obj->setDiskRadius(m_ICP_max_distance, m_ICP_max_distance-0.5);
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	if (m_win) {
		m_edge_counter.setVisualizationWindow(m_win);
	}

	// register the types of edges that are going to be displayed in the
	// visualization window
	{
		// total edges / loop closures
		double offset_y_total_edges, offset_y_loop_closures;
		int text_index_total_edges, text_index_loop_closures;

		this->assignTextMessageParameters(&offset_y_total_edges, &text_index_total_edges);
		//cout << "in GraphSlamEngine:	" << endl
		//<< "offset_y_total_edges: " << offset_y_total_edges << endl
		//<< "text_index_total_edges: " << text_index_total_edges << endl;

		// constraint types..

		const char* strings[] = {"odometry", "ICP", "Visual"};
		//vector<string> vec_strings(strings, strings + 3);

		// register all the edge types
		vector<string> vec_edge_types;
		vec_edge_types.push_back("Odometry"); m_edge_counter.addEdgeType("Odometry");
		vec_edge_types.push_back("ICP");			m_edge_counter.addEdgeType("ICP");
		vec_edge_types.push_back("Visual");		m_edge_counter.addEdgeType("Visual");

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
		//cout << "in GraphSlamEngine:	" << endl
		//<< "offset_y_loop_closures: " << offset_y_loop_closures << endl
		//<< "text_index_loop_closures: " << text_index_loop_closures <<endl;

		if (m_win) {
			// add all the parameters to the EdgeCounter_t object
			m_edge_counter.setTextMessageParams(name_to_offset_y, name_to_text_index,
					offset_y_total_edges, text_index_total_edges,
					offset_y_loop_closures, text_index_loop_closures,
					m_font_name, m_font_size);
		}
	}

	// various flags initialization
	m_autozoom_active = true;

	MRPT_END
}

template<class GRAPH_t>
bool GraphSlamEngine_t<GRAPH_t>::parseRawlogFile() {
	MRPT_START

	if (!m_has_read_config)
		THROW_EXCEPTION("Config file has not been provided yet.\nExiting...");
	if (!fileExists(m_rawlog_fname))
		THROW_EXCEPTION("parseRawlogFile: Inputted rawlog file ( "
				<< m_rawlog_fname << " ) not found");
 // good to go..

	/**
	 * Variables initialization
	 */

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

	size_t curr_GT_poses_index = 0;

	// Read the first rawlog pair / observation explicitly to register the laser
	// scan of the root node
	{
		CRawlog::getActionObservationPairOrObservation(
				rawlog_file,
				action,								// Possible out var: Action of a a pair action / obs
				observations,					// Possible out var: obs's of a pair actin		 / obs
				observation,					// Possible out var
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
				action,								// Possible out var: Action of a a pair action / obs
				observations,					// Possible out var: obs's of a pair actin		 / obs
				observation,					// Possible out var
				curr_rawlog_entry ) ) {

		// process action & observations
		if (observation.present()) {
			// Read a single observation from the rawlog
			// (Format #2 rawlog file)
			//TODO Implement 2nd format
			THROW_EXCEPTION("Observation-only format is not yet supported");
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
			 * Use the dataset timestamp otherwise fallback to mrpt::system::now()
			 */
			TTimeStamp	timestamp = robot_move->timestamp;
			if (timestamp != INVALID_TIMESTAMP && m_win) {
				m_win->addTextMessage(5,-m_offset_y_timestamp,
						format("Simulated time: %s", timeLocalToString(timestamp).c_str()),
						TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ m_text_index_timestamp );
			}
			else if (m_win) {
				m_win->addTextMessage(5,-m_offset_y_timestamp,
						format("Wall time: %s", timeLocalToString(system::now()).c_str()),
						TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ m_text_index_timestamp );
			}

			// add to the odometry PointCloud and visualize it
			{
				// fill in the odometry_poses vecotr
				pose_t* odometry_pose = new pose_t;
				*odometry_pose = curr_odometry_only_pose;
				m_odometry_poses.push_back(odometry_pose);

				if (m_visualize_odometry_poses) {
					assert(m_win && 
							"Visualization of data was requested but no CDisplayWindow3D pointer was given");

					COpenGLScenePtr scene = m_win->get3DSceneAndLock();

					CRenderizablePtr obj = scene->getByName("odometry_poses_cloud");
					CPointCloudPtr odometry_poses_cloud = static_cast<CPointCloudPtr>(obj);
					odometry_poses_cloud->insertPoint(m_odometry_poses.back()->x(),
																						m_odometry_poses.back()->y(),
																						0 );

					m_win->unlockAccess3DScene();
					m_win->forceRepaint();
				}

				delete odometry_pose;
			}

			// add to the GT PointCloud and visualize it
			// check that GT vector is not depleted
			if (m_visualize_GT && 
					curr_GT_poses_index < m_GT_poses.size()) {
				assert(m_win &&
						"Visualization of data was requested but no CDisplayWindow3D pointer was given");

				COpenGLScenePtr scene = m_win->get3DSceneAndLock();

				CRenderizablePtr obj = scene->getByName("GT_cloud");
				CPointCloudPtr GT_cloud = static_cast<CPointCloudPtr>(obj);

				// get current point by matching the timestamp
				pose_t* curr_pose = m_GT_poses[curr_GT_poses_index++];
				GT_cloud->insertPoint(curr_pose->x(),
															curr_pose->y(),
															0 );

				m_win->unlockAccess3DScene();
				m_win->forceRepaint();
			}

			/**
			 * Fixed intervals odometry edge insertion
			 * Determine whether to insert a new pose in the graph given the
			 * distance and angle thresholds
			 */

			// update the current estimated pose of the robot
			{
				CCriticalSectionLocker m_graph_lock(&m_graph_section);
				
				m_curr_estimated_pose = m_graph.nodes[from] + since_prev_node_PDF.getMeanVal();
			}
			// odometry criterion
			if ( (last_pose_inserted.distanceTo(m_curr_estimated_pose) > m_distance_threshold) ||
					fabs(wrapToPi(last_pose_inserted.phi() - m_curr_estimated_pose.phi())) > m_angle_threshold ) {
				
				from = m_nodeID_max;
				TNodeID to = ++m_nodeID_max;

				// build the relative edge and insert it
				{
					//constraint_t rel_edge;

					{
						CCriticalSectionLocker m_graph_lock(&m_graph_section);

						m_graph.nodes[to] = m_graph.nodes[from] + since_prev_node_PDF.getMeanVal();
						m_graph.insertEdgeAtEnd(from, to, since_prev_node_PDF);
						m_edge_counter.addEdge("Odometry");
					}
				}

				/**
				 * add ICP constraint with "some" of the previous nodes
				 */
				CObservation2DRangeScanPtr curr_laser_scan =
					observations->getObservationByClass<CObservation2DRangeScan>();
				m_nodes_to_laser_scans[m_nodeID_max] = curr_laser_scan;

				// build the list of nodes against which we check possible ICP
				// constraint
				std::set<TNodeID> nodes_to_check_ICP;
				{
					CCriticalSectionLocker m_graph_lock(&m_graph_section);

					// call it once every iteration
					m_graph.dijkstra_nodes_estimate(); 
				}
				if (m_ICP_use_distance_criterion) {
					this->getNearbyNodesOf(&nodes_to_check_ICP,
							m_nodeID_max,
							&m_ICP_max_distance,
							m_ICP_use_distance_criterion);
				}
				else {
					this->getNearbyNodesOf(&nodes_to_check_ICP,
							m_nodeID_max,
							&m_ICP_prev_nodes,
							m_ICP_use_distance_criterion);
				}

				for (set<TNodeID>::const_iterator node_it = nodes_to_check_ICP.begin();
						node_it != nodes_to_check_ICP.end(); ++node_it) {

					CObservation2DRangeScanPtr prev_laser_scan = m_nodes_to_laser_scans[*node_it];
					constraint_t rel_edge;
					double ICP_goodness = this->getICPEdge(*node_it, to, &rel_edge);
					if (ICP_goodness > m_ICP_goodness_thres) {
						{
							CCriticalSectionLocker m_graph_lock(&m_graph_section);
							m_graph.insertEdge(*node_it, to, rel_edge);

							//VERBOSE_COUT << "Added ICP constraint betwen nodes "
								//<< to << ", " << *node_it << "." << endl;
							//VERBOSE_COUT << "Edge: " << endl << rel_edge << endl;
							//VERBOSE_COUT << "ICP goodness: " << ICP_goodness << endl;
						}

						// register a loop closing constraint if distance of nodes larger
						// than the predefined m_loop_closing_min_nodeid_diff
						bool is_loop_closure = (to - *node_it >= m_loop_closing_min_nodeid_diff) ? 1 : 0;
						m_edge_counter.addEdge("ICP", is_loop_closure);
					}
				}

				// join the previous optimization thread
				joinThread(m_thread_optimize);

				// optimize the graph - run on a seperate thread
				m_thread_optimize = createThreadFromObjectMethod(/*obj = */this, 
						/* func = */&GraphSlamEngine_t::optimizeGraph, &m_graph);

				// update the visualization window
				if (m_visualize_optimized_graph) {
					assert(m_win && 
							"Visualization of data was requested but no CDisplayWindow3D pointer was given");

					visualizeGraph(m_graph);
					updateCurPosViewport(m_graph);
				}


				// update robot model
				if (m_win) {
					COpenGLScenePtr scene = m_win->get3DSceneAndLock();

					CRenderizablePtr obj = scene->getByName("robot_model");
					CSetOfObjectsPtr robot_obj = static_cast<CSetOfObjectsPtr>(obj);

					CCriticalSectionLocker m_graph_lock(&m_graph_section);

					robot_obj->setPose(m_graph.nodes[m_graph.nodeCount()-1]);

					m_win->unlockAccess3DScene();
				}
				// update ICP_max_distance Disk
				if (m_win && m_ICP_use_distance_criterion && m_ICP_max_distance > 0) {
					COpenGLScenePtr scene = m_win->get3DSceneAndLock();

					CRenderizablePtr obj = scene->getByName("ICP_max_distance");
					CDiskPtr disk_obj = static_cast<CDiskPtr>(obj);

					CCriticalSectionLocker m_graph_lock(&m_graph_section);

					disk_obj->setPose(m_graph.nodes[m_graph.nodeCount()-1]);

					m_win->unlockAccess3DScene();
				}


				// reset the relative PDF
				{
					CCriticalSectionLocker m_graph_lock(&m_graph_section);

					since_prev_node_PDF.cov_inv = init_path_uncertainty;
					since_prev_node_PDF.mean = pose_t(0, 0, 0);
					last_pose_inserted = m_graph.nodes[to];
				}



			} // IF ODOMETRY_CRITERION
		} // ELSE FORMAT #1


		/** 
		 * query for events and take coresponding actions
		 */
		this->queryObserverForEvents();

		if (m_request_to_exit) {
			cout << "Halting execution... " << endl;
			joinThread(m_thread_optimize);
			return false; // exit the parseRawlogFile method
		}

		/**
		 * Reduce edges
		 */
		if (m_edge_counter.getTotalNumOfEdges() % m_num_of_edges_for_collapse == 0) {
			CCriticalSectionLocker m_graph_lock(&m_graph_section);
			//cout << "Collapsing duplicated edges..." << endl;

			int removed_edges = m_graph.collapseDuplicatedEdges();
			m_edge_counter.setRemovedEdges(removed_edges);
		}



	} // WHILE CRAWLOG FILE
	return true; // function execution completed successfully
MRPT_END
} // END OF FUNCTION

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::optimizeGraph(GRAPH_t* gr) {
	MRPT_START

	CTicTac optimization_timer;
	optimization_timer.Tic();

	CCriticalSectionLocker m_graph_lock(&m_graph_section);

	//cout << "In optimizeGraph: threadID: " << getCurrentThreadId()<< endl;

	//cout << "Executing the graph optimization" << endl;
	graphslam::TResultInfoSpaLevMarq	levmarq_info;

	// Execute the optimization
	graphslam::optimize_graph_spa_levmarq(
			*gr,
			levmarq_info,
			NULL,  // List of nodes to optimize. NULL -> all but the root node.
			m_optimization_params,
			&levMarqFeedback<GRAPH_t>); // functor feedback

	double elapsed_time = optimization_timer.Tac();
	//VERBOSE_COUT << "Optimization of graph took: " << elapsed_time << "s" << endl;

	MRPT_END
}

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::visualizeGraph(const GRAPH_t& gr) {
	MRPT_START

	assert(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	//cout << "Inside the visualizeGraph function" << endl;
	
	CCriticalSectionLocker m_graph_lock(&m_graph_section);

	string gr_name = graph_to_name[&gr];
	const TParametersDouble* viz_params = graph_to_viz_params[&gr];

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

	// Autozoom to the graph?
	if (m_autozoom_active) {
		this->autofitObjectInView(graph_obj);
	}

	MRPT_END
}

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::readConfigFile(const string& fname) {
	MRPT_START

	VERBOSE_COUT << "Reading the .ini file... " << endl;

	CConfigFile cfg_file(fname);

	if (m_rawlog_fname.empty()) {
		m_rawlog_fname = cfg_file.read_string(
				/*section_name = */ "MappingApplication",
				/*var_name = */ "rawlog_file",
				/*default_value = */ "", /*failIfNotFound = */ true);
	}


	// Section: GeneralConfiguration
	// ////////////////////////////////
	m_output_dir_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"output_dir_fname",
			"graphslam_engine_results", false);
	m_user_decides_about_output_dir = cfg_file.read_bool(
			"GeneralConfiguration",
			"user_decides_about_output_dir",
			true, false);
	m_do_debug =	cfg_file.read_bool(
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

	// Section: Optimization
	// ////////////////////////////
	m_optimization_params["verbose"] = cfg_file.read_bool(
			"Optimization",
			"verbose",
			0, false);
	m_optimization_params["profiler"] = cfg_file.read_bool(
			"Optimization",
			"profiler",
			0, false);
		m_optimization_params["max_iterations"] = cfg_file.read_double(
			"Optimization",
			"max_iterations",
			100, false);
	m_optimization_params["scale_hessian"] = cfg_file.read_double(
			"Optimization",
			"scale_hessian",
			0.2, false);
	m_optimization_params["tau"] = cfg_file.read_double(
			"Optimization",
			"tau",
			1e-3, false);


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

	//// Section: ICP
	//// ////////////////////////////////
	//mapBuilder.ICP_params.loadFromConfigFile ( iniFile, "ICP");
	m_ICP.options.loadFromConfigFile(cfg_file, "ICP");

	/** 
	 * Criterion for building the list of nodes for ICP checks. Uses either
	 * distance from current node or number of nodes prior/after current node
	 * If none, or both of ICP_max_distance, ICP_prev_nodes parameters are
	 * defined in the .ini file, throw exception
	 */
	bool ICP_max_distance_exists = false;
	bool ICP_prev_nodes_exists = false;
	vector<string> keys;
	cfg_file.getAllKeys("ICP", keys);
	if (std::find(keys.begin(), keys.end(), "ICP_max_distance") != keys.end()) {
		m_ICP_max_distance = cfg_file.read_double(
				"ICP",
				"ICP_max_distance",
				5, false);
		ICP_max_distance_exists = true;
	}
	if (std::find(keys.begin(), keys.end(), "ICP_prev_nodes") != keys.end()) {
		m_ICP_prev_nodes = cfg_file.read_int(
				"ICP",
				"ICP_prev_nodes",
				5, false);
		ICP_prev_nodes_exists = true;
	}

	if (ICP_max_distance_exists && ICP_prev_nodes_exists ||
			!ICP_max_distance_exists && !ICP_prev_nodes_exists) {
		THROW_EXCEPTION("User should specify either the ICP_prev_nodes"
				<< "or the ICP_max distance parameter")
	}
	m_ICP_use_distance_criterion = ICP_max_distance_exists;

	m_ICP_goodness_thres = cfg_file.read_double(
			"ICP",
			"ICP_goodness_thres",
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

	// GT configuration / visualization
	m_visualize_GT = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_ground_truth",
			1, false);
	m_fname_GT = cfg_file.read_string(
			"VisualizationParameters",
			"ground_truth_file",
			"", false);


	m_has_read_config = true;
	MRPT_END
}

// TODO - check for new value from above
template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::printProblemParams() const {
	MRPT_START

	assert(m_has_read_config);

	stringstream ss_out;

	ss_out << "-----------------------------------------------------------" << endl;
	ss_out << " Graphslam_engine: Problem Parameters " << endl;
	ss_out << " Config filename                 = "
		<< m_config_fname << endl;
	ss_out << " Rawlog filename                 = "
		<< m_rawlog_fname << endl;
	ss_out << " Output directory                = "
		<< m_output_dir_fname << endl;
	ss_out << " User decides about output dir   = "
		<< m_user_decides_about_output_dir << endl;
	ss_out << " Debug mode                      = "
		<< m_do_debug << endl;
	ss_out << " save_graph_fname                = "
		<< m_save_graph_fname << endl;
	ss_out << " do_pose_graph_only              = "
		<<	m_do_pose_graph_only << endl;
	ss_out << " optimizer                       = "
		<< m_optimizer << endl;
	ss_out << " Loop closing alg                = "
		<< m_loop_closing_alg << endl;
	ss_out << " Loop Closing min. node distance = "
		<< m_loop_closing_alg << endl;
	ss_out << " Decider alg                     = "
		<< m_decider_alg << endl;
	ss_out << " Distance threshold              = "
		<< m_distance_threshold << " m" << endl;
	ss_out << " Angle threshold                 = "
		<< RAD2DEG(m_angle_threshold) << " deg" << endl;
	ss_out << "ICP Goodness threshold           = "
		<< m_ICP_goodness_thres << endl;
	if (m_ICP_use_distance_criterion) {
		ss_out << "Maximum distance for ICP check  = "
			<< m_ICP_max_distance << endl;
	}
	else {
		ss_out << "Num of previous nodes  for ICP  = "
			<< m_ICP_prev_nodes << endl;
	}
	ss_out << "Visualize odometry               = " 
		<< m_visualize_odometry_poses << endl;
	ss_out << "Visualize optimized path         = " 
		<< m_visualize_odometry_poses << endl;
	ss_out << "Visualize Ground Truth           = " 
		<< m_visualize_odometry_poses << endl;
	ss_out << "Ground Truth filename            = "
		<< m_fname_GT << endl;
	ss_out << "-----------------------------------------------------------" << endl;
	ss_out << endl;

	cout << ss_out.str(); ss_out.str("");

	m_ICP.options.dumpToConsole();
	cout << "-----------[ Optimization Parameters ]-----------" << endl;
	m_optimization_params.dumpToConsole();
	cout << "-----------[ Graph Visualization Parameters ]-----------" << endl;
	m_optimized_graph_viz_params.dumpToConsole();

	ss_out << "-----------------------------------------------------------" << endl;
	cout << ss_out.str(); ss_out.str("");

	//mrpt::system::pause();

	MRPT_END
}

template<class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::initOutputDir() {
	MRPT_START

	VERBOSE_COUT << "Setting up Output directory: " << m_output_dir_fname << endl;

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
			switch (answer_int) 
			{
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

	VERBOSE_COUT << "Setting up file: " << fname << endl;

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
		THROW_EXCEPTION("Error while trying to open " <<	fname)
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

	pose_t initial_pose;
	{
		CCriticalSectionLocker m_graph_lock(&m_graph_section);

		// use the difference of the node positions as an initial alignment
		// estimation (dijkstra_nodes_estimate has to be run from the caller
		// function
		initial_pose = m_graph.nodes[to] - m_graph.nodes[from];
	}

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
	
	// return the edge regardless of the goodness of the alignment
	rel_edge->copyFrom(*pdf);  	return info.goodness;

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

	assert(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	pose_t curr_robot_pose;
	{
		CCriticalSectionLocker m_graph_lock(&m_graph_section);
		curr_robot_pose = gr.nodes.find(gr.nodeCount()-1)->second; // get the last added pose
	}

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	COpenGLViewportPtr viewp = scene->getViewport("curr_robot_pose_viewport");
	viewp->getCamera().setPointingAt(CPose3D(curr_robot_pose));

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	//VERBOSE_COUT << "Updated the \"current_pos\" viewport" << endl;

	MRPT_END
}

template <class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::BuildGroundTruthMap(const std::string& fname_GT) {
	MRPT_START

	VERBOSE_COUT << "Parsing the ground truth file textfile.." << endl;
	assert(fileExists(fname_GT) && "Ground truth file was not found.");

	// open the file, throw exception 
	CFileInputStream* file_GT = new CFileInputStream(fname_GT);
	m_in_streams[fname_GT] = file_GT;


	if (file_GT->fileOpenCorrectly()) {
		string curr_line; 
		size_t line_num = 0;

		// parse the file - get timestamp and pose and fill in the pose_t vector
		for (size_t line_num = 1; file_GT->readLine(curr_line); line_num++) {
			vector<string> curr_tokens;
			system::tokenize(curr_line, " ", curr_tokens);

			// check the current pose dimensions
			if (curr_tokens.size() != constraint_t::state_length + 1) {
				THROW_EXCEPTION("Wrong length of curent pose at line " << line_num);
			}
			pose_t *curr_pose = new pose_t(atof(curr_tokens[1].c_str()),
					atof(curr_tokens[2].c_str()),
					atof(curr_tokens[3].c_str()) );
			m_GT_poses.push_back(curr_pose);
		}
	}
	else {
		THROW_EXCEPTION("BuildGroundTruthMap: Can't open GT file (" << fname_GT
				<< ")")
	}

	MRPT_END
}

template <class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::autofitObjectInView(const CSetOfObjectsPtr& graph_obj) {
	MRPT_START

		
	assert(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	CGridPlaneXYPtr obj_grid = graph_obj->CSetOfObjects::getByClass<CGridPlaneXY>();
	if (obj_grid) {
		float x_min,x_max, y_min,y_max;
		obj_grid->getPlaneLimits(x_min,x_max, y_min,y_max);
		const float z_min = obj_grid->getPlaneZcoord();
		m_win->setCameraPointingToPoint( 0.5*(x_min+x_max), 0.5*(y_min+y_max), z_min );
		m_win->setCameraZoom( 2.0f * std::max(10.0f, std::max(x_max-x_min, y_max-y_min) ) );
	}
	m_win->setCameraAzimuthDeg(0);
	m_win->setCameraElevationDeg(75);

	MRPT_END
}

template <class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::queryObserverForEvents() {
	assert(m_win_observer &&
			"queryObserverForEvents method was called even though no Observer object was provided");

	const TParameters<bool>* events_occurred = m_win_observer->returnEventsStruct();
	m_autozoom_active = !(*events_occurred)["mouse_clicked"];
	m_request_to_exit = (*events_occurred)["request_to_exit"];

}

template <class GRAPH_t>
void GraphSlamEngine_t<GRAPH_t>::getNearbyNodesOf(set<TNodeID> *lstNodes, 
		const TNodeID& cur_nodeID,
		void* num_nodes_or_distance, 
		bool use_distance_criterion /* = true */) {

	CCriticalSectionLocker m_graph_lock(&m_graph_section);

	if (use_distance_criterion) {
		double* p_distance = static_cast<double*>(num_nodes_or_distance);

		if (*p_distance > 0) {
			for (TNodeID nodeID = 0; nodeID <= m_nodeID_max; ++nodeID) {
				double curr_distance = m_graph.nodes[nodeID].distanceTo(
						m_graph.nodes[cur_nodeID]);
				//cout << "Distance for ICP is: " << curr_distance << endl;
				if (curr_distance <= *p_distance) {
					lstNodes->insert(nodeID);
				}
			}
		}
		else { // check against all nodes 
			m_graph.getAllNodes(*lstNodes);
		}

	}
	else  {
		int* p_num_nodes = static_cast<int*>(num_nodes_or_distance);

		if (*p_num_nodes > 0) {
			// add nodes previous to the current 
			for (int a_node = cur_nodeID-1; // should be int, and not unsigned int as it may get < 0 
					(a_node != (cur_nodeID-1 - *p_num_nodes)) && (a_node >= 0); --a_node) {
				lstNodes->insert(a_node);
			}
			// add nodes after the current - if any in the graph
			if (m_graph.nodeCount()-1  > cur_nodeID) {
				for (TNodeID a_node = cur_nodeID+1;
						a_node < cur_nodeID+1 + *p_num_nodes && a_node <= m_graph.nodeCount()-1;
						a_node++) {
					cout << "Adding node after current!" << endl;
					lstNodes->insert(a_node);
				}
			}
		}
		else { // check against all nodes
			m_graph.getAllNodes(*lstNodes);
		}
	}
}

