/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPGoodnessERD_IMPL_H
#define CICPGoodnessERD_IMPL_H


using namespace mrpt::graphslam::deciders;

// Ctors, Dtors
// //////////////////////////////////

template<class GRAPH_t>
CICPGoodnessERD_t<GRAPH_t>::CICPGoodnessERD_t():
	m_search_disk_color(TColor(142, 142, 56)),
	m_consecutive_invalid_format_instances_thres(20), // large threshold just to make sure
	kConversionSensorLabel("RGBD_TO_2D_SCAN"), // 3D=>2D conversion parameter
	kConversionAngleSup(DEG2RAD(5)),           // 3D=>2D conversion parameter
	kConversionAngleInf(DEG2RAD(5)),           // 3D=>2D conversion parameter
	kConversionOversamplingRatio(1.2)          // 3D=>2D conversion parameter
{
	MRPT_START;

	this->initCICPGoodnessERD_t();

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::initCICPGoodnessERD_t() {
	MRPT_START;

	m_win = NULL;
	m_win_manager = NULL;
	m_graph = NULL;

	m_initialized_visuals = false;
	m_initialized_rgbd_viewports = false;
	m_just_inserted_loop_closure = false;
	m_contains_scans3D = false;

	// start ICP constraint registration only when 
	// nodeCount > m_last_total_num_of_nodes
	m_last_total_num_of_nodes = 2;

	m_edge_types_to_nums["ICP"] = 0;
	m_edge_types_to_nums["LC"] = 0;

	m_checked_for_usuable_dataset = false;
	m_consecutive_invalid_format_instances = 0;

	std::cout << "CCICPGoodnessERD: Initialized class object" << std::endl;

	MRPT_END;
}
template<class GRAPH_t>
CICPGoodnessERD_t<GRAPH_t>::~CICPGoodnessERD_t() { }

// Method implementations
// //////////////////////////////////

template<class GRAPH_t> void CICPGoodnessERD_t<GRAPH_t>::updateDeciderState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	
	// check possible prior node registration
	bool registered_new_node = false;

	if (m_last_total_num_of_nodes < m_graph->nodeCount()) {
		registered_new_node = true;
		m_last_total_num_of_nodes = m_graph->nodeCount();
		//std::cout << "[CICPGoodnessERD:] Registered new node. " << std::endl;
	}

	if (observation.present()) { // observation-only rawlog format
			if (IS_CLASS(observation, CObservation2DRangeScan)) {
				m_last_laser_scan2D =
					static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);
			}
			if (IS_CLASS(observation, CObservation3DRangeScan)) {
				m_last_laser_scan3D =
					static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);
				// just load the range/intensity images - CGraphSlanEngine takes care
				// of the path
				m_last_laser_scan3D->load();
				this->convert3DTo2DRangeScan(
						/*from = */ m_last_laser_scan3D,
						/*to   = */ &m_last_laser_scan2D);
				//cout << "Conversion successful: 3DRangeScan=>2DRangeScan.. " << endl;

				m_contains_scans3D = true;
			}

			// add the last laser_scan
			if (registered_new_node && m_last_laser_scan2D) {
				m_nodes_to_laser_scans[m_graph->nodeCount()-1] = m_last_laser_scan2D;
				std::cout << "[CICPGoodnessERD:] Added laser scans of nodeID: "
					<< m_graph->nodeCount()-1 << std::endl;
			}
	}
	else { // action-observations rawlog format
		// append current laser scan
		m_last_laser_scan2D =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (registered_new_node && m_last_laser_scan2D) {
			m_nodes_to_laser_scans[m_graph->nodeCount()-1] = m_last_laser_scan2D;
			//std::cout << "Added laser scans of nodeID: "
			//<< m_graph->nodeCount()-1 << std::endl;
		}
	}

	// edge registration procedure - same for both rawlog formats
	if (registered_new_node) {
		// get set of nodes within predefined distance for ICP
		std::set<mrpt::utils::TNodeID> nodes_to_check_ICP;
		this->getNearbyNodesOf(
				&nodes_to_check_ICP,
				m_graph->nodeCount()-1,
				params.ICP_max_distance);
		std::cout << "Found * " << nodes_to_check_ICP.size() 
			<< " * nodes close to nodeID: " << m_graph->nodeCount()-1 << endl;

		// reset the loop_closure flag and run registration
		m_just_inserted_loop_closure = false;
		registered_new_node = false;
		checkRegistrationCondition(nodes_to_check_ICP);
	}

	if (!m_checked_for_usuable_dataset) {
		this->checkIfInvalidDataset(action, observations, observation);
	}

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::checkRegistrationCondition(
		const std::set<mrpt::utils::TNodeID>& nodes_set) {
	MRPT_START;

	// try adding ICP constraints with each node in the previous set
	for (set<mrpt::utils::TNodeID>::const_iterator 
			node_it = nodes_set.begin();
			node_it != nodes_set.end(); ++node_it) {

		// get the ICP edge between current and last node
		constraint_t rel_edge;
		double ICP_goodness = this->getICPEdge(
				*node_it,
				m_graph->nodeCount()-1,
				&rel_edge);

		// criterion for registering a new node
		if (ICP_goodness > params.ICP_goodness_thresh) {
			this->registerNewEdge(
					*node_it, m_graph->nodeCount()-1, rel_edge);
			m_edge_types_to_nums["ICP"]++;
			// in case of loop closure
			if (abs(m_graph->nodeCount()-1 - *node_it) > params.LC_min_nodeid_diff) {
				m_edge_types_to_nums["LC"]++;
				m_just_inserted_loop_closure = true;
			}
		}
	}

	MRPT_END;
}


template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::registerNewEdge(
    const mrpt::utils::TNodeID& from, 
    const mrpt::utils::TNodeID& to,
    const constraint_t& rel_edge ) {
  MRPT_START;

  //cout << "[CICPGoodnessERD:] Inserting new Edge: " << from << " -> " << to << endl;
	m_graph->insertEdge(from,  to, rel_edge);

	MRPT_END;
}
template<class GRAPH_t>
double CICPGoodnessERD_t<GRAPH_t>::getICPEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		constraint_t* rel_edge ) {
	MRPT_START;

	// get the laser scan measurements - first check if they exist
	CObservation2DRangeScanPtr prev_laser_scan;
	CObservation2DRangeScanPtr curr_laser_scan;

	std::map<const mrpt::utils::TNodeID, 
		mrpt::obs::CObservation2DRangeScanPtr>::const_iterator search =
			m_nodes_to_laser_scans.find(from); 
	if (search != m_nodes_to_laser_scans.end()) {
		prev_laser_scan = search->second;
	}
	else {
		std::cout << "Unable to find laser scan of NodeID: " 
			<< from << std::endl;
		return 0.0;
	}
	search =	m_nodes_to_laser_scans.find(to); 
	if (search != m_nodes_to_laser_scans.end()) {
		curr_laser_scan = search->second;
	}
	else {
		std::cout << "Unable to find laser scan of NodeID: " 
			<< to << std::endl;
		return 0.0;
	}

	// Uses TParams::ICP member variable
	CSimplePointsMap m1,m2;
	float running_time;
	CICP::TReturnInfo info;

	pose_t initial_pose;
	// use the difference of the node positions as an initial alignment
	// estimation (dijkstra_nodes_estimate has to be run from the caller
	// function)
	initial_pose = m_graph->nodes[to] - m_graph->nodes[from];

	m1.insertObservation(&(*prev_laser_scan));
	m2.insertObservation(&(*curr_laser_scan));

	CPosePDFPtr pdf = params.icp.Align(
			&m1,
			&m2,
			initial_pose,
			&running_time,
			(void*)&info);

	// return the edge regardless of the goodness of the alignment
	rel_edge->copyFrom(*pdf);  	
	return info.goodness;

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::getNearbyNodesOf(
		set<TNodeID> *nodes_set,
		const TNodeID& cur_nodeID,
		double distance ) {
	MRPT_START;

	if (distance > 0) {
		// check all but the last node.
		for (TNodeID nodeID = 0; nodeID < m_graph->nodeCount()-1; ++nodeID) {
			double curr_distance = m_graph->nodes[nodeID].distanceTo(
					m_graph->nodes[cur_nodeID]);
			//std::cout << "testing against node: " << nodeID << std::endl;
			//std::cout << "\tcurr_distance: " << curr_distance << std::endl;
			if (curr_distance <= distance) {
				nodes_set->insert(nodeID);
			}
		}
	}
	else { // check against all nodes 
		m_graph->getAllNodes(*nodes_set);
	}

	MRPT_END;
}


template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	MRPT_START;

	m_graph = graph;

	std::cout << "[CICPGoodnessERD:] Fetched the graph successfully" 
		<< std::endl;

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::setRawlogFname(const std::string& rawlog_fname){
	MRPT_START;

	m_rawlog_fname = rawlog_fname;
	std::cout << "[CICPGoodnessERD:] Fetched the rawlog filename successfully: "
		<< m_rawlog_fname << std::endl;

	// find the directory of the 3Dscan images in case we are working with
	// Cobservation3DRangeScans
	cout << "Trying to fetch 3D scans external storage directory.. " << endl;
	std::string rawlog_fname_noext = system::extractFileName(m_rawlog_fname);
	std::string rawlog_dir = system::extractFileDirectory(rawlog_fname);
	std::string img_external_storage_dir = 
		rawlog_dir + rawlog_fname_noext + "_Images/";

	if (system::directoryExists(img_external_storage_dir)) {
		params.scans_img_external_dir = img_external_storage_dir;
		cout << "3D scans external storage: " << params.scans_img_external_dir 
			<< endl;
	}
	else {
		cout << "Couldn't find 3D scans external storage: " << img_external_storage_dir << endl;
	}




	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::setWindowManagerPtr(
		mrpt::gui::CWindowManager_t* win_manager) {
	m_win_manager = win_manager;
}
template<class GRAPH_t> void
CICPGoodnessERD_t<GRAPH_t>::setCDisplayWindowPtr(
		mrpt::gui::CDisplayWindow3D* win) {
	MRPT_START;

	m_win = win;

	std::cout << "[CICPGoodnessERD:] Fetched the CDisplayWindow successfully" 
		<< std::endl;

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::getEdgesStats(
		std::map<const std::string, int>* edge_types_to_nums) {
	MRPT_START;

	*edge_types_to_nums = m_edge_types_to_nums;

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::initializeVisuals() {
	MRPT_START;
	assert(params.has_read_config &&
			"[CICPGoodnessERD:] Configuration parameters aren't loaded yet");
	std::cout << "Initializing CICPGoodnessERD visuals" << std::endl;

	// ICP_max_distance disk
	if (m_win &&  params.ICP_max_distance > 0) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CDiskPtr obj = CDisk::Create();
		pose_t initial_pose;
		obj->setPose(initial_pose);
		obj->setName("ICP_max_distance");
		obj->setColor(m_search_disk_color);
		obj->setDiskRadius(params.ICP_max_distance, params.ICP_max_distance-0.5);
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	// laser scan visualization
	if (params.visualize_laser_scans) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CPlanarLaserScanPtr laser_scan_viz = mrpt::opengl::CPlanarLaserScan::Create();
		laser_scan_viz->enablePoints(true);
		laser_scan_viz->enableLine(true);
		laser_scan_viz->enableSurface(true);
		laser_scan_viz->setSurfaceColor(0, 0.2, 0.8);

		laser_scan_viz->setName("laser_scan_viz");

		scene->insert(laser_scan_viz);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	// max distance disk - textMessage
	if (m_win && m_win_manager && params.ICP_max_distance > 0) {
		m_win_manager->assignTextMessageParameters(
				&m_offset_y_search_disk,
				&m_text_index_search_disk);

		m_win_manager->addTextMessage(5,-m_offset_y_search_disk,
				format("ICP Edges search radius"),
				mrpt::utils::TColorf(m_search_disk_color),
				/* unique_index = */ m_text_index_search_disk );
	}

	m_initialized_visuals = true;
	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::updateVisuals() {
	MRPT_START;
	assert(m_initialized_visuals);
	//std::cout << "Updating CICPGoodnessERD visuals" << std::endl;

	// update ICP_max_distance Disk
	if (m_win && params.ICP_max_distance > 0) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CRenderizablePtr obj = scene->getByName("ICP_max_distance");
		CDiskPtr disk_obj = static_cast<CDiskPtr>(obj);

		disk_obj->setPose(m_graph->nodes[m_graph->nodeCount()-1]);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}
	
	// update laser scan visual
	if (params.visualize_laser_scans && m_last_laser_scan2D) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();
		
		CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		CPlanarLaserScanPtr laser_scan_viz = static_cast<CPlanarLaserScanPtr>(obj);

		// set the scan contents
		laser_scan_viz->setScan(*m_last_laser_scan2D);

		// set the pose of the laser scan
		typename GRAPH_t::global_poses_t::const_iterator search = 
			m_graph->nodes.find(m_graph->nodeCount()-1);
		if (search != m_graph->nodes.end()) {
			laser_scan_viz->setPose(m_graph->nodes[m_graph->nodeCount()-1]);
			// put the laser scan underneath the graph, so that you can still
			// visualize the loop closures with the nodes ahead
			laser_scan_viz->setPose(CPose3D(
						laser_scan_viz->getPoseX(), laser_scan_viz->getPoseY(), -0.5,
						DEG2RAD(laser_scan_viz->getPoseYaw()), 
						DEG2RAD(laser_scan_viz->getPosePitch()), 
						DEG2RAD(laser_scan_viz->getPoseRoll())
						));
		}

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	// RGB image visual
	if (m_contains_scans3D) {
		// initialize the viewport if not there
		if (!m_initialized_rgbd_viewports) {
			cout << "Initializing the RGBD viewports..." << endl;

			// intensity viewport
			if (params.enable_intensity_viewport) {
				COpenGLScenePtr scene = m_win->get3DSceneAndLock();
				opengl::COpenGLViewportPtr viewp_intensity;

				viewp_intensity = scene->createViewport("viewp_intensity");
				// TODO - assign position using window_manager
				viewp_intensity->setViewportPosition(0.78,0.56,0.20,0.20);

				m_win->unlockAccess3DScene();
				m_win->forceRepaint();
			}

			// range viewport
			if (params.enable_range_viewport) {
				COpenGLScenePtr scene = m_win->get3DSceneAndLock();
				opengl::COpenGLViewportPtr viewp_range;

				viewp_range = scene->createViewport("viewp_range");
				// TODO - assign position using window_manager
				viewp_range->setViewportPosition(0.78,0.34,0.20,0.20);

				m_win->unlockAccess3DScene();
				m_win->forceRepaint();
			}

			m_initialized_rgbd_viewports = true;
		}

		//// load the externally stored images
		//m_last_laser_scan3D->load();

		// in either case update them..
		// Show intensity image:
		if (m_last_laser_scan3D->hasIntensityImage && params.enable_intensity_viewport) {
			mrpt::utils::CImage img  = m_last_laser_scan3D->intensityImage;

			// in case it is externally stored - find the correct file name
			this->correct3DScanImageFname(&img, ".png");

			COpenGLScenePtr scene = m_win->get3DSceneAndLock();
			COpenGLViewportPtr viewp_intensity = scene->getViewport("viewp_intensity");
			//viewp_intensity->setImageView_fast(m_last_laser_scan3D->intensityImage);
			viewp_intensity->setImageView_fast(img);
			m_win->unlockAccess3DScene();
			m_win->forceRepaint();
		}

		//// show the range image
		//if (m_last_laser_scan3D->hasRangeImage && params.enable_range_viewport) {

			//// make this a static class member
			//CMatrixFloat range2D;
			//mrpt::utils::CImage img;
	
			//range2D = m_last_laser_scan3D->rangeImage * (1.0/5.0); // TODO - without the magic number?
			//img.setFromMatrix(range2D);

			//COpenGLScenePtr scene = m_win->get3DSceneAndLock();
			//COpenGLViewportPtr viewp_range = scene->getViewport("viewp_range");
			//viewp_range->setImageView_fast(img);
			//m_win->unlockAccess3DScene();
			//m_win->forceRepaint();
		//}

	}
	MRPT_END;
}
template<class GRAPH_t>
bool CICPGoodnessERD_t<GRAPH_t>::justInsertedLoopClosure() {
	return m_just_inserted_loop_closure;
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::checkIfInvalidDataset(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);

	if (observation.present()) { // FORMAT #2
		if (IS_CLASS(observation, CObservation2DRangeScan) || 
				IS_CLASS(observation, CObservation3DRangeScan)) {
			m_checked_for_usuable_dataset = true;
			return;
		}
		else {
			m_consecutive_invalid_format_instances++;
		}
	}
	else {
		// TODO - what if it's in this format but only has odometry information?
		m_checked_for_usuable_dataset = true;
		return;
	}
	if (m_consecutive_invalid_format_instances > m_consecutive_invalid_format_instances_thres) {
		std::cout << "[CICPGoodnessERD:] Can't find usuable data in the given dataset." 
			<< std::endl;
		std::cout << "Make sure dataset contains valid CObservation2DRangeScan/CObservation3DRangeScan data." 
			<< std::endl;
		mrpt::system::sleep(5000);
		m_checked_for_usuable_dataset = true;
	}

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::convert3DTo2DRangeScan(
		CObservation3DRangeScanPtr& scan3D_in,
		CObservation2DRangeScanPtr* scan2D_out /*= NULL*/) {

	// create the 2D laser scan first
	*scan2D_out = CObservation2DRangeScan::Create();

	// TODO - does it change the 3D range scan? const .. 
	if (scan3D_in->hasRangeImage) {
		scan3D_in->convertTo2DScan(**scan2D_out, 
				kConversionSensorLabel, 
				kConversionAngleSup, 
				kConversionAngleInf, 
				kConversionOversamplingRatio);
	}

}



template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::correct3DScanImageFname(
		mrpt::utils::CImage* img,
		std::string extension /*= ".png" */ ) {

	if (!params.scans_img_external_dir.empty()) {
		// fetch the correct absolute path of the image
		string relative_fname = img->getExternalStorageFile();
		string fname = system::extractFileName(relative_fname);
		fname = params.scans_img_external_dir + fname + extension;

		// set the image file path appropriately
		img->setExternalStorage(fname);
	}
}

// TParameter
// //////////////////////////////////

template<class GRAPH_t>
CICPGoodnessERD_t<GRAPH_t>::TParams::TParams():
	has_read_config(false)
{ }

template<class GRAPH_t>
CICPGoodnessERD_t<GRAPH_t>::TParams::~TParams() {
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ Goodness-based ICP Edge Registration ]------------------\n");
	out.printf("ICP goodness threshold         = %.2f%% \n", ICP_goodness_thresh*100);
	out.printf("ICP max radius for edge search = %.2f\n", ICP_max_distance);
	out.printf("Min. node difference for LC    = %d\n", LC_min_nodeid_diff);
	out.printf("Visualize laser scans          = %d\n", visualize_laser_scans);
	out.printf("Enable intensity img viewport  = %d\n", enable_intensity_viewport);
	out.printf("Enable range img viewport      = %d\n", enable_range_viewport);
	out.printf("Visualize laser scans          = %d\n", visualize_laser_scans);
	out.printf("3DScans Image Directory        = %s\n", scans_img_external_dir.c_str());
	out.printf("ICP Configuration:\n");

	icp.options.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
    const std::string& section) {
  MRPT_START;

	ICP_max_distance = source.read_double(
			section,
			"ICP_max_distance",
			10, false);
	ICP_goodness_thresh = source.read_double(
			section,
			"ICP_goodness_thresh",
	 		0.75, false);
  LC_min_nodeid_diff = source.read_int(
  		section,
 			"LC_min_nodeid_diff",
 			10, false);
  visualize_laser_scans = source.read_bool(
  		"VisualizationParameters",
 			"visualize_laser_scans",
 			true, false);
  enable_intensity_viewport= source.read_bool(
  		"VisualizationParameters",
 			"enable_intensity_viewport",
 			true, false);
  enable_range_viewport = source.read_bool(
  		"VisualizationParameters",
 			"enable_range_viewport",
 			true, false);
 	scans_img_external_dir = source.read_string(
 			section,
 			"scan_images_external_directory",
 			"", false);

	// load the icp parameters - from "ICP" section explicitly
	icp.options.loadFromConfigFile(source, "ICP");

	std::cout << "[CICPGoodnessERD:] Successfully loaded CICPGoodnessERD parameters. " 
		<< std::endl;
	has_read_config = true;

	MRPT_END;
}


#endif /* end of include guard: CICPGoodnessERD_IMPL_H */
