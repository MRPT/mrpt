#ifndef CRANGESCANEDGEREGISTRATIONDECIDER_IMPL_H
#define CRANGESCANEDGEREGISTRATIONDECIDER_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
CRangeScanEdgeRegistrationDecider<GRAPH_T>::
CRangeScanEdgeRegistrationDecider():
	m_last_total_num_nodes(0),
	m_use_mahal_distance_init_ICP(true),
	m_keystroke_laser_scans("l"),
	m_planar_laser_scan_obj_name("laser_scan_viz"),
	m_icp_goodness_thresh_is_fixed(false)
{
	m_mahal_distance_ICP_odom_win.resizeWindow(200); // use the last X mahal. distance values
	m_goodness_threshold_win.resizeWindow(200);

	pose_t p_unused;
	initClassInternal(p_unused);
}

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initClassInternal(
		const mrpt::poses::CPose2D& p_unused) {
} // end of initClassInternal

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initClassInternal(
		const mrpt::poses::CPose3D& p_unused) {
	using namespace mrpt::utils;

	// keep the last path - change back to it after rawlog parsing
	m_img_prev_path_base = CImage::IMAGES_PATH_BASE;

	{
		m_info_params.setRawlogFile(this->m_rawlog_fname);
		m_info_params.parseFile();
		// set the rate at which we read from the GT poses vector
		int num_of_objects = std::atoi(
				m_info_params.fields["Overall number of objects"].c_str());

		MRPT_LOG_INFO_STREAM("Overall number of objects in rawlog: " << num_of_objects);
	}


	std::string rawlog_fname_noext = system::extractFileName(this->m_rawlog_fname);
	std::string rawlog_dir = system::extractFileDirectory(this->m_rawlog_fname);
	std::string m_img_external_storage_dir = rawlog_dir + rawlog_fname_noext
		+ "_Images/";
	CImage::IMAGES_PATH_BASE = m_img_external_storage_dir;

} // end of initClassInternal

template<class GRAPH_T>
CRangeScanEdgeRegistrationDecider<GRAPH_T>::~CRangeScanEdgeRegistrationDecider() {

	MRPT_LOG_DEBUG_STREAM("Changing back the CImage PATH");
	mrpt::utils::CImage::IMAGES_PATH_BASE = m_img_prev_path_base;

}

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::loadParams(
		const std::string& source_fname) {
	MRPT_START;

	parent_t::loadParams(source_fname);
	range_ops_t::params.loadFromConfigFileName(source_fname, "ICP");

	mrpt::utils::CConfigFile source(source_fname);

	std::string section_viz = "VisualizationParameters";
	std::string section_erd = "EdgeRegistrationDeciderParameters";

	// visualization/coloring of laser scans surface
	m_visualize_laser_scans = source.read_bool(
			section_viz,
			"visualize_laser_scans",
			true, false);

	const uint64_t ls_color =  source.read_uint64_t(
			section_viz,
			"laser_scans_color",
			0x0014FF, false);
	const uint64_t ls_opac =  source.read_uint64_t(
			section_viz,
			"laser_scans_opac",
			0xFF, false);
	m_laser_scans_color = mrpt::utils::TColor(ls_color, ls_opac);

	m_consec_icp_constraint_factor = source.read_double(
			section_erd,
			"consec_icp_constraint_factor",
			0.90, false);

	// fixed threshold - if <= 0  an adaptive threshold is going to be used
	m_ICP_goodness_thresh = source.read_double(
			section_erd,
			"ICP_goodness_thresh",
			-1, false);

	m_use_mahal_distance_init_ICP = source.read_bool(
			section_erd,
			"use_mahal_distance_init_ICP",
			true, false);

	// viewports
	m_enable_range_viewport = source.read_bool(
			section_viz,
			"enable_range_viewport",
			true, false);
	m_enable_intensity_viewport = source.read_bool(
			section_viz,
			"enable_intensity_viewport",
			true, false);


	MRPT_END;
} // end of loadParams

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::printParams() const {
	MRPT_START;
	using namespace std;

	parent_t::printParams();
	range_ops_t::params.dumpToConsole();

	cout << "Visualize laser scans                       = "
			<< (m_visualize_laser_scans? "TRUE": "FALSE") << endl;
	cout << "Scan-matching ICP Constraint factor         = "
		<< m_consec_icp_constraint_factor << endl;
	cout << "Enable range img viewport       = "
		<< ( m_enable_range_viewport ? "TRUE" : "FALSE" ) << endl;
	cout << "Enable intensity img viewport   = "
		<< ( m_enable_intensity_viewport ? "TRUE" : "FALSE" ) << endl;

	MRPT_END;
}

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initLaserScansVisualization() {
	MRPT_START;

	// laser scan visualization
	if (m_visualize_laser_scans) {
		mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

		mrpt::opengl::CPlanarLaserScanPtr laser_scan_viz = 
			mrpt::opengl::CPlanarLaserScan::Create();
		laser_scan_viz->enablePoints(true);
		laser_scan_viz->enableLine(true);
		laser_scan_viz->enableSurface(true);
		laser_scan_viz->setSurfaceColor(m_laser_scans_color);

		laser_scan_viz->setName(m_planar_laser_scan_obj_name);

		scene->insert(laser_scan_viz);
		this->m_win->unlockAccess3DScene();
		this->m_win->forceRepaint();
	}

	MRPT_END;
} // end of initLaserScansVisualization

template<class GRAPH_T>
bool CRangeScanEdgeRegistrationDecider<GRAPH_T>::fillNodePropsFromGroupParams(
		const mrpt::utils::TNodeID& nodeID,
		const std::map<mrpt::utils::TNodeID, node_props_t>& group_params,
		node_props_t* node_props) {
	ASSERT_(node_props);

	//MRPT_LOG_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
	//MRPT_LOG_DEBUG_STREAM("Running fillNodePropsFromGroupParams for nodeID \""
		//<< nodeID << "\"");

	// Make sure that the given nodeID exists in the group_params
	typename std::map<mrpt::utils::TNodeID, node_props_t>::const_iterator
		search = group_params.find(nodeID);
	bool res = false;
	if (search == group_params.end()) {
		//MRPT_LOG_DEBUG_STREAM("Operation failed.");
	}
	else {
		*node_props = search->second;
		res = true;
		//MRPT_LOG_DEBUG_STREAM("Properties filled: " << node_props->getAsString());
	}

	//MRPT_LOG_DEBUG_STREAM("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
	return res;

} // end of fillNodePropsFromGroupParams

template<class GRAPH_T>
bool CRangeScanEdgeRegistrationDecider<GRAPH_T>::getICPEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		constraint_t* rel_edge,
		mrpt::slam::CICP::TReturnInfo* icp_info,
		const TGetICPEdgeAdParams* ad_params/*=NULL*/) {
	MRPT_START;
	ASSERT_(rel_edge);
	this->m_time_logger.enter("getICPEdge");

	using namespace mrpt::obs;
	using namespace mrpt::utils;
	using namespace std;
	using namespace mrpt::graphslam::detail;

	// fetch the relevant laser scans and poses of the nodeIDs
	// If given in the additional params struct, use those values instead of
	// searching in the class std::map(s)
	CObservation2DRangeScanPtr from_scan, to_scan;
	global_pose_t from_pose;
	global_pose_t to_pose;

	// from-node parameters
	const node_props_t* from_params = ad_params ? &ad_params->from_params : NULL;
	bool from_success = this->getPropsOfNodeID(from, &from_pose, from_scan, from_params);
	// to-node parameters
	const node_props_t* to_params = ad_params ? &ad_params->to_params : NULL;
	bool to_success = this->getPropsOfNodeID(to, &to_pose, to_scan, to_params);


	if (!from_success || !to_success) {
		MRPT_LOG_DEBUG_STREAM(
			"Either node #" << from <<
			" or node #" << to <<
			" doesn't contain a valid LaserScan. Ignoring this...");
		return false;
	}

	// make use of initial node position difference for the ICP edge
	// from_node pose
	pose_t initial_estim;
	if (ad_params) {
		initial_estim = ad_params->init_estim;
	}
	else {
		initial_estim = to_pose - from_pose;
 	}

	MRPT_LOG_DEBUG_STREAM("from_pose: " << from_pose
		<< "| to_pose: " << to_pose
		<< "| init_estim: " << initial_estim);

	this->_getICPEdge(
			*from_scan, *to_scan, rel_edge, &initial_estim, icp_info);
	MRPT_LOG_DEBUG_STREAM("*************");

	this->m_time_logger.leave("getICPEdge");
	return true;
	MRPT_END;
} // end of getICPEdge

template<class GRAPH_T>
bool CRangeScanEdgeRegistrationDecider<GRAPH_T>::getPropsOfNodeID(
		const mrpt::utils::TNodeID& nodeID,
		global_pose_t* pose,
		mrpt::obs::CObservation2DRangeScanPtr& scan,
		const node_props_t* node_props/*=NULL*/) const {

	// make sure output instances are valid
	ASSERT_(pose);

	bool filled_pose = false;
	bool filled_scan = false;

	if (node_props) {
		// Pose
		MRPT_TODO("Use approximatelyEqual instead of !=")
		if (node_props->pose != global_pose_t()) {
			*pose = node_props->pose;
			filled_pose = true;
		}
		// LaserScan
		if (node_props->scan.present()) {
			scan = node_props->scan;
			filled_scan = true;
		}
	} // end if node_props

	// TODO - What if the node_props->pose is indeed 0?
	ASSERTMSG_(!(filled_pose ^ filled_scan),
			format(
				"Either BOTH or NONE of the filled_pose, filled_scan should be set."
				"NodeID:  [%lu]", static_cast<unsigned long>(nodeID)));

	//
	// fill with class data if not yet available
	//
	if (!filled_pose) {
		// fill with class data if not yet available
		typename GRAPH_T::global_poses_t::const_iterator search =
			this->m_graph->nodes.find(nodeID);
		if (search != this->m_graph->nodes.end()) {
			*pose = search->second;
			filled_pose = true;
		}
		else {
			MRPT_LOG_WARN_STREAM("pose not found for nodeID: " << nodeID);
		}
	}
	if (!filled_scan) {
		typename nodes_to_scans2D_t::const_iterator search =
			this->m_nodes_to_laser_scans2D.find(nodeID);
		if (search != this->m_nodes_to_laser_scans2D.end()) {
			scan = search->second;
			filled_scan = true;
		}
	}

	return filled_pose && filled_scan;
} // end of getPropsOfNodeID


template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::addScanMatchingEdges(
		const mrpt::utils::TNodeID& curr_nodeID) {
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::obs;
	using namespace mrpt::math;

	// get a list of nodes to check ICP against
	MRPT_LOG_DEBUG_STREAM("Adding ICP Constraints for nodeID: " <<
		curr_nodeID);

	std::set<TNodeID> nodes_set;
	this->fetchNodeIDsForScanMatching(curr_nodeID, &nodes_set);


	// try adding ICP constraints with each node in the previous set
	for (std::set<TNodeID>::const_iterator node_it = nodes_set.begin();
			node_it != nodes_set.end(); ++node_it) {

		constraint_t rel_edge;
		mrpt::slam::CICP::TReturnInfo icp_info;

		MRPT_LOG_DEBUG_STREAM("Fetching laser scan for nodes: " << *node_it
				<<  "==> " << curr_nodeID);

		bool found_edge = this->getICPEdge(
				*node_it,
				curr_nodeID,
				&rel_edge,
				&icp_info);
		if (!found_edge) continue;

		// Goodness value
		//
		// keep track of the recorded goodness values
		// TODO - rethink on these condition.
		if (!m_icp_goodness_thresh_is_fixed && (
					!isNaN(icp_info.goodness) || 
					icp_info.goodness != 0)) {
				m_goodness_threshold_win.addNewMeasurement(icp_info.goodness);
		}
		double goodness_thresh = this->getGoodnessThresh();
		bool accept_goodness = icp_info.goodness > goodness_thresh;
		MRPT_LOG_DEBUG_STREAM(
				"Curr. Goodness: " << icp_info.goodness
				<< "|\t Threshold: " << goodness_thresh << 
				" => " << (accept_goodness? "ACCEPT" : "REJECT"));

		// Mahalanobis distance
		//
		// make sure that the suggested edge makes sense with regards to current
		// graph config - check against the current position difference
		bool accept_mahal_distance = (m_use_mahal_distance_init_ICP ?
				this->mahalanobisDistanceInitToICPEdge(
					*node_it, curr_nodeID, rel_edge) :
				true);

		// criterion for registering a new node
		if (accept_goodness && accept_mahal_distance) {
			this->registerNewEdge(*node_it, curr_nodeID, rel_edge);
		}
	}

	MRPT_END;
} // end of addScanMatchingEdges

template<class GRAPH_T>
double CRangeScanEdgeRegistrationDecider<GRAPH_T>::getGoodnessThresh() const {
	double thresh;

	if (!m_icp_goodness_thresh_is_fixed) {
		thresh =
			m_goodness_threshold_win.getMedian() * m_consec_icp_constraint_factor;
	}
	else {
		thresh = m_ICP_goodness_thresh_fixed;
	}

	return thresh;
}

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initializeVisuals() {
	MRPT_START;

	// assign a text message for displaying the ICP goodness value
	this->m_win_manager->assignTextMessageParameters(
			/* offset_y*	= */ &m_offset_y_icp_goodness,
			/* text_index* = */ &m_text_index_icp_goodness);

	if (m_visualize_laser_scans) { this->initLaserScansVisualization(); }

	// keystrokes
	this->m_win_observer->registerKeystroke(
			m_keystroke_laser_scans,
			"Toggle LaserScans Visualization");

	MRPT_END;
} // end of initializeVisuals

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initViewports() {
	pose_t p_unused;
	this->initViewportsInternal(p_unused);
}

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::updateViewports() {
	pose_t p_unused;
	this->updateViewportsInternal(p_unused);
}


template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initViewportsInternal(
		const mrpt::poses::CPose2D& p_unused) {
} // end if initViewportsInternal

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initViewportsInternal(
		const mrpt::poses::CPose3D& p_unused) {

	if (m_enable_range_viewport) {
		this->initRangeImageViewport();
	}
	if (m_enable_intensity_viewport) {
		this->initIntensityImageViewport();
	}
} // end if initViewportsInternal

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::updateViewportsInternal(
		const mrpt::poses::CPose2D& p_unused) {
} // end if updateViewportsInternal

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::updateViewportsInternal(
		const mrpt::poses::CPose3D& p_unused) {

	if (m_enable_range_viewport &&
			!m_last_laser_scan3D.null() &&
			m_last_laser_scan3D->hasRangeImage) {
		this->updateRangeImageViewport();
	}

	if (m_enable_intensity_viewport &&
			!m_last_laser_scan3D.null() &&
			m_last_laser_scan3D->hasIntensityImage) {
		this->updateIntensityImageViewport();
	}

} // end if updateViewportsInternal


template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initRangeImageViewport() {
	MRPT_START;
	using namespace mrpt::opengl;

	COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_range;

	viewp_range = scene->createViewport("viewp_range");
	double x,y,h,w;
	this->m_win_manager->assignViewportParameters(&x, &y, &w, &h);
	viewp_range->setViewportPosition(x, y, h, w);

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
} // end of initRangeImageViewport

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::updateRangeImageViewport() {
	MRPT_START;
	using namespace mrpt::math;
	using namespace mrpt::opengl;

	CMatrixFloat range2D;
	mrpt::utils::CImage img;

	// load the image if not already loaded..
	m_last_laser_scan3D->load();
	range2D = m_last_laser_scan3D->rangeImage * (1.0f/5.0);
	img.setFromMatrix(range2D);

	COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_range = scene->getViewport("viewp_range");
	viewp_range->setImageView(img);
	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
} // end of updateRangeImageViewport


template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initIntensityImageViewport() {
	MRPT_START;
	using namespace mrpt::opengl;

	COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_intensity;

	viewp_intensity = scene->createViewport("viewp_intensity");
	double x, y, w, h;
	this->m_win_manager->assignViewportParameters(&x, &y, &w, &h);
	viewp_intensity->setViewportPosition(x, y, w, h);

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
} // end of initIntensityImageViewport

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::updateIntensityImageViewport() {
	MRPT_START;
	using namespace mrpt::opengl;

	// load the image if not already loaded..
	m_last_laser_scan3D->load();

	COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_intensity = scene->getViewport("viewp_intensity");
	viewp_intensity->setImageView(m_last_laser_scan3D->intensityImage);
	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
} // end of updateIntensityImageViewport



template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::updateVisuals() {
	MRPT_START;

	if (m_visualize_laser_scans) { this->updateLaserScansVisualization(); }

	if (!this->m_icp_goodness_thresh_is_fixed) {
		std::string title = mrpt::format(
				"ICP Goodness threshold: %0.3f",
				this->getGoodnessThresh());

		// update the indicated ICP goodness
		this->m_win_manager->addTextMessage(5,-m_offset_y_icp_goodness,
				title,
				mrpt::utils::TColorf(1, 1, 1),
				/* unique_index = */ this->m_text_index_icp_goodness);
	}

	MRPT_END;
} // end of updateVisuals

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::
updateLaserScansVisualization() {
	MRPT_START;
	using namespace mrpt::poses;

	// update laser scan visual
	if (m_visualize_laser_scans && !m_last_laser_scan2D.null()) {
		mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

		mrpt::opengl::CRenderizablePtr obj = scene->getByName(m_planar_laser_scan_obj_name);
		mrpt::opengl::CPlanarLaserScanPtr laser_scan_viz =
			static_cast<mrpt::opengl::CPlanarLaserScanPtr>(obj);
		ASSERT_(laser_scan_viz.present());
		laser_scan_viz->setScan(*m_last_laser_scan2D);

		// set the pose of the laser scan
		typename GRAPH_T::global_poses_t::const_iterator search =
			this->m_graph->nodes.find(this->m_graph->nodeCount()-1);
		// TODO - change the opacity of the laserscan so that I can see underneath
		// it
		if (search != this->m_graph->nodes.end()) {
			const CPose3D p = CPose3D(search->second.getPoseMean());
			laser_scan_viz->setPose(mrpt::poses::CPose3D(
						p.x(),
						p.y(),
						-0.30,
						p.yaw(),
						p.pitch(),
						p.roll()));
		}

		this->m_win->unlockAccess3DScene();
		this->m_win->forceRepaint();
	} // end if

	MRPT_END;
} // end of updateLaserScansVisualization

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	MRPT_START;

	// laser scans
	if (events_occurred.at(m_keystroke_laser_scans)) {
		this->toggleLaserScansVisualization();
	}

	MRPT_END;
} // end of notifyOfWindowEvents

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::toggleLaserScansVisualization() {
	MRPT_START;
	using namespace mrpt::utils;
	this->logFmt(LVL_INFO, "Toggling LaserScans visualization...");

	mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

	if (m_visualize_laser_scans) {
		mrpt::opengl::CRenderizablePtr obj = scene->getByName(m_planar_laser_scan_obj_name);
		obj->setVisibility(!obj->isVisible());
	}
	else {
		parent_t::dumpVisibilityErrorMsg("visualize_laser_scans");
	}

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
} // end of toggleLaserScansVisualization

template<class GRAPH_T>
bool CRangeScanEdgeRegistrationDecider<GRAPH_T>::
mahalanobisDistanceInitToICPEdge(
		const mrpt::utils::TNodeID& from, const mrpt::utils::TNodeID& to, const
		constraint_t& rel_edge) {
	MRPT_START;

	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::utils;

	// mean difference
	pose_t initial_estim =
		this->m_graph->nodes.at(to) - this->m_graph->nodes.at(from);
	dynamic_vector<double> mean_diff;
	(rel_edge.getMeanVal()-initial_estim).getAsVector(mean_diff);

	// covariance matrix
	inf_mat_t cov_mat; rel_edge.getCovariance(cov_mat);

	// mahalanobis distance computation
	double mahal_distance = mrpt::math::mahalanobisDistance2(mean_diff, cov_mat);
	bool mahal_distance_null = isNaN(mahal_distance);
	if (!mahal_distance_null) {
		m_mahal_distance_ICP_odom_win.addNewMeasurement(mahal_distance);
	}

	//double threshold = m_mahal_distance_ICP_odom_win.getMean() +
		//2*m_mahal_distance_ICP_odom_win.getStdDev();
	double threshold = m_mahal_distance_ICP_odom_win.getMedian()*4;
	bool accept_edge = (threshold >= mahal_distance && !mahal_distance_null) ? true : false;

	//cout << "Suggested Edge: " << rel_edge.getMeanVal() << "|\tInitial Estim.: " << initial_estim
		//<< "|\tMahalanobis Dist: " << mahal_distance << "|\tThresh.: " << threshold
		//<< " => " << (accept_edge? "ACCEPT": "REJECT") << endl;

	return accept_edge;
	MRPT_END;
} // end of mahalanobisDistanceInitToICPEdge

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::initMiscActions() {

	// I put it here because it runs after visuals have been initiated
	// so the text message can be displayed at the correct position
	if (m_ICP_goodness_thresh > 0) {
		this->fixICPGoodnessThresh(m_ICP_goodness_thresh);
	}

	// Textmessage: Mahalanobis distance - Initial-ICP
	if (this->m_win_manager) {
		const std::string title = mrpt::format(
				"Use mahalanobis distance ICP-init: %s",
				(m_use_mahal_distance_init_ICP? "True" : "False"));
		this->m_win_manager->assignTextMessageParameters(
				/* offset_y*	= */ &m_offset_y_mahal,
				/* text_index* = */ &m_text_index_mahal);
		this->m_win_manager->addTextMessage(5,-m_offset_y_mahal,
				title,
				mrpt::utils::TColorf(1, 1, 1),
				/* unique_index = */ m_text_index_mahal);
	}

}


template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::fixICPGoodnessThresh(
		const double goodness) {

	this->m_ICP_goodness_thresh_fixed = goodness;
	m_icp_goodness_thresh_is_fixed = true;

	std::string title = mrpt::format(
			"ICP Goodness threshold: %0.3f",
			this->getGoodnessThresh());

	if (this->m_win_manager) {
		// update the indicated ICP goodness
		this->m_win_manager->addTextMessage(5,-m_offset_y_icp_goodness,
				title,
				mrpt::utils::TColorf(0, 0, 1),
				/* unique_index = */ m_text_index_icp_goodness);
	}

	MRPT_LOG_WARN_STREAM("Using fixed goodness threshold: " <<
			this->m_ICP_goodness_thresh_fixed);
} // end of fixICPGoodenssThresh

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::alignOpticalWithMRPTFrame() {
	MRPT_START;
	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::utils;

	// aligning GT (optical) frame with the MRPT frame
	// Set the rotation matrix from the corresponding RPY angles
	// MRPT Frame: X->forward; Y->Left; Z->Upward
	// Optical Frame: X->Right; Y->Downward; Z->Forward

	// rotz
	double anglez = DEG2RAD(0.0);
	const double tmpz[] = {
		cos(anglez),     -sin(anglez), 0,
		sin(anglez),     cos(anglez),  0,
		0,               0,            1  };
	CMatrixDouble rotz(3, 3, tmpz);

	// roty
	double angley = DEG2RAD(0.0);
	//double angley = DEG2RAD(90.0);
	const double tmpy[] = {
		cos(angley),      0,      sin(angley),
		0,                1,      0,
		-sin(angley),     0,      cos(angley)  };
	CMatrixDouble roty(3, 3, tmpy);

	// rotx
	//double anglex = DEG2RAD(-90.0);
	double anglex = DEG2RAD(0.0);
	const double tmpx[] = {
		1,        0,               0,
		0,        cos(anglex),     -sin(anglex),
		0,        sin(anglex),     cos(anglex)  };
	CMatrixDouble rotx(3, 3, tmpx);

	stringstream ss_out;
	ss_out << "\nConstructing the rotation matrix for the GroundTruth Data..."
		<< endl;
	m_rot_TUM_to_MRPT = rotz * roty * rotx;

	ss_out << "Rotation matrices for optical=>MRPT transformation" << endl;
	ss_out << "rotz: " << endl << rotz << endl;
	ss_out << "roty: " << endl << roty << endl;
	ss_out << "rotx: " << endl << rotx << endl;
	ss_out << "Full rotation matrix: " << endl << m_rot_TUM_to_MRPT << endl;

	MRPT_LOG_DEBUG_STREAM(ss_out);

	MRPT_END;
} // end of alignOpticalWithMRPTFrame


// TRGBDInfoFileParams
// ////////////////////////////////

template<class GRAPH_T>
CRangeScanEdgeRegistrationDecider<GRAPH_T>::
TRGBDInfoFileParams::TRGBDInfoFileParams(const std::string& rawlog_fname) {

	this->setRawlogFile(rawlog_fname);
	this->initTRGBDInfoFileParams();
} // end of TRGBDInfoFileParams::TRGBDInfoFileParams
template<class GRAPH_T>
CRangeScanEdgeRegistrationDecider<GRAPH_T>::
TRGBDInfoFileParams::TRGBDInfoFileParams() {
	this->initTRGBDInfoFileParams();
} // end of TRGBDInfoFileParams::TRGBDInfoFileParams
template<class GRAPH_T>
CRangeScanEdgeRegistrationDecider<GRAPH_T>::
TRGBDInfoFileParams::~TRGBDInfoFileParams() { }

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::TRGBDInfoFileParams::setRawlogFile(
		const std::string& rawlog_fname) {

	// get the correct info filename from the rawlog_fname
	std::string dir = mrpt::system::extractFileDirectory(rawlog_fname);
	std::string rawlog_filename = mrpt::system::extractFileName(rawlog_fname);
	std::string name_prefix = "rawlog_";
	std::string name_suffix = "_info.txt";
	info_fname = dir + name_prefix + rawlog_filename + name_suffix;
} // end of TRGBDInfoFileParams::setRawlogFile

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::
TRGBDInfoFileParams::initTRGBDInfoFileParams() {
	// fields to use
	fields["Overall number of objects"] = "";
	fields["Observations format"] = "";
} // end of TRGBDInfoFileParams::initTRGBDInfoFileParams

template<class GRAPH_T>
void CRangeScanEdgeRegistrationDecider<GRAPH_T>::TRGBDInfoFileParams::parseFile() {
	ASSERT_FILE_EXISTS_(info_fname);
	using namespace std;
	using namespace mrpt::utils;

	// open file
	CFileInputStream info_file(info_fname);
	ASSERTMSG_(info_file.fileOpenCorrectly(),
			"\nTRGBDInfoFileParams::parseFile: Couldn't open info file\n");

	string curr_line;
	size_t line_cnt = 0;

	// parse until you find an empty line.
	while (true) {
		info_file.readLine(curr_line);
		line_cnt++;
		if (curr_line.size() == 0)
			break;
	}

	// parse the meaningful data
	while (info_file.readLine(curr_line)) {
		// split current line at ":"
		vector<string> curr_tokens;
		mrpt::system::tokenize(curr_line, ":", curr_tokens);

		ASSERT_EQUAL_(curr_tokens.size(), 2);

		// evaluate the name. if name in info struct then fill the corresponding
		// info struct parameter with the value_part in the file.
		std::string literal_part = mrpt::system::trim(curr_tokens[0]);
		std::string value_part   = mrpt::system::trim(curr_tokens[1]);

		for (std::map<std::string, std::string>::iterator it = fields.begin();
				it != fields.end(); ++it) {
			if (mrpt::system::strCmpI(it->first, literal_part)) {
				it->second = value_part;
			}
		}

		line_cnt++;
	}

} // end of TRGBDInfoFileParams::parseFile

////////////////////////////////////////////////////////////////////////////////


} } } // end of namespaces

#endif /* end of include guard: CRANGESCANEDGEREGISTRATIONDECIDER_IMPL_H */
