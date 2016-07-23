/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */


#ifndef CLOOPCLOSERERD_IMPL_H
#define CLOOPCLOSERERD_IMPL_H


namespace mrpt { namespace graphslam { namespace deciders {

// Ctors, Dtors
// //////////////////////////////////
template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::CLoopCloserERD():
	m_laser_scans_color(0, 20, 255),
	m_consecutive_invalid_format_instances_thres(20) // high threshold just to make sure
{
	MRPT_START;
	this->initCLoopCloserERD();
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initCLoopCloserERD() {
	MRPT_START;

	m_win = NULL;
	m_win_manager = NULL;
	m_graph = NULL;

	m_initialized_visuals = false;
	m_just_inserted_loop_closure = false;

	// start the edge registration procedure only when this num is surpassed
	// nodeCount > m_last_total_num_of_nodes
	m_last_total_num_of_nodes = 5;

	m_edge_types_to_nums["ICP2D"] = 0;
	m_edge_types_to_nums["LC"] = 0;

	m_checked_for_usuable_dataset = false;
	m_consecutive_invalid_format_instances = 0;

	m_out_logger.setName("CLoopCloserERD");
	m_out_logger.setLoggingLevel(mrpt::utils::LVL_DEBUG); // defalut level of logger

	m_out_logger.log("Initialized class object");

	MRPT_END;
}
template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::~CLoopCloserERD() { }



// Methods implementations
// //////////////////////////////////

template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::opengl;

	// check possible prior node registration
	bool registered_new_node = false;

	if (m_last_total_num_of_nodes < m_graph->nodeCount()) {
		registered_new_node = true;
		m_last_total_num_of_nodes = m_graph->nodeCount();
		m_out_logger.log("New node has been registered!");
	}

	// TODO - carve this out...

	// TODO - remove this
	return false;

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::registerNewEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		const constraint_t& rel_edge ) {
	MRPT_START;
	m_out_logger.logFmt("Registering new edge: %lu => %lu\n"
			"\t%s\n"
			"\t%f\n", from, to, 
			rel_edge.getMeanVal().asString().c_str(),
			rel_edge.getMeanVal().norm());

	m_graph->insertEdge(from,  to, rel_edge);
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	MRPT_START;
	m_graph = graph;
	m_out_logger.log("Fetched the graph successfully");
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::setRawlogFname(const std::string& rawlog_fname){
	MRPT_START;

	m_rawlog_fname = rawlog_fname;
	m_out_logger.logFmt("Fetched the rawlog filename successfully: %s",
			m_rawlog_fname.c_str());

	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager) {
	m_win_manager = win_manager;

	// may still be null..
	if (m_win_manager) {
		m_win = m_win_manager->win;
		m_win_observer = m_win_manager->observer;

		m_out_logger.log("Fetched the window manager, window observer  successfully.");
	}

}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	MRPT_START;

	// I know the key exists - I put it there explicitly
	if (events_occurred.find(params.keystroke_laser_scans)->second) {
		this->toggleLaserScansVisualization();
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::toggleLaserScansVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(m_win_manager, "No CWindowManager* was provided");

	m_out_logger.log("Toggling LaserScans visualization...");

	mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	if (params.visualize_laser_scans) {
		mrpt::opengl::CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		dumpVisibilityErrorMsg("visualize_laser_scans");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}


template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::getEdgesStats(
		std::map<const std::string, int>* edge_types_to_num) const {
	MRPT_START;
	*edge_types_to_num = m_edge_types_to_nums;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initializeVisuals() {
	MRPT_START;
	m_out_logger.log("Initializing visuals");
	m_time_logger.enter("CLoopCloserERD::Visuals");

	ASSERTMSG_(params.has_read_config,
			"Configuration parameters aren't loaded yet");
	ASSERTMSG_(m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(m_win_manager, "No CWindowManager* was provided");
	ASSERTMSG_(m_win_observer, "No CWindowObserver* was provided");

	m_win_observer->registerKeystroke(params.keystroke_laser_scans,
			"Toggle LaserScans Visualization");
	// TODO - include visualization of the partitioning process
	// TODO - include visualization of the Olson LC
	// TODO - indicate number of node groups

	// laser scan visualization
	if (params.visualize_laser_scans) {
		mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		mrpt::opengl::CPlanarLaserScanPtr laser_scan_viz = 
			mrpt::opengl::CPlanarLaserScan::Create();
		laser_scan_viz->enablePoints(true);
		laser_scan_viz->enableLine(true);
		laser_scan_viz->enableSurface(true);
		laser_scan_viz->setSurfaceColor(
				m_laser_scans_color.R,
				m_laser_scans_color.G,
				m_laser_scans_color.B,
				m_laser_scans_color.A);

		laser_scan_viz->setName("laser_scan_viz");

		scene->insert(laser_scan_viz);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	m_initialized_visuals = true;
	m_time_logger.leave("CLoopCloserERD::Visuals");
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateVisuals() {
	MRPT_START;
	ASSERT_(m_initialized_visuals);
	m_out_logger.log("Updating visuals");
	m_time_logger.enter("CLoopCloserERD::Visuals");

	// update laser scan visual
	if (params.visualize_laser_scans && !m_last_laser_scan2D.null()) {
		mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		mrpt::opengl::CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		mrpt::opengl::CPlanarLaserScanPtr laser_scan_viz =
			static_cast<mrpt::opengl::CPlanarLaserScanPtr>(obj);

		laser_scan_viz->setScan(*m_last_laser_scan2D);

		// set the pose of the laser scan
		typename GRAPH_t::global_poses_t::const_iterator search =
			m_graph->nodes.find(m_graph->nodeCount()-1);
		if (search != m_graph->nodes.end()) {
			laser_scan_viz->setPose(m_graph->nodes[m_graph->nodeCount()-1]);
			// put the laser scan underneath the graph, so that you can still
			// visualize the loop closures with the nodes ahead
			laser_scan_viz->setPose(mrpt::poses::CPose3D(
						laser_scan_viz->getPoseX(), laser_scan_viz->getPoseY(), -0.3,
						mrpt::utils::DEG2RAD(laser_scan_viz->getPoseYaw()),
						mrpt::utils::DEG2RAD(laser_scan_viz->getPosePitch()),
						mrpt::utils::DEG2RAD(laser_scan_viz->getPoseRoll())
						));
		}

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	m_time_logger.leave("CLoopCloserERD::Visuals");
	MRPT_END;
}
template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::justInsertedLoopClosure() const {
	return m_just_inserted_loop_closure;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::checkIfInvalidDataset(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	using namespace mrpt::obs;

	if (observation.present()) { // FORMAT #2
		if (IS_CLASS(observation, CObservation2DRangeScan)) {
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
	if (m_consecutive_invalid_format_instances > 
			m_consecutive_invalid_format_instances_thres) {
		m_out_logger.log("Can't find usuable data in the given dataset.\nMake sure dataset contains valid CObservation2DRangeScan/CObservation3DRangeScan data.",
				mrpt::utils::LVL_ERROR);
		mrpt::system::sleep(5000);
		m_checked_for_usuable_dataset = true;
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::dumpVisibilityErrorMsg(
		std::string viz_flag, int sleep_time /* = 500 milliseconds */) {
	MRPT_START;

	m_out_logger.log(format("Cannot toggle visibility of specified object.\n "
			"Make sure that the corresponding visualization flag ( %s "
			") is set to true in the .ini file.\n",
			viz_flag.c_str()).c_str(), mrpt::utils::LVL_ERROR);
	mrpt::system::sleep(sleep_time);

	MRPT_END;
}


template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;

	params.loadFromConfigFileName(source_fname,
			"EdgeRegistrationDeciderParameters");
	range_scanner_t::params.loadFromConfigFile(source, "ICP");

	// set the logging level if given by the user
	mrpt::utils::CConfigFile source(source_fname);
	int min_verbosity_level = source.read_int(
			"EdgeRegistrationDeciderParameters",
			"class_verbosity",
			1, false);
	m_out_logger.setMinLoggingLevel(mrpt::utils::VerbosityLevel(min_verbosity_level));

	m_out_logger.log("Successfully loaded parameters. ");
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::printParams() const {
	MRPT_START;
	params.dumpToConsole();
	range_scanner_t::params.dumpToConsole();

	m_out_logger.log("Printed the relevant parameters");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	std::stringstream class_props_ss;
	class_props_ss << "Pairwise Consistency of ICP Edges - Registration Procedure Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = m_out_logger.getAsString();

	// merge the individual reports
	report_str->clear();

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
}



// TParameter
// //////////////////////////////////


template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TParams::TParams():
	keystroke_laser_scans("l"),
	has_read_config(false)
{ }

template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TParams::~TParams() { }

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;
	// TODO - Implement this for LoopCloser

	out.printf("------------------[Pairwise Consistency of ICP Edges - Registration Procedure Summary]------------------\n");
	out.printf("ICP goodness threshold      = %.2f%% \n", ICP_goodness_thresh*100);
	out.printf("Min. node difference for LC = %d\n", LC_min_nodeid_diff);
	out.printf("Visualize laser scans       = %s\n",
			visualize_laser_scans? "TRUE": "FALSE");

	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
		const std::string& section) {
	MRPT_START;

	LC_min_nodeid_diff = source.read_int(
			"GeneralConfiguration",
			"LC_min_nodeid_diff",
			30, false);
	ICP_goodness_thresh = source.read_double(
			section,
			"ICP_goodness_thresh",
			0.75, false);
	visualize_laser_scans = source.read_bool(
			"VisualizationParameters",
			"visualize_laser_scans",
			true, false);

	// TODO - Implement this for LoopCloser

	has_read_config = true;
	MRPT_END;
}

} } } // end of namespaces

#endif /* end of include guard: CLOOPCLOSERERD_IMPL_H */
