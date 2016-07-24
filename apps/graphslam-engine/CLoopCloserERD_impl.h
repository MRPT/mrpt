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
	m_consecutive_invalid_format_instances_thres(20), // high threshold just to make sure
	m_balloon_elevation(3),
	m_balloon_radius(0.5),
	m_balloon_std_color(153, 0, 153),
	m_balloon_curr_color(102, 0, 102),
	m_connecting_lines_color(m_balloon_std_color)
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
	m_threshold_to_start = m_last_total_num_of_nodes = 0;

	m_edge_types_to_nums["ICP2D"] = 0;
	m_edge_types_to_nums["LC"] = 0;

	m_checked_for_usuable_dataset = false;
	m_consecutive_invalid_format_instances = 0;

	m_partitions_full_update = false;

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
	m_time_logger.enter("CLoopCloserERD::updateState");
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::opengl;
	using namespace mrpt::poses;

	// check possible prior node registration
	bool registered_new_node = false;

	// was a new node registered?
	if (m_last_total_num_of_nodes < m_graph->nodeCount()) {
		registered_new_node = true;
		m_last_total_num_of_nodes = m_graph->nodeCount();
		m_out_logger.log("New node has been registered!");
	}

	if (observation.present()) { // observation-only rawlog format
		if (IS_CLASS(observation, CObservation2DRangeScan)) {
			// update last laser scan to use
			m_last_laser_scan2D = 
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);

		}
	}
	else { // action-observations format
		// TODO - going to implement this.
	}


	if (registered_new_node) {
		// register the new node-laserScan pair
		m_nodes_to_laser_scans2D[m_graph->nodeCount()-1] = m_last_laser_scan2D;

		// update the partitioned map
		if ((m_graph->nodeCount() % 50) == 0) {
			m_partitions_full_update = true;
		}
		else {
			m_partitions_full_update = false;
		}
		this->updateMapPartitions(m_partitions_full_update);

	}

	m_time_logger.enter("CLoopCloserERD::updateState");
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
	if (events_occurred.at(params.keystroke_laser_scans)) {
		this->toggleLaserScansVisualization();
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initMapPartitionsVisualization() {
	using namespace mrpt;
	using namespace mrpt::gui;
	using namespace mrpt::math;
	using namespace mrpt::opengl;

	// textmessage
	m_win_manager->assignTextMessageParameters(
			/* offset_y*	= */ &m_offset_y_map_partitions,
			/* text_index* = */ &m_text_index_map_partitions);


	// just add an empty CSetOfObjects in the scene - going to populate it later
	CSetOfObjectsPtr map_partitions_obj = CSetOfObjects::Create();
	map_partitions_obj->setName("map_partitions");

	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();
	scene->insert(map_partitions_obj);
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateMapPartitionsVisualization() {
	using namespace mrpt;
	using namespace mrpt::gui;
	using namespace mrpt::math;
	using namespace mrpt::opengl;
	using namespace mrpt::poses;

	// textmessage
	std::stringstream title;
	title << "# Partitions: " << m_curr_partitions.size();
	m_win_manager->addTextMessage(5,-m_offset_y_map_partitions,
			title.str(),
			mrpt::utils::TColorf(m_balloon_std_color),
			/* unique_index = */ m_text_index_map_partitions);

	// update the partitioning visualization
	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

	// fetch the partitions CSetOfObjects
	CSetOfObjectsPtr map_partitions_obj;
	{
		CRenderizablePtr obj = scene->getByName("map_partitions");
		// do not check for null ptr - must be properly created in the init* method
		map_partitions_obj = static_cast<CSetOfObjectsPtr>(obj);
	}

	int partitionID = 0;
	bool partition_contains_last_node = false;
	for (partitions_t::const_iterator p_it = m_curr_partitions.begin();
			p_it != m_curr_partitions.end(); ++p_it, ++partitionID) {
		m_out_logger.logFmt("Working on Partition #%d", partitionID);
		vector_uint nodes_list = *p_it;

		// fetch the current partition object if it exists - create otherwise
		std::string partition_obj_name = mrpt::format("partition_%d", partitionID);
		std::string balloon_obj_name = mrpt::format("#%d", partitionID);

		CRenderizablePtr obj = map_partitions_obj->getByName(partition_obj_name);
		CSetOfObjectsPtr curr_partition_obj;
		if (obj) {
			m_out_logger.logFmt(
					"\tFetching CSetOfObjects partition object for partition #%d",
					partitionID);
			curr_partition_obj = static_cast<CSetOfObjectsPtr>(obj);
		}
		else {
			m_out_logger.logFmt(
					"\tCreating a new CSetOfObjects partition object for partition #%d",
					partitionID);
			curr_partition_obj = CSetOfObjects::Create();
			curr_partition_obj->setName(partition_obj_name);

			m_out_logger.logFmt("\t\tCreating a new CSphere balloon object");
			CSpherePtr balloon_obj = CSphere::Create();
			balloon_obj->setName(balloon_obj_name);
			balloon_obj->setRadius(m_balloon_radius);
			balloon_obj->setColor_u8(m_balloon_std_color);
			balloon_obj->enableShowName();

			curr_partition_obj->insert(balloon_obj);

			// set of lines connecting the graph nodes to the balloon
			m_out_logger.logFmt("\t\tCreating set of lines that will connect to the Balloon");
			CSetOfLinesPtr connecting_lines_obj = CSetOfLines::Create();
			connecting_lines_obj->setName("connecting_lines");
			connecting_lines_obj->setColor_u8(m_connecting_lines_color);
			connecting_lines_obj->setLineWidth(0.4);

			curr_partition_obj->insert(connecting_lines_obj);

			// add the created CSetOfObjects to the total CSetOfObjects responsible
			// for the map partitioning
			map_partitions_obj->insert(curr_partition_obj);
			m_out_logger.logFmt("\tInserted new CSetOfObjects successfully");
		}
		// up to now the CSetOfObjects exists and the balloon inside it as well..

		std::pair<double, double> centroid_coords;
		this->computeCentroidOfNodesVector(nodes_list, &centroid_coords);

		// finding the partition in which the last node is in
		if (std::find(nodes_list.begin(), nodes_list.end(), m_graph->nodeCount()-1)
				!= nodes_list.end()) {
			partition_contains_last_node = true;
		}
		else {
			partition_contains_last_node = false;
		}
		TPoint3D balloon_location(centroid_coords.first, centroid_coords.second,
				m_balloon_elevation);

		m_out_logger.logFmt("\tUpdating the balloon position");
		// set the balloon properties
		CSpherePtr balloon_obj;
		{
			// place the partitions baloon at the centroid elevated by a fixed Z value
			CRenderizablePtr obj = curr_partition_obj->getByName(balloon_obj_name);
			balloon_obj = static_cast<CSpherePtr>(obj);
			balloon_obj->setLocation(balloon_location);
			if (partition_contains_last_node)
				balloon_obj->setColor_u8(m_balloon_curr_color);
			else
				balloon_obj->setColor_u8(m_balloon_std_color);
		}

		m_out_logger.logFmt("\tUpdating the lines connecting nodes to balloon");
		// set the lines connecting the nodes of the partition to the partition
		// balloon - set it from scratch all the times since the node positions
		// tend to change according to the dijkstra position estimation
		CSetOfLinesPtr connecting_lines_obj;
		{
			// place the partitions baloon at the centroid elevated by a fixed Z value
			CRenderizablePtr obj = curr_partition_obj->getByName("connecting_lines");
			connecting_lines_obj = static_cast<CSetOfLinesPtr>(obj);

			connecting_lines_obj->clear();

			for (vector_uint::const_iterator it = nodes_list.begin();
					it != nodes_list.end(); ++it) {
				CPose3D curr_pose(m_graph->nodes.at(*it));
				TPoint3D curr_node_location(curr_pose);

				TSegment3D connecting_line(curr_node_location, balloon_location);
				connecting_lines_obj->appendLine(connecting_line);
			}

		}
		m_out_logger.logFmt("Done working on partition #%d", partitionID);
	}
	m_out_logger.logFmt("Done working on the partitions visualization.");


	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::computeCentroidOfNodesVector(const vector_uint& nodes_list,
		std::pair<double, double>* centroid_coords) {
	MRPT_START;

	// get the poses and find the centroid so that we can place the baloon over
	// and at their center
	double centroid_x = 0;
	double centroid_y = 0;
	for (vector_uint::const_iterator node_it = nodes_list.begin();
			node_it != nodes_list.end(); ++node_it) {
		pose_t curr_node_pos = m_graph->nodes.at(*node_it);
		centroid_x +=  curr_node_pos.x();
		centroid_y +=  curr_node_pos.y();

	}

	// normalize by the size - assign to the given pair
	centroid_coords->first = centroid_x/static_cast<double>(nodes_list.size());
	centroid_coords->second = centroid_y/static_cast<double>(nodes_list.size());

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initLaserScansVisualization() {
	MRPT_START;

	m_win_observer->registerKeystroke(params.keystroke_laser_scans,
			"Toggle LaserScans Visualization");

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

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateLaserScansVisualization() {
	MRPT_START;

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

	// TODO - include visualization of the partitioning process
	// TODO - include visualization of the Olson LC
	// TODO - indicate number of node groups

	this->initLaserScansVisualization();
	this->initMapPartitionsVisualization();

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

	this->updateLaserScansVisualization();
	this->updateMapPartitionsVisualization();

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

	m_partitioner.options.loadFromConfigFileName(source_fname, "EdgeRegistrationDeciderParameters");
	params.loadFromConfigFileName(source_fname, "EdgeRegistrationDeciderParameters");
	range_scanner_t::params.loadFromConfigFileName(source_fname, "ICP");

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
	m_partitioner.options.dumpToConsole();
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

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateMapPartitions(bool full_update /* = false */) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;

	m_time_logger.enter("CLoopCloser:updateMapPartitions");
	
 nodes_to_scans2D_t nodes_to_scans;
	if (full_update) {
		m_out_logger.log("updateMapPartitions: Full partitionint of map was issued", LVL_INFO);

		// clear the existing partitions and recompute the partitioned map for all
		// the nodes
		m_partitioner.clear();
		nodes_to_scans = m_nodes_to_laser_scans2D;
	}
	else {
		// just use the last node-laser scan pair
		nodes_to_scans[0] = m_nodes_to_laser_scans2D.at(m_graph->nodeCount()-1);
	}

	// for each one of the above nodes - add its position and correspoding
	// laserScan to the partitioner object
	for (nodes_to_scans2D_t::const_iterator it = nodes_to_scans.begin();
			it != nodes_to_scans.end(); ++it) {
		if ((it->second).null()) { continue; } // if laserScan invalud go to next...


		// pose
		pose_t curr_pose = m_graph->nodes.at(it->first);
		mrpt::poses::CPosePDFPtr posePDF(new constraint_t(curr_pose));

		// laser scan
		mrpt::obs::CSensoryFramePtr sf = mrpt::obs::CSensoryFrame::Create();
		sf->insert(it->second);

		m_partitioner.addMapFrame(sf, posePDF);
	}

	// update the last partitions list
	size_t n = m_curr_partitions.size();
	m_last_partitions.resize(n);
	for (int i = 0; i < n; i++)	{
		m_last_partitions[i] = m_curr_partitions[i];
	}
	//update current partitions list
	m_partitioner.updatePartitions(m_curr_partitions);

	m_out_logger.log("Updated map partitions successfully.");
	m_time_logger.leave("CLoopCloser:updateMapPartitions");
	MRPT_END;
}


template<class GRAPH_t>
template<class T>
void CLoopCloserERD<GRAPH_t>::printVectorOfVectors(const T& t) const{
	int i = 0;
	for (typename T::const_iterator it = t.begin(); it  != t.end(); ++i, ++it) {
		printf("Vector %d/%lu:\n\t", i, t.size());
		this->printVector(*it);
	}
}

// TODO - what happesn if I have them the opposite way?
template<class GRAPH_t>
template<class T>
void CLoopCloserERD<GRAPH_t>::printVector(const T& t) const {
	for (typename T::const_iterator it = t.begin(); it != t.end(); ++it) {
		std::cout << *it << ", ";
	}
	std::cout << std::endl;

}

// TODO - either use it or lose it
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
