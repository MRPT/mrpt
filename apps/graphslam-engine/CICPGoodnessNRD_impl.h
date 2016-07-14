/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPGOODNESSNRD_IMPL_H
#define CICPGOODNESSNRD_IMPL_H

using namespace mrpt::graphslam::deciders;

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::CICPGoodnessNRD_t():
	params(*this) // pass reference to self when initializing the parameters
{
	this->initCICPGoodnessNRD_t();
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::initCICPGoodnessNRD_t() {

	m_first_time_call2D = true;
	m_first_time_call3D = true;
	m_is_using_3DScan = false;

	m_graph = NULL;

	// Current node registration decider *decides* how many nodes are there
	// currently in the graph (no need to ask m_graph->nodeCount..
	m_nodeID_max  = INVALID_NODEID;

	m_curr_timestamp = INVALID_TIMESTAMP;

	// I am sure of the initial position, set to identity matrix
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 0.0 };
	InfMat init_path_uncertainty(tmp);
	m_since_prev_node_PDF.cov_inv = init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();

	m_out_logger.setName("CICPGoodnessNRD");
	m_out_logger.setLoggingLevel(LVL_DEBUG);

	m_out_logger.log("Initialized class object", LVL_DEBUG);
}
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::~CICPGoodnessNRD_t() { }

template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::updateDeciderState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {
	MRPT_START;
	m_time_logger.enter("CICPGoodnessNRD::updateDeciderState");

	bool registered_new_node = false;

	MRPT_UNUSED_PARAM(action);

	if (observation.present()) { // Observation-Only Rawlog


		// 3D Range Scan
		if (IS_CLASS(observation, CObservation3DRangeScan)) {
			m_curr_laser_scan3D =
				static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);
			m_curr_laser_scan3D->load();
			m_curr_laser_scan3D->project3DPointsFromDepthImage();

			// first_time call 
			// Initialize the m_last_laser_scan as well
			if (m_first_time_call3D) {
				m_last_laser_scan3D = m_curr_laser_scan3D;
				m_first_time_call3D = false;

				// do not check for node registration - not enough data yet.
				m_time_logger.leave("CICPGoodnessNRD::updateDeciderState");
				return false;
			}

			m_is_using_3DScan = true;
		}
		// 2D Range Scan
		else if (IS_CLASS(observation, CObservation2DRangeScan)) {
			m_curr_laser_scan2D =
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);

			// first_time call 
			// Initialize the m_last_laser_scan as well
			if (m_first_time_call2D) {
				m_last_laser_scan2D = m_curr_laser_scan2D;
				m_first_time_call2D = false;

				// do not check for node registration - not enough data yet.
				m_time_logger.leave("CICPGoodnessNRD::updateDeciderState");
				return false;
			}

			m_is_using_3DScan = false;
		}

		if (IS_CLASS(observation, CObservation3DRangeScan) ||
				IS_CLASS(observation, CObservation2DRangeScan) ) {
			// Ignore the timestamps of the CObservationOdometry objects in the
			// datasets. They aren't always in acending order when combined with the
			// CObaservation*DRangeScan objects
			m_curr_timestamp = observation->timestamp;
			m_prev_timestamp = m_curr_timestamp;

		}

	}
	else { // Action/Observations Rawlog
		// update the timestamps
		CActionRobotMovement2DPtr robot_move =
			action->getBestMovementEstimation();
		if (robot_move) {
			m_prev_timestamp = m_curr_timestamp;
			m_curr_timestamp = robot_move->timestamp;
		}

		// 2D Range Scan
		m_curr_laser_scan2D =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (m_curr_laser_scan2D) {

			// first_time call 
			// Initialize the m_last_laser_scan as well
			if (m_first_time_call2D) {
				m_last_laser_scan2D = m_curr_laser_scan2D;
				m_first_time_call2D = false;

				// do not check for node registration - not enough data yet.
				m_time_logger.leave("CICPGoodnessNRD::updateDeciderState");
				return false;
			}

			m_is_using_3DScan = false;
		}
		// 3D Range Scan
		m_curr_laser_scan3D =
			observations->getObservationByClass<CObservation3DRangeScan>();
		if (m_curr_laser_scan3D) {

			// first_time call 
			// Initialize the m_last_laser_scan as well
			if (m_first_time_call3D) {
				m_last_laser_scan3D = m_curr_laser_scan3D;
				m_first_time_call3D = false;

				// do not check for node registration - not enough data yet.
				m_time_logger.leave("CICPGoodnessNRD::updateDeciderState");
				return false;
			}

			m_is_using_3DScan = true;
		}

	}

	registered_new_node = this->checkRegistrationCondition();

	// reset the relative PDF since the previous registered node
	if (registered_new_node) {
		m_since_prev_node_PDF = constraint_t();
	}

	// TODO - implement checkIfInvalidDataset

	m_time_logger.leave("CICPGoodnessNRD::updateDeciderState");
	return registered_new_node;

	MRPT_END;
}

template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::checkRegistrationCondition() {
	MRPT_START;

	m_out_logger.log("In checkRegistrationCondition2D..");
	bool registered_new_node = false;



	// get an initial estimation of your next ICP constraint using the
	// pose_estimator object
	pose_t curr_estimated_pose;
	pose_t initial_ICP_estimation;
	//bool is_trusted = pose_estimator.getLatestRobotPose(curr_estimated_pose);

	//if (is_trusted)
		//initial_ICP_estimation


	constraint_t rel_edge;
	mrpt::slam::CICP::TReturnInfo icp_info;
	// decide on which data to use.
	if ( m_is_using_3DScan ) {
		this->getICPEdge(
				*m_last_laser_scan3D,
				*m_curr_laser_scan3D,
				&rel_edge,
				NULL,
				&icp_info);
	}
	else {
		this->getICPEdge(
				*m_last_laser_scan2D,
				*m_curr_laser_scan2D,
				&rel_edge,
				NULL,
				&icp_info);
	}

	// append current ICP edge to the sliding window
	m_ICP_sliding_win.addNewMeasurement(icp_info.goodness);
	//m_ICP_sliding_win.dumpToConsole();

	m_out_logger.log(mrpt::format(
				"Current ICP constraint: \n\tEdge: %s\n\tNorm: %f", 
				rel_edge.getMeanVal().asString().c_str(), 
				rel_edge.getMeanVal().norm()), LVL_DEBUG);

	// Criterions for updating PDF since last registered node
	// - ICP goodness > threshold goodness
	if (m_ICP_sliding_win.evaluateICPgoodness(icp_info.goodness) ) {
		m_since_prev_node_PDF += rel_edge;

		// udpate the last laser scan
		if (m_is_using_3DScan) {
			m_last_laser_scan3D = m_curr_laser_scan3D;
		}
		else {
			m_last_laser_scan2D = m_curr_laser_scan2D;
		}

		// Update the pose estimator object
		m_out_logger.log("Updating the pose_estimator object... ", LVL_DEBUG);
		pose_t curr_estimated_pose;
		pose_estimator.getLatestRobotPose(curr_estimated_pose);
		curr_estimated_pose += rel_edge.getMeanVal();

// 		 // update the pose estimation
// 		 //TODO have an estimation of velocity of your own. See Giannakoglou notes..
// 		 // Not all timestamps are in correct order..
// 		pose_estimator.processUpdateNewOdometry(
// 				curr_estimated_pose,
// 				m_curr_timestamp,
// 				/* hasVelocities = */ false);
		

		// Criterions for adding a new node
		// - Covered distance since last node > registration_max_distance
		// - Angle difference since last node > registration_max_angle
		bool use_angle_difference_node_reg = true;
		bool use_distance_node_reg = true;

		bool angle_crit = false;
		if (use_angle_difference_node_reg) {
			angle_crit = fabs(wrapToPi(m_since_prev_node_PDF.getMeanVal().phi())) >
				params.registration_max_angle;
		}
		bool max_registration_distance_crit = false;
		if (use_distance_node_reg) {
			max_registration_distance_crit =
				m_since_prev_node_PDF.getMeanVal().norm() > params.registration_max_distance;
		}

		if (max_registration_distance_crit || angle_crit) {

			registered_new_node = true;
			this->registerNewNode();
		}
	}

	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::registerNewNode() {
	MRPT_START;

	mrpt::utils::TNodeID from = m_nodeID_max;
	mrpt::utils::TNodeID to = ++m_nodeID_max;

	m_out_logger.log(mrpt::format("Registered new node:\n\t%lu => %lu\n\tEdge: %s",
				from, to, m_since_prev_node_PDF.getMeanVal().asString().c_str()), LVL_DEBUG);

	m_graph->nodes[to] = m_graph->nodes[from] + m_since_prev_node_PDF.getMeanVal();
  m_graph->insertEdgeAtEnd(from, to, m_since_prev_node_PDF);

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	m_graph = graph;

	// get the last registrered node + corresponding pose - root
	m_nodeID_max = m_graph->root;

	m_out_logger.log("CICPGoodnessNRD: Fetched the graph successfully", LVL_DEBUG);
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;

	params.loadFromConfigFileName(source_fname,
			"NodeRegistrationDeciderParameters");
	m_ICP_sliding_win.loadFromConfigFileName(source_fname,
			"NodeRegistrationDeciderParameters");

	// set the logging level if given by the user
	CConfigFile source(source_fname);
	// Minimum verbosity level of the logger
	int min_verbosity_level = source.read_int(
			"NodeRegistrationDeciderParameters",
			"class_verbosity",
			1, false);
	m_out_logger.setMinLoggingLevel(VerbosityLevel(min_verbosity_level));

	m_out_logger.log("Successfully loaded parameters.", LVL_DEBUG);

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::printParams() const {
	MRPT_START;

	params.dumpToConsole();
	m_ICP_sliding_win.dumpToConsole();

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "ICP Goodness-based Registration Procedure Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = m_out_logger.getAsString();

	// merge the individual reports
	report_str->clear();

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	// loggers results
	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
}


// TParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::TParams::TParams(decider_t& d):
	decider(d)
{ }
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::TParams::~TParams() { }
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ ICP Fixed Intervals Node Registration ]------------------\n");
	out.printf("Max distance for registration = %.2f m\n",
			registration_max_distance);
	out.printf("Max Angle for registration    = %.2f deg\n",
			RAD2DEG(registration_max_angle));

	decider.range_scanner_t::params.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
  MRPT_START;

	registration_max_distance = source.read_double( section,
			"registration_max_distance",
			0.5 /* meter */, false);
	registration_max_angle = source.read_double( section,
			"registration_max_angle",
			10 /* degrees */, false);
	registration_max_angle = DEG2RAD(registration_max_angle);

	// load the icp parameters - from "ICP" section explicitly
	decider.range_scanner_t::params.loadFromConfigFile(source, "ICP");

	MRPT_END;
}


#endif /* end of include guard: CICPGOODNESSNRD_IMPL_H */
