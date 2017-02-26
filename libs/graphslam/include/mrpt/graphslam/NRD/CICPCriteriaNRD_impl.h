/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CICPCRITERIANRD_IMPL_H
#define CICPCRITERIANRD_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CICPCriteriaNRD<GRAPH_t>::CICPCriteriaNRD():
	params(*this) // pass reference to self when initializing the parameters
	//m_mahal_distance_ICP_odom("Mahalanobis dist (ICP - odom)")
{
	this->initCICPCriteriaNRD();
}
template<class GRAPH_t>
void CICPCriteriaNRD<GRAPH_t>::initCICPCriteriaNRD() {
	using namespace mrpt::utils;

	m_first_time_call2D = true;
	m_first_time_call3D = true;
	m_is_using_3DScan = false;

	m_use_angle_difference_node_reg = true;
	m_use_distance_node_reg = true;


	m_graph = NULL;

	// Current node registration decider *decides* how many nodes are there
	// currently in the graph (no need to ask m_graph->nodeCount)..
	m_nodeID_max  = INVALID_NODEID;

	{
		// I am sure of the initial position, set to identity matrix
		double tmp[] = {
			1.0, 0.0, 0.0,
			0.0, 1.0 ,0.0,
			0.0, 0.0, 1.0 };
		InfMat init_path_uncertainty(tmp);
		m_since_prev_node_PDF.cov_inv = init_path_uncertainty;
		m_since_prev_node_PDF.mean = pose_t();
	}

	{
		// I am sure of the initial position, set to identity matrix
		double tmp[] = {
			1.0, 0.0, 0.0,
			0.0, 1.0 ,0.0,
			0.0, 0.0, 1.0 };
		InfMat init_path_uncertainty(tmp);
		// odometry only estimation initialization
		m_latest_odometry_PDF.mean = pose_t();
		m_latest_odometry_PDF.cov_inv = init_path_uncertainty;

		//m_mahal_distance_ICP_odom.resizeWindow(1000); // use the last X mahalanobis distance values
	}

	m_times_used_ICP = 0;
	m_times_used_odom = 0;

	// logger initialization
	this->logging_enable_keep_record = true;
	this->setLoggerName("CICPCriteriaNRD");
	this->logFmt(LVL_DEBUG, "Initialized class object");
}
template<class GRAPH_t>
CICPCriteriaNRD<GRAPH_t>::~CICPCriteriaNRD() {
}

template<class GRAPH_t>
typename GRAPH_t::constraint_t::type_value
CICPCriteriaNRD<GRAPH_t>::getCurrentRobotPosEstimation() const {
	MRPT_START;

	mrpt::utils::TNodeID from = m_nodeID_max;
	return m_graph->nodes.find(from)->second +
		m_since_prev_node_PDF.getMeanVal();

	MRPT_END;
}

template<class GRAPH_t>
bool CICPCriteriaNRD<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	m_time_logger.enter("CICPCriteriaNRD::updateState");

	using namespace mrpt::obs;
	using namespace mrpt::poses;

	bool registered_new_node = false;

	if (observation.present()) { // Observation-Only Rawlog
		// delegate the action to the method responsible
		if (IS_CLASS(observation, CObservation2DRangeScan) ) { // 2D
			mrpt::obs::CObservation2DRangeScanPtr curr_laser_scan =
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);
			registered_new_node = updateState2D(curr_laser_scan);

		}
		else if (IS_CLASS(observation, CObservation3DRangeScan) ) { // 3D
			CObservation3DRangeScanPtr curr_laser_scan =
				static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);
			registered_new_node = updateState3D(curr_laser_scan);
		}
		else if (IS_CLASS(observation, CObservationOdometry) ) { // odometry
			// if it exists use the odometry information to reject wrong ICP matches
			CObservationOdometryPtr obs_odometry =
				static_cast<CObservationOdometryPtr>(observation);

			// not incremental - gives the absolute odometry reading - no InfMat
			// either
			m_curr_odometry_only_pose = obs_odometry->odometry;
			m_latest_odometry_PDF.mean =
				m_curr_odometry_only_pose - m_last_odometry_only_pose;

		}

	}
	else { // action-observations rawlog
		// Action part
		if (action->getBestMovementEstimation()) {
			// if it exists use the odometry information to reject wrong ICP matches
			mrpt::obs::CActionRobotMovement2DPtr robot_move =
				action->getBestMovementEstimation();
			mrpt::poses::CPosePDFPtr increment = robot_move->poseChange.get_ptr();
			mrpt::poses::CPosePDFGaussianInf increment_gaussian;
			increment_gaussian.copyFrom(*increment);
			m_latest_odometry_PDF += increment_gaussian;
		}

		// observations part
		if (observations->getObservationByClass<CObservation2DRangeScan>()) { // 2D
			CObservation2DRangeScanPtr curr_laser_scan =
				observations->getObservationByClass<CObservation2DRangeScan>();
			registered_new_node = updateState2D(curr_laser_scan);
		}
		else if (observations->getObservationByClass<CObservation3DRangeScan>()){	// 3D - EXPERIMENTAL, has not been tested
			CObservation3DRangeScanPtr curr_laser_scan =
				observations->getObservationByClass<CObservation3DRangeScan>();
			registered_new_node = updateState3D(curr_laser_scan);
		}

	}

	// reset the constraint since the last registered node
	if (registered_new_node) {
		m_since_prev_node_PDF = constraint_t();
	}

	// TODO - implement checkIfInvalidDataset
	m_time_logger.leave("CICPCriteriaNRD::updateState");
	return registered_new_node;

	MRPT_END;
}

template<class GRAPH_t>
bool CICPCriteriaNRD<GRAPH_t>::updateState2D(
		mrpt::obs::CObservation2DRangeScanPtr scan2d) {
	MRPT_START;
	bool registered_new_node = false;

	m_curr_laser_scan2D = scan2d;
	if (m_last_laser_scan2D.null()) {
		// initialize the last_laser_scan here - afterwards updated inside the
		// checkRegistrationCondition*D method
		m_last_laser_scan2D = m_curr_laser_scan2D;
	}
	else {
		registered_new_node = checkRegistrationCondition2D();
	}

	return registered_new_node;
	MRPT_END;
}
template<class GRAPH_t>
bool CICPCriteriaNRD<GRAPH_t>::checkRegistrationCondition2D() {
	MRPT_START;

	using namespace mrpt::math;
	using namespace mrpt::utils;

	bool registered_new_node = false;
	this->logFmt(LVL_DEBUG, "In checkRegistrationCondition2D..");

	constraint_t rel_edge;
	mrpt::slam::CICP::TReturnInfo icp_info;

	this->getICPEdge(
			*m_last_laser_scan2D,
			*m_curr_laser_scan2D,
			&rel_edge,
			NULL,
			&icp_info);

	// Debugging directives
	this->logFmt(LVL_DEBUG,
			">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
	this->logFmt(LVL_DEBUG,
			"ICP Alignment operation:\tnIterations: %d\tgoodness: %.f\n",
			icp_info.nIterations, icp_info.goodness);

	this->logFmt(LVL_DEBUG, "Current ICP constraint: \n\tEdge: %s\n\tNorm: %f",
				rel_edge.getMeanVal().asString().c_str(),
				rel_edge.getMeanVal().norm());
	this->logFmt(LVL_DEBUG, "Corresponding Odometry constraint: \n\tEdge: %s\n\tNorm: %f",
				m_latest_odometry_PDF.getMeanVal().asString().c_str(),
				m_latest_odometry_PDF.getMeanVal().norm());

	// evaluate the mahalanobis distance of the above..
	// If over an (adaptive) threshold, trust the odometry
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 1.0 };
	constraint_t latest_odometry_PDF(m_latest_odometry_PDF.getMeanVal(),
			CMatrixDouble33(tmp)/* cov= unity */ );
	double mahal_distance = rel_edge.mahalanobisDistanceTo(latest_odometry_PDF);
	//m_mahal_distance_ICP_odom.addNewMeasurement(mahal_distance);

	// TODO - Find out a proper criterion
	// How do I filter out the "bad" 2DRangeScans?
	//double mahal_distance_lim = m_mahal_distance_ICP_odom.getMedian();
	//double mahal_distance_lim = m_mahal_distance_ICP_odom.getMean();
	//double mahal_distance_lim =
		//m_mahal_distance_ICP_odom.getMean() + m_mahal_distance_ICP_odom.getStdDev();
	double mahal_distance_lim = 0.18; // visual introspection

	// check whether to use ICP or odometry Edge.
	// if the norm of the odometry edge is 0 => no odometry edge available. =>
	// use ICP
	if (mahal_distance < mahal_distance_lim ||
			m_latest_odometry_PDF.getMeanVal().norm() == 0) {
		this->logFmt(LVL_DEBUG, "Using ICP Edge");
		m_times_used_ICP++;
	}
	else {
		this->logFmt(LVL_DEBUG, "Using Odometry Edge");
		rel_edge.copyFrom(m_latest_odometry_PDF);
		m_times_used_odom++;
	}
	this->logFmt(LVL_DEBUG, "\tMahalanobis Distance = %f", mahal_distance);
	this->logFmt(LVL_DEBUG, "Times that the ICP Edge was used: %d/%d",
			m_times_used_ICP, m_times_used_ICP + m_times_used_odom);

	// update the Probability Density Function until last registered
	// node
	m_since_prev_node_PDF += rel_edge;
	m_last_laser_scan2D = m_curr_laser_scan2D;
	registered_new_node = this->checkRegistrationCondition();

	// reset the odometry tracking as well.
	m_last_odometry_only_pose = m_curr_odometry_only_pose;
	m_latest_odometry_PDF.mean = pose_t();


	this->logFmt(LVL_DEBUG,
			"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
	return registered_new_node;
	MRPT_END;
}
template<class GRAPH_t>
bool CICPCriteriaNRD<GRAPH_t>::updateState3D(
		mrpt::obs::CObservation3DRangeScanPtr scan3d) {
	MRPT_START;
	bool registered_new_node = false;

	m_curr_laser_scan3D = scan3d;
	m_curr_laser_scan3D->load();
	m_curr_laser_scan3D->project3DPointsFromDepthImage();

	if (m_last_laser_scan3D.null()) {
		// initialize the last_laser_scan here - afterwards updated inside the
		// checkRegistrationCondition*D method
		m_last_laser_scan3D = m_curr_laser_scan3D;
	}
	else {
		registered_new_node = checkRegistrationCondition3D();
	}

	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
bool CICPCriteriaNRD<GRAPH_t>::checkRegistrationCondition3D() {
	MRPT_START;
	using namespace mrpt::utils;
	bool registered_new_node = false;

	this->logFmt(mrpt::utils::LVL_DEBUG, "In checkRegistrationCondition3D..");

	constraint_t* rel_edge = new constraint_t;
	mrpt::slam::CICP::TReturnInfo icp_info;

	this->getICPEdge(
			*m_last_laser_scan3D,
			*m_curr_laser_scan3D,
			rel_edge,
			NULL,
			&icp_info);

	// Criterions for updating PDF since last registered node
	// - ICP goodness > threshold goodness
	// - Small Z displacement

	if (!rel_edge) {
		this->logFmt(LVL_DEBUG, "NULL rel_edge from getICPEdge procedure");
		return false;
	}

	this->logFmt(LVL_DEBUG, "Current ICP constraint: \n\tEdge: %s\n\tNorm: %f",
				rel_edge->getMeanVal().asString().c_str(),
				rel_edge->getMeanVal().norm());
	std::string debug_msg = mrpt::format(
			"ICP Alignment operation:\n\tnIterations: %d\\n\tquality: %f\n\tgoodness: %.f\n",
			icp_info.nIterations,
			icp_info.quality,
			icp_info.goodness);
		this->logFmt(LVL_DEBUG, "%s", debug_msg.c_str());

	m_since_prev_node_PDF += *rel_edge;
	m_last_laser_scan3D = m_curr_laser_scan3D;
	registered_new_node = this->checkRegistrationCondition();

	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
bool CICPCriteriaNRD<GRAPH_t>::checkRegistrationCondition() {
	MRPT_START;
	using namespace mrpt::utils;

	bool registered_new_node = false;
	this->logFmt(LVL_DEBUG, "In checkRegistrationCondition");
	using namespace mrpt::math;

	// Criterions for adding a new node
	// - Covered distance since last node > registration_max_distance
	// - Angle difference since last node > registration_max_angle

	bool angle_crit = false;
	if (m_use_angle_difference_node_reg) {
		angle_crit = fabs(wrapToPi(m_since_prev_node_PDF.getMeanVal().phi())) >
			params.registration_max_angle;
	}
	bool distance_crit = false;
	if (m_use_distance_node_reg) {
		distance_crit =
			m_since_prev_node_PDF.getMeanVal().norm() > params.registration_max_distance;
	}

	// actual check
	if (distance_crit || angle_crit) {
		registered_new_node = true;
		this->registerNewNode();
	}

	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaNRD<GRAPH_t>::registerNewNode() {
	MRPT_START;

	using namespace mrpt::utils;

	mrpt::utils::TNodeID from = m_nodeID_max;
	mrpt::utils::TNodeID to = ++m_nodeID_max;

	m_graph->nodes[to] = this->getCurrentRobotPosEstimation();
	m_graph->insertEdgeAtEnd(from, to, m_since_prev_node_PDF);

	this->logFmt(LVL_DEBUG,
			"Registered new node:\n\t%lu => %lu\n\tEdge: %s",
			from,
			to,
			m_since_prev_node_PDF.getMeanVal().asString().c_str());

	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaNRD<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	using namespace mrpt::utils;

	m_graph = graph;
	// get the last registrered node + corresponding pose - root
	m_nodeID_max = m_graph->root;

	this->logFmt(LVL_DEBUG, "Fetched the graph successfully");
}
template<class GRAPH_t>
void CICPCriteriaNRD<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;

	using namespace mrpt::utils;

	params.loadFromConfigFileName(source_fname,
			"NodeRegistrationDeciderParameters");
	//m_mahal_distance_ICP_odom.loadFromConfigFileName(source_fname,
			//"NodeRegistrationDeciderParameters");

	// set the logging level if given by the user
	CConfigFile source(source_fname);
	// Minimum verbosity level of the logger
	int min_verbosity_level = source.read_int(
			"NodeRegistrationDeciderParameters",
			"class_verbosity",
			1, false);
	this->setMinLoggingLevel(VerbosityLevel(min_verbosity_level));

	this->logFmt(LVL_DEBUG, "Successfully loaded parameters.");
	MRPT_END;
}
template<class GRAPH_t>
void CICPCriteriaNRD<GRAPH_t>::printParams() const {
	MRPT_START;

	params.dumpToConsole();
	//m_mahal_distance_ICP_odom.dumpToConsole();

	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaNRD<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;

	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "ICP Goodness-based Registration Procedure Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

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
CICPCriteriaNRD<GRAPH_t>::TParams::TParams(decider_t& d):
	decider(d)
{ }
template<class GRAPH_t>
CICPCriteriaNRD<GRAPH_t>::TParams::~TParams() { }
template<class GRAPH_t>
void CICPCriteriaNRD<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	using namespace mrpt::utils;
	using namespace mrpt::math;

	out.printf("------------------[ ICP Fixed Intervals Node Registration ]------------------\n");
	out.printf("Max distance for registration = %.2f m\n",
			registration_max_distance);
	out.printf("Max Angle for registration    = %.2f deg\n",
			RAD2DEG(registration_max_angle));

	decider.range_scanner_t::params.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CICPCriteriaNRD<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
		const std::string &section) {
	MRPT_START;

	using namespace mrpt::utils;
	using namespace mrpt::math;

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

} } } // end of namespace


#endif /* end of include guard: CICPCRITERIANRD_IMPL_H */
