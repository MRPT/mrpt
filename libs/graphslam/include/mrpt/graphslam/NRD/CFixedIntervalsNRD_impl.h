/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CFIXEDINTERVALSNRD_IMPL_H
#define CFIXEDINTERVALSNRD_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CFixedIntervalsNRD<GRAPH_t>::CFixedIntervalsNRD():
	m_consecutive_invalid_format_instances_thres(20) // large threshold just to make sure
{
	this->initCFixedIntervalsNRD();
}
template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::initCFixedIntervalsNRD() {
	using namespace mrpt::utils;

	m_win = NULL;
	m_graph = NULL;

	m_prev_registered_node = INVALID_NODEID;

	// I am sure of the initial position, set to identity matrix
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 1.0 };
	InfMat init_path_uncertainty(tmp);
	m_since_prev_node_PDF.cov_inv = init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();

	m_checked_for_usuable_dataset = false;
	m_consecutive_invalid_format_instances = 0;

	this->logging_enable_keep_record = true;
	this->setLoggerName("CFixedIntervalsNRD");
	this->setMinLoggingLevel(LVL_DEBUG);

	this->logFmt(LVL_DEBUG, "IntervalsNRD: Initialized class object");
}
template<class GRAPH_t>
CFixedIntervalsNRD<GRAPH_t>::~CFixedIntervalsNRD() { }

// Member function implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
typename GRAPH_t::constraint_t::type_value
CFixedIntervalsNRD<GRAPH_t>::getCurrentRobotPosEstimation() const {
	return m_curr_estimated_pose;
}

template<class GRAPH_t>
bool CFixedIntervalsNRD<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {
	MRPT_START;
	using namespace mrpt::obs;
	using namespace mrpt::math;
	using namespace mrpt::utils;

	//cout << "in updateState..." << endl;
	// don't use the measurements in this implementation
	MRPT_UNUSED_PARAM(observations);

	if (observation.present()) { // FORMAT #2
		m_observation_only_rawlog = true;

		if (IS_CLASS(observation, CObservationOdometry)) {

			CObservationOdometryPtr obs_odometry =
				static_cast<CObservationOdometryPtr>(observation);
			// not incremental - gives the absolute odometry reading
			m_curr_odometry_only_pose = obs_odometry->odometry;
			this->logFmt(LVL_DEBUG, "Current odometry-only pose: %s",
					m_curr_odometry_only_pose.asString().c_str());

			// I don't have any information about the covariane of the move in
			// observation-only format
			m_since_prev_node_PDF.mean =
				m_curr_odometry_only_pose - m_last_odometry_only_pose;
		}
	} // IF FORMAT #2 - observation-only
	else { // FORMAT #1
		m_observation_only_rawlog = false;

		if (action->getBestMovementEstimation() ) {
			mrpt::obs::CActionRobotMovement2DPtr robot_move = action->getBestMovementEstimation();
			mrpt::poses::CPosePDFPtr increment = robot_move->poseChange.get_ptr();
			pose_t increment_pose = increment->getMeanVal();
			InfMat increment_inf_mat;
			increment->getInformationMatrix(increment_inf_mat);

			// update the relative PDF of the path since the LAST node was inserted
			constraint_t incremental_constraint(increment_pose, increment_inf_mat);
			m_since_prev_node_PDF += incremental_constraint;
		}
	} // ELSE - FORMAT #1

	m_curr_estimated_pose = m_graph->nodes[m_prev_registered_node] +
		m_since_prev_node_PDF.getMeanVal();

	bool registered = this->checkRegistrationCondition();

	if (registered) {
		if (m_observation_only_rawlog) {
			// keep track of the odometry-only pose_t at the last inserted graph node
			m_last_odometry_only_pose = m_curr_odometry_only_pose;
		}
		m_since_prev_node_PDF = constraint_t();
	}

	if (!m_checked_for_usuable_dataset) {
		this->checkIfInvalidDataset(action, observations, observation );
	}

	return registered;

	MRPT_END;
}

template<class GRAPH_t>
bool CFixedIntervalsNRD<GRAPH_t>::checkRegistrationCondition() {
	MRPT_START;
	using namespace mrpt::math;

	bool registered = false;

	//cout << "in checkRegistrationCondition..." << endl;

	pose_t last_pose_inserted = m_graph->nodes[m_prev_registered_node];

	// odometry criterion
	if ( (last_pose_inserted.distanceTo(m_curr_estimated_pose)
				> params.registration_max_distance) ||
			(fabs(wrapToPi(last_pose_inserted.phi() - m_curr_estimated_pose.phi()))
			 > params.registration_max_angle ) ) {

		// register the new node
		registered = true;
		this->registerNewNode();
	}

	return registered;

	MRPT_END;
}

template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::registerNewNode() {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;

	this->logFmt(LVL_DEBUG, "In registerNewNode...");

	mrpt::utils::TNodeID from = m_prev_registered_node;
	mrpt::utils::TNodeID to = ++m_prev_registered_node;

	m_graph->nodes[to] = m_graph->nodes[from] + m_since_prev_node_PDF.getMeanVal();
	m_graph->insertEdgeAtEnd(from, to, m_since_prev_node_PDF);

	MRPT_LOG_DEBUG_STREAM << "Registered new node:" << endl <<
		"\t" << from << " => " << to << endl <<
		"\tEdge: " << m_since_prev_node_PDF.getMeanVal().asString();

	MRPT_END;
}
template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	using namespace mrpt::utils;

	m_graph = graph;
	// get the last registrered node + corresponding pose - root
	m_prev_registered_node = m_graph->root;
	this->logFmt(LVL_DEBUG, "Fetched the graph successfully");
}

template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::checkIfInvalidDataset(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::utils;

	MRPT_UNUSED_PARAM(observations);
	MRPT_UNUSED_PARAM(action);

	if (observation.present()) { // FORMAT #2
		if (IS_CLASS(observation, CObservationOdometry)) {
			m_checked_for_usuable_dataset = true;
			return;
		}
		else {
			m_consecutive_invalid_format_instances++;
		}
	}
	else {
		// TODO - make a real check here
		m_checked_for_usuable_dataset = true;
		return;
	}

	if (m_consecutive_invalid_format_instances > m_consecutive_invalid_format_instances_thres) {
		this->logFmt(LVL_ERROR,
				"Can't find usuable data in the given dataset.\nMake sure dataset contains valid odometry data.");
		mrpt::system::sleep(5000);
		m_checked_for_usuable_dataset = true;
	}

	MRPT_END;
}

template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;
	using namespace mrpt::utils;

	params.loadFromConfigFileName(source_fname,
			"NodeRegistrationDeciderParameters");

	// set the logging level if given by the user
	CConfigFile source(source_fname);
	int min_verbosity_level = source.read_int(
			"NodeRegistrationDeciderParameters",
			"class_verbosity",
			1, false);
	this->setMinLoggingLevel(VerbosityLevel(min_verbosity_level));


	this->logFmt(LVL_DEBUG, "Successfully loaded parameters.");

	MRPT_END;
}

template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::printParams() const {
	MRPT_START;
	params.dumpToConsole();

	MRPT_END;
}

template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;
	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "Fixed Intervals odometry-based Node Registration Decider Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	// configuration parameters
	*report_str += params.getAsString();
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
CFixedIntervalsNRD<GRAPH_t>::TParams::TParams() {
}
template<class GRAPH_t>
CFixedIntervalsNRD<GRAPH_t>::TParams::~TParams() {
}
template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;
	out.printf("%s", this->getAsString().c_str());
	MRPT_END;
}
template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
		const std::string &section) {
	MRPT_START;
	using namespace mrpt::math;
	using namespace mrpt::utils;

	registration_max_distance = source.read_double( section,
			"registration_max_distance",
			0.5 /* meter */, false);
	registration_max_angle = source.read_double( section,
			"registration_max_angle",
			60 /* degrees */, false);
	registration_max_angle = DEG2RAD(registration_max_angle);

	MRPT_END;
}

template<class GRAPH_t>
void CFixedIntervalsNRD<GRAPH_t>::TParams::getAsString(std::string* params_out) const {
	MRPT_START;
	using namespace mrpt::math;
	using namespace mrpt::utils;

	double max_angle_deg = RAD2DEG(registration_max_angle);
	params_out->clear();

	*params_out += "------------------[ Fixed Intervals Node Registration ]------------------\n";
	*params_out += mrpt::format("Max distance for registration = %.2f m\n", registration_max_distance);
	*params_out += mrpt::format("Max angle for registration    = %.2f deg\n", max_angle_deg);

	MRPT_END;
}
template<class GRAPH_t>
std::string CFixedIntervalsNRD<GRAPH_t>::TParams::getAsString() const {
	MRPT_START;

	std::string str;
	this->getAsString(&str);
	return str;

	MRPT_END;
}

} } } // end of namespaces

#endif /* end of include guard: CFIXEDINTERVALSNRD_IMPL_H */
