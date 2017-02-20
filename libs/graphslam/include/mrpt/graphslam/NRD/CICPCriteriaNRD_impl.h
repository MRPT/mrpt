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

template<class GRAPH_T>
CICPCriteriaNRD<GRAPH_T>::CICPCriteriaNRD():
	params(*this) // pass reference to self when initializing the parameters
	//m_mahal_distance_ICP_odom("Mahalanobis dist (ICP - odom)")
{
	using namespace mrpt::utils;
	using namespace mrpt::math;
	this->initializeLoggers("CICPCriteriaNRD");

	m_is_using_3DScan = false;

	m_use_angle_difference_node_reg = true;
	m_use_distance_node_reg = true;
	this->resetPDF(&m_latest_odometry_PDF);

	//m_mahal_distance_ICP_odom.resizeWindow(1000); // use the last X mahalanobis distance values

	m_times_used_ICP = 0;
	m_times_used_odom = 0;

	this->logFmt(LVL_DEBUG, "Initialized class object");
}
template<class GRAPH_T>
CICPCriteriaNRD<GRAPH_T>::~CICPCriteriaNRD() {
}

template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	this->m_time_logger.enter("updateState");

	using namespace mrpt::obs;
	using namespace mrpt::poses;

	bool registered_new_node = false;

	if (observation.present()) { // Observation-Only Rawlog
		// delegate the action to the method responsible
		if (IS_CLASS(observation, CObservation2DRangeScan) ) { // 2D
			mrpt::obs::CObservation2DRangeScanPtr curr_laser_scan =
				static_cast<CObservation2DRangeScanPtr>(observation);
			registered_new_node = updateState2D(curr_laser_scan);

		}
		else if (IS_CLASS(observation, CObservation3DRangeScan) ) { // 3D
			CObservation3DRangeScanPtr curr_laser_scan =
				static_cast<CObservation3DRangeScanPtr>(observation);
			registered_new_node = updateState3D(curr_laser_scan);
		}
		else if (IS_CLASS(observation, CObservationOdometry) ) { // odometry
			// if it exists use the odometry information to reject wrong ICP
			// matches
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
			CActionRobotMovement2DPtr robot_move =
				action->getBestMovementEstimation();
			CPosePDFPtr increment = robot_move->poseChange.get_ptr();
			CPosePDFGaussianInf increment_gaussian;
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

	this->m_time_logger.leave("updateState");
	return registered_new_node;

	MRPT_END;
} // end of updateState

template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::updateState2D(
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
} // end of updateState2D

template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::checkRegistrationCondition2D() {
	MRPT_START;

	using namespace mrpt::math;
	using namespace mrpt::utils;

	bool registered_new_node = false;

	// Constraint that *may* update incrementally the m_since_prev_node_PDF.
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
	double mahal_distance = rel_edge.mahalanobisDistanceTo(m_latest_odometry_PDF);
	//m_mahal_distance_ICP_odom.addNewMeasurement(mahal_distance);

	// TODO - Find out a proper criterion
	// How do I filter out the "bad" 2DRangeScans?
	//double mahal_distance_lim = m_mahal_distance_ICP_odom.getMedian();
	//double mahal_distance_lim = m_mahal_distance_ICP_odom.getMean();
	//double mahal_distance_lim =
		//m_mahal_distance_ICP_odom.getMean() + m_mahal_distance_ICP_odom.getStdDev();
	double mahal_distance_lim = 0.18; // visual introspection

	//
	// check whether to use ICP or odometry Edge.
	//
	// if the norm of the odometry edge is 0, no odometry edge available
	// => use ICP
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

	// update the PDF until last registered node
	this->m_since_prev_node_PDF += rel_edge;
	m_last_laser_scan2D = m_curr_laser_scan2D;
	registered_new_node = this->checkRegistrationCondition();

	// reset the odometry tracking as well.
	m_last_odometry_only_pose = m_curr_odometry_only_pose;
	this->resetPDF(&m_latest_odometry_PDF);

	this->logFmt(LVL_DEBUG,
			"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
	return registered_new_node;
	MRPT_END;
} // end of checkRegistrationCondition2D

template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::updateState3D(
		mrpt::obs::CObservation3DRangeScanPtr scan3d) {
	THROW_EXCEPTION("Not yet implemented.");
	return false;
} // end of updateState3D

template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::checkRegistrationCondition3D() {
	THROW_EXCEPTION("Not yet implemented.");
	return false;
} // end of checkRegistrationCondition3D


template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::checkRegistrationCondition() {
	MRPT_START;
	using namespace mrpt::utils;
	this->logFmt(LVL_DEBUG, "In checkRegistrationCondition");
	using namespace mrpt::math;

	// Criterions for adding a new node
	// - Covered distance since last node > registration_max_distance
	// - Angle difference since last node > registration_max_angle

	bool angle_crit = false;
	if (m_use_angle_difference_node_reg) {
		angle_crit = fabs(wrapToPi(this->m_since_prev_node_PDF.getMeanVal().phi())) >
			params.registration_max_angle;
	}
	bool distance_crit = false;
	if (m_use_distance_node_reg) {
		distance_crit =
			this->m_since_prev_node_PDF.getMeanVal().norm() >
			params.registration_max_distance; }

	// actual check
	bool registered = false;
	if (distance_crit || angle_crit) {
		registered = this->registerNewNodeAtEnd();
	}

	return registered;
	MRPT_END;
} // end of checkRegistrationCondition

template<class GRAPH_T>
void CICPCriteriaNRD<GRAPH_T>::loadParams(const std::string& source_fname) {
	MRPT_START;
	parent_t::loadParams(source_fname);

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
} // end of loadParams

template<class GRAPH_T>
void CICPCriteriaNRD<GRAPH_T>::printParams() const {
	MRPT_START;
	parent_t::printParams();

	params.dumpToConsole();
	//m_mahal_distance_ICP_odom.dumpToConsole();

	MRPT_END;
} // end of printParams

template<class GRAPH_T>
void CICPCriteriaNRD<GRAPH_T>::getDescriptiveReport(
		std::string* report_str) const {
	MRPT_START;
	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "ICP Goodness-based Registration Procedure Summary: "
		<< std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = this->m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();
	parent_t::getDescriptiveReport(report_str);

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	// loggers results
	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
} // end of getDescriptiveReport


// TParams
//////////////////////////////////////////////////////////////
template<class GRAPH_T>
CICPCriteriaNRD<GRAPH_T>::TParams::TParams(decider_t& d):
	decider(d)
{ }
template<class GRAPH_T>
CICPCriteriaNRD<GRAPH_T>::TParams::~TParams() { }
template<class GRAPH_T>
void CICPCriteriaNRD<GRAPH_T>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	using namespace mrpt::utils;
	using namespace mrpt::math;

	out.printf(
			"------------------[ ICP Fixed Intervals Node Registration ]------------------\n");
	out.printf("Max distance for registration = %.2f m\n",
			registration_max_distance);
	out.printf("Max Angle for registration    = %.2f deg\n",
			RAD2DEG(registration_max_angle));

	decider.range_ops_t::params.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_T>
void CICPCriteriaNRD<GRAPH_T>::TParams::loadFromConfigFile(
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
	decider.range_ops_t::params.loadFromConfigFile(source, "ICP");

	MRPT_END;
}

} } } // end of namespace


#endif /* end of include guard: CICPCRITERIANRD_IMPL_H */
