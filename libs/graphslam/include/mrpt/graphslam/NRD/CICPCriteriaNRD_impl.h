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
	m_times_used_ICP(0),
	m_times_used_odom(0)
	//m_mahal_distance_ICP_odom("Mahalanobis dist (ICP - odom)")
{
	using namespace mrpt::utils;
	using namespace mrpt::math;
	this->initializeLoggers("CICPCriteriaNRD");

	this->resetPDF(&m_latest_odometry_PDF);
	//m_mahal_distance_ICP_odom.resizeWindow(1000); // use the last X mahalanobis distance values

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
		mrpt::obs::CObservation2DRangeScanPtr scan) {
	MRPT_START;
	bool registered_new_node = false;

	m_curr_laser_scan2D = scan;
	if (m_last_laser_scan2D.null()) {
		// initialize the last_laser_scan here - afterwards updated inside the
		// checkRegistrationConditionLS*D method
		m_last_laser_scan2D = m_curr_laser_scan2D;
	}
	else {
		registered_new_node = checkRegistrationConditionLS(
				m_last_laser_scan2D, m_curr_laser_scan2D);
	}

	return registered_new_node;
	MRPT_END;
} // end of updateState2D

template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::updateState3D(
		mrpt::obs::CObservation3DRangeScanPtr scan) {
	MRPT_START;
	bool registered_new_node = false;

	m_curr_laser_scan3D = scan;
	if (m_last_laser_scan3D.null()) {
		// initialize the last_laser_scan here - afterwards updated inside the
		// checkRegistrationConditionLS*D method
		m_last_laser_scan3D = m_curr_laser_scan3D;
	}
	else {
		registered_new_node = checkRegistrationConditionLS(
				m_last_laser_scan3D, m_curr_laser_scan3D);
	}

	return registered_new_node;
	MRPT_END;
} // end of updateState3D


template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::checkRegistrationConditionLS(
		mrpt::obs::CObservation2DRangeScanPtr& last_laser_scan,
		mrpt::obs::CObservation2DRangeScanPtr& curr_laser_scan) {
	MRPT_START;

	using namespace mrpt::math;
	using namespace mrpt::utils;

	bool registered_new_node = false;

	// Constraint that *may* update incrementally the m_since_prev_node_PDF.
	constraint_t rel_edge;
	mrpt::slam::CICP::TReturnInfo icp_info;

	this->_getICPEdge(
			*last_laser_scan,
			*curr_laser_scan,
			&rel_edge,
			NULL,
			&icp_info);

	// evaluate the mahalanobis distance of the above..
	// If over an (adaptive) threshold, trust the odometry
	MRPT_TODO("Do not check Mahal distance if I don't have odometry at all");
	double mahal_distance = rel_edge.mahalanobisDistanceTo(m_latest_odometry_PDF);
	//m_mahal_distance_ICP_odom.addNewMeasurement(mahal_distance);

	// TODO - Find out a proper criterion
	// How do I filter out the "bad" 2DRangeScans?
	//double mahal_distance_lim = m_mahal_distance_ICP_odom.getMedian();
	//double mahal_distance_lim = m_mahal_distance_ICP_odom.getMean();
	//double mahal_distance_lim =
		//m_mahal_distance_ICP_odom.getMean() + m_mahal_distance_ICP_odom.getStdDev();
	MRPT_TODO("Fix this criterion");
	double mahal_distance_lim = 0.18; // visual introspection

	//
	// check whether to use ICP or odometry Edge.
	//
	// if the norm of the odometry edge is 0, no odometry edge available
	// => use ICP
	if (mahal_distance < mahal_distance_lim ||
			approximatelyEqual(m_latest_odometry_PDF.getMeanVal().norm(), 0.0)) {
		m_times_used_ICP++;
	}
	else {
		rel_edge.copyFrom(m_latest_odometry_PDF);
		m_times_used_odom++;
	}

	// update the PDF until last registered node
	this->m_since_prev_node_PDF += rel_edge;
	last_laser_scan = curr_laser_scan;

	// check the actual criterion
	registered_new_node = this->checkRegistrationCondition();

	// reset the odometry tracking as well.
	m_last_odometry_only_pose = m_curr_odometry_only_pose;
	this->resetPDF(&m_latest_odometry_PDF);

	return registered_new_node;
	MRPT_END;
} // end of checkRegistrationConditionLS

template<class GRAPH_T>
bool CICPCriteriaNRD<GRAPH_T>::checkRegistrationConditionLS(
		mrpt::obs::CObservation3DRangeScanPtr& last_laser_scan,
		mrpt::obs::CObservation3DRangeScanPtr& curr_laser_scan) {

	bool registered_new_node = false;

	// Constraint that *may* update incrementally the m_since_prev_node_PDF.
	constraint_t rel_edge;
	mrpt::slam::CICP::TReturnInfo icp_info;

	this->_getICPEdge(
			*last_laser_scan,
			*curr_laser_scan,
			&rel_edge,
			NULL,
			&icp_info);

	// update the PDF until last registered node
	this->m_since_prev_node_PDF += rel_edge;
	last_laser_scan = curr_laser_scan;

	// register the new node
	registered_new_node = this->checkRegistrationCondition();

	// reset the odometry tracking as well.
	m_last_odometry_only_pose = m_curr_odometry_only_pose;
	this->resetPDF(&m_latest_odometry_PDF);

	return registered_new_node;

} // end of checkRegistrationConditionLS


template<class GRAPH_T>
void CICPCriteriaNRD<GRAPH_T>::loadParams(const std::string& source_fname) {
	parent_t::loadParams(source_fname);
} // end of loadParams

template<class GRAPH_T>
void CICPCriteriaNRD<GRAPH_T>::printParams() const {
	parent_t::printParams();
} // end of printParams

template<class GRAPH_T>
void CICPCriteriaNRD<GRAPH_T>::getDescriptiveReport(
		std::string* report_str) const {
	MRPT_START;
	using namespace std;

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "ICP Goodness-based Registration Procedure Summary: "
		<< std::endl;
	class_props_ss << this->header_sep << std::endl;

	// time and output logging
	const std::string time_res = this->m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();
	parent_t::getDescriptiveReport(report_str);

	*report_str += class_props_ss.str();
	*report_str += this->report_sep;

	// loggers results
	*report_str += time_res;
	*report_str += this->report_sep;

	*report_str += output_res;
	*report_str += this->report_sep;

	MRPT_END;
} // end of getDescriptiveReport

} } } // end of namespace

#endif /* end of include guard: CICPCRITERIANRD_IMPL_H */
