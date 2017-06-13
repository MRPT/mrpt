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

template<class GRAPH_T>
CFixedIntervalsNRD<GRAPH_T>::CFixedIntervalsNRD() {
	using namespace mrpt::utils;
	this->initializeLoggers("CFixedIntervalsNRD");

	this->logFmt(LVL_DEBUG, "IntervalsNRD: Initialized class object");
}
template<class GRAPH_T>
CFixedIntervalsNRD<GRAPH_T>::~CFixedIntervalsNRD() { }

// Member function implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_T>
bool CFixedIntervalsNRD<GRAPH_T>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {
	MRPT_START;
	using namespace mrpt::obs;
	using namespace mrpt::math;
	using namespace mrpt::utils;
	using namespace mrpt::poses;

	// don't use the measurements in this implementation
	MRPT_UNUSED_PARAM(observations);

	if (observation.present()) { // FORMAT #2 - observation-only format
		m_observation_only_rawlog = true;

		if (IS_CLASS(observation, CObservationOdometry)) {

			CObservationOdometryPtr obs_odometry =
				static_cast<CObservationOdometryPtr>(observation);
			// not incremental - gives the absolute odometry reading
			m_curr_odometry_only_pose = pose_t(obs_odometry->odometry);
			this->logFmt(LVL_DEBUG, "Current odometry-only pose: %s",
					m_curr_odometry_only_pose.asString().c_str());

			// I don't have any information about the covariane of the move in
			// observation-only format
			this->m_since_prev_node_PDF.mean =
				m_curr_odometry_only_pose - m_last_odometry_only_pose;
		}
	}
	else { // FORMAT #1 - action-observation format
		m_observation_only_rawlog = false;

		mrpt::poses::CPose3DPDFGaussian move_pdf;
		bool found = action->getFirstMovementEstimation(move_pdf);
		if (found) {
			// update the relative PDF of the path since the LAST node was inserted
			constraint_t incr_constraint;
			incr_constraint.copyFrom(move_pdf);
			this->m_since_prev_node_PDF += incr_constraint;
		}
	} // ELSE - FORMAT #1

	bool registered_new_node = this->checkRegistrationCondition();

	if (registered_new_node) {
		if (m_observation_only_rawlog) {
			// keep track of the odometry-only pose_t at the last inserted graph node
			m_last_odometry_only_pose = m_curr_odometry_only_pose;
		}
	}

	return registered_new_node;

	MRPT_END;
} // end of updateState

template<class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::loadParams(const std::string& source_fname) {
	parent_t::loadParams(source_fname);
}

template<class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::printParams() const {
	parent_t::printParams();
}

template<class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;
	using namespace std;

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "Strategy: " <<
		"Fixed Odometry-based Intervals" << std::endl;
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
}

} } } // end of namespaces

#endif /* end of include guard: CFIXEDINTERVALSNRD_IMPL_H */
