/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/config/CConfigFile.h>

namespace mrpt::graphslam::deciders
{
// Ctors, Dtors
//////////////////////////////////////////////////////////////

template <class GRAPH_T>
CFixedIntervalsNRD<GRAPH_T>::CFixedIntervalsNRD()
{
	this->initializeLoggers("CFixedIntervalsNRD");
}

// Member function implementations
//////////////////////////////////////////////////////////////

template <class GRAPH_T>
bool CFixedIntervalsNRD<GRAPH_T>::updateState(
	mrpt::obs::CActionCollection::Ptr action,
	mrpt::obs::CSensoryFrame::Ptr observations,
	mrpt::obs::CObservation::Ptr observation)
{
	MRPT_START;
	using namespace mrpt::obs;
	using namespace mrpt::math;
	using namespace mrpt::poses;

	// don't use the measurements in this implementation
	MRPT_UNUSED_PARAM(observations);

	if (observation)
	{  // FORMAT #2 - observation-only format
		m_observation_only_rawlog = true;

		if (IS_CLASS(observation, CObservationOdometry))
		{
			CObservationOdometry::Ptr obs_odometry =
				std::dynamic_pointer_cast<CObservationOdometry>(observation);
			// not incremental - gives the absolute odometry reading
			m_curr_odometry_only_pose = pose_t(obs_odometry->odometry);
			MRPT_LOG_DEBUG_FMT(
				"Current odometry-only pose: %s",
				m_curr_odometry_only_pose.asString().c_str());

			// I don't have any information about the covariane of the move in
			// observation-only format
			this->m_since_prev_node_PDF.mean =
				m_curr_odometry_only_pose - m_last_odometry_only_pose;
		}
	}
	else
	{  // FORMAT #1 - action-observation format
		m_observation_only_rawlog = false;

		mrpt::poses::CPose3DPDFGaussian move_pdf;
		bool found = action->getFirstMovementEstimation(move_pdf);
		if (found)
		{
			// update the relative PDF of the path since the LAST node was
			// inserted
			constraint_t incr_constraint;
			incr_constraint.copyFrom(move_pdf);
			this->m_since_prev_node_PDF += incr_constraint;
		}
	}  // ELSE - FORMAT #1

	bool registered = this->checkRegistrationCondition();

	if (registered)
	{
		if (m_observation_only_rawlog)
		{
			// keep track of the odometry-only pose_t at the last inserted graph
			// node
			m_last_odometry_only_pose = m_curr_odometry_only_pose;
		}
	}

	return registered;

	MRPT_END;
}  // end of updateState

template <class GRAPH_T>
bool CFixedIntervalsNRD<GRAPH_T>::checkRegistrationCondition()
{
	MRPT_START;

	// check that a node has already been registered - if not, default to
	// (0,0,0)
	pose_t last_pose_inserted =
		this->m_prev_registered_nodeID != INVALID_NODEID
			? this->m_graph->nodes.at(this->m_prev_registered_nodeID)
			: pose_t();

	// odometry criterion
	bool registered = false;

	if (this->checkRegistrationCondition(
			last_pose_inserted, this->getCurrentRobotPosEstimation()))
	{
		registered = this->registerNewNodeAtEnd();
	}

	return registered;
	MRPT_END;
}  // end of checkRegistrationCondition

template <class GRAPH_T>
bool CFixedIntervalsNRD<GRAPH_T>::checkRegistrationCondition(
	const mrpt::poses::CPose2D& p1, const mrpt::poses::CPose2D& p2) const
{
	using namespace mrpt::math;

	bool res = false;
	if ((p1.distanceTo(p2) > params.registration_max_distance) ||
		(fabs(wrapToPi(p1.phi() - p2.phi())) > params.registration_max_angle))
	{
		res = true;
	}

	return res;
}  // end of checkRegistrationCondition2D

template <class GRAPH_T>
bool CFixedIntervalsNRD<GRAPH_T>::checkRegistrationCondition(
	const mrpt::poses::CPose3D& p1, const mrpt::poses::CPose3D& p2) const
{
	using namespace mrpt::math;

	bool res = false;
	if ((p1.distanceTo(p2) > params.registration_max_distance) ||
		(fabs(wrapToPi(p1.roll() - p2.roll())) >
		 params.registration_max_angle) ||
		(fabs(wrapToPi(p1.pitch() - p2.pitch())) >
		 params.registration_max_angle) ||
		(fabs(wrapToPi(p1.yaw() - p2.yaw())) > params.registration_max_angle))
	{
		res = true;
	}

	return res;
}  // end of checkRegistrationCondition3D

template <class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::loadParams(const std::string& source_fname)
{
	MRPT_START;
	parent_t::loadParams(source_fname);

	params.loadFromConfigFileName(
		source_fname, "NodeRegistrationDeciderParameters");

	// set the logging level if given by the user
	mrpt::config::CConfigFile source(source_fname);
	int min_verbosity_level = source.read_int(
		"NodeRegistrationDeciderParameters", "class_verbosity", 1, false);
	this->setMinLoggingLevel(mrpt::system::VerbosityLevel(min_verbosity_level));

	MRPT_LOG_DEBUG("Successfully loaded parameters.");
	MRPT_END;
}

template <class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::printParams() const
{
	MRPT_START;
	parent_t::printParams();
	params.dumpToConsole();

	MRPT_END;
}

template <class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::getDescriptiveReport(
	std::string* report_str) const
{
	MRPT_START;
	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "Strategy: "
				   << "Fixed Odometry-based Intervals" << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = this->m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();
	parent_t::getDescriptiveReport(report_str);

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

template <class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::TParams::dumpToTextStream(
	std::ostream& out) const
{
	MRPT_START;
	out << mrpt::format("%s", this->getAsString().c_str());
	MRPT_END;
}
template <class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::TParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	MRPT_START;
	using namespace mrpt::math;

	registration_max_distance = source.read_double(
		section, "registration_max_distance", 0.5 /* meter */, false);
	registration_max_angle = source.read_double(
		section, "registration_max_angle", 60 /* degrees */, false);
	registration_max_angle = DEG2RAD(registration_max_angle);

	MRPT_END;
}

template <class GRAPH_T>
void CFixedIntervalsNRD<GRAPH_T>::TParams::getAsString(
	std::string* params_out) const
{
	MRPT_START;
	using namespace mrpt::math;

	double max_angle_deg = RAD2DEG(registration_max_angle);
	params_out->clear();

	*params_out +=
		"------------------[ Fixed Intervals Node Registration "
		"]------------------\n";
	*params_out += mrpt::format(
		"Max distance for registration = %.2f m\n", registration_max_distance);
	*params_out += mrpt::format(
		"Max angle for registration    = %.2f deg\n", max_angle_deg);

	MRPT_END;
}
template <class GRAPH_T>
std::string CFixedIntervalsNRD<GRAPH_T>::TParams::getAsString() const
{
	MRPT_START;

	std::string str;
	this->getAsString(&str);
	return str;

	MRPT_END;
}
}  // namespace mrpt::graphslam::deciders
