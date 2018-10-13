/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

namespace mrpt::graphslam::deciders
{
template <class GRAPH_T>
bool CIncrementalNodeRegistrationDecider<GRAPH_T>::checkRegistrationCondition()
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

	if (this->checkRegistrationConditionPose(
			last_pose_inserted, this->getCurrentRobotPosEstimation()))
	{
		registered = this->registerNewNodeAtEnd();
	}

	return registered;
	MRPT_END;
}  // end of checkRegistrationCondition

template <class GRAPH_T>
bool CIncrementalNodeRegistrationDecider<GRAPH_T>::
	checkRegistrationConditionPose(
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
}  // end of checkRegistrationConditionPose

template <class GRAPH_T>
bool CIncrementalNodeRegistrationDecider<GRAPH_T>::
	checkRegistrationConditionPose(
		const mrpt::poses::CPose3D& p1, const mrpt::poses::CPose3D& p2) const
{
	using namespace mrpt::math;

	std::cout << "In checkRegistrationConditionPose:\np1: " << p1.asString()
			  << "\np2: " << p1.asString() << std::endl;

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
}  // end of checkRegistrationConditionPose

template <class GRAPH_T>
void CIncrementalNodeRegistrationDecider<GRAPH_T>::loadParams(
	const std::string& source_fname)
{
	MRPT_START;
	parent_t::loadParams(source_fname);

	params.loadFromConfigFileName(
		source_fname, "NodeRegistrationDeciderParameters");

	MRPT_END;
}

template <class GRAPH_T>
void CIncrementalNodeRegistrationDecider<GRAPH_T>::printParams() const
{
	MRPT_START;
	parent_t::printParams();
	params.dumpToConsole();

	MRPT_END;
}

template <class GRAPH_T>
void CIncrementalNodeRegistrationDecider<GRAPH_T>::getDescriptiveReport(
	std::string* report_str) const
{
	MRPT_START;
	using namespace std;

	*report_str += params.getAsString();
	*report_str += this->report_sep;

	MRPT_END;
}

template <class GRAPH_T>
void CIncrementalNodeRegistrationDecider<GRAPH_T>::TParams::dumpToTextStream(
	std::ostream& out) const
{
	MRPT_START;
	out << mrpt::format("%s", this->getAsString().c_str());
	MRPT_END;
}

template <class GRAPH_T>
void CIncrementalNodeRegistrationDecider<GRAPH_T>::TParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	MRPT_START;
	using namespace mrpt::math;

	registration_max_distance = source.read_double(
		section, "registration_max_distance", 0.5 /* meter */, false);
	registration_max_angle = source.read_double(
		section, "registration_max_angle", 15 /* degrees */, false);
	registration_max_angle = DEG2RAD(registration_max_angle);

	MRPT_END;
}

template <class GRAPH_T>
void CIncrementalNodeRegistrationDecider<GRAPH_T>::TParams::getAsString(
	std::string* params_out) const
{
	MRPT_START;
	using namespace mrpt::math;

	double max_angle_deg = RAD2DEG(registration_max_angle);
	params_out->clear();

	*params_out +=
		"------------------[ Fixed Intervals Incremental Node Registration "
		"]------------------\n";
	*params_out += mrpt::format(
		"Max distance for registration = %.2f m\n", registration_max_distance);
	*params_out += mrpt::format(
		"Max angle for registration    = %.2f deg\n", max_angle_deg);

	MRPT_END;
}

template <class GRAPH_T>
std::string CIncrementalNodeRegistrationDecider<GRAPH_T>::TParams::getAsString()
	const
{
	MRPT_START;

	std::string str;
	this->getAsString(&str);
	return str;

	MRPT_END;
}
}  // namespace mrpt::graphslam::deciders
