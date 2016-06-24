/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CRANGESCANREGISTRATIONDECIDER_IMPL_H
#define CRANGESCANREGISTRATIONDECIDER_IMPL_H

using namespace mrpt::graphslam::deciders;

template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::getICPEdge(
		const CObservation2DRangeScan& prev_laser_scan,
		const CObservation2DRangeScan& curr_laser_scan,
		constraint_t* rel_edge,
		mrpt::poses::CPose2D* initial_pose_in/* = NULL */,
		mrpt::slam::CICP::TReturnInfo* icp_info/* = NULL */) {
	MRPT_START;

	mrpt::maps::CSimplePointsMap m1,m2;
	float running_time;
	mrpt::slam::CICP::TReturnInfo info;

	// have them initialized prior - and then just clear them
	m1.insertObservation(&prev_laser_scan);
	m2.insertObservation(&curr_laser_scan);

	// If given, use initial_pose_in as a first guess for the ICP
	mrpt::poses::CPose2D initial_pose;
	if (initial_pose_in) {
		initial_pose = *initial_pose_in;
	}

	mrpt::poses::CPosePDFPtr pdf = params.icp.Align(
			&m1,
			&m2,
			initial_pose,
			&running_time,
			(void*)&info);

	// return the edge regardless of the goodness of the alignment
	rel_edge->copyFrom(*pdf);

	// if given, fill the TReturnInfo Struct
	if (icp_info) {
		icp_info->nIterations = info.nIterations;
		icp_info->goodness = info.goodness;
		icp_info->quality = info.quality;
		icp_info->cbSize = info.cbSize;
	}

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::getICPEdge(
		const CObservation3DRangeScan& prev_laser_scan,
		const CObservation3DRangeScan& curr_laser_scan,
		constraint_t* rel_edge,
		mrpt::poses::CPose2D* initial_pose_in /* =NULL */,
		mrpt::slam::CICP::TReturnInfo* icp_info /* =NULL */) {
	MRPT_START;

	mrpt::maps::CSimplePointsMap m1,m2;
	float running_time;
	mrpt::slam::CICP::TReturnInfo info;

	m1.insertObservation(&prev_laser_scan);
	m2.insertObservation(&curr_laser_scan);

	// If given, use initial_pose_in as a first guess for the ICP
	mrpt::poses::CPose3D initial_pose;
	if (initial_pose_in) {
		initial_pose = *initial_pose_in;
	}

	mrpt::poses::CPose3DPDFPtr pdf = params.icp.Align3D(
			&m1,
			&m2,
			initial_pose,
			&running_time,
			(void*)&info);

	// return the edge regardless of the goodness of the alignment
	// copy fro the 3D PDF
	rel_edge->copyFrom(*pdf);

	// if given, fill the TReturnInfo Struct
	if (icp_info) {
		icp_info->nIterations = info.nIterations;
		icp_info->goodness = info.goodness;
		icp_info->quality = info.quality;
		icp_info->cbSize = info.cbSize;
	}


	MRPT_END;
}

// TParameter
// //////////////////////////////////

template<class GRAPH_t>
CRangeScanRegistrationDecider_t<GRAPH_t>::TParams::TParams():
	has_read_config(false)
{ }

template<class GRAPH_t>
CRangeScanRegistrationDecider_t<GRAPH_t>::TParams::~TParams() {
}

template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	icp.options.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
    const std::string& section) {
  MRPT_START;

	// load the icp parameters - from "ICP" section explicitly
	icp.options.loadFromConfigFile(source, "ICP");

	std::cout << "[CRangeScanRegistrationDecider:] Successfully loaded parameters. " 
		<< std::endl;
	has_read_config = true;

	MRPT_END;
}

#endif /* end of include guard: CRANGESCANREGISTRATIONDECIDER_IMPL_H */
