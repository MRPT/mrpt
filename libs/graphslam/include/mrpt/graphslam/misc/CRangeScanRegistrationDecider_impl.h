/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CRANGESCANREGISTRATIONDECIDER_IMPL_H
#define CRANGESCANREGISTRATIONDECIDER_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_t>
void CRangeScanRegistrationDecider<GRAPH_t>::getICPEdge(
		const mrpt::obs::CObservation2DRangeScan& from,
		const mrpt::obs::CObservation2DRangeScan& to,
		constraint_t* rel_edge,
		const mrpt::poses::CPose2D* initial_pose_in/* = NULL */,
		mrpt::slam::CICP::TReturnInfo* icp_info/* = NULL */) {
	MRPT_START;

	mrpt::maps::CSimplePointsMap m1,m2;
	float running_time;
	mrpt::slam::CICP::TReturnInfo info;

	// have them initialized prior - and then just clear them
	m1.insertObservation(&from);
	m2.insertObservation(&to);

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
	if (icp_info) *icp_info = info;

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider<GRAPH_t>::getICPEdge(
		const mrpt::obs::CObservation3DRangeScan& from,
		const mrpt::obs::CObservation3DRangeScan& to,
		constraint_t* rel_edge,
		const mrpt::poses::CPose2D* initial_pose_in /* =NULL */,
		mrpt::slam::CICP::TReturnInfo* icp_info /* =NULL */) {
	MRPT_START;

	ASSERTMSG_(from.hasRangeImage,
			mrpt::format("Laser scan doesn't contain valid range image"));
	ASSERTMSG_(to.hasRangeImage,
			mrpt::format("Laser scan doesn't contain valid range image"));

	// TODO - have this as a class member
	mrpt::maps::CSimplePointsMap m1,m2;
	float running_time;
	mrpt::slam::CICP::TReturnInfo info;

	m1.insertObservation(&from);
	m2.insertObservation(&to);

	//this->decimatePointsMap(&m1, [> keep every = */ 40, /* low_lim = <] 5000);
	//this->decimatePointsMap(&m2, [> keep every = */ 40, /* low_lim = <] 5000);

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
	// copy from the 3D PDF
	// NULLIFY contraint_t if Z displacement is high (normally this should be 0)
	// since we are moving in 2D
	if (fabs(pdf->getMeanVal().z()) < 0.1) {
		rel_edge->copyFrom(*pdf);
	}
	else {
		rel_edge = NULL;
	}

	// if given, fill the TReturnInfo Struct
	if (icp_info) *icp_info = info;


	MRPT_END;
}

template<class GRAPH_t>
void CRangeScanRegistrationDecider<GRAPH_t>::decimatePointsMap(
		mrpt::maps::CPointsMap* m,
		size_t keep_point_every, /* = 4 */
		size_t low_lim /* = 8000 */) {
	MRPT_START;

	size_t map_size = m->size();

	if (low_lim) {
		// check if current keep_point_every variable is too large
		size_t conservative_keep_point_every = map_size / low_lim;
		keep_point_every = std::min(keep_point_every, conservative_keep_point_every);
	}

	// insert a false every "keep_point_every" points
	std::vector<bool> deletion_mask(map_size, true);
	for (size_t i = 0; i != map_size; ++i) {
		if (i % keep_point_every == 0) {
			deletion_mask[i] = false;
		}
	}
	m->applyDeletionMask(deletion_mask);

	std::cout << "Map size: " << map_size << " => " << m->size() << std::endl;

	MRPT_END;
}

template<class GRAPH_t>
bool CRangeScanRegistrationDecider<GRAPH_t>::convert3DTo2DRangeScan(
		mrpt::obs::CObservation3DRangeScanPtr& scan3D_in,
		mrpt::obs::CObservation2DRangeScanPtr* scan2D_out /*= NULL*/) {
	MRPT_START;

	bool success = false;
	// if it doesn't exist, create it
	if ( (*scan2D_out).null() ) {
		*scan2D_out = mrpt::obs::CObservation2DRangeScan::Create();
	}

	if (scan3D_in->hasRangeImage) {
		scan3D_in->convertTo2DScan(**scan2D_out, params.conversion_params);
		success = true;
	}
	else {
		std::cout << "No valid rangeImage found" << std::endl;
		success = false;
	}

	return success;
	MRPT_END;
}

// TParameter
// //////////////////////////////////

template<class GRAPH_t>
CRangeScanRegistrationDecider<GRAPH_t>::TParams::TParams():
	has_read_config(false)
{ }

template<class GRAPH_t>
CRangeScanRegistrationDecider<GRAPH_t>::TParams::~TParams() {
}

template<class GRAPH_t>
void CRangeScanRegistrationDecider<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("3D=>2D LaserScan Conversion Sensor label       = %s\n",
			conversion_params.sensorLabel.c_str());
	out.printf("3D=>2D LaserScan Conversion angle sup          = %.2f deg\n",
			mrpt::utils::RAD2DEG(conversion_params.angle_sup));
	out.printf("3D=>2D LaserScan Conversion angle inf          = %.2f deg\n",
			mrpt::utils::RAD2DEG(conversion_params.angle_inf));
	out.printf("3D=>2D LaserScan Conversion oversampling ratio = %.2f\n",
			conversion_params.oversampling_ratio);
	out.printf("3D=>2D LaserScan Conversion Z minimum          = %.2f\n",
			conversion_params.z_min);

	icp.options.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
		const std::string& section) {
	MRPT_START;

	conversion_params.sensorLabel = source.read_string(
			section,
			"conversion_sensor_label",
			"KINECT_TO_2D_SCAN", false);
	conversion_params.angle_sup = mrpt::utils::DEG2RAD(source.read_double(
				section,
				"conversion_angle_sup",
				10, false));
	conversion_params.angle_inf  = mrpt::utils::DEG2RAD(source.read_double(
				section,
				"conversion_angle_inf",
				10, false));
	conversion_params.oversampling_ratio = source.read_double(
			section,
			"conversion_oversampling_ratio",
			1.1, false);
	conversion_params.z_min = source.read_double(
			section,
			"conversion_z_min",
			0, false); // TODO - is this accurate?


	// load the icp parameters - from "ICP" section explicitly
	icp.options.loadFromConfigFile(source, "ICP");

	has_read_config = true;

	MRPT_END;
}

} } } // end of namespaces

#endif /* end of include guard: CRANGESCANREGISTRATIONDECIDER_IMPL_H */
