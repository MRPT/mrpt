/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>

namespace mrpt::obs
{
/** Used in CObservation3DRangeScan::convertTo2DScan()
 * \ingroup mrpt_obs_grp
 */
struct T3DPointsTo2DScanParams
{
	/** The sensor label that will have the newly created observation. */
	std::string sensorLabel;
	/** (Default=5 degrees) [Only if use_origin_sensor_pose=false] The upper &
	 * lower half-FOV angle (in radians). */
	double angle_sup, angle_inf;
	/** (Default:-inf, +inf) [Only if use_origin_sensor_pose=true] Only obstacle
	 * points with Z coordinates within the range [z_min,z_max] will be taken
	 * into account. */
	double z_min, z_max;
	/** (Default=1.2=120%) How many more laser scans rays to create (read docs
	 * for CObservation3DRangeScan::convertTo2DScan()). */
	double oversampling_ratio{1.2};

	/** (Default:false) If `false`, the conversion will be such that the 2D
	 * observation pose on the robot coincides with that in the original 3D
	 * range scan.
	 * If `true`, the sensed points will be "reprojected" as seen from a sensor
	 * pose at the robot/vehicle frame origin  (and angle_sup, angle_inf will be
	 * ignored) */
	bool use_origin_sensor_pose{false};

	T3DPointsTo2DScanParams();
};
}  // namespace mrpt::obs