/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/poses/CPose3D.h>

#include <map>
#include <string>

namespace mrpt::poses
{
using SensorToPoseMap = std::map<std::string, mrpt::poses::CPose3D>;

/**
 * Alternative to sensor_poses_from_yaml_file() where the yaml map inside
 * `sensors: ...` is directly passed programatically.
 *
 * \sa CPose3D, mrpt::obs::CObservation, sensor_poses_from_yaml_file()
 * \ingroup poses_grp
 */
SensorToPoseMap sensor_poses_from_yaml(
	const mrpt::containers::yaml& d,
	const std::string& referenceFrame = "base_link");

/** Utility to parse a YAML file with the extrinsic calibration of sensors.
 *
 *  Each YAML map entry defines a sensorLabel, and for each one an `extrinsics`
 *  map containing the SE(3) relative pose between the `parent` frame and this
 *  sensor. The pose is given as a quaternion and a translation.
 *
 *  The expected file contents is like:
 *
 *  \verbatim
 *  # My YAML file:
 *  sensors:  # Note: sensor_poses_from_yaml() expects **this node** as input
 *    camera:
 *      extrinsics:
 *        quaternion: [qx, qy, qz, qw]
 *        translation: [tx, ty, tz]
 *    parent: base_link
 *    imu:
 *      extrinsics:
 *        quaternion: [qx, qy, qz, qw]
 *        translation: [tx, ty, tz]
 *    parent: camera
 *  \endverbatim
 *
 * Following the common ROS conventions, the robot reference frame is assumed
 * to be `base_link` (default).
 *
 * Of course, this mechanism of defining a tree of sensor poses in a YAML file
 * only works for static (rigid) sensor assemblies, where the transformations
 * between them is always static.
 *
 * The data is returned as a `std::map` from sensor labels to poses within the
 * robot reference frame.
 *
 * This function takes as input the YAML filename to load.
 *
 * \sa CPose3D, mrpt::obs::CObservation, sensor_poses_from_yaml()
 * \ingroup poses_grp
 */
inline SensorToPoseMap sensor_poses_from_yaml_file(
	const std::string& filename,
	const std::string& referenceFrame = "base_link")
{
	const auto d = mrpt::containers::yaml::FromFile(filename);
	ASSERT_(d.has("sensors"));
	return sensor_poses_from_yaml(d["sensors"], referenceFrame);
}

}  // namespace mrpt::poses
