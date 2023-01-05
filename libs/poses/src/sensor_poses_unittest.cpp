/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/config.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/sensor_poses.h>

#if MRPT_HAS_FYAML

// clang-format off

const auto sample_YAML_extrinsics1 = std::string(R"xxx(
sensors:
  lidar1:
    extrinsics:
      quaternion: [-0.5003218001035493, 0.5012125349997221, -0.5001966939080825, 0.49826434600894337]
      translation: [0.0506, 0.04, -0.002]
    parent: imu
  imu:
    extrinsics:
      quaternion: [0.0, 0.0, 0.0, 1.0]
      translation: [1.0, 0.0, 0.0]
    parent: base_link
)xxx");

// clang-format on

TEST(sensor_poses, from_yaml)
{
	const auto d = mrpt::containers::yaml::FromText(sample_YAML_extrinsics1);

	const auto s2p = mrpt::poses::sensor_poses_from_yaml(d["sensors"]);

	EXPECT_EQ(s2p.size(), 3U);

	const std::map<std::string, mrpt::poses::CPose3D> gt_poses = {
		{"base_link", mrpt::poses::CPose3D::Identity()},
		{"lidar1",
		 mrpt::poses::CPose3D::FromQuaternionAndTranslation(
			 {0.49826434600894337, -0.5003218001035493, 0.5012125349997221,
			  -0.5001966939080825},
			 1.0506, 0.04, -0.002)},
		{"imu", mrpt::poses::CPose3D::FromTranslation(1.0, 0.0, 0.0)},
	};

	for (const auto& kv : gt_poses)
	{
		EXPECT_NEAR(
			mrpt::poses::Lie::SE<3>::log(s2p.at(kv.first) - kv.second).norm(),
			.0, 1e-3);
	}
}

#endif
