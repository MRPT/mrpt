/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"	// Precompiled headers
//
#include <mrpt/poses/sensor_poses.h>

#include <map>
#include <queue>
#include <set>

mrpt::poses::SensorToPoseMap mrpt::poses::sensor_poses_from_yaml(
	const mrpt::containers::yaml& d, const std::string& referenceFrame)
{
	MRPT_START

	std::map<std::string, std::set<std::string>> childrenFrames;
	std::map<std::pair<std::string, std::string>, mrpt::poses::CPose3D>
		parentToChildrenEdges;

	ASSERT_(d.isMap());
	for (const auto& e : d.asMap())
	{
		const auto sensorLabel = e.first.as<std::string>();

		ASSERT_(e.second.isMap());
		const auto& em = e.second.asMap();

		ASSERT_(em.count("extrinsics") != 0);
		ASSERT_(em.count("parent") != 0);
		const auto& extrs = em.at("extrinsics");
		const auto parentFrame = em.at("parent").as<std::string>();

		ASSERT_(extrs.isMap());
		const auto& extrm = extrs.asMap();

		ASSERT_(extrm.count("quaternion") != 0);
		ASSERT_(extrm.count("translation") != 0);

		const std::vector<double> read_quaternion =
			mrpt::containers::yaml(extrm.at("quaternion"))
				.toStdVector<double>();
		ASSERT_EQUAL_(read_quaternion.size(), 4);

		const mrpt::math::CQuaternionDouble q(
			read_quaternion.at(3), read_quaternion.at(0), read_quaternion.at(1),
			read_quaternion.at(2));

		const auto t = mrpt::math::TPoint3D::FromVector(
			mrpt::containers::yaml(extrm.at("translation"))
				.toStdVector<double>());

		const auto relPose =
			mrpt::poses::CPose3D::FromQuaternionAndTranslation(q, t);

#if 0
		std::cout << "sensor: " << sensorLabel << " tr: " << t
				  << " q: " << q.inMatlabFormat() << " parent: " << parentFrame
				  << std::endl;
#endif

		// Store data in temporary graph structure for spanning tree:
		childrenFrames[parentFrame].insert(sensorLabel);
		parentToChildrenEdges[{parentFrame, sensorLabel}] = relPose;
	}

	mrpt::poses::SensorToPoseMap s2p;

	// reference:
	s2p[referenceFrame] = mrpt::poses::CPose3D::Identity();

	std::queue<std::string> pendingFrames;

	pendingFrames.push(referenceFrame);

	while (!pendingFrames.empty())
	{
		const auto curFrame = pendingFrames.front();
		pendingFrames.pop();

		if (childrenFrames.count(curFrame) == 0) continue;

		const auto& children = childrenFrames.at(curFrame);
		// process poses and add to queue:
		for (const auto& child : children)
		{
			const auto& relPose = parentToChildrenEdges.at({curFrame, child});
			s2p[child] = s2p[curFrame] + relPose;

			pendingFrames.push(child);
		}
	}

	return s2p;

	MRPT_END
}
