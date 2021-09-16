/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt::gtsam_wrappers
{
/** @name mrpt_gtsam_wrappers GTSAM <-> MRPT conversion functions
 *  \note New in MRPT 2.3.3
 *  \sa <mrpt/math/gtsam_wrappers.h>
 * @{ */

static gtsam::Pose3 toPose3(const mrpt::poses::CPose3D& p)
{
	return gtsam::Pose3(
		gtsam::Rot3(p.getRotationMatrix().asEigen()),
		gtsam::Point3(p.x(), p.y(), p.z()));
}

static mrpt::math::TPose3D toTPose3D(const gtsam::Rot3& p)
{
	const auto R = p.matrix();
	return mrpt::poses::CPose3D(
			   mrpt::math::CMatrixDouble33(R),
			   mrpt::math::CVectorFixedDouble<3>())
		.asTPose();
}

static gtsam::Pose3 toPose3(const mrpt::math::TPose3D& p)
{
	return gtsam::Pose3(
		gtsam::Rot3(p.getRotationMatrix().asEigen()),
		gtsam::Point3(p.x, p.y, p.z));
}

static mrpt::math::TPose3D toTPose3D(const gtsam::Pose3& p)
{
	const auto HM = p.matrix();
	const auto H = mrpt::math::CMatrixDouble44(HM);
	return mrpt::poses::CPose3D(H).asTPose();
}

/** @} */

}  // namespace mrpt::gtsam_wrappers
