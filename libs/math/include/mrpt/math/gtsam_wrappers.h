/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/NavState.h>	// Velocity3
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>

namespace mrpt::gtsam_wrappers
{
/** @name mrpt_gtsam_wrappers GTSAM <-> MRPT conversion functions
 *  \note New in MRPT 2.3.3
 *  \sa <mrpt/poses/gtsam_wrappers.h>
 * @{ */

static gtsam::Point3 toPoint3(const mrpt::math::TPoint3D& p)
{
	return gtsam::Point3(p.x, p.y, p.z);
}

static mrpt::math::TPoint3D toTPoint3(const gtsam::Point3& p)
{
	return {p.x(), p.y(), p.z()};
}

static mrpt::math::TTwist3D toTTwist3D(const gtsam::Velocity3& v)
{
	mrpt::math::TTwist3D t;
	t.vx = v.x();
	t.vy = v.y();
	t.vz = v.z();
	return t;
}
static std::array<double, 3> toVelArray(const gtsam::Velocity3& v)
{
	return {v.x(), v.y(), v.z()};
}

static mrpt::math::CMatrixDouble66 to_mrpt_se3_cov6(
	const gtsam::Matrix6& se3_cov)
{
	mrpt::math::CMatrixDouble66 C;
	// MRPT : X ,Y,Z,YAW,PITCH,ROLL
	// GTSAM: RX RY RZ X Y Z
	constexpr unsigned int mapping[6] = {5, 4, 3, 0, 1, 2};
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			C(mapping[i], mapping[j]) = se3_cov(i, j);

	return C;
}

static gtsam::Matrix6 to_gtsam_se3_cov6_isotropic(
	const mrpt::math::CMatrixDouble66& se3_cov)
{
	gtsam::Matrix6 C;
	// MRPT : X ,Y,Z,YAW,PITCH,ROLL
	// GTSAM: RX RY RZ X Y Z
	constexpr unsigned int mapping[6] = {5, 4, 3, 0, 1, 2};
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
		{
			C(i, j) = se3_cov(mapping[i], mapping[j]);
			if (i != j) ASSERT_EQUAL_(C(i, j), 0);
		}

	return C;
}

/** @} */

}  // namespace mrpt::gtsam_wrappers
