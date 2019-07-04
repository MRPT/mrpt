/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGrid.h>
//#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

template class mrpt::CTraitsTest<CPose3DPDFGrid>;

const auto bb_min =
	mrpt::math::TPose3D(1, 2, 3, DEG2RAD(-20), DEG2RAD(-30), DEG2RAD(-40));
const auto bb_max =
	mrpt::math::TPose3D(3, 4, 5, DEG2RAD(20), DEG2RAD(30), DEG2RAD(40));

TEST(CPose3DPDFGrid, uniformDistribution)
{
	const auto gt_mean =
		mrpt::math::TPose3D(2, 3, 4, DEG2RAD(.0), DEG2RAD(.0), DEG2RAD(.0));

	const double res_xyz = 0.25;
	const double res_YPR = mrpt::DEG2RAD(10.0);

	auto grid = mrpt::poses::CPose3DPDFGrid(bb_min, bb_max, res_xyz, res_YPR);

	grid.uniformDistribution();

	auto [COV, MEAN] = grid.getCovarianceAndMean();

	EXPECT_NEAR(MEAN.x(), gt_mean.x, 1e-4);
	EXPECT_NEAR(MEAN.y(), gt_mean.y, 1e-4);
	EXPECT_NEAR(MEAN.z(), gt_mean.z, 1e-4);

	EXPECT_NEAR(MEAN.yaw(), gt_mean.yaw, 1e-4);
	EXPECT_NEAR(MEAN.pitch(), gt_mean.pitch, 1e-4);
	EXPECT_NEAR(MEAN.roll(), gt_mean.roll, 1e-4);

	for (int i = 0; i < 3; i++) EXPECT_GT(COV(i, i), mrpt::square(0.25));
	for (int i = 4; i < 6; i++)
		EXPECT_GT(COV(i, i), mrpt::square(mrpt::DEG2RAD(3.0)));
}

TEST(CPose3DPDFGrid, setManualPDF)
{
	const auto gt_mean = mrpt::math::TPose3D(
		2, 3, 4, DEG2RAD(9.0), DEG2RAD(34.0), DEG2RAD(12.0));

	const double res_xyz = 0.25;
	const double res_YPR = mrpt::DEG2RAD(10.0);

	auto grid = mrpt::poses::CPose3DPDFGrid(bb_min, bb_max, res_xyz, res_YPR);

	grid.fill(0);

	*grid.getByPos(gt_mean) = .1;

	grid.normalize();

	auto [COV, MEAN] = grid.getCovarianceAndMean();

	EXPECT_NEAR(MEAN.x(), gt_mean.x, 0.2);
	EXPECT_NEAR(MEAN.y(), gt_mean.y, 0.2);
	EXPECT_NEAR(MEAN.z(), gt_mean.z, 0.2);

	EXPECT_NEAR(MEAN.yaw(), gt_mean.yaw, 0.1);
	EXPECT_NEAR(MEAN.pitch(), gt_mean.pitch, 0.1);
	EXPECT_NEAR(MEAN.roll(), gt_mean.roll, 0.1);

	for (int i = 0; i < 6; i++) EXPECT_LT(COV(i, i), mrpt::square(0.05));
}
