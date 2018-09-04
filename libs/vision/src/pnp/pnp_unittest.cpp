/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Dense>
#include <mrpt/vision/pnp_algos.h>

// Opencv 2.3 had a broken <opencv/eigen.h> in Ubuntu 14.04 Trusty => Disable
// PNP classes
#include <mrpt/config.h>
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM < 0x240
#undef MRPT_HAS_OPENCV
#define MRPT_HAS_OPENCV 0
#endif

#if MRPT_HAS_OPENCV

class CPnPTest : public ::testing::Test
{
   public:
	mrpt::vision::pnp::CPnP cpnp;

	Eigen::MatrixXd obj_pts, img_pts, pose_est;
	Eigen::Matrix3d R, I3, R_est;
	Eigen::Vector3d t, t_est;
	int n;

	void SetUp() override
	{
		n = 6;

		obj_pts = Eigen::MatrixXd::Zero(3, n);
		img_pts = Eigen::MatrixXd::Zero(3, n);
		obj_pts << 0, 20, 10, 15, 14, 16, 0, 0, -10, -5, -2, -5, 0, 40, 0, 24,
			21, 13;

		R << -0.3536, 0.3536, -0.8660, 0.9330, 0.0670, -0.3536, -0.0670,
			-0.9330, -0.3536;

		t << 20, -30, 100;

		for (int i = 0; i < n; i++)
		{
			img_pts.col(i) = (R * obj_pts.col(i) + t);
			img_pts.col(i) /= img_pts(2, i);
		}

		I3 = Eigen::MatrixXd::Identity(3, 3);

		pose_est = Eigen::MatrixXd::Zero(6, 1);
	}
	void TearDown() override {}
};

TEST_F(CPnPTest, p3p_TEST)
{
	cpnp.p3p(obj_pts, img_pts, n, I3, pose_est);

	t_est << pose_est(0), pose_est(1), pose_est(2);

	double err_t = (t - t_est).norm();

	EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, rpnp_TEST)
{
	cpnp.rpnp(obj_pts, img_pts, n, I3, pose_est);

	t_est << pose_est(0), pose_est(1), pose_est(2);

	double err_t = (t - t_est).norm();

	EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, ppnp_TEST)
{
	cpnp.ppnp(obj_pts, img_pts, n, I3, pose_est);

	t_est << pose_est(0), pose_est(1), pose_est(2);

	double err_t = (t - t_est).norm();

	EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, posit_TEST)
{
	cpnp.posit(obj_pts, img_pts, n, I3, pose_est);

	t_est << pose_est(0), pose_est(1), pose_est(2);

	double err_t = (t - t_est).norm();

	EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, lhm_TEST)
{
	cpnp.lhm(obj_pts, img_pts, n, I3, pose_est);

	t_est << pose_est(0), pose_est(1), pose_est(2);

	double err_t = (t - t_est).norm();

	EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, dls_TEST)
{
	cpnp.dls(obj_pts, img_pts, n, I3, pose_est);

	t_est << pose_est(0), pose_est(1), pose_est(2);

	double err_t = (t - t_est).norm();

	EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, epnp_TEST)
{
	cpnp.epnp(obj_pts, img_pts, n, I3, pose_est);

	t_est << pose_est(0), pose_est(1), pose_est(2);

	double err_t = (t - t_est).norm();

	EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, DISABLED_upnp_TEST)
{
	cpnp.upnp(obj_pts, img_pts, n, I3, pose_est);

	t_est << pose_est(0), pose_est(1), pose_est(2);

	double err_t = (t - t_est).norm();

	EXPECT_LE(err_t, 2);
}

#endif
