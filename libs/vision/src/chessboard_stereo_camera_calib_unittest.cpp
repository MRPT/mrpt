/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/config.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <test_mrpt_common.h>

#if MRPT_HAS_OPENCV
TEST(Vision, checkerBoardStereoCalibration)
#else
TEST(Vision, DISABLED_checkerBoardStereoCalibration)
#endif
{
	using namespace std::string_literals;

	mrpt::vision::TCalibrationStereoImageList images;
	mrpt::vision::TStereoCalibParams params;
	mrpt::vision::TStereoCalibResults out;

	// Test: calib with real data:
	const auto dir =
		mrpt::UNITTEST_BASEDIR + "/share/mrpt/datasets/stereo-calib/"s;

	const unsigned int NUM_IMGS = 4;
	images.resize(NUM_IMGS);
	for (unsigned int i = 0; i < NUM_IMGS; i++)
	{
		auto fil = dir + mrpt::format("%u_left.jpg", i);
		if (!images[i].left.img_original.loadFromFile(fil))
		{
			GTEST_FAIL() << "Error loading: " << fil;
			return;
		}
		fil = dir + mrpt::format("%u_right.jpg", i);
		if (!images[i].right.img_original.loadFromFile(fil))
		{
			GTEST_FAIL() << "Error loading: " << fil;
			return;
		}
	}

	// Pattern images from the "Malaga Urban Dataset 2010" (BumbleBee2 camera)
	params.check_size_x = 6;
	params.check_size_y = 9;
	params.optimize_k1 = true;
	params.optimize_k2 = true;
	params.optimize_k3 = true;
	params.optimize_t1 = true;
	params.optimize_t2 = true;
	params.check_squares_length_X_meters = 0.034;
	params.check_squares_length_Y_meters = 0.034;
	params.verbose = false;

	bool ok = mrpt::vision::checkerBoardStereoCalibration(images, params, out);

	// Checks:
	EXPECT_LT(out.final_rmse, 3.0);
	EXPECT_GT(out.final_iters, 10UL);

	EXPECT_NEAR(out.cam_params.rightCameraPose.x, 0.1194, 0.005);
	EXPECT_NEAR(out.cam_params.rightCameraPose.y, 0.00, 0.005);
	EXPECT_NEAR(out.cam_params.rightCameraPose.z, 0.01, 0.005);
	EXPECT_NEAR(out.cam_params.rightCameraPose.qr, 0.99, 0.01);

	EXPECT_EQ(out.image_pair_was_used.size(), NUM_IMGS);
	for (unsigned int i = 0; i < NUM_IMGS; i++)
		EXPECT_TRUE(out.image_pair_was_used[i]) << "i=" << i;

#if 0  // Debug:
	std::cout << out.right2left_camera_pose.asString() << "\n";
	for (unsigned int i = 0; i < images.size(); i++)
	{
		images[i].left.img_checkboard.saveToFile(
			mrpt::format("/tmp/stereo_%03u_check_L.png", i));
		images[i].right.img_checkboard.saveToFile(
			mrpt::format("/tmp/stereo_%03u_check_R.png", i));
	}
	std::cout << out.cam_params.dumpAsText();
#endif

	EXPECT_TRUE(ok);
}

#if MRPT_HAS_OPENCV
TEST(Vision, checkerBoardStereoCalibration_empty)
#else
TEST(Vision, DISABLED_checkerBoardStereoCalibration_empty)
#endif
{
	mrpt::vision::TCalibrationStereoImageList images;
	mrpt::vision::TStereoCalibParams params;
	mrpt::vision::TStereoCalibResults out;
	params.verbose = false;

	const bool ok =
		mrpt::vision::checkerBoardStereoCalibration(images, params, out);
	EXPECT_FALSE(ok);
}
