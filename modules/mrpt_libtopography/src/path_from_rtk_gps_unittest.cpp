/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/topography.h>
#include <test_mrpt_common.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::topography;
using namespace std;

TEST(TopographyReconstructPathFrom3RTK, sampleDataset)
{
  mrpt::poses::CPose3DInterpolator robot_path;

  mrpt::obs::CRawlog rawlog;

  const string dataset_fil =
      UNITTEST_BASEDIR() + string("/share/mrpt/datasets/test_rtk_path.rawlog");
  if (!mrpt::system::fileExists(dataset_fil))
  {
    cerr << "WARNING: Skipping test due to missing file: " << dataset_fil << "\n";
    return;
  }
  if (!rawlog.loadFromRawLogFile(dataset_fil))
  {
    cerr << "WARNING: Skipping test due to error loading file: " << dataset_fil << "\n";
  }
  else
  {
    mrpt::topography::TPathFromRTKInfo rtk_path_info;

    // -------------------------------------------
    // Run path reconstruction:
    // -------------------------------------------
    mrpt::topography::path_from_rtk_gps(
        robot_path, rawlog,
        0,                  // first entry
        rawlog.size() - 1,  // last entry
        false,              // Isn't a GUI
        false,              // disableGPSInterp
        1,                  // path_smooth_filter_size
        &rtk_path_info);

    EXPECT_EQ(robot_path.size(), 75u);

    // clang-format off
		// Expected values:
		// 1226225355.000000 279.696222 216.622980 9.213152 0.195764 -0.031973 -0.042048
		// 1226225380.000000 377.086918 233.311009 10.474043 0.178932 -0.025085 -0.013734
    // clang-format on
    const auto t1 = mrpt::Clock::fromDouble(1226225355.0);
    const auto t2 = mrpt::Clock::fromDouble(1226225380.0);
    const CPose3D pose_GT_1(279.696222, 216.622980, 9.213152, 0.195764, -0.031973, -0.042048);
    const CPose3D pose_GT_2(377.086918, 233.311009, 10.474043, 0.178932, -0.025085, -0.013734);

    CPose3D pose1, pose2;
    bool valid;
    robot_path.interpolate(t1, pose1, valid);
    EXPECT_TRUE(valid);

    robot_path.interpolate(t2, pose2, valid);
    EXPECT_TRUE(valid);

    CVectorDouble p1vec(12), p2vec(12);
    pose1.getAs12Vector(p1vec);
    pose2.getAs12Vector(p2vec);

    CVectorDouble p1vec_gt(12), p2vec_gt(12);
    pose_GT_1.getAs12Vector(p1vec_gt);
    pose_GT_2.getAs12Vector(p2vec_gt);
    EXPECT_NEAR((p1vec - p1vec_gt).sum_abs(), 0, 1e-3);
    EXPECT_NEAR((p2vec - p2vec_gt).sum_abs(), 0, 1e-3);
  }
}
