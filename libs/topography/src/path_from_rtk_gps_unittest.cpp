/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/topography.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::topography;
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}


TEST(TopographyReconstructPathFrom3RTK, sampleDataset )
{
	mrpt::poses::CPose3DInterpolator robot_path;

	mrpt::obs::CRawlog  rawlog;

	const string dataset_fil = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/share/mrpt/datasets/test_rtk_path.rawlog");
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
		mrpt::topography::TPathFromRTKInfo	rtk_path_info;

		// -------------------------------------------
		// Run path reconstruction:
		// -------------------------------------------
		mrpt::topography::path_from_rtk_gps(
			robot_path,
			rawlog,
			0, // first entry
			rawlog.size()-1, // last entry
			false, // Isn't a GUI
			false, // disableGPSInterp
			1, // path_smooth_filter_size
			&rtk_path_info );

		EXPECT_EQ(robot_path.size(),75u);

		// Expected values:
		//1226225355.000000 279.705647 216.651473 8.517821 0.194222 -0.083873 -0.045293
		//1226225380.000000 377.095830 233.343569 9.724171 0.177037 -0.073565 -0.019024
		const mrpt::system::TTimeStamp t1 = mrpt::system::time_tToTimestamp( 1226225355.000000 );
		const mrpt::system::TTimeStamp t2 = mrpt::system::time_tToTimestamp( 1226225380.000000 );
		const CPose3D pose_GT_1(279.696, 216.623, 9.21315, 0.195764, -0.0319733, -0.0420478 );
		const CPose3D pose_GT_2(377.087, 233.311, 10.474,0.178932,-0.0212096,-0.0154982 );

		CPose3D pose1,pose2;
		bool    valid;
		robot_path.interpolate(t1,pose1,valid);
		EXPECT_TRUE(valid);

		robot_path.interpolate(t2,pose2,valid);
		EXPECT_TRUE(valid);

		CVectorDouble p1vec(12), p2vec(12);
		pose1.getAs12Vector(p1vec);
		pose2.getAs12Vector(p2vec);

		CVectorDouble p1vec_gt(12), p2vec_gt(12);
		pose_GT_1.getAs12Vector(p1vec_gt);
		pose_GT_2.getAs12Vector(p2vec_gt);

		EXPECT_NEAR( (p1vec - p1vec_gt).array().abs().sum(), 0, 1e-3);
		EXPECT_NEAR( (p2vec - p2vec_gt).array().abs().sum(), 0, 1e-3);
	}
}
