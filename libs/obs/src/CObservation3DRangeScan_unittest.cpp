/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/containers/copy_container_typecasting.h>
#include <mrpt/math/CHistogram.h>
#include <mrpt/config.h>
#include <gtest/gtest.h>

// Defined in tests/test_main.cpp
namespace mrpt
{
extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
}

using namespace mrpt;
using namespace std;

#define TEST_RANGEIMG_WIDTH 32
#define TEST_RANGEIMG_HEIGHT 24

void fillSampleObs(
	mrpt::obs::CObservation3DRangeScan& obs,
	mrpt::obs::T3DPointsProjectionParams& pp, int test_case)
{
	obs.hasRangeImage = true;
	obs.rangeImage_setSize(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);

	obs.rangeImage.setZero();

	for (int r = 10; r < 16; r++)
		for (int c = 10; c <= r; c++) obs.rangeImage(r, c) = r;

	// Test case:
	pp.PROJ3D_USE_LUT = (test_case & 1) != 0;
	pp.USE_SSE2 = (test_case & 2) != 0;
	pp.takeIntoAccountSensorPoseOnRobot = (test_case & 4) != 0;
}

TEST(CObservation3DRangeScan, Project3D_noFilter)
{
	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;

	for (int i = 0; i < 8; i++)  // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);

		o.project3DPointsFromDepthImageInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), 21U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMinMax1)
{
	mrpt::math::CMatrix fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
		fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	fMin(12, 12) = 11.5f;
	fMax(12, 12) = 12.5f;  // pass
	fMin(14, 14) = 15.5f;
	fMax(14, 14) = 16.5f;  // don't pass
	// Rest of points: filter=0.0f -> no filtering

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i = 0; i < 16; i++)  // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);
		fp.rangeCheckBetween = (i & 8) != 0;

		o.project3DPointsFromDepthImageInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), 20U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMinMaxAllBetween)
{
	mrpt::math::CMatrix fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
		fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r = 10; r < 16; r++)
		for (int c = 10; c < 16; c++)
		{
			fMin(r, c) = r - 0.1f;  // All points actually lie in between
			fMax(r, c) = r + 0.1f;
		}

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i = 0; i < 16; i++)  // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);
		fp.rangeCheckBetween = (i & 8) != 0;

		o.project3DPointsFromDepthImageInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), fp.rangeCheckBetween ? 21U : 0U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMinMaxNoneBetween)
{
	mrpt::math::CMatrix fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
		fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r = 10; r < 16; r++)
		for (int c = 10; c < 16; c++)
		{
			fMin(r, c) = r + 1.1f;  // No point lies in between
			fMax(r, c) = r + 1.2f;
		}

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i = 0; i < 16; i++)  // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);
		fp.rangeCheckBetween = (i & 8) != 0;

		o.project3DPointsFromDepthImageInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), fp.rangeCheckBetween ? 0U : 21U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMin)
{
	mrpt::math::CMatrix fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r = 10; r < 16; r++)
		for (int c = 10; c < 16; c++)
			fMin(r, c) =
				14.5f;  // Only last row of points should pass this filter

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;

	for (int i = 0; i < 8; i++)  // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);

		o.project3DPointsFromDepthImageInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), 6U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMax)
{
	mrpt::math::CMatrix fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r = 10; r < 16; r++)
		for (int c = 10; c < 16; c++)
			fMax(r, c) =
				11.5f;  // Only first 2 rows of points should pass this filter

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_max = &fMax;

	for (int i = 0; i < 8; i++)  // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);

		o.project3DPointsFromDepthImageInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), 3U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

// We need OPENCV to read the image internal to CObservation3DRangeScan,
// so skip this test if built without opencv.
#if MRPT_HAS_OPENCV

TEST(CObservation3DRangeScan, LoadAndCheckFloorPoints)
{
	const string rawlog_fil = MRPT_GLOBAL_UNITTEST_SRC_DIR +
							  string("/tests/test-3d-obs-ground.rawlog");
	if (!mrpt::system::fileExists(rawlog_fil))
	{
		GTEST_FAIL() << "ERROR: test due to missing file: " << rawlog_fil
					 << "\n";
		return;
	}

	// Load sample 3D scan from file:
	mrpt::obs::CSensoryFrame sf;
	mrpt::io::CFileGZInputStream f(rawlog_fil);
	mrpt::serialization::archiveFrom(f) >> sf;

	auto obs = sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>();

	// Depth -> 3D points:
	mrpt::maps::CSimplePointsMap pts;
	mrpt::obs::T3DPointsProjectionParams pp;
	obs->project3DPointsFromDepthImageInto(pts, pp);
	// rotate to account for the sensor pose:
	pts.changeCoordinatesReference(obs->sensorPose);

	mrpt::math::CHistogram hist(-0.15, 0.15, 30);
	std::vector<double> ptsz;
	mrpt::containers::copy_container_typecasting(
		pts.getPointsBufferRef_z(), ptsz);
	hist.add(ptsz);

	std::vector<double> bin_x, bin_count;
	hist.getHistogram(bin_x, bin_count);

	/*
	Hist[0] x=-0.15 count= 0
	Hist[1] x=-0.139655 count= 0
	Hist[2] x=-0.12931 count= 0
	Hist[3] x=-0.118966 count= 0
	Hist[4] x=-0.108621 count= 0
	Hist[5] x=-0.0982759 count= 0
	Hist[6] x=-0.087931 count= 0
	Hist[7] x=-0.0775862 count= 0
	Hist[8] x=-0.0672414 count= 0
	Hist[9] x=-0.0568966 count= 0
	Hist[10] x=-0.0465517 count= 1
	Hist[11] x=-0.0362069 count= 47
	Hist[12] x=-0.0258621 count= 942
	Hist[13] x=-0.0155172 count= 5593
	Hist[14] x=-0.00517241 count= 12903
	Hist[15] x=0.00517241 count= 9116
	Hist[16] x=0.0155172 count= 4666
	Hist[17] x=0.0258621 count= 2677
	Hist[18] x=0.0362069 count= 595
	Hist[19] x=0.0465517 count= 4
	Hist[20] x=0.0568966 count= 0
	Hist[21] x=0.0672414 count= 0
	Hist[22] x=0.0775862 count= 0
	Hist[23] x=0.087931 count= 0
	Hist[24] x=0.0982759 count= 0
	Hist[25] x=0.108621 count= 0
	Hist[26] x=0.118966 count= 0
	Hist[27] x=0.12931 count= 0
	Hist[28] x=0.139655 count= 0
	Hist[29] x=0.15 count= 0
	//for (unsigned int i=0;i<bin_x.size();i++)
	//std::cout << "Hist[" << i << "] x=" << bin_x[i] << " count= " <<
	bin_count[i] << std::endl;
	*/

	EXPECT_LE(bin_count[11], 100);
	EXPECT_LE(bin_count[12], 1000);
	EXPECT_GE(bin_count[14], 12000);
	EXPECT_LE(bin_count[18], 700);
	EXPECT_LE(bin_count[19], 20);
}
#endif
