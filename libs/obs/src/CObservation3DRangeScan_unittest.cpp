/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/config.h>
#include <mrpt/containers/copy_container_typecasting.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CHistogram.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

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
	mrpt::math::CMatrixF fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
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
	mrpt::math::CMatrixF fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
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
	mrpt::math::CMatrixF fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
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
	mrpt::math::CMatrixF fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
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
	mrpt::math::CMatrixF fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
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
	const string rawlog_fil =
		UNITTEST_BASEDIR + string("/tests/test-3d-obs-ground.rawlog");
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
	mrpt::obs::T3DPointsProjectionParams pp;
	pp.takeIntoAccountSensorPoseOnRobot = true;
	mrpt::math::CHistogram hist(-0.15, 0.15, 30);
	std::vector<double> ptsz;
	std::vector<double> bin_x, bin_count;

	// decimation=1
	{
		mrpt::maps::CSimplePointsMap pts;
		obs->project3DPointsFromDepthImageInto(pts, pp);

		mrpt::containers::copy_container_typecasting(
			pts.getPointsBufferRef_z(), ptsz);
		hist.add(ptsz);
		hist.getHistogram(bin_x, bin_count);

		/*
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

	// decimation=8
	{
		mrpt::maps::CSimplePointsMap pts;
		pp.decimation = 8;
		obs->project3DPointsFromDepthImageInto(pts, pp);

		mrpt::containers::copy_container_typecasting(
			pts.getPointsBufferRef_z(), ptsz);
		hist.clear();
		hist.add(ptsz);
		hist.getHistogram(bin_x, bin_count);

		// for (unsigned int i = 0; i < bin_x.size(); i++)
		//	std::cout << "Hist[" << i << "] x=" << bin_x[i]
		//   << " count= " << bin_count[i] << std::endl;
		/*
		Hist[11] x=-0.0362069 count= 0
		Hist[12] x=-0.0258621 count= 7
		Hist[13] x=-0.0155172 count= 24
		Hist[14] x=-0.00517241 count= 130
		Hist[15] x=0.00517241 count= 234
		Hist[16] x=0.0155172 count= 129
		Hist[17] x=0.0258621 count= 85
		Hist[18] x=0.0362069 count= 32
		Hist[19] x=0.0465517 count= 3
		 */
		EXPECT_LE(bin_count[11], 2);
		EXPECT_LE(bin_count[12], 10);
		EXPECT_GE(bin_count[14], 100);
		EXPECT_LE(bin_count[18], 40);
		EXPECT_LE(bin_count[19], 5);
	}
}
#endif
