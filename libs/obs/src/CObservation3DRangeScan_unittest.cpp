/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation3DRangeScan.h>

#include <gtest/gtest.h>

using namespace mrpt;
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}

#define TEST_RANGEIMG_WIDTH  32
#define TEST_RANGEIMG_HEIGHT 24

void fillSampleObs(mrpt::obs::CObservation3DRangeScan &obs,mrpt::obs::T3DPointsProjectionParams &pp, int test_case)
{
	obs.hasRangeImage = true;
	obs.rangeImage_setSize(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH);

	obs.rangeImage.setZero();

	for (int r=10;r<16;r++)
		for (int c=10;c<=r;c++)
			obs.rangeImage(r,c) = r;

	// Test case:
	pp.PROJ3D_USE_LUT = (test_case&1)!=0;
	pp.USE_SSE2 = (test_case&2)!=0;
	pp.takeIntoAccountSensorPoseOnRobot = (test_case&4)!=0;
}


TEST(CObservation3DRangeScan, Project3D_noFilter)
{
	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;

	for (int i=0;i<8;i++) // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan  o;
		fillSampleObs(o,pp,i);

		o.project3DPointsFromDepthImageInto(o,pp,fp);
		EXPECT_EQ(o.points3D_x.size(),21U) << " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMinMax1)
{
	mrpt::math::CMatrix fMax(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH),fMin(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH);
	fMin(12,12) = 11.5f; fMax(12,12) = 12.5f; // pass
	fMin(14,14) = 15.5f; fMax(14,14) = 16.5f; // don't pass
	// Rest of points: filter=0.0f -> no filtering

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i=0;i<16;i++) // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan  o;
		fillSampleObs(o,pp,i);
		fp.rangeCheckBetween = (i&8)!=0;

		o.project3DPointsFromDepthImageInto(o,pp,fp);
		EXPECT_EQ(o.points3D_x.size(),20U) << " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMinMaxAllBetween)
{
	mrpt::math::CMatrix fMax(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH),fMin(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r=10;r<16;r++)
		for (int c=10;c<16;c++) 
		{
			fMin(r,c) = r-0.1f;  // All points actually lie in between
			fMax(r,c) = r+0.1f;
		}

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i=0;i<16;i++) // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan  o;
		fillSampleObs(o,pp,i);
		fp.rangeCheckBetween = (i&8)!=0;
		
		o.project3DPointsFromDepthImageInto(o,pp,fp);
		EXPECT_EQ(o.points3D_x.size(),fp.rangeCheckBetween ? 21U:0U ) << " testcase flags: i=" << i << std::endl;
	}
}


TEST(CObservation3DRangeScan, Project3D_filterMinMaxNoneBetween)
{
	mrpt::math::CMatrix fMax(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH),fMin(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r=10;r<16;r++)
		for (int c=10;c<16;c++) 
		{
			fMin(r,c) = r+1.1f;  // No point lies in between
			fMax(r,c) = r+1.2f;
		}

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i=0;i<16;i++) // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan  o;
		fillSampleObs(o,pp,i);
		fp.rangeCheckBetween = (i&8)!=0;
		
		o.project3DPointsFromDepthImageInto(o,pp,fp);
		EXPECT_EQ(o.points3D_x.size(),fp.rangeCheckBetween ? 0U:21U ) << " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMin)
{
	mrpt::math::CMatrix fMin(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r=10;r<16;r++)
		for (int c=10;c<16;c++)
			fMin(r,c) = 14.5f;  // Only last row of points should pass this filter

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;

	for (int i=0;i<8;i++) // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan  o;
		fillSampleObs(o,pp,i);
		
		o.project3DPointsFromDepthImageInto(o,pp,fp);
		EXPECT_EQ(o.points3D_x.size(), 6U ) << " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMax)
{
	mrpt::math::CMatrix fMax(TEST_RANGEIMG_HEIGHT,TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r=10;r<16;r++)
		for (int c=10;c<16;c++)
			fMax(r,c) = 11.5f;  // Only first 2 rows of points should pass this filter

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_max = &fMax;

	for (int i=0;i<8;i++) // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan  o;
		fillSampleObs(o,pp,i);
		
		o.project3DPointsFromDepthImageInto(o,pp,fp);
		EXPECT_EQ(o.points3D_x.size(), 3U ) << " testcase flags: i=" << i << std::endl;
	}
}
