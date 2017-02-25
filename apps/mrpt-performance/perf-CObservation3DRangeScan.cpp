/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/random.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/system/filesystem.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace std;

const string rgbd_test_rawlog_file = 
#ifdef MRPT_DATASET_DIR
	MRPT_DATASET_DIR  "/tests_rgbd.rawlog";
#else
	""
#endif
;

void generateRandomMaskImage(mrpt::math::CMatrix &m, const unsigned int nrows, const unsigned int ncols)
{
	m.resize(nrows,ncols);
	for (unsigned r=0;r<nrows;r++)
		for (unsigned c=0;c<ncols;c++)
			m(r,c) = static_cast<float>( mrpt::random::randomGenerator.drawUniform(0.0, 3.0) );
}


double obs3d_test_depth_to_3d(int a, int b)
{
	CObservation3DRangeScan obs1;
	CFileGZInputStream(rgbd_test_rawlog_file) >> obs1;

	CTimeLogger timlog;


	T3DPointsProjectionParams pp;
	pp.PROJ3D_USE_LUT = (a & 0x01)!=0;
	pp.USE_SSE2 = (a & 0x02)!=0;

	TRangeImageFilterParams fp;
	mrpt::math::CMatrix minF, maxF;
	if (b&0x01) {
		generateRandomMaskImage(minF, obs1.rangeImage.rows(),obs1.rangeImage.cols());
		fp.rangeMask_min = &minF;
	}
	if (b&0x02) {
		generateRandomMaskImage(maxF, obs1.rangeImage.rows(),obs1.rangeImage.cols());
		fp.rangeMask_max = &maxF;
	}
	
	for (int i=0;i<100;i++) {
		CObservation3DRangeScan obs = obs1;
		if (!pp.PROJ3D_USE_LUT || i>0) { timlog.enter("run"); }
		obs.project3DPointsFromDepthImageInto(obs, pp, fp );
		if (!pp.PROJ3D_USE_LUT || i>0) { timlog.leave("run"); }
	}
	const double t = timlog.getMeanTime("run");
	timlog.clear(true);
	return t;
}

double obs3d_test_depth_to_2d_scan(int useMinFilter, int useMaxFilter)
{
	CObservation3DRangeScan obs1;
	CFileGZInputStream(rgbd_test_rawlog_file) >> obs1;

	CTimeLogger timlog;

	T3DPointsTo2DScanParams sp;
	sp.sensorLabel = "mysensor";

	TRangeImageFilterParams fp;
	mrpt::math::CMatrix minF, maxF;
	if (useMinFilter) {
		generateRandomMaskImage(minF, obs1.rangeImage.rows(),obs1.rangeImage.cols());
		fp.rangeMask_min = &minF;
	}
	if (useMaxFilter) {
		generateRandomMaskImage(maxF, obs1.rangeImage.rows(),obs1.rangeImage.cols());
		fp.rangeMask_max = &maxF;
	}

	for (int i=0;i<10;i++) {
		CObservation3DRangeScan obs = obs1;
		CObservation2DRangeScan fake2d;
		timlog.enter("run");
		obs.convertTo2DScan(fake2d, sp, fp);
		timlog.leave("run");
	}
	const double t = timlog.getMeanTime("run");
	timlog.clear(true);
	return t;
}

// ------------------------------------------------------
// register_tests_CObservation3DRangeScan
// ------------------------------------------------------
void register_tests_CObservation3DRangeScan()
{
	if (mrpt::system::fileExists(rgbd_test_rawlog_file)) {
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (no LUT,w/o SSE2)",obs3d_test_depth_to_3d, 0x00,0) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (no LUT,w/SSE2)",obs3d_test_depth_to_3d, 0x02, 0 ) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (LUT,w/o SSE2)",obs3d_test_depth_to_3d, 0x01,0) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (LUT,w/SSE2)",obs3d_test_depth_to_3d, 0x03, 0) );

		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (no LUT,w/o SSE2,minFilter)",obs3d_test_depth_to_3d, 0x00,0x01) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (no LUT,w/SSE2,minFilter)",obs3d_test_depth_to_3d, 0x02, 0x01 ) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (LUT,w/o SSE2,minFilter)",obs3d_test_depth_to_3d, 0x01,0x01) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (LUT,w/SSE2,minFilter)",obs3d_test_depth_to_3d, 0x03, 0x01) );

		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (no LUT,w/o SSE2,maxFilter)",obs3d_test_depth_to_3d, 0x00,0x02) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (no LUT,w/SSE2,maxFilter)",obs3d_test_depth_to_3d, 0x02, 0x02 ) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (LUT,w/o SSE2,maxFilter)",obs3d_test_depth_to_3d, 0x01,0x02) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (LUT,w/SSE2,maxFilter)",obs3d_test_depth_to_3d, 0x03, 0x02) );

		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (no LUT,w/o SSE2,min/maxFilter)",obs3d_test_depth_to_3d, 0x00,0x03) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (no LUT,w/SSE2,min/maxFilter)",obs3d_test_depth_to_3d, 0x02, 0x03 ) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (LUT,w/o SSE2,min/maxFilter)",obs3d_test_depth_to_3d, 0x01,0x03) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->3D (LUT,w/SSE2,min/maxFilter)",obs3d_test_depth_to_3d, 0x03, 0x03) );

		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->2D scan",obs3d_test_depth_to_2d_scan ) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->2D scan + min_filter",obs3d_test_depth_to_2d_scan, 1, 0 ) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->2D scan + max_filter",obs3d_test_depth_to_2d_scan, 0, 1 ) );
		lstTests.push_back( TestData("3DRangeScan: 320x240 Depth->2D scan + min/max_filters",obs3d_test_depth_to_2d_scan, 1, 1 ) );		
	}
}

