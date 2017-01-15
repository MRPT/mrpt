/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/random.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;


// ------------------------------------------------------
//				Benchmark Point Maps
// ------------------------------------------------------
double pointmap_test_0(int a1, int a2)
{
	// test 0: insert scan
	// ----------------------------------------

	// prepare the laser scan:
	CObservation2DRangeScan	scan1;
	scan1.aperture = M_PIf;
	scan1.rightToLeft = true;
	scan1.loadFromVectors( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]), SCAN_RANGES_1, SCAN_VALID_1);

	CSimplePointsMap  pt_map;

	pt_map.insertionOptions.minDistBetweenLaserPoints = 0.03f;
	CPose3D pose;

	CTicTac	 tictac;
	for (int n=0;n<a2;n++)
	{
		pt_map.clear();
		for (long i=0;i<a1;i++)
		{
			pose.setFromValues( pose.x()+0.04, pose.y()+0.08,0, pose.yaw()+0.02);
			pt_map.insertObservation(&scan1, &pose);
		}
	}
	return tictac.Tac()/a2;
}

double pointmap_test_1(int a1, int a2)
{
	// test 1: insert scan
	// ----------------------------------------

	// prepare the laser scan:
	CObservation2DRangeScan	scan1;
	scan1.aperture = M_PIf;
	scan1.rightToLeft = true;
	scan1.loadFromVectors( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]), SCAN_RANGES_1, SCAN_VALID_1);

	CSimplePointsMap  pt_map;

	pt_map.insertionOptions.minDistBetweenLaserPoints = 0.03f;
	CPose3D pose;

	CTicTac	 tictac;
	for (long i=0;i<a1;i++)
	{
		pose.setFromValues( pose.x()+0.04, pose.y()+0.08,0, pose.yaw()+0.02);

		pt_map.insertObservation(&scan1, &pose);
	}

	const unsigned N_REPS = 25;

	if (a2==0)
		return tictac.Tac();
	else if (a2==1)
	{ // 2d kd-tree
		float x,y, dist2;

		tictac.Tic();

		for (unsigned k=N_REPS;k!=0;--k)
			/*size_t idx = */ pt_map.kdTreeClosestPoint2D(5.0, 6.0, x,y, dist2);

		return tictac.Tac()/N_REPS;
	}
	else
	{ // 3d kd-tree
		float x,y,z, dist2;
		tictac.Tic();

		for (unsigned k=N_REPS;k!=0;--k)
			/*size_t idx =*/ pt_map.kdTreeClosestPoint3D(5.0, 6.0, 1.0, x,y,z, dist2);

		return tictac.Tac()/N_REPS;
	}
}

double pointmap_test_2(int a1, int a2)
{
	// test 2: insert scan + kd_tree in each iteration
	// --------------------------------------------------

	// prepare the laser scan:
	CObservation2DRangeScan	scan1;
	scan1.aperture = M_PIf;
	scan1.rightToLeft = true;
	scan1.loadFromVectors( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]), SCAN_RANGES_1, SCAN_VALID_1);

	CTicTac	 tictac;
	const unsigned N_TIMES = 20;

	for (unsigned n=N_TIMES;n!=0;--n)
	{
		CSimplePointsMap  pt_map;

		pt_map.insertionOptions.minDistBetweenLaserPoints = 0.03f;
		CPose3D pose;
		for (long i=0;i<a1;i++)
		{
			pose.setFromValues( pose.x()+0.04, pose.y()+0.08,0, pose.yaw()+0.02);
			pt_map.insertObservation(&scan1, &pose);
			if (a2==1)
			{ // 2d kd-tree
				float x,y, dist2;
				/*size_t idx =*/ pt_map.kdTreeClosestPoint2D(5.0, 6.0, x,y, dist2);
			}
			else
			{ // 3d kd-tree
				float x,y,z, dist2;
				/*size_t idx = */pt_map.kdTreeClosestPoint3D(5.0, 6.0, 1.0, x,y,z, dist2);
			}
		}
	}

	return tictac.Tac()/N_TIMES;
}

double pointmap_test_3(int a1, int a2)
{
	// test 3: query2D
	// ----------------------------------------

	// prepare the laser scan:
	CObservation2DRangeScan	scan1;
	scan1.aperture = M_PIf;
	scan1.rightToLeft = true;
	scan1.loadFromVectors( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]), SCAN_RANGES_1, SCAN_VALID_1);


	CSimplePointsMap  pt_map;

	pt_map.insertionOptions.minDistBetweenLaserPoints = 0.03f;
	CPose3D pose;
	for (long i=0;i<a1;i++)
	{
		pose.setFromValues( pose.x()+0.04, pose.y()+0.08,0, pose.yaw()+0.02);
		pt_map.insertObservation(&scan1, &pose);
	}

	CTicTac	 tictac;
	float x0=-5;
	float y0=-4;
	float sq;
	float x,y;
	for (long i=0;i<a2;i++)
	{
		pt_map.kdTreeClosestPoint2D(x0,y0,x,y,sq);
		x0+=0.05f;
		y0+=0.05f;
		if (x0>20) x0=-10;
		if (y0>20) y0=-10;
	}

	return tictac.Tac()/a2;
}

double pointmap_test_4(int a1, int a2)
{
	// test 4: computeMatchingWith2D
	// ----------------------------------------

	// prepare the laser scan:
	CObservation2DRangeScan	scan1;
	scan1.aperture = M_PIf;
	scan1.rightToLeft = true;
	scan1.loadFromVectors( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]), SCAN_RANGES_1, SCAN_VALID_1);


	CSimplePointsMap  pt_map;
	CSimplePointsMap  pt_map2;
	pt_map.insertionOptions.minDistBetweenLaserPoints = 0.03f;
	pt_map2.insertionOptions.minDistBetweenLaserPoints = 0.03f;

	CPose3D pose;
	pt_map.insertObservation(&scan1, &pose);

	CPose3D pose2(0.05,0.04,0, DEG2RAD(4), 0,0);
	pt_map2.insertObservation(&scan1, &pose2);

	const CPose2D nullPose(0,0,0);
	TMatchingPairList	correspondences;
	TMatchingParams matchParams;
	TMatchingExtraResults matchExtraResults;
	matchParams.maxDistForCorrespondence = 0.10f;
	matchParams.maxAngularDistForCorrespondence = 0;

	CTicTac	 tictac;
	for (long i=0;i<a1;i++)
	{
		pt_map.determineMatching2D(
			&pt_map2,			// The other map
			nullPose,	// The other map's pose
			correspondences,
			matchParams, matchExtraResults);
	}
	return tictac.Tac()/a1;
}

double pointmap_test_5(int a1, int a2)
{
	// test 5: boundingBox
	// ----------------------------------------

	// prepare the laser scan:
	CObservation2DRangeScan	scan1;
	scan1.aperture = M_PIf;
	scan1.rightToLeft = true;
	scan1.loadFromVectors( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]), SCAN_RANGES_1, SCAN_VALID_1);


	CSimplePointsMap  pt_map;

	pt_map.insertionOptions.minDistBetweenLaserPoints = 0.03f;
	CPose3D pose;
	for (long i=0;i<a1;i++)
	{
		pose.setFromValues( pose.x()+0.04, pose.y()+0.08,0, pose.yaw()+0.02);
		pt_map.insertObservation(&scan1, &pose);
	}

	CTicTac	 tictac;

	float x0,x1, y0,y1, z0,z1;
	for (long i=0;i<a2;i++)
	{
		// Modify the map so the bounding box cache is invalidated:
		pt_map.setPoint(0, 0,0,0);

		pt_map.boundingBox(x0,x1, y0,y1, z0,z1);
	}

	return tictac.Tac()/a2;
}


// ------------------------------------------------------
// register_tests_pointmaps
// ------------------------------------------------------
void register_tests_pointmaps()
{
	lstTests.push_back( TestData("pointmap: insert 100 scans",pointmap_test_0, 100, 2000 ) );

	lstTests.push_back( TestData("pointmap: build 2D kd-tree of 1 scan",pointmap_test_1, 1, 1 ) );
	lstTests.push_back( TestData("pointmap: build 2D kd-tree of 100 scan",pointmap_test_1, 100, 1 ) );
	lstTests.push_back( TestData("pointmap: build 2D kd-tree of 1000 scan",pointmap_test_1, 1000, 1 ) );

	lstTests.push_back( TestData("pointmap: build 3D kd-tree of 1 scan",pointmap_test_1, 1, 2 ) );
	lstTests.push_back( TestData("pointmap: build 3D kd-tree of 100 scan",pointmap_test_1, 100, 2 ) );
	lstTests.push_back( TestData("pointmap: build 3D kd-tree of 1000 scan",pointmap_test_1, 1000, 2 ) );

	lstTests.push_back( TestData("pointmap: kd-tree 2d query on 10 scans",pointmap_test_3, 10, 1000 ) );
	lstTests.push_back( TestData("pointmap: kd-tree 2d query on 1000 scans",pointmap_test_3, 1000, 1000 ) );
	lstTests.push_back( TestData("pointmap: kd-tree 2d query on 10000 scans",pointmap_test_3, 10000, 100 ) );


	lstTests.push_back( TestData("pointmap: (insert scan+2D kd-tree query) x 10",pointmap_test_2,  10,  1 ) );
	lstTests.push_back( TestData("pointmap: (insert scan+2D kd-tree query) x 50",pointmap_test_2,  50,  1 ) );
	//lstTests.push_back( TestData("pointmap: (insert scan+2D kd-tree query) x 100",pointmap_test_2, 100, 1 ) );
	lstTests.push_back( TestData("pointmap: (insert scan+3D kd-tree query) x 10",pointmap_test_2,  10,  2 ) );
	lstTests.push_back( TestData("pointmap: (insert scan+3D kd-tree query) x 50",pointmap_test_2,  50,  2 ) );
	//lstTests.push_back( TestData("pointmap: (insert scan+3D kd-tree query) x 100",pointmap_test_2, 100, 2 ) );

	lstTests.push_back( TestData("pointmap: computeMatchingWith2D",pointmap_test_4, 5000 ) );

	lstTests.push_back( TestData("pointmap: boundingBox (10 scans)",pointmap_test_5, 10, 50000 ) );
	lstTests.push_back( TestData("pointmap: boundingBox (1000 scans)",pointmap_test_5, 1000, 5000 ) );

}

