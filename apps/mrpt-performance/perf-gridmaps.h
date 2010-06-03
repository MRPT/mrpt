/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::random;
using namespace std;


// ------------------------------------------------------
//				BenchmarkGridmaps
// ------------------------------------------------------
double grid_test_1(int a1, int a2)
{
	COccupancyGridMap2D		gridMap(-20,20,-20,20, 0.05);

	// test 1: getcell
	// ----------------------------------------
	const long N = 10000000;
	float	p=0;

	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		p += gridMap.getCell( 0, 0 );
	}
	return tictac.Tac()/N;
}

double grid_test_2(int a1, int a2)
{
	COccupancyGridMap2D		gridMap(-20,20,-20,20, 0.05);
	// test 2: setcell
	// ----------------------------------------
	const long N = 10000000;

	float	p=0.8f;

	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		gridMap.setCell( 0, 0, p );
	}
	return tictac.Tac()/N;
}

double grid_test_3(int a1, int a2)
{
	COccupancyGridMap2D		gridMap(-20,20,-20,20, 0.05);
	const long N = 1000000;
	float	p=0.57f;

	CTicTac tictac;
	for (long i=0;i<N;i++)
	{
		gridMap.updateCell( 0, 0, p );
	}
	return tictac.Tac()/N;
}


double grid_test_4(int a1, int a2)
{
	COccupancyGridMap2D		gridMap(-20,20,-20,20, 0.05);

	// test 4: updateCell_fast
	// ----------------------------------------
	const long N = 10000000;

	float	p=0.57f;
	COccupancyGridMap2D::cellType  logodd_obs = COccupancyGridMap2D::p2l( p );
	COccupancyGridMap2D::cellType  *theMapArray = gridMap.getRow(0);
	unsigned  theMapSize_x = gridMap.getSizeX();
	COccupancyGridMap2D::cellType   logodd_thres_occupied = OCCGRID_CELLTYPE_MIN+logodd_obs;

	CTicTac tictac;
	for (long i=0;i<N;i++)
	{
		COccupancyGridMap2D::updateCell_fast_occupied( 2, 2, logodd_obs,logodd_thres_occupied, theMapArray, theMapSize_x);
	}
	return tictac.Tac()/N;
}

double grid_test_5_6(int a1, int a2)
{
	randomGenerator.randomize(333);

	// prepare the laser scan:
	CObservation2DRangeScan	scan1;
	scan1.aperture = M_PIf;
	scan1.rightToLeft = true;
	scan1.validRange.resize( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]) );
	scan1.scan.resize( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]) );

	memcpy( &scan1.scan[0], SCAN_RANGES_1, sizeof(SCAN_RANGES_1) );
	memcpy( &scan1.validRange[0], SCAN_VALID_1, sizeof(SCAN_VALID_1) );

	COccupancyGridMap2D		gridmap(-20,20,-20,20, 0.05);
	gridmap.insertionOptions.wideningBeamsWithDistance = a1!=0;
	const long N = 3000;
	CTicTac tictac;
	for (long i=0;i<N;i++)
	{
		CPose2D  pose(
			randomGenerator.drawUniform(-1.0,1.0),
			randomGenerator.drawUniform(-1.0,1.0),
			randomGenerator.drawUniform(-M_PI,M_PI) );
		CPose3D  pose3D(pose);

		gridmap.insertObservation( &scan1, &pose3D );
	}
	return tictac.Tac()/N;
}

double grid_test_7(int a1, int a2)
{
	COccupancyGridMap2D		gridmap(-20,20,-20,20, 0.05);
	CTicTac tictac;

	gridmap.resizeGrid(-30,30,-40,40);

	return tictac.Tac();
}

double grid_test_8(int a1, int a2)
{
	randomGenerator.randomize(333);

	// prepare the laser scan:
	CObservation2DRangeScan	scan1;
	scan1.aperture = M_PIf;
	scan1.rightToLeft = true;
	scan1.validRange.resize( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]) );
	scan1.scan.resize( sizeof(SCAN_RANGES_1)/sizeof(SCAN_RANGES_1[0]) );

	memcpy( &scan1.scan[0], SCAN_RANGES_1, sizeof(SCAN_RANGES_1) );
	memcpy( &scan1.validRange[0], SCAN_VALID_1, sizeof(SCAN_VALID_1) );

	COccupancyGridMap2D		gridmap(-20,20,-20,20, 0.05);

	// test 8: Likelihood computation
	const long N = 5000;

	CPose3D pose3D(0,0,0);
	gridmap.insertObservation( &scan1, &pose3D );

	double R = 0;
	CTicTac tictac;
	for (long i=0;i<N;i++)
	{
		CPose2D  pose(
			randomGenerator.drawUniform(-1.0,1.0),
			randomGenerator.drawUniform(-1.0,1.0),
			randomGenerator.drawUniform(-M_PI,M_PI) );
		R+=gridmap.computeObservationLikelihood(&scan1,pose);
	}
	return tictac.Tac()/N;
}

// ------------------------------------------------------
// register_tests_grids
// ------------------------------------------------------
void register_tests_grids()
{
	lstTests.push_back( TestData("gridmap2D: getCell",grid_test_1) ); 
	lstTests.push_back( TestData("gridmap2D: setCell",grid_test_2) ); 
	lstTests.push_back( TestData("gridmap2D: updateCell",grid_test_3) ); 
	lstTests.push_back( TestData("gridmap2D: updateCell_fast_occupied",grid_test_4) ); 
	lstTests.push_back( TestData("gridmap2D: insert scan w/o widening",grid_test_5_6, 0) ); 
	lstTests.push_back( TestData("gridmap2D: insert scan with widening",grid_test_5_6, 1) ); 
	lstTests.push_back( TestData("gridmap2D: resize",grid_test_7) ); 
	lstTests.push_back( TestData("gridmap2D: computeLikelihood",grid_test_8) ); 
}

