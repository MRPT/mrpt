/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/random.h>
#include <mrpt/tfest.h>

#include "common.h"


using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;


typedef std::vector< std::vector< double > > TPoints;

// ------------------------------------------------------
//				Generate both sets of points
// ------------------------------------------------------
void generate_points( TPoints &pA, TPoints &pB )
{
	const double		Dx = 0.5;
	const double		Dy = 1.5;
	const double		Dz = 0.75;

	const double		yaw = DEG2RAD(10);
	const double		pitch = DEG2RAD(20);
	const double		roll = DEG2RAD(5);

	pA.resize( 5 );		// A set of points at "A" reference system
	pB.resize( 5 );		// A set of points at "B" reference system

	pA[0].resize(3);	pA[0][0] = 0.0;		pA[0][1] = 0.5;		pA[0][2] = 0.4;
	pA[1].resize(3);	pA[1][0] = 1.0;		pA[1][1] = 1.5;		pA[1][2] = -0.1;
	pA[2].resize(3);	pA[2][0] = 1.2;		pA[2][1] = 1.1;		pA[2][2] = 0.9;
	pA[3].resize(3);	pA[3][0] = 0.7;		pA[3][1] = 0.3;		pA[3][2] = 3.4;
	pA[4].resize(3);	pA[4][0] = 1.9;		pA[4][1] = 2.5;		pA[4][2] = -1.7;

	CPose3DQuat qPose( CPose3D( Dx, Dy, Dz, yaw, pitch, roll ) );
	for( unsigned int i = 0; i < 5; ++i )
	{
		pB[i].resize( 3 );
		qPose.inverseComposePoint( pA[i][0], pA[i][1], pA[i][2], pB[i][0], pB[i][1], pB[i][2] );
	}

} // end generate_points

// ------------------------------------------------------
//				Generate a list of matched points
// ------------------------------------------------------
void generate_list_of_points( const TPoints &pA, const TPoints &pB, TMatchingPairList &list )
{
	TMatchingPair		pair;
	for( unsigned int i = 0; i < 5; ++i )
	{
		pair.this_idx	= pair.other_idx = i;
		pair.this_x		= pA[i][0];
		pair.this_y		= pA[i][1];
		pair.this_z		= pA[i][2];

		pair.other_x	= pB[i][0];
		pair.other_y	= pB[i][1];
		pair.other_z	= pB[i][2];

		list.push_back( pair );
	}
} // end generate_list_of_points

// ------------------------------------------------------
//				Genreate a vector of matched points
// ------------------------------------------------------
void generate_vector_of_points(  const TPoints &pA, const TPoints &pB, vector<mrpt::math::TPoint3D> &ptsA, vector<mrpt::math::TPoint3D> &ptsB  )
{
	// The input vector: inV = [pA1x, pA1y, pA1z, pB1x, pB1y, pB1z, ... ]
	ptsA.resize( pA.size() );
	ptsB.resize( pA.size() );
	for( unsigned int i = 0; i < pA.size(); ++i )
	{
		ptsA[i] = mrpt::math::TPoint3D( pA[i][0], pA[i][1], pA[i][2] );
		ptsB[i] = mrpt::math::TPoint3D( pB[i][0], pB[i][1], pB[i][2] );
	}
} // end generate_vector_of_points

// ------------------------------------------------------
//				Benchmark: using CPose3DQuat
// ------------------------------------------------------
double scan_matching_test_1( int a1, int a2 )
{
	TPoints	pA, pB;
	generate_points( pA, pB );

	TMatchingPairList list;
	generate_list_of_points( pA, pB, list );

	CPose3DQuat out;
	double	scale;

	const size_t	N = a1;
	CTicTac			tictac;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		mrpt::tfest::se3_l2(list,out,scale);

	const double T = tictac.Tac()/N;
	return T;
}

// ------------------------------------------------------
//				Benchmark: using vectors
// ------------------------------------------------------
double scan_matching_test_3( int a1, int a2 )
{
	TPoints	pA, pB;
	generate_points( pA, pB );

	vector<mrpt::math::TPoint3D> ptsA, ptsB;
	generate_vector_of_points( pA, pB, ptsA, ptsB);

	const size_t	N = a1;
	CTicTac			tictac;

	mrpt::poses::CPose3DQuat out_pose;
	double out_scale;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		mrpt::tfest::se3_l2(ptsA,ptsB,out_pose,out_scale);

	const double T = tictac.Tac()/N;
	return T;
}


// ------------------------------------------------------
//				Benchmark:  leastSquareErrorRigidTransformation
// ------------------------------------------------------
double scan_matching_test_4( int nCorrs, int nRepets )
{
	TPoints	pA, pB;
	generate_points( pA, pB );

	vector<double> qu;

	TMatchingPairList	in_correspondences;
	mrpt::math::TPose2D out_pose;

	in_correspondences.resize(nCorrs);
	for (int i=0;i<nCorrs;i++)
	{
		TMatchingPair & m= in_correspondences[i];
		m.this_idx = i;
		m.other_idx = i;
		m.this_x = mrpt::random::randomGenerator.drawUniform(-10,10);
		m.this_y = mrpt::random::randomGenerator.drawUniform(-10,10);
		m.this_z = mrpt::random::randomGenerator.drawUniform(-10,10);
		m.other_x = mrpt::random::randomGenerator.drawUniform(-10,10);
		m.other_y = mrpt::random::randomGenerator.drawUniform(-10,10);
		m.other_z = mrpt::random::randomGenerator.drawUniform(-10,10);
	}

	const size_t	N = nRepets;
	CTicTac			tictac;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
		mrpt::tfest::se2_l2(in_correspondences, out_pose);
	}

	const double T = tictac.Tac()/N;
	return T;
}

// ------------------------------------------------------
// register_tests_scan_matching
// ------------------------------------------------------
void register_tests_scan_matching()
{
	lstTests.push_back( TestData("tfest: se3_l2 [CPose3DQuat]", scan_matching_test_1 , 1e4 ) );
	lstTests.push_back( TestData("tfest: se3_l2 [vector TPoint3D]", scan_matching_test_3 , 1e4) );

	lstTests.push_back( TestData("tfest: se2_l2 [x10 corrs]", scan_matching_test_4,  10, 1e6 ) );
	lstTests.push_back( TestData("tfest: se2_l2 [x100 corrs]", scan_matching_test_4,  100, 1e6 ) );
	lstTests.push_back( TestData("tfest: se2_l2 [x1000 corrs]", scan_matching_test_4,  1000, 1e5 ) );
}
