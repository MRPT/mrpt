/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

#include <mrpt/scanmatching.h>
#include <mrpt/base.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::scanmatching;
using namespace std;

typedef std::vector< std::vector< double > > TPoints;

// ------------------------------------------------------
//				Generate both sets of points
// ------------------------------------------------------
CPose3DQuat generate_points( TPoints &pA, TPoints &pB )
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

	CPose3DQuat qPose = CPose3DQuat(CPose3D( Dx, Dy, Dz, yaw, pitch, roll ));
	for( unsigned int i = 0; i < 5; ++i )
	{
		pB[i].resize( 3 );
		qPose.inverseComposePoint( pA[i][0], pA[i][1], pA[i][2], pB[i][0], pB[i][1], pB[i][2] );
	}

	return qPose;

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
void generate_vector_of_points(  const TPoints &pA, const TPoints &pB, vector_double &inV )
{
	// The input vector: inV = [pA1x, pA1y, pA1z, pB1x, pB1y, pB1z, ... ]
	inV.resize( 30 );
	for( unsigned int i = 0; i < 5; ++i )
	{
		inV[6*i+0] = pA[i][0]; inV[6*i+1] = pA[i][1]; inV[6*i+2] = pA[i][2];
		inV[6*i+3] = pB[i][0]; inV[6*i+4] = pB[i][1]; inV[6*i+5] = pB[i][2];
	}
} // end generate_vector_of_points




// Load data from constant file and check for exact match.
TEST(LSRigidTrans6D, CPose3D)
{
	TPoints	pA, pB;										// The input points
	CPose3DQuat qPose = generate_points( pA, pB );

	TMatchingPairList list;
	generate_list_of_points( pA, pB, list );			// Generate a list of matched points

	CPose3D			out;								// Output CPose3D for the LSRigidTransformation
	double			scale;								// Output scale value

	// Take the x,y,z,yaw,pitch,roll values for comparing
	double quat_yaw, quat_pitch, quat_roll;
	qPose.quat().rpy( quat_roll, quat_pitch, quat_yaw );
	const double quat_x = qPose.x();
	const double quat_y = qPose.y();
	const double quat_z = qPose.z();
	// --

	/*bool res1 =*/
	scanmatching::leastSquareErrorRigidTransformation6D( list, out, scale );
	const double err = sqrt(	square(out.x() - quat_x) + square(out.y() - quat_y) + square(out.z() - quat_z) +
								square(out.yaw() - quat_yaw) + square(out.pitch() - quat_pitch) + square(out.roll() - quat_roll) );
	EXPECT_TRUE( err< 1e-6 )
		<< "Applied quaternion: " << endl << qPose << endl
		<< "Out CPose3D: " << endl << out << " [Err: " << err << "]" << endl;
}

TEST(LSRigidTrans6D, CPose3DQuat)
{
	TPoints	pA, pB;										// The input points
	CPose3DQuat qPose = generate_points( pA, pB );

	TMatchingPairList list;
	generate_list_of_points( pA, pB, list );			// Generate a list of matched points

	CPose3DQuat		outQuat;							// Output CPose3DQuat for the LSRigidTransformation
	double			scale;								// Output scale value

	/*bool res2 =*/
	scanmatching::leastSquareErrorRigidTransformation6D( list, outQuat, scale );

	double err = 0.0;
	if( (qPose[3]*outQuat[3] > 0 && qPose[4]*outQuat[4] > 0 && qPose[5]*outQuat[5] > 0 && qPose[6]*outQuat[6] > 0) ||
		(qPose[3]*outQuat[3] < 0 && qPose[4]*outQuat[4] < 0 && qPose[5]*outQuat[5] < 0 && qPose[6]*outQuat[6] < 0) )
	{
		for( unsigned int i = 0; i < 7; ++i )
			err += square( std::fabs(qPose[i])-std::fabs(outQuat[i]) );
		err = sqrt(err);
		EXPECT_TRUE( err< 1e-6 )
			<< "Applied quaternion: " << endl << qPose << endl
			<< "Out CPose3DQuat: " << endl << outQuat << " [Err: " << err << "]" << endl;
	}
	else
	{
		GTEST_FAIL( )
		<< "Applied quaternion: " << endl << qPose << endl
		<< "Out CPose3DQuat: " << endl << outQuat << endl;
	}

}

TEST(LSRigidTrans6D, vector)
{
	TPoints	pA, pB;										// The input points
	CPose3DQuat qPose = generate_points( pA, pB );

	vector_double inV;
	generate_vector_of_points( pA, pB, inV );			// Generate a vector of matched points

	vector_double	qu;									// Output quaternion for the Horn Method

	HornMethod( inV, qu, false );

	double err = 0.0;
	if( (qPose[3]*qu[3] > 0 && qPose[4]*qu[4] > 0 && qPose[5]*qu[5] > 0 && qPose[6]*qu[6] > 0) ||
		(qPose[3]*qu[3] < 0 && qPose[4]*qu[4] < 0 && qPose[5]*qu[5] < 0 && qPose[6]*qu[6] < 0) )
	{

		for( unsigned int i = 0; i < 7; ++i )
			err += square( std::fabs(qPose[i])-std::fabs(qu[i]) );
		err = sqrt(err);
		EXPECT_TRUE( err< 1e-6 )
			<< "Applied quaternion: " << endl << qPose << endl
			<< "Out CPose3DQuat: " << endl << qu << " [Err: " << err << "]" << endl;
	}
	else
	{
		GTEST_FAIL( )
			<< "Applied quaternion: " << endl << qPose << endl
			<< "Out CPose3DQuat: " << endl << qu << endl;
	}
} // end
