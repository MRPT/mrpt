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

#include <mrpt/scanmatching.h>  // Precompiled header


#include <mrpt/scanmatching/scan_matching.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/random.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/CQuaternion.h>

#include <algorithm>

using namespace mrpt;
using namespace mrpt::scanmatching;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace std;

/*---------------------------------------------------------------
	HornMethod
  ---------------------------------------------------------------*/
double scanmatching::HornMethod(
	const vector_double		&inVector,
	vector_double			&outVector,				// The output vector
	bool forceScaleToUnity )
{
	MRPT_START

	vector_double input;
	input.resize( inVector.size() );
	input = inVector;
	outVector.resize( 7 );

	// Compute the centroids
	TPoint3D	cL(0,0,0), cR(0,0,0);

	const size_t nMatches = input.size()/6;
	ASSERT_EQUAL_(input.size()%6, 0)

	for( unsigned int i = 0; i < nMatches; i++ )
	{
		cL.x += input[i*6+3];
		cL.y += input[i*6+4];
		cL.z += input[i*6+5];

		cR.x += input[i*6+0];
		cR.y += input[i*6+1];
		cR.z += input[i*6+2];
	}

	ASSERT_ABOVE_(nMatches,0)
	const double F = 1.0/nMatches;
	cL *= F;
	cR *= F;

	CMatrixDouble33 S; // S.zeros(); // Zeroed by default

	// Substract the centroid and compute the S matrix of cross products
	for( unsigned int i = 0; i < nMatches; i++ )
	{
		input[i*6+3] -= cL.x;
		input[i*6+4] -= cL.y;
		input[i*6+5] -= cL.z;

		input[i*6+0] -= cR.x;
		input[i*6+1] -= cR.y;
		input[i*6+2] -= cR.z;

		S.get_unsafe(0,0) += input[i*6+3]*input[i*6+0];
		S.get_unsafe(0,1) += input[i*6+3]*input[i*6+1];
		S.get_unsafe(0,2) += input[i*6+3]*input[i*6+2];

		S.get_unsafe(1,0) += input[i*6+4]*input[i*6+0];
		S.get_unsafe(1,1) += input[i*6+4]*input[i*6+1];
		S.get_unsafe(1,2) += input[i*6+4]*input[i*6+2];

		S.get_unsafe(2,0) += input[i*6+5]*input[i*6+0];
		S.get_unsafe(2,1) += input[i*6+5]*input[i*6+1];
		S.get_unsafe(2,2) += input[i*6+5]*input[i*6+2];
	}

	// Construct the N matrix
	CMatrixDouble44 N; // N.zeros(); // Zeroed by default

	N.set_unsafe( 0,0,S.get_unsafe(0,0) + S.get_unsafe(1,1) + S.get_unsafe(2,2) );
	N.set_unsafe( 0,1,S.get_unsafe(1,2) - S.get_unsafe(2,1) );
	N.set_unsafe( 0,2,S.get_unsafe(2,0) - S.get_unsafe(0,2) );
	N.set_unsafe( 0,3,S.get_unsafe(0,1) - S.get_unsafe(1,0) );

	N.set_unsafe( 1,0,N.get_unsafe(0,1) );
	N.set_unsafe( 1,1,S.get_unsafe(0,0) - S.get_unsafe(1,1) - S.get_unsafe(2,2) );
	N.set_unsafe( 1,2,S.get_unsafe(0,1) + S.get_unsafe(1,0) );
	N.set_unsafe( 1,3,S.get_unsafe(2,0) + S.get_unsafe(0,2) );

	N.set_unsafe( 2,0,N.get_unsafe(0,2) );
	N.set_unsafe( 2,1,N.get_unsafe(1,2) );
	N.set_unsafe( 2,2,-S.get_unsafe(0,0) + S.get_unsafe(1,1) - S.get_unsafe(2,2) );
	N.set_unsafe( 2,3,S.get_unsafe(1,2) + S.get_unsafe(2,1) );

	N.set_unsafe( 3,0,N.get_unsafe(0,3) );
	N.set_unsafe( 3,1,N.get_unsafe(1,3) );
	N.set_unsafe( 3,2,N.get_unsafe(2,3) );
	N.set_unsafe( 3,3,-S.get_unsafe(0,0) - S.get_unsafe(1,1) + S.get_unsafe(2,2) );

	// q is the quaternion correspondent to the greatest eigenvector of the N matrix (last column in Z)
	CMatrixDouble44 Z, D;
	vector_double v;

	N.eigenVectors( Z, D );
	Z.extractCol( Z.getColCount()-1, v );

	ASSERTDEB_( fabs( sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3] ) - 1.0 ) < 0.1 );

	// Make q_r > 0
	if( v[0] < 0 ){ v[0] *= -1;	v[1] *= -1;	v[2] *= -1;	v[3] *= -1;	}

	CPose3DQuat q;		// Create a pose rotation with the quaternion
	for(unsigned int i = 0; i < 4; i++ )			// insert the quaternion part
		outVector[i+3] = q[i+3] = v[i];

	// Compute scale
	double	num = 0.0;
	double	den = 0.0;
	for( unsigned int i = 0; i < nMatches; i++ )
	{
		num += square( input[i*6+0] ) + square( input[i*6+1] ) + square( input[i*6+2] );
		den += square( input[i*6+3] ) + square( input[i*6+4] ) + square( input[i*6+5] );
	} // end-for

	// The scale:
	double s = std::sqrt( num/den );

	// Enforce scale to be 1
	if( forceScaleToUnity )
		s = 1.0;

	TPoint3D pp;
	q.composePoint( cL.x, cL.y, cL.z, pp.x, pp.y, pp.z );
	pp*=s;

	outVector[0] = cR.x - pp.x;	// X
	outVector[1] = cR.y - pp.y;	// Y
	outVector[2] = cR.z - pp.z;	// Z

	return s; // return scale
	MRPT_END
}

//! \overload
double scanmatching::HornMethod(
	const vector_double      &inPoints,
	mrpt::poses::CPose3DQuat &outQuat,
	bool                      forceScaleToUnity )
{
	vector_double outV;
	const double s = HornMethod(inPoints,outV,forceScaleToUnity);
	for (int i=0;i<7;i++)
		outQuat[i]=outV[i];
	return s;
}


//*---------------------------------------------------------------
//	leastSquareErrorRigidTransformation6D
//  ---------------------------------------------------------------*/
bool  scanmatching::leastSquareErrorRigidTransformation6D(
	const TMatchingPairList	&in_correspondences,
	CPose3DQuat							&out_transformation,
	double								&out_scale,
	const bool 							forceScaleToUnity)
{

	MRPT_START
	if( in_correspondences.size() < 3 )
		THROW_EXCEPTION( "[leastSquareErrorRigidTransformation6D]: Error: at least 3 correspondences must be provided" );

	CPoint3D cL, cR;
	CMatrixD S, N;
	CMatrixD Z, D;

	vector_double v;
	const size_t nMatches = in_correspondences.size();
	double s; // Scale

	// Compute the centroid
	TMatchingPairList::const_iterator	itMatch;

	for(itMatch = in_correspondences.begin(); itMatch != in_correspondences.end(); itMatch++)
	{
		cL.x_incr( itMatch->other_x );
		cL.y_incr( itMatch->other_y );
		cL.z_incr( itMatch->other_z );

		cR.x_incr( itMatch->this_x );
		cR.y_incr( itMatch->this_y );
		cR.z_incr( itMatch->this_z );
	}
	const double F = 1.0/nMatches;
	cL *= F;
	cR *= F;

	TMatchingPairList			auxList( in_correspondences );
	TMatchingPairList::iterator auxIt;
	// Substract the centroid
	for( auxIt = auxList.begin(); auxIt != auxList.end(); auxIt++ )
	{
		auxIt->other_x -= cL.x();
		auxIt->other_y -= cL.y();
		auxIt->other_z -= cL.z();

		auxIt->this_x -= cR.x();
		auxIt->this_y -= cR.y();
		auxIt->this_z -= cR.z();
	}

	// Compute the S matrix of products
	S.setSize(3,3);
	S.fill(0);
	for( auxIt = auxList.begin(); auxIt != auxList.end(); auxIt++ )
	{
		S(0,0) += auxIt->other_x * auxIt->this_x;
		S(0,1) += auxIt->other_x * auxIt->this_y;
		S(0,2) += auxIt->other_x * auxIt->this_z;

		S(1,0) += auxIt->other_y * auxIt->this_x;
		S(1,1) += auxIt->other_y * auxIt->this_y;
		S(1,2) += auxIt->other_y * auxIt->this_z;

		S(2,0) += auxIt->other_z * auxIt->this_x;
		S(2,1) += auxIt->other_z * auxIt->this_y;
		S(2,2) += auxIt->other_z * auxIt->this_z;
	}

	N.setSize(4,4);
	N.fill(0);

	N(0,0) = S(0,0) + S(1,1) + S(2,2);
	N(0,1) = S(1,2) - S(2,1);
	N(0,2) = S(2,0) - S(0,2);
	N(0,3) = S(0,1) - S(1,0);

	N(1,0) = N(0,1);
	N(1,1) = S(0,0) - S(1,1) - S(2,2);
	N(1,2) = S(0,1) + S(1,0);
	N(1,3) = S(2,0) + S(0,2);

	N(2,0) = N(0,2);
	N(2,1) = N(1,2);
	N(2,2) = -S(0,0) + S(1,1) - S(2,2);
	N(2,3) = S(1,2) + S(2,1);

	N(3,0) = N(0,3);
	N(3,1) = N(1,3);
	N(3,2) = N(2,3);
	N(3,3) = -S(0,0) - S(1,1) + S(2,2);

	// q is the quaternion correspondent to the greatest eigenvector of the N matrix (last column in Z)
	N.eigenVectors( Z, D );
	Z.extractCol( Z.getColCount()-1, v );

	CPose3DQuat q;
	for(unsigned int i = 0; i < 4; i++ )			// Set out_transformation [rotation]
		out_transformation[i+3] = q[i+3] = v[i];

	// Compute scale
	double	num = 0.0;
	double	den = 0.0;
	for( auxIt = auxList.begin(); auxIt != auxList.end(); auxIt++ )
	{
		den += square(auxIt->other_x) + square(auxIt->other_y) + square(auxIt->other_z);
		num += square(auxIt->this_x) + square(auxIt->this_y) + square(auxIt->this_z);
	}
	s = sqrt( num/den );

	// Enforce scale to be 1
	out_scale = s;
	if (forceScaleToUnity)
		s = 1.0;

	CPoint3D pp, aux;
	q.composePoint( cL.x(), cL.y(), cL.z(), pp.x(), pp.y(), pp.z() );
	pp*=s;

	// Set out_transformation [traslation]
	out_transformation[0] = cR.x() - pp.x();	// X
	out_transformation[1] = cR.y() - pp.y();	// Y
	out_transformation[2] = cR.z() - pp.z();	// Z

	MRPT_END

	return true;

/*---------------------------------------------------------------
			leastSquareErrorRigidTransformation in 6D
  ---------------------------------------------------------------*/
	// Algorithm:
	// 0. Preliminary
	//		pLi = { pLix, pLiy, pLiz }
	//		pRi = { pRix, pRiy, pRiz }
	// -------------------------------------------------------
	// 1. Find the centroids of the two sets of measurements:
	//		cL = (1/n)*sum{i}( pLi )		cL = { cLx, cLy, cLz }
	//		cR = (1/n)*sum{i}( pRi )		cR = { cRx, cRy, cRz }
	//
	// 2. Substract centroids from the point coordinates:
	//		pLi' = pLi - cL					pLi' = { pLix', pLiy', pLiz' }
	//		pRi' = pRi - cR					pRi' = { pRix', pRiy', pRiz' }
	//
	// 3. For each pair of coordinates (correspondences) compute the nine possible products:
	//		pi1 = pLix'*pRix'		pi2 = pLix'*pRiy'		pi3 = pLix'*pRiz'
	//		pi4 = pLiy'*pRix'		pi5 = pLiy'*pRiy'		pi6 = pLiy'*pRiz'
	//		pi7 = pLiz'*pRix'		pi8 = pLiz'*pRiy'		pi9 = pLiz'*pRiz'
	//
	// 4. Compute S components:
	//		Sxx = sum{i}( pi1 )		Sxy = sum{i}( pi2 )		Sxz = sum{i}( pi3 )
	//		Syx = sum{i}( pi4 )		Syy = sum{i}( pi5 )		Syz = sum{i}( pi6 )
	//		Szx = sum{i}( pi7 )		Szy = sum{i}( pi8 )		Szz = sum{i}( pi9 )
	//
	// 5. Compute N components:
	//			[ Sxx+Syy+Szz	Syz-Szy			Szx-Sxz			Sxy-Syx		 ]
	//			[ Syz-Szy		Sxx-Syy-Szz		Sxy+Syx			Szx+Sxz		 ]
	//		N = [ Szx-Sxz		Sxy+Syx			-Sxx+Syy-Szz	Syz+Szy		 ]
	//			[ Sxy-Syx		Szx+Sxz			Syz+Szy			-Sxx-Syy+Szz ]
	//
	// 6. Rotation represented by the quaternion eigenvector correspondent to the higher eigenvalue of N
	//
	// 7. Scale computation (symmetric expression)
	//		s = sqrt( sum{i}( square(abs(pRi')) / sum{i}( square(abs(pLi')) ) )
	//
	// 8. Translation computation (distance between the Right centroid and the scaled and rotated Left centroid)
	//		t = cR-sR(cL)
}
