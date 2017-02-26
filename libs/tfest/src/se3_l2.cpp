/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "tfest-precomp.h"  // Precompiled headers

#include <mrpt/tfest/se3.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// "Closed-form solution of absolute orientation using unit quaternions", BKP Horn, Journal of the Optical Society of America, 1987.
// Algorithm:
// 0. Preliminary
//		pLi = { pLix, pLiy, pLiz }
//		pRi = { pRix, pRiy, pRiz }
// -------------------------------------------------------
// 1. Find the centroids of the two sets of measurements:
//		ct_others = (1/n)*sum{i}( pLi )		ct_others = { cLx, cLy, cLz }
//		ct_this = (1/n)*sum{i}( pRi )		ct_this = { cRx, cRy, cRz }
//
// 2. Substract centroids from the point coordinates:
//		pLi' = pLi - ct_others					pLi' = { pLix', pLiy', pLiz' }
//		pRi' = pRi - ct_this					pRi' = { pRix', pRiy', pRiz' }
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
//		t = ct_this-sR(ct_others)

/*---------------------------------------------------------------
                    se3_l2  (old "HornMethod()")
  ---------------------------------------------------------------*/
bool se3_l2_internal(
	std::vector<mrpt::math::TPoint3D> & points_this,   // IN/OUT: It gets modified!
	std::vector<mrpt::math::TPoint3D> & points_other,  // IN/OUT: It gets modified!
	mrpt::poses::CPose3DQuat    & out_transform,
	double                     & out_scale,
	bool                         forceScaleToUnity)
{
	MRPT_START

	ASSERT_EQUAL_(points_this.size(),points_other.size())

	// Compute the centroids
	TPoint3D	ct_others(0,0,0), ct_this(0,0,0);
	const size_t nMatches = points_this.size();
	
	if (nMatches<3)
		return false; // Nothing we can estimate without 3 points!!

	for(size_t i = 0; i < nMatches; i++ )
	{
		ct_others += points_other[i];
		ct_this += points_this [i];
	}

	const double F = 1.0/nMatches;
	ct_others *= F;
	ct_this *= F;

	CMatrixDouble33 S; // Zeroed by default

	// Substract the centroid and compute the S matrix of cross products
	for(size_t i = 0; i < nMatches; i++ )
	{
		points_this[i]  -= ct_this;
		points_other[i] -= ct_others;

		S.get_unsafe(0,0) += points_other[i].x * points_this[i].x;
		S.get_unsafe(0,1) += points_other[i].x * points_this[i].y;
		S.get_unsafe(0,2) += points_other[i].x * points_this[i].z;

		S.get_unsafe(1,0) += points_other[i].y * points_this[i].x;
		S.get_unsafe(1,1) += points_other[i].y * points_this[i].y;
		S.get_unsafe(1,2) += points_other[i].y * points_this[i].z;

		S.get_unsafe(2,0) += points_other[i].z * points_this[i].x;
		S.get_unsafe(2,1) += points_other[i].z * points_this[i].y;
		S.get_unsafe(2,2) += points_other[i].z * points_this[i].z;
	}

	// Construct the N matrix
	CMatrixDouble44 N; // Zeroed by default

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
	vector<double> v;

	N.eigenVectors( Z, D );
	Z.extractCol( Z.getColCount()-1, v );

	ASSERTDEB_( fabs( sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3] ) - 1.0 ) < 0.1 );

	// Make q_r > 0
	if( v[0] < 0 ){ v[0] *= -1;	v[1] *= -1;	v[2] *= -1;	v[3] *= -1;	}

	// out_transform: Create a pose rotation with the quaternion
	for(unsigned int i = 0; i < 4; i++ )			// insert the quaternion part
		out_transform[i+3] = v[i];

	// Compute scale
	double s;
	if( forceScaleToUnity )
	{
		s = 1.0; // Enforce scale to be 1
	}
	else
	{
		double	num = 0.0;
		double	den = 0.0;
		for(size_t i = 0; i < nMatches; i++ )
		{
			num += square( points_other[i].x ) + square( points_other[i].y ) + square( points_other[i].z );
			den += square( points_this[i].x )  + square( points_this[i].y )  + square( points_this[i].z );
		} // end-for

		// The scale:
		s = std::sqrt( num/den );
	}

	TPoint3D pp;
	out_transform.composePoint( ct_others.x, ct_others.y, ct_others.z, pp.x, pp.y, pp.z );
	pp*=s;

	out_transform[0] = ct_this.x - pp.x;	// X
	out_transform[1] = ct_this.y - pp.y;	// Y
	out_transform[2] = ct_this.z - pp.z;	// Z

	out_scale=s; // return scale
	return true;

	MRPT_END
} // end se3_l2_internal()


bool tfest::se3_l2(
	const std::vector<mrpt::math::TPoint3D> & in_points_this,
	const std::vector<mrpt::math::TPoint3D> & in_points_other,
	mrpt::poses::CPose3DQuat   & out_transform,
	double                     & out_scale,
	bool                         forceScaleToUnity)
{
	// make a copy because we need it anyway to substract the centroid and to provide a unified interface to TMatchingList API
	std::vector<mrpt::math::TPoint3D> points_this = in_points_this; 
	std::vector<mrpt::math::TPoint3D> points_other= in_points_other;

	return se3_l2_internal(points_this,points_other,out_transform,out_scale,forceScaleToUnity);
}

bool tfest::se3_l2(
	const mrpt::utils::TMatchingPairList  & corrs,
	mrpt::poses::CPose3DQuat   & out_transform,
	double                     & out_scale,
	bool                         forceScaleToUnity)
{
	// Transform data types:
	const size_t N = corrs.size();
	std::vector<mrpt::math::TPoint3D> points_this(N), points_other(N);
	for (size_t i=0;i<N;i++) 
	{
		points_this[i].x = corrs[i].this_x;
		points_this[i].y = corrs[i].this_y;
		points_this[i].z = corrs[i].this_z;
		points_other[i].x = corrs[i].other_x;
		points_other[i].y = corrs[i].other_y;
		points_other[i].z = corrs[i].other_z;
	}
	return se3_l2_internal(points_this,points_other,out_transform,out_scale,forceScaleToUnity);
}

