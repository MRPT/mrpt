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

#include <mrpt/vision.h>  // Precompiled headers


#include <mrpt/vision/utils.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/CFeature.h>

#include <mrpt/poses/CPoint3D.h>
#include <mrpt/slam/CLandmarksMap.h>
#include <mrpt/slam/CObservationVisualLandmarks.h>
#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/slam/CObservationBearingRange.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/geometry.h>
//#include <Eigen/Core>
//using namespace Eigen;

#include "do_opencv_includes.h"

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

#ifdef MRPT_OS_WINDOWS
    #include <process.h>
    #include <windows.h>		// TODO: This is temporary!!!
#endif

const int NOT_ASIG = 0;
const int ASG_FEAT = 1;
const int AMB_FEAT = 2;
const int FEAT_FREE = -1;

void _updateCounter( const vector<int> &idx, CFeatureList &list)
{
    ASSERT_( idx.size() == list.size() );

    for( int k = 0; k < (int)idx.size(); ++k )
    {
        if( idx[k] == FEAT_FREE )
        {
            list[k]->nTimesNotSeen++;
            list[k]->nTimesLastSeen++;
        }
        else
        {
            list[k]->nTimesSeen++;
            list[k]->nTimesLastSeen = 0;
        }
    }
}
/*-------------------------------------------------------------
					buildIntrinsicParamsMatrix
-------------------------------------------------------------*/
CMatrixDouble33  vision::buildIntrinsicParamsMatrix(
	const double focalLengthX,
	const double focalLengthY,
	const double centerX,
	const double centerY)
{
	CMatrixDouble33 A;

	A(0,0) = focalLengthX;
	A(1,1) = focalLengthY;
	A(2,2) = 1;

	A(0,2) = centerX;
	A(1,2) = centerY;

	return A;
}

/*-------------------------------------------------------------
					pixelTo3D
-------------------------------------------------------------*/
TPoint3D  vision::pixelTo3D(const vision::TPixelCoordf &xy, const CMatrixDouble33 &A)
{
	TPoint3D	res;

	// Build the vector:
	res.x = xy.x - A.get_unsafe(0,2);
	res.y = xy.y - A.get_unsafe(1,2);
	res.z = A.get_unsafe(0,0);

	// Normalize:
	const double	u = res.norm();
	ASSERT_(u!=0)
	res*=1.0/u;

	return res;
}

/*-------------------------------------------------------------
					defaultIntrinsicParamsMatrix
-------------------------------------------------------------*/
CMatrixDouble33 vision::defaultIntrinsicParamsMatrix(
					unsigned int camIndex,
					unsigned int resX,
					unsigned int resY)
{
	float		fx,fy,cx,cy;

	switch(camIndex)
	{
	case 0:
		// Bumblebee:
		fx=0.79345f;	fy=1.05793f;
		cx=0.55662f;	cy=0.52692f;
		break;

	case 1:
		// Sony:
		fx=0.95666094f;	fy=1.3983423f;
		cx=0.54626328f;	cy=0.4939191f;
		break;

	default:
		{
			THROW_EXCEPTION_CUSTOM_MSG1( "Unknown camera index!! for 'camIndex'=%u",camIndex );
		}
	}

	return buildIntrinsicParamsMatrix(	resX * fx,	resY * fy,
										resX * cx,	resY * cy );

}

/*-------------------------------------------------------------
					addFeaturesToImage
-------------------------------------------------------------*/
void  vision::addFeaturesToImage( const CImage &inImg, const CFeatureList &theList, CImage &outImg )
{
	outImg = inImg;												// Create a copy of the input image
	for( CFeatureList::const_iterator it = theList.begin(); it != theList.end(); ++it )
		outImg.rectangle( (*it)->x-5, (*it)->y-5, (*it)->x+5, (*it)->y+5, TColor(255,0,0) );
}

/*-------------------------------------------------------------
					projectMatchedFeatures
-------------------------------------------------------------*/
void  vision::projectMatchedFeatures(
	CFeatureList					    &leftList,		// The left of matched features (matches must be ordered!)
	CFeatureList					    &rightList,		// The right of matched features (matches must be ordered!)
	const vision::TStereoSystemParams	&param,			// Parameters for the stereo system
	mrpt::slam::CLandmarksMap			&landmarks )	// Output map of 3D landmarks
{
	MRPT_START;
	ASSERT_( leftList.size() == rightList.size() );

	landmarks.clear();								// Assert that the output CLandmarksMap is clear

	CFeatureList::iterator		itListL, itListR;
	float						stdPixel2	= square( param.stdPixel );
	float						stdDisp2	= square( param.stdDisp );

	// Main loop
	for(itListL = leftList.begin(), itListR = rightList.begin(); itListL != leftList.end();)
	{
		float disp = ( (*itListL)->x - (*itListR)->x );	// Disparity
		if ( disp < 1e-9 )							// Filter out too far points
		{
			itListL = leftList.erase( itListL );		// Erase the match : itListL = leftList.erase( itListL );
			itListR = rightList.erase( itListR );		// Erase the match : itListR = rightList.erase( itListR );
		}
		else													// This
		{
			// Too much distant features are not taken into account
			float x3D = ( (*itListL)->x - param.K(0,2) ) * ( (param.baseline) ) / disp;
			float y3D = ( (*itListL)->y - param.K(1,2) ) * ( (param.baseline) ) / disp;
			float z3D = ( param.K(0,0) ) * ( (param.baseline) ) / disp;

			// Filter out bad points
			if ( (z3D < param.minZ) || (z3D > param.maxZ) )
			{
				itListL = leftList.erase( itListL );		// Erase the match : (*itListL) = leftList.erase( (*itListL) );
				itListR = rightList.erase( itListR );		// Erase the match : (*itListR) = rightList.erase( (*itListR) );
			}
			else
			{
				TPoint3D	p3D(x3D,y3D,z3D);

				// STORE THE OBTAINED LANDMARK
				CLandmark	lm;

				TPoint3D	norm3D = p3D;
				norm3D *= -1/norm3D.norm();

				lm.normal = norm3D;
				lm.pose_mean = p3D;
				lm.ID = (*itListL)->ID;

				// If the matched landmarks has a (SIFT or SURF) descriptor, asign the left one to the landmark.
				// TO DO: Assign the mean value of the descriptor (between the matches)
				lm.features.resize(2);
				lm.features[0] = *itListL;
				lm.features[1] = *itListR;

				// Compute the covariance matrix for the landmark
				switch( param.uncPropagation )
				{
				case TStereoSystemParams::Prop_Linear:
				{

					float foc2	= square( param.K(0,0) );
					float c0	= param.K(0,2);
					float r0	= param.K(1,2);
					float base2	= square( param.baseline );
					float disp2	= square( (*itListL)->x - (*itListR)->x );

					lm.pose_cov_11 = stdPixel2*base2/disp2 + stdDisp2*base2*square( (*itListL)->x - c0 )/square(disp2);
					lm.pose_cov_12 = stdDisp2*base2*( (*itListL)->x - c0 )*( (*itListL)->y - r0 )/square(disp2);
					lm.pose_cov_13 = stdDisp2*base2*sqrt(foc2)*( (*itListL)->x - c0 )/square(disp2);
					lm.pose_cov_22 = stdPixel2*base2/disp2 + stdDisp2*base2*square( (*itListL)->y - r0 )/square(disp2);
					lm.pose_cov_23 = stdDisp2*base2*sqrt(foc2)*( (*itListL)->y - r0 )/square(disp2);
					lm.pose_cov_33 = stdDisp2*foc2*base2/square(disp2);
				} // end case 'Prop_Linear'
				break;

				case TStereoSystemParams::Prop_UT:
				{
					// Parameters
					unsigned int			Na = 3;
					unsigned int			i;

					float					k = param.factor_k;

					float					w0 = k/(Na + k);
					float					w1 = 1/(2*(Na + k));

					CMatrix					Pa(3,3);
					CMatrix					L(3,3);

					Pa.fill(0);
					Pa(0,0) = Pa(1,1) = ( Na + k ) * square( param.stdPixel );
					Pa(2,2) = ( Na + k ) * square( param.stdDisp );

					// Cholesky decomposition
					Pa.chol(L); // math::chol(Pa,L);

					vector<TPoint3D>	B;				// B group
					poses::TPoint3D			meanB;			// Mean value of the B group
					CMatrix					Pb;				// Covariance of the B group

					B.resize( 2*Na + 1 );	// Set of output values
					Pb.fill(0);				// Reset the output covariance

					vector_float			vAux, myPoint;	// Auxiliar vectors
					vector_float			meanA;			// Mean value of the A group

					vAux.resize(3);			// Set the variables size
					meanA.resize(3);
					myPoint.resize(3);

					// Mean input value: (c,r,d)
					meanA[0] = (*itListL)->x;
					meanA[1] = (*itListL)->y;
					meanA[2] = disp;

					// Output mean
					meanB.x = w0*x3D; meanB.y = w0*y3D; meanB.z = w0*z3D;	// Add to the mean
					B[0].x = x3D; B[0].y = y3D; B[0].z = z3D;				// Insert into B

					for( i = 1; i <= 2*Na; i++ )
					{
						// Form the Ai value
						if( i <= Na )
						{
							L.extractRowAsCol( i-1, vAux );						// Extract the proper row
							myPoint[0] = meanA[0] + vAux[0];
							myPoint[1] = meanA[1] + vAux[1];
							myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							L.extractRowAsCol( (i-Na)-1, vAux );					// Extract the proper row
							myPoint[0] = meanA[0] - vAux[0];
							myPoint[1] = meanA[1] - vAux[1];
							myPoint[2] = meanA[2] - vAux[2];
						}

						// Pass the Ai through the functions:
						x3D = ( myPoint[0] - param.K(0,2) ) * ( (param.baseline) ) / myPoint[2];
						y3D = ( myPoint[1] - param.K(1,2) ) * ( (param.baseline) ) / myPoint[2];
						z3D = ( param.K(0,0) ) * ( (param.baseline) ) / myPoint[2];

						// Add to the B mean computation and the B vector
						meanB.x = meanB.x + w1*x3D;
						meanB.y = meanB.y + w1*y3D;
						meanB.z = meanB.z + w1*z3D;

						B[i].x = x3D;
						B[i].y = y3D;
						B[i].z = z3D;

					} // end for 'i'

					// Output covariance
					for( i = 0; i <= 2*Na; i++ )
					{
						float		weight = w1;
						CMatrix		v(3,1);

						if( i == 0 )				// The weight for the mean value of A is w0
							weight = w0;

						v(0,0) = B[i].x - meanB.x;
						v(1,0) = B[i].y - meanB.y;
						v(2,0) = B[i].z - meanB.z;

						Pb = Pb + weight*(v*~v);
					} // end for 'i'

					// Store it in the landmark
					lm.pose_cov_11 = Pb(0,0);
					lm.pose_cov_12 = Pb(0,1);
					lm.pose_cov_13 = Pb(0,2);
					lm.pose_cov_22 = Pb(1,1);
					lm.pose_cov_23 = Pb(1,2);
					lm.pose_cov_33 = Pb(2,2);
				} // end case 'Prop_UT'
				break;

				case TStereoSystemParams::Prop_SUT:
				{
					// Parameters
					unsigned int			Na = 3;
					unsigned int			i;

					float					a = param.factor_a;
					float					b = param.factor_b;
					float					k = param.factor_k;

					float					lambda = square(a)*(Na + k) - Na;

					float					w0_m = lambda/(Na + lambda);
					float					w0_c = w0_m + (1 - square(a) + b);
					float					w1 = 1/(2*(Na + lambda));

					CMatrix					Pa(3,3);
					CMatrix					L(3,3);

					Pa.fill(0);
					Pa(0,0) = Pa(1,1) = ( Na + lambda ) * square( param.stdPixel );
					Pa(2,2) = ( Na + lambda ) * square( param.stdDisp );

					// Cholesky decomposition
					Pa.chol(L); //math::chol(Pa,L);

					vector<TPoint3D>	B;				// B group
					poses::TPoint3D		meanB;			// Mean value of the B group
					CMatrix				Pb;				// Covariance of the B group

					B.resize( 2*Na + 1 );	// Set of output values
					Pb.fill(0);				// Reset the output covariance

					vector_float		vAux, myPoint;	// Auxiliar vectors
					vector_float		meanA;			// Mean value of the A group

					vAux.resize(3);			// Set the variables size
					meanA.resize(3);
					myPoint.resize(3);

					// Mean input value: (c,r,d)
					meanA[0] = (*itListL)->x;
					meanA[1] = (*itListL)->y;
					meanA[2] = disp;

					// Output mean
					meanB.x = w0_m*x3D; meanB.y = w0_m*y3D; meanB.z = w0_m*z3D;		// Add to the mean
					B[0].x = x3D; B[0].y = y3D; B[0].z = z3D;						// Insert into B

					for( i = 1; i <= 2*Na; i++ )
					{
						// Form the Ai value
						if( i <= Na )
						{
							L.extractRowAsCol( i-1, vAux );						// Extract the proper row
							myPoint = meanA + vAux;
							//myPoint[0] = meanA[0] + vAux[0];
							//myPoint[1] = meanA[1] + vAux[1];
							//myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							L.extractRowAsCol( (i-Na)-1, vAux );					// Extract the proper row
							myPoint = meanA - vAux;
							//myPoint[0] = meanA[0] - vAux[0];
							//myPoint[1] = meanA[1] - vAux[1];
							//myPoint[2] = meanA[2] - vAux[2];
						}

						// Pass the Ai through the functions:
						x3D = ( myPoint[0] - param.K(0,2) ) * ( (param.baseline) ) / myPoint[2];
						y3D = ( myPoint[1] - param.K(1,2) ) * ( (param.baseline) ) / myPoint[2];
						z3D = ( param.K(0,0) ) * ( (param.baseline) ) / myPoint[2];

						// Add to the B mean computation and the B vector
						meanB.x = meanB.x + w1*x3D;
						meanB.y = meanB.y + w1*y3D;
						meanB.z = meanB.z + w1*z3D;

						B[i].x = x3D;
						B[i].y = y3D;
						B[i].z = z3D;

					} // end for 'i'

					// Output covariance
					for( i = 0; i <= 2*Na; i++ )
					{
						float		weight = w1;
						CMatrix		v(3,1);

						if( i == 0 )				// The weight for the mean value of A is w0
							weight = w0_c;

						v(0,0) = B[i].x - meanB.x;
						v(1,0) = B[i].y - meanB.y;
						v(2,0) = B[i].z - meanB.z;

						Pb = Pb + weight*(v*~v);
					} // end for 'i'

					// Store it in the landmark
					lm.pose_cov_11 = Pb(0,0);
					lm.pose_cov_12 = Pb(0,1);
					lm.pose_cov_13 = Pb(0,2);
					lm.pose_cov_22 = Pb(1,1);
					lm.pose_cov_23 = Pb(1,2);
					lm.pose_cov_33 = Pb(2,2);
				} // end case 'Prop_SUT'
				break;

				} // end switch
				landmarks.landmarks.push_back( lm );
				itListL++;
				itListR++;
			} // end else ( (z3D > param.minZ) && (z3D < param.maxZ) )
		} // end else
	} // end for 'i'

	MRPT_END;
}
/*-------------------------------------------------------------
					projectMatchedFeatures
-------------------------------------------------------------*/
void  vision::projectMatchedFeatures(
	CMatchedFeatureList			        &mfList,		// The set of matched features
	const vision::TStereoSystemParams	&param,			// Parameters for the stereo system
	mrpt::slam::CLandmarksMap			&landmarks )	// Output map of 3D landmarks
{
	MRPT_START;

	landmarks.clear();								// Assert that the output CLandmarksMap is clear

	CMatchedFeatureList::iterator		itList;
	float								stdPixel2	= square( param.stdPixel );
	float								stdDisp2	= square( param.stdDisp );

	// Main loop
	for(itList = mfList.begin(); itList != mfList.end();)
	{
		float disp = ( itList->first->x - itList->second->x );	// Disparity
		if ( disp < 1e-9 )										// Filter out too far points
			itList = mfList.erase( itList );					// Erase the match : itList = mfList.erase( itList );

		else													// This
		{
			// Too much distant features are not taken into account
			float x3D = ( itList->first->x - param.K(0,2) ) * ( (param.baseline) ) / disp;
			float y3D = ( itList->first->y - param.K(1,2) ) * ( (param.baseline) ) / disp;
			float z3D = ( param.K(0,0) ) * ( (param.baseline) ) / disp;

			// Filter out bad points
			if ( (z3D < param.minZ) || (z3D > param.maxZ) )
				itList = mfList.erase( itList );					// Erase the match : itList = mfList.erase( itList );
			else
			{
				TPoint3D	p3D(x3D,y3D,z3D);

				// STORE THE OBTAINED LANDMARK
				CLandmark	lm;

				TPoint3D	norm3D = p3D;
				norm3D *= -1/norm3D.norm();

				lm.normal = norm3D;
				lm.pose_mean = p3D;
				lm.ID = itList->first->ID;

				// If the matched landmarks has a (SIFT or SURF) descriptor, asign the left one to the landmark.
				// TO DO: Assign the mean value of the descriptor (between the matches)
				lm.features.resize(1);
				lm.features[0] = (*itList).first;

				// Compute the covariance matrix for the landmark
				switch( param.uncPropagation )
				{
				case TStereoSystemParams::Prop_Linear:
				{

					float foc2	= square( param.K(0,0) );
					float c0	= param.K(0,2);
					float r0	= param.K(1,2);
					float base2	= square( param.baseline );
					float disp2	= square( itList->first->x - itList->second->x );

					lm.pose_cov_11 = stdPixel2*base2/disp2 + stdDisp2*base2*square( itList->first->x - c0 )/square(disp2);
					lm.pose_cov_12 = stdDisp2*base2*( itList->first->x - c0 )*( itList->first->y - r0 )/square(disp2);
					lm.pose_cov_13 = stdDisp2*base2*sqrt(foc2)*( itList->first->x - c0 )/square(disp2);
					lm.pose_cov_22 = stdPixel2*base2/disp2 + stdDisp2*base2*square( itList->first->y - r0 )/square(disp2);
					lm.pose_cov_23 = stdDisp2*base2*sqrt(foc2)*( itList->first->y - r0 )/square(disp2);
					lm.pose_cov_33 = stdDisp2*foc2*base2/square(disp2);
				} // end case 'Prop_Linear'
				break;

				case TStereoSystemParams::Prop_UT:
				{
					// Parameters
					unsigned int			Na = 3;
					unsigned int			i;

					float					k = param.factor_k;

					float					w0 = k/(Na + k);
					float					w1 = 1/(2*(Na + k));

					CMatrix					Pa(3,3);
					CMatrix					L(3,3);

					Pa.fill(0);
					Pa(0,0) = Pa(1,1) = ( Na + k ) * square( param.stdPixel );
					Pa(2,2) = ( Na + k ) * square( param.stdDisp );

					// Cholesky decomposition
					Pa.chol(L); // math::chol(Pa,L);

					vector<TPoint3D>	B;				// B group
					poses::TPoint3D			meanB;			// Mean value of the B group
					CMatrix					Pb;				// Covariance of the B group

					B.resize( 2*Na + 1 );	// Set of output values
					Pb.fill(0);				// Reset the output covariance

					vector_float			vAux, myPoint;	// Auxiliar vectors
					vector_float			meanA;			// Mean value of the A group

					vAux.resize(3);			// Set the variables size
					meanA.resize(3);
					myPoint.resize(3);

					// Mean input value: (c,r,d)
					meanA[0] = itList->first->x;
					meanA[1] = itList->first->y;
					meanA[2] = disp;

					// Output mean
					meanB.x = w0*x3D; meanB.y = w0*y3D; meanB.z = w0*z3D;	// Add to the mean
					B[0].x = x3D; B[0].y = y3D; B[0].z = z3D;				// Insert into B

					for( i = 1; i <= 2*Na; i++ )
					{
						// Form the Ai value
						if( i <= Na )
						{
							L.extractRowAsCol( i-1, vAux );						// Extract the proper row
							myPoint[0] = meanA[0] + vAux[0];
							myPoint[1] = meanA[1] + vAux[1];
							myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							L.extractRowAsCol( (i-Na)-1, vAux );					// Extract the proper row
							myPoint[0] = meanA[0] - vAux[0];
							myPoint[1] = meanA[1] - vAux[1];
							myPoint[2] = meanA[2] - vAux[2];
						}

						// Pass the Ai through the functions:
						x3D = ( myPoint[0] - param.K(0,2) ) * ( (param.baseline) ) / myPoint[2];
						y3D = ( myPoint[1] - param.K(1,2) ) * ( (param.baseline) ) / myPoint[2];
						z3D = ( param.K(0,0) ) * ( (param.baseline) ) / myPoint[2];

						// Add to the B mean computation and the B vector
						meanB.x = meanB.x + w1*x3D;
						meanB.y = meanB.y + w1*y3D;
						meanB.z = meanB.z + w1*z3D;

						B[i].x = x3D;
						B[i].y = y3D;
						B[i].z = z3D;

					} // end for 'i'

					// Output covariance
					for( i = 0; i <= 2*Na; i++ )
					{
						float		weight = w1;
						CMatrix		v(3,1);

						if( i == 0 )				// The weight for the mean value of A is w0
							weight = w0;

						v(0,0) = B[i].x - meanB.x;
						v(1,0) = B[i].y - meanB.y;
						v(2,0) = B[i].z - meanB.z;

						Pb = Pb + weight*(v*~v);
					} // end for 'i'

					// Store it in the landmark
					lm.pose_cov_11 = Pb(0,0);
					lm.pose_cov_12 = Pb(0,1);
					lm.pose_cov_13 = Pb(0,2);
					lm.pose_cov_22 = Pb(1,1);
					lm.pose_cov_23 = Pb(1,2);
					lm.pose_cov_33 = Pb(2,2);
				} // end case 'Prop_UT'
				break;

				case TStereoSystemParams::Prop_SUT:
				{
					// Parameters
					unsigned int			Na = 3;
					unsigned int			i;

					float					a = param.factor_a;
					float					b = param.factor_b;
					float					k = param.factor_k;

					float					lambda = square(a)*(Na + k) - Na;

					float					w0_m = lambda/(Na + lambda);
					float					w0_c = w0_m + (1 - square(a) + b);
					float					w1 = 1/(2*(Na + lambda));

					CMatrix					Pa(3,3);
					CMatrix					L(3,3);

					Pa.fill(0);
					Pa(0,0) = Pa(1,1) = ( Na + lambda ) * square( param.stdPixel );
					Pa(2,2) = ( Na + lambda ) * square( param.stdDisp );

					// Cholesky decomposition
					Pa.chol(L); //math::chol(Pa,L);

					vector<TPoint3D>	B;				// B group
					poses::TPoint3D		meanB;			// Mean value of the B group
					CMatrix				Pb;				// Covariance of the B group

					B.resize( 2*Na + 1 );	// Set of output values
					Pb.fill(0);				// Reset the output covariance

					vector_float		vAux, myPoint;	// Auxiliar vectors
					vector_float		meanA;			// Mean value of the A group

					vAux.resize(3);			// Set the variables size
					meanA.resize(3);
					myPoint.resize(3);

					// Mean input value: (c,r,d)
					meanA[0] = itList->first->x;
					meanA[1] = itList->first->y;
					meanA[2] = disp;

					// Output mean
					meanB.x = w0_m*x3D; meanB.y = w0_m*y3D; meanB.z = w0_m*z3D;		// Add to the mean
					B[0].x = x3D; B[0].y = y3D; B[0].z = z3D;						// Insert into B

					for( i = 1; i <= 2*Na; i++ )
					{
						// Form the Ai value
						if( i <= Na )
						{
							L.extractRowAsCol( i-1, vAux );						// Extract the proper row
							myPoint = meanA + vAux;
							//myPoint[0] = meanA[0] + vAux[0];
							//myPoint[1] = meanA[1] + vAux[1];
							//myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							L.extractRowAsCol( (i-Na)-1, vAux );					// Extract the proper row
							myPoint = meanA - vAux;
							//myPoint[0] = meanA[0] - vAux[0];
							//myPoint[1] = meanA[1] - vAux[1];
							//myPoint[2] = meanA[2] - vAux[2];
						}

						// Pass the Ai through the functions:
						x3D = ( myPoint[0] - param.K(0,2) ) * ( (param.baseline) ) / myPoint[2];
						y3D = ( myPoint[1] - param.K(1,2) ) * ( (param.baseline) ) / myPoint[2];
						z3D = ( param.K(0,0) ) * ( (param.baseline) ) / myPoint[2];

						// Add to the B mean computation and the B vector
						meanB.x = meanB.x + w1*x3D;
						meanB.y = meanB.y + w1*y3D;
						meanB.z = meanB.z + w1*z3D;

						B[i].x = x3D;
						B[i].y = y3D;
						B[i].z = z3D;

					} // end for 'i'

					// Output covariance
					for( i = 0; i <= 2*Na; i++ )
					{
						float		weight = w1;
						CMatrix		v(3,1);

						if( i == 0 )				// The weight for the mean value of A is w0
							weight = w0_c;

						v(0,0) = B[i].x - meanB.x;
						v(1,0) = B[i].y - meanB.y;
						v(2,0) = B[i].z - meanB.z;

						Pb = Pb + weight*(v*~v);
					} // end for 'i'

					// Store it in the landmark
					lm.pose_cov_11 = Pb(0,0);
					lm.pose_cov_12 = Pb(0,1);
					lm.pose_cov_13 = Pb(0,2);
					lm.pose_cov_22 = Pb(1,1);
					lm.pose_cov_23 = Pb(1,2);
					lm.pose_cov_33 = Pb(2,2);
				} // end case 'Prop_SUT'
				break;

				} // end switch
				landmarks.landmarks.push_back( lm );
				itList++;
			} // end else ( (z3D > param.minZ) && (z3D < param.maxZ) )
		} // end else
	} // end for 'i'

	MRPT_END;
} // end of projectMatchedFeatures



/*-------------------------------------------------------------
					deleteRepeatedFeats
-------------------------------------------------------------*/
void  vision::deleteRepeatedFeats( CFeatureList & feat_list )
{
	CFeatureList::iterator	itList1,itList2;
	float					lx = 0.0, ly = 0.0;

	// Look for repeated features in the feat_list of features
	for( itList1 = feat_list.begin(); itList1 != feat_list.end(); itList1++)
	{
		lx = (*itList1)->x;
		ly = (*itList1)->y;
		for( itList2 = itList1; itList2 != feat_list.end(); itList2++)
		{
			if( (lx == (*itList2)->x && ly == (*itList2)->y ) && ( (*itList2)->x > 0.0f && (*itList2)->y > 0.0f ))
			{
				(*itList2)->x = -1.0f;
				(*itList2)->y = -1.0f;
			} // end if
		} // end for
	} // end for

	// Delete the repeated features
	for( itList1 = feat_list.begin(); itList1 != feat_list.end();)
	{
		if( (*itList1)->x == -1.0f && (*itList1)->y == -1.0f )
			itList1 = feat_list.erase( itList1 );
		else
			itList1++;
	} // end for
} // end deleteRepeatedFeats


/*------------------------------------------------------------
					getDispersion
-------------------------------------------------------------*/
void vision::getDispersion( const CFeatureList &list, vector_float &std, vector_float &mean )
{
	std.assign(2,0);
	mean.assign(2,0);

	CFeatureList::const_iterator it;
	double varx = 0, vary = 0;

	for( it = list.begin(); it != list.end(); it++ )
	{
		mean[0] += (*it)->x;
		mean[1] += (*it)->y;
	}
	mean[0] /= list.size();
	mean[1] /= list.size();

	for( it = list.begin(); it != list.end(); it++ )
	{
		varx += square( (*it)->x - mean[0] );
		vary += square( (*it)->y - mean[1] );
	}
	varx /= list.size();
	vary /= list.size();

	std[0] = sqrt( varx );
	std[1] = sqrt( vary );
} // end getDispersion

/*------------------------------------------------------------
					rowChecking
-------------------------------------------------------------*/
void  vision::rowChecking( CFeatureList &leftList,
							    CFeatureList &rightList,
								float threshold )
{
	ASSERT_( leftList.size() == rightList.size() );

	// By now: row checking -> Delete bad correspondences
	std::cout << std::endl << "[ROW CHECKING]------------------------------------------" << std::endl;

	CFeatureList::iterator	itLeft, itRight;
	for( itLeft = leftList.begin(), itRight = rightList.begin(); itLeft != leftList.end(); )
	{
		if( (*itLeft)->x < 0 || (*itLeft)->y < 0 ||
			(*itRight)->x < 0 || (*itRight)->y < 0 ||
			fabs( (*itLeft)->y - (*itRight)->y ) > threshold )
		{
			std::cout << "[Erased Feature] Row Dif: " << fabs( (*itLeft)->y - (*itRight)->y ) << std::endl;
			itLeft = leftList.erase( itLeft );
			itRight = rightList.erase( itRight );
		} // end if
		else
		{
			itLeft++;
			itRight++;
		}
	} // end for

	std::cout << "------------------------------------------" << std::endl;

	std::cout << "Tracked features: " << leftList.size() << " and " << rightList.size() << std::endl;
	ASSERT_( leftList.size() == rightList.size() );

} // end rowChecking


/*-------------------------------------------------------------
						computeMsd
-------------------------------------------------------------*/
double  vision::computeMsd( const TMatchingPairList &feat_list,
								 const mrpt::poses::CPose3D &Rt )
{
	CMatrixDouble44 mat;
	Rt.getHomogeneousMatrix(mat);
	double	acum = 0.0;

	TMatchingPairList::const_iterator it;
	TPoint3D  err;
	for( it = feat_list.begin(); it != feat_list.end(); it++ )
	{
		err.x = it->other_x - (it->this_x*mat.get_unsafe(0,0) + it->this_y*mat.get_unsafe(0,1) + it->this_z*mat.get_unsafe(0,2) + Rt.x());
		err.y = it->other_y - (it->this_x*mat.get_unsafe(1,0) + it->this_y*mat.get_unsafe(1,1) + it->this_z*mat.get_unsafe(1,2) + Rt.y());
		err.z = it->other_z - (it->this_x*mat.get_unsafe(2,0) + it->this_y*mat.get_unsafe(2,1) + it->this_z*mat.get_unsafe(2,2) + Rt.z());

		acum += err.norm();

	} // end for
	return( acum/feat_list.size() );
} // end msd


/*-------------------------------------------------------------
					TROI Constructors
-------------------------------------------------------------*/
vision::TROI::TROI(): xMin(0), xMax(0), yMin(0), yMax(0), zMin(0), zMax(0)
{}

vision::TROI::TROI(float x1, float x2, float y1, float y2, float z1, float z2) : xMin(x1), xMax(x2), yMin(y1), yMax(y2), zMin(z1), zMax(z2)
{}

/*-------------------------------------------------------------
					TImageROI Constructors
-------------------------------------------------------------*/
vision::TImageROI::TImageROI():
	xMin(0), xMax(0),
	yMin(0), yMax(0)
{}

vision::TImageROI::TImageROI( float x1, float x2, float y1, float y2 ):
	xMin(x1), xMax(x2),
	yMin(y1), yMax(y2)
{}

/*-------------------------------------------------------------
					openCV_cross_correlation
-------------------------------------------------------------*/
void  vision::openCV_cross_correlation(
	const CImage	&img,
	const CImage	&patch_img,
	size_t				&x_max,
	size_t				&y_max,
	double				&max_val,
	int					x_search_ini,
	int					y_search_ini,
	int					x_search_size,
	int					y_search_size)
{
	MRPT_START;

#if MRPT_HAS_OPENCV
	double		mini;
	CvPoint		min_point,max_point;

	bool entireImg = (x_search_ini<0 || y_search_ini<0 || x_search_size<0 || y_search_size<0);

	CImage  im, patch_im;

	if( img.isColor() && patch_img.isColor() )
	{
		img.grayscale(im);
		patch_img.grayscale(patch_im);
	}
	else
	{
		ASSERT_(!img.isColor() && !patch_img.isColor())

		im.setFromIplImageReadOnly( img.getAsIplImage() );
		patch_im.setFromIplImageReadOnly( patch_img.getAsIplImage() );
	}

	const int im_w = im.getWidth();
	const int im_h = im.getHeight();
	const int patch_w = patch_im.getWidth();
	const int patch_h = patch_im.getHeight();

	if (entireImg)
	{
		x_search_size = im_w - patch_w;
		y_search_size = im_h - patch_h;
	}

	// JLBC: Perhaps is better to raise the exception always??
	if ((x_search_ini + x_search_size  + patch_w)>im_w)
		x_search_size -= (x_search_ini + x_search_size + patch_w) - im_w;

	if ((y_search_ini + y_search_size  + patch_h)>im_h)
		y_search_size -= (y_search_ini + y_search_size  + patch_h) - im_h;

	ASSERT_( (x_search_ini + x_search_size + patch_w)<=im_w )
	ASSERT_( (y_search_ini + y_search_size + patch_h)<=im_h )

	IplImage *result = cvCreateImage(cvSize(x_search_size+1,y_search_size+1),IPL_DEPTH_32F, 1);

	CImage  img_region_to_search;

	if (entireImg)
	{
		// Just a pointer to the original img:
		img_region_to_search.setFromImageReadOnly( im );
	}
	else
	{
		im.extract_patch(
			img_region_to_search,
			x_search_ini,   // start corner
			y_search_ini ,
			patch_w+x_search_size,  // sub-image size
			patch_h+y_search_size
			);
 	}

	// Compute cross correlation:
	cvMatchTemplate(
		img_region_to_search.getAsIplImage(),
		patch_im.getAsIplImage(),
		result,
		CV_TM_CCORR_NORMED
		);

	// Find the max point:
	cvMinMaxLoc(result,&mini,&max_val,&min_point,&max_point,NULL);
	x_max = max_point.x+x_search_ini+(round(patch_w-1)>>1);
	y_max = max_point.y+y_search_ini+(round(patch_h-1)>>1);

	// Free memory:
	cvReleaseImage( &result );

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
}

/********************************************************************************************/
/********************************************FLIP********************************************/
void  vision::flip(CImage		&img)
{
	MRPT_START;

#if MRPT_HAS_OPENCV
	cvFlip((IplImage*)img.getAsIplImage());	// More params exists, they could be added in the future?
	img.setOriginTopLeft(true);
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
}

/****************************************** FAMD ********************************************/

void  vision::cloudsToMatchedList(
		const mrpt::slam::CObservationVisualLandmarks &cloud1,
		const mrpt::slam::CObservationVisualLandmarks &cloud2,
		TMatchingPairList &outList)
{
	CLandmarksMap::TCustomSequenceLandmarks::const_iterator	itLand1, itLand2;
	TMatchingPair								pair;

	for(itLand1 = cloud1.landmarks.landmarks.begin(); itLand1 != cloud1.landmarks.landmarks.end(); itLand1++)
		for(itLand2 = cloud2.landmarks.landmarks.begin(); itLand2 != cloud2.landmarks.landmarks.end(); itLand2++)
			if( itLand1->ID == itLand2->ID )
			{
				// Match found!
				pair.this_idx = pair.other_idx = (unsigned int)itLand1->ID;

				pair.this_x = itLand1->pose_mean.x;
				pair.this_y = itLand1->pose_mean.y;
				pair.this_z = itLand1->pose_mean.z;

				pair.other_x = itLand2->pose_mean.x;
				pair.other_y = itLand2->pose_mean.y;
				pair.other_z = itLand2->pose_mean.z;

				outList.push_back( pair );
			} // end if
}


float vision::computeMainOrientation( const CImage &image,
										   const unsigned int &x,
										   const unsigned int &y )
{
	MRPT_START;
	float orientation=0;
	if( ( x-1 >= 0 ) && ( y-1 >= 0 ) && ( x+1 < image.getWidth() ) && ( y+1 < image.getHeight() ) )
		orientation = (float) atan2( (double)*image(x,y+1) - (double)*image(x,y-1), (double)*image(x+1,y) - (double)*image(x-1,y) );

	// Convert from [-pi,pi] to [0,2pi]

	return orientation;


	MRPT_END;
} // end vision::computeMainOrientation

// computeGradientAndOrientation.- Computes both the (approximated) gradient magnitude and orientation for a certain point within the image
// return: true = OK; false = if the keypoint is located at the image border (where the gradient cannot be computed).
bool vision::computeGradient( const CImage &image,
        const unsigned int &x, const unsigned int &y,
        double &mag, double &ori )                           // x = col, y = row
{
    if( x > 0 && x < image.getWidth()-1 && y > 0 && y < image.getHeight()-1 )
    {
        float d1, d2;
        //----------
        //| 0|-1| 0|
        //----------
        //|-1| 0| 1|
        //----------
        //| 0| 1| 0|
        //----------

        // According to Hess's implementation (Lowe does: d2 = image.getAsFloat(x,y+1) - px4 = image.getAsFloat(x,y-1); )
        d1 = image.getAsFloat(x+1,y) - image.getAsFloat(x-1,y);
        d2 = image.getAsFloat(x,y-1) - image.getAsFloat(x,y+1);

        mag = sqrt( d1*d1 + d2*d2 );
        ori  = atan2( d2, d1 );      // From -pi to pi
        return true;
    }
    else return false;
}

/*-------------------------------------------------------------
					computeMainOrientations
-------------------------------------------------------------*/
// Compute a set of main orientations (if there are more than one) of a certain keypoint within the image.
// return: 1 = OK; -1 = keypoint is too close the image margin; -2: other error
int vision::computeMainOrientations( const CImage &image,
                                 const unsigned int &x,
                                 const unsigned int &y,
                                 const unsigned int &patchSize,
                                 std::vector<double> &orientations,
                                 const double &sigma )
{
    MRPT_START

    // Pre operations
    orientations.clear();

    // Local variables:
    const unsigned int NBINS    = 36;
    const int hPatchSize        = patchSize/2;

    vector<double> oris( NBINS, 0.0 );
    int mx = (int)x, my = (int)y;

    // Check if there's no margin problems when computing the orientation
    if( mx-(hPatchSize+1) < 0 || my-(hPatchSize+1) < 0 || mx+(hPatchSize+1) > (int)image.getWidth() || my+(hPatchSize+1) > (int)image.getHeight() )
        return -1;      // Feature is too close to the image's border

    // For each pixel within the patch (patchSize should be 23x23):
    // Compute local gradients (magnitude and orientation)
    // The orientation histogram is weighted with a Gaussian with sigma = 7.5 (Cheklov's Thesis)
    double exp_denom = 2.0 * sigma *sigma;
    double bin_size = M_2PI/NBINS;
    double mag, ori;
    for( int ii = -hPatchSize; ii <= hPatchSize; ++ii )
        for( int jj = -hPatchSize; jj <= hPatchSize; ++jj )
            if( computeGradient( image, mx+ii, my+jj, mag, ori ) )
            {
                ori += M_PI;        // ori: from 0 to 2*pi
                double w = mag*exp( -( ii*ii + jj*jj ) / exp_denom );

                int bin             = ((int)(floor(ori/bin_size))) % NBINS;    // from 0 to 35
                double bin_center   = bin*bin_size;
                double dif          = ori-bin_center;
                int nxbin;
                if( dif > 0 )
                    nxbin = bin == NBINS-1 ? 0 : bin+1;             // the other involved bin is the next one
                else
                    nxbin = bin == 0 ? NBINS-1 : bin-1;             // the other involved bin is the previous one

                double nbin_center = nxbin*bin_size;
                double dif2        = ori-nbin_center;

                oris[bin]   += w*fabs(dif2)/bin_size;               // dif2 == 0 means that "ori" is equal to "bin_center"
                oris[nxbin] += w*fabs(dif)/bin_size;                // dif == 0 means that "ori" is equal to "nbin_center"
            } // end-if

    // Search for the maximum
    unsigned int   mxbin   = 0;
    double         mxori   = 0.0;
    for( unsigned int k = 0; k < oris.size(); ++k )
    {
        if( oris[k] > mxori )
        {
            mxori = oris[k];
            mxbin = k;
        }
    } // end-for

    // Compute the peaks of the histogram of orientations
    double hist_mag_th = 0.8*mxori;
    for( unsigned int k = 0; k < oris.size(); ++k )
    {
        double pv = k == 0 ? oris[oris.size()-1] : oris[k-1];
        double nv = k == oris.size()-1 ? 0 : oris[k+1];

        if( oris[k] > pv && oris[k] > nv && oris[k] > hist_mag_th )
        {
            // Check this formulae:
            // double A    = (pv-nv)/(4*oris[k]+2*nv-2*pv-4*oris[k]);
            // double peak = A/(1+2*A);

            // Interpolate the peak
            double int_bin = k + 0.5 * (pv-nv)/(pv-2.0*oris[k]+nv); // Hess formulae
			int_bin = ( int_bin < 0 ) ? NBINS + int_bin : ( int_bin >= NBINS ) ? int_bin - NBINS : int_bin;
			double int_ori = ( ( M_2PI * int_bin ) / NBINS ) - M_PI;    // Back to -pi:pi
            orientations.push_back( int_ori );
        }
    }
    return 1;

    MRPT_END
} // end vision::computeMainOrientation

/*-------------------------------------------------------------
					interpolateHistEntry
-------------------------------------------------------------*/
void vision::interpolateHistEntry(
        vector<double> &oris,
        const double &cbin, const double &rbin, const double &obin,
        const double &mag,
        const int &d,
        const int &n )
{
    CTimeLogger logger;
    logger.disable();
    // Spatial bin
    std::vector<int>    colIndex, rowIndex;
    std::vector<double> colBinDistance, rowBinDistance;

    logger.enter("Main loop");
    colIndex.reserve(2);
    colBinDistance.reserve(2);
    rowIndex.reserve(2);
    rowBinDistance.reserve(2);
    for( int bin = 0; bin < d; bin++ )
    {
        double binCenter    = bin-1.5;                          // Center of the bin
        double colDistance  = fabs( cbin - binCenter );         // Distance to the center of the bin
        if( colDistance < 1.0 )                                 // If it is close enough
        {
            colIndex.push_back( bin );
            colBinDistance.push_back( 1-colDistance );
        }

        double rowDistance = fabs( rbin - binCenter );
        if( rowDistance < 1.0 )
        {
            rowIndex.push_back( bin );
            rowBinDistance.push_back( 1-rowDistance );
        }

        if( colIndex.size() == 2 && rowIndex.size() == 2 )       // We have found all we need (stop looping)
            break;
    } // end for

    logger.leave("Main loop");
    ASSERT_( colIndex.size() <= 2 && rowIndex.size() <= 2 );

    // Orientation bin
    std::vector<int>    oriIndex(2);
    std::vector<double> oriBinDistance(2);
    oriIndex[0]         = floor( obin );
    oriIndex[1]         = oriIndex[0] == n-1 ? 0 : oriIndex[0]+1;
    oriBinDistance[0]   = 1 - (obin-oriIndex[0]);
    oriBinDistance[1]   = 1 - oriBinDistance[0];

    // The entry can affect to 2 (1x1x2), 4 (2x1x2 or 1x2x2) or 8 (2x2x2) bins
    // Insert into the histogram
    logger.enter("Insertion in histogram");
    for( unsigned int k = 0; k < colIndex.size(); ++k )
        for( unsigned int l = 0; l < rowIndex.size(); ++l )
            for( unsigned int m = 0; m < 2; ++m )
            {
                if( colIndex[k] >= 0 && rowIndex[l] >= 0 )
                {
                    unsigned int idx = (unsigned int)(n*colIndex[k] + rowIndex[l]*n*d + oriIndex[m]);
                    oris[idx] += mag*colBinDistance[k]*rowBinDistance[l]*oriBinDistance[m];
                } // end if
            } // end for
    logger.leave("Insertion in histogram");
} // end of interpolateHistEntry

/*-------------------------------------------------------------
					computeHistogramOfOrientations
-------------------------------------------------------------*/
void vision::computeHistogramOfOrientations(
            const CImage &image,                            // the image
            const unsigned int &x, const unsigned int &y,   // the position of the keypoint (x = col, y = row )
            const unsigned int &patchSize,                  // the size of the patch
            const double &orientation,                      // the orientation that must be applied to the patch
            vector<int> &descriptor,                        // the OUTPUT descriptor
            const TMultiResDescOptions &opts )
{
    CTimeLogger tlogger;
    tlogger.disable();
    MRPT_START

    // The patch is 23x23
    int     w               = 16;   // width of the used patch
    int     Bp              = 4;    // Number of spatial bins
    int     n               = 8;    // Number of frequential bins (per spatial bin) --> Descriptor size: 4 x 4 x 8 = 128
    double  cos_t           = cos( orientation );
	double  sin_t           = sin( orientation );
	double  bins_per_rad    = n / M_2PI;
	double  exp_denom       = opts.sg3*opts.sg3*2;
	int     radius          = patchSize/2;
	vector<double> oris( 128, 0.0 );

//     For each pixel in the 23x23 patch:
//     a. find its rotated position
//     b. determine its proper spatial bin (and the other affected one)
//     c. rotate its orientation with respect the main orientation
//     d. determine its frequential bin
//     e. compute the weight
    const double kp = ((double)(Bp+1))/((double)w); // [0.31250] Constant for mapping between -8,8 -> -2.5/2.5
    double mag, ori;
    double cbin, rbin, obin;
    for( int c = -radius; c <= radius; ++c )
        for( int r = -radius; r <= radius; ++r )
        {
            cbin = kp*c*cos_t - kp*r*sin_t;
            rbin = kp*c*sin_t + kp*r*cos_t;

            if( cbin > -2.5 && cbin < 2.5 && rbin > -2.5 && rbin < 2.5 )
            {
                tlogger.enter("computeGradient");
                bool res = vision::computeGradient( image, x+c, y+r, mag, ori );
                tlogger.leave("computeGradient");
                if( res )
				{
				    ori -= orientation;
					while( ori < 0.0 )
						ori += M_2PI;
					while( ori >= M_2PI )
						ori -= M_2PI;

					obin = ori*bins_per_rad;
					w = exp( -(c*c + r*r) / exp_denom );
					tlogger.enter("interpolate");
                    vision::interpolateHistEntry( oris, cbin, rbin, obin, mag, Bp, n );
                    tlogger.leave("interpolate");
				} // end if
            } // end
        } // end-for

    // Normalize
    tlogger.enter("normalize");
    double sum = 0.0;
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        sum += oris[ii]*oris[ii];
    sum = 1/sqrt(sum);
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        oris[ii] *= sum;

    // Crop to "crop_value"
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        oris[ii] = min(opts.cropValue,oris[ii]);

    // Normalize again -> we have the descriptor!
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        sum += oris[ii]*oris[ii];
    sum = 1/sqrt(sum);
    for( unsigned int ii = 0; ii < oris.size(); ++ii )
        oris[ii] *= sum;

    // Convert it to std::vector<int>
    descriptor.resize( oris.size() );
    for( unsigned int ii = 0; ii < descriptor.size(); ++ii )
        descriptor[ii] = (int)(255.0f*oris[ii]);

    tlogger.leave("normalize");

    MRPT_END
} // end vision::computeHistogramOfOrientations

/*-------------------------------------------------------------
					matchMultiResolutionFeatures
-------------------------------------------------------------*/
int vision::matchMultiResolutionFeatures(
        const CFeatureList              & list1,
        const CFeatureList              & list2,
        const CImage                    & rightImage,
        vector<int>                     & leftMatchingIdx,
        vector<int>                     & rightMatchingIdx,
        vector<int>                     & outScales,
        const TMultiResDescMatchOptions & matchOpts,
        const TMultiResDescOptions      & computeOpts )
{
    MRPT_START
    // "List1" has a set of features with their depths computed
    // "List2" has a set of FAST features detected in the next frame (with their depths)
    // We perform a prediction of the "List1" features and try to find its matches within List2
    // --------------------------------------------------------
    // Algortihm summary:
    // --------------------------------------------------------
    // For each feature in List1 we find a search region: by now a fixed window e.g. 20x20 pixels
    // TO DO: Non-maximal supression according to the feature score
    // From the remaining set, we compute the main orientation and check its consistency with the List1 feature
    // NEW: compute the corresponding feature in right image and compute its depth
    // From the remaining set, we compute the descriptor at scale 1.0 and determine the set of scales where to look for
    // If the distance between descriptors is below a certain threshold, we've found a match.
    // --------------------------------------------------------

    // General variables
    CTimeLogger logger;
    logger.disable();

    // Preliminary tasks
    leftMatchingIdx.resize(list1.size(),FEAT_FREE);
    rightMatchingIdx.resize(list2.size(),FEAT_FREE);
    outScales.resize(list1.size(),-1);
    vector<double> dist_corrs(list1.size(),1.0);

    // Local variables
    int hWindow = matchOpts.searchAreaSize/2;   // Half of the search window width
    int imageW = rightImage.getWidth();         // Image width
    int imageH = rightImage.getHeight();        // Image height
    int leftFeatCounter = 0;
    int rightFeatCounter = 0;                   // Counter for features
    vector<double> orientations;
    int patchSize = computeOpts.basePSize;
    int minDist, minIdx, minScale;
    double maxResponse;
    int nMatches = 0;

    CFeatureList::const_iterator it1, it2;
//    cout << "Starting the loop" << endl;
    for( leftFeatCounter = 0, it1 = list1.begin(); it1 != list1.end(); ++it1, ++leftFeatCounter )
    {
//        cout << leftFeatCounter << endl;
        if( !(*it1)->descriptors.hasDescriptorMultiSIFT() )
            continue;

        double sRegionX0 = max( (*it1)->x-hWindow, 0.0f);
        double sRegionX1 = min( (*it1)->x+hWindow, (float)imageW );
        double sRegionY0 = max( (*it1)->y-hWindow, 0.0f);
        double sRegionY1 = min( (*it1)->y+hWindow, (float)imageH );

        minDist = 1e6;
        minIdx  = -1;
        minScale = -1;
        maxResponse = 0;
        int ridx = 0;

        for( rightFeatCounter = 0, it2 = list2.begin(); it2 != list2.end(); ++it2, ++rightFeatCounter )
        {
            // FILTER 1: Search region
            if( (*it2)->x < sRegionX0 || (*it2)->x > sRegionX1 || (*it2)->y < sRegionY0 || (*it2)->y > sRegionY1 )
                continue;

            // Compute main orientations
//            logger.enter("cmorientation");
            vision::computeMainOrientations( rightImage, (*it2)->x, (*it2)->y, patchSize, orientations, computeOpts.sg2 );
//            logger.leave("cmorientation");

            // Compute the proper scales where to look for
            int firstScale = max( 0, (int)matchOpts.lowScl1 );
            int lastScale = min( (int)(*it1)->multiScales.size()-1, (int)matchOpts.highScl1 );
            if( matchOpts.useDepthFilter )
                vision::setProperScales( (*it1), (*it2), firstScale, lastScale );

            // Search for consistency of the orientation
            for( int k1 = firstScale; k1 <= lastScale; ++k1 )
                for( int k2 = 0; k2 < (int)(*it1)->multiOrientations[k1].size(); ++k2 )
                    for( int k3 = 0; k3 < (int)orientations.size(); ++k3 )  // FILTER 2: Orientation
                        if( fabs( (*it1)->multiOrientations[k1][k2] - orientations[k3] ) < matchOpts.oriThreshold ||
                            fabs( M_2PI - fabs( (*it1)->multiOrientations[k1][k2] - orientations[k3] ) ) < matchOpts.oriThreshold )
                        {
                            // Orientation check passed
                            // FILTER 3: Feature response
                            // has it a better score than its 8 neighbours?
//                            logger.enter("non-max");
                            std::vector< int > out_idx;
                            std::vector< float > out_dist_sqr;
                            maxResponse = (*it2)->response;
                            bool isMax = true;
                            list2.kdTreeNClosestPoint2DIdx(	(*it2)->x, (*it2)->y, 8, out_idx, out_dist_sqr );
                            for( int kdcounter = 0; kdcounter < (int)out_idx.size(); ++kdcounter )
                            {
                                if( out_dist_sqr[kdcounter] > 1.4142 )
                                    continue;

                                if( list2[out_idx[kdcounter]]->response > maxResponse )
                                {
                                    maxResponse = list2[out_idx[kdcounter]]->response;
                                    isMax = false;
                                    break;
                                }
                            }
                            logger.leave("non-max");

                            if( !isMax )
                                continue;

                            // Candidate found at scale "k1" and orientation "k2" (list1) and orientation "k3" (list2)
                            // Compute descriptor
                            vector<int> desc;
//                            logger.enter("chorientations");
                            vision::computeHistogramOfOrientations( rightImage, (*it2)->x, (*it2)->y, patchSize, orientations[k3], desc, computeOpts );
//                            logger.leave("chorientations");

                            // FILTER 4: Descriptor distance
                            int dist = 0;
                            for( int n = 0; n < (int)(*it1)->descriptors.multiSIFTDescriptors[k1][k2].size(); n++ )
                                dist += square( (*it1)->descriptors.multiSIFTDescriptors[k1][k2][n] - desc[n] );

                            if( dist < minDist )
                            {
                                minDist     = dist;
                                ridx        = rightFeatCounter;
                                maxResponse = (*it2)->response;
                                minScale    = k1;

//                                minauxfscale = auxfscale;
//                                minauxlscale = auxlscale;

//                                mindepth1    = (*it1)->depth;
//                                mindepth2    = (*it2)->depth;
                            }
                        } // end if
        } // end for it2
        if( minDist < matchOpts.matchingThreshold )
        {
            // AVOID COLLISIONS IN A GOOD MANNER
            int auxIdx = rightMatchingIdx[ ridx ];
		    if( auxIdx != FEAT_FREE && minDist < dist_corrs[ auxIdx ] )
		    {
                // We've found a better match
                dist_corrs[ leftFeatCounter ]       = minDist;
                leftMatchingIdx[ leftFeatCounter ]  = ridx;
                rightMatchingIdx[ ridx ]            = leftFeatCounter;
                outScales[ leftFeatCounter ]        = minScale;
                dist_corrs[ auxIdx ]                = 1.0;
                leftMatchingIdx[ auxIdx ]           = FEAT_FREE;
//                cout << "Better match at scale: " << minScale << endl;
		    } // end-if
		    else
		    {
////		        cout << "New match found: [R] " << minRightIdx << " with [L] " << minLeftIdx << "(" << minVal << ")" << endl;
		        rightMatchingIdx[ ridx ]            = leftFeatCounter;
		        leftMatchingIdx[ leftFeatCounter ]  = ridx;
		        dist_corrs[ leftFeatCounter ]       = minDist;
		        outScales[ leftFeatCounter ]        = minScale;
		        nMatches++;
//		        cout << "Match found at scale: " << minScale << endl;
		    }
            // Match found
//            leftMatchingIdx.push_back( leftFeatCounter );
//            rightMatchingIdx.push_back( minIdx );
//            outScales.push_back( minScale );
//
//            cout << "Match found: [" << leftFeatCounter << "," << minIdx;
//            cout << "] at scale " << minScale << " [" << minauxfscale << "," << minauxlscale << "]";
//            cout << " with depths: [" << mindepth1 << "," << mindepth2 << "]" << endl;
        } // end if
    } // end for it1
    return nMatches;
    MRPT_END
} // end matchMultiResolutionFeatures

/*-------------------------------------------------------------
					matchMultiResolutionFeatures
-------------------------------------------------------------*/
int vision::matchMultiResolutionFeatures(
        const CFeatureList                          & list1,
        const CFeatureList                          & list2,
        vector<int>                                 & idx_right_corrs,
        vector<double>                              & dist_corrs,
        vector<double>                              & lscl_corrs,
        vector<double>                              & rscl_corrs,
        vector<double>                              & lori_corrs,
        vector<double>                              & rori_corrs,
        const TMultiResDescMatchOptions             & opts )
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211
    CTimeLogger logger;
    //logger.disable();
    // *************************************************************
    // Important notes and algorithm:
    // *************************************************************
    // List1 is taken as a data base of keypoints with their descriptors at a set of different scales
    // (default: 7 scales at resolutions: 0.5,0.8,1.0,1.2,1.5,1.8,2.0)
    // List2 is taken as a list of query keypoints with only one scale (1.0)
    // Matching is performed as follows:
    // 1. for each query keypoint in list2 compute the scales from list1 where to lookfor
    //      [useDepthFilter + depths]
    // 2. for each of the selected scales, check if orientation is consistent
    //      [useOriFilter + oriThreshold]
    // 3. for each of the consistent orientations, compute the SSD between the descriptors
    // 4. if the distance is compliant with the specified parameters -> match is found
    // 5. check if keypoint in list1 hasn't been already assigned to any other keypoint in list2.
    // 6. fill output vectors
    // *************************************************************
    size_t list_size1 = list1.size();
    size_t list_size2 = list2.size();

    ASSERT_( list_size1 > 0 && list_size2 > 0 );

    // prepare the output vectors
    int nMatchesFound = 0;

    vector<int> idx_left_corrs( list_size1, FEAT_FREE );

    idx_right_corrs.clear();
    dist_corrs.clear();
    lscl_corrs.clear();
    rscl_corrs.clear();
    lori_corrs.clear();
    rori_corrs.clear();

    idx_right_corrs.resize( list_size2, FEAT_FREE );
    dist_corrs.resize( list_size2 );
    lscl_corrs.resize( list_size1 );
    rscl_corrs.resize( list_size2 );
    lori_corrs.resize( list_size1 );
    rori_corrs.resize( list_size2 );

    // Dump matching options
    cout << "Multi matching options: " << endl;
    cout << " · Threshold: " << opts.matchingThreshold << endl;
    cout << " · Use depth filter?: ";
    if( opts.useDepthFilter ) cout << "yes";
    else cout << "no";
    cout << endl;

    cout << " · Use orientation filter?: ";
    if( opts.useOriFilter ) cout << "yes - th: " << opts.oriThreshold;
    else cout << "no";
    cout << endl;
    cout << "--------------------------------------------------------------------" << endl;

    FILE *f;
    char buf[50];


    CFeatureList::const_iterator  it1, it2;
    unsigned int                  lFeat, rFeat = 0;
    for( it2 = list2.begin(); it2 != list2.end(); ++it2, ++rFeat )
    {
        mrpt::system::os::sprintf( buf, 50, "imgs/dist_%d.txt", rFeat );
        f = mrpt::system::os::fopen( buf, "wt" );

        double min_dist = 1e6, min_dist2 = 1e6;
        double lscl = 0.0, rscl = 0.0, lori = 0.0, rori = 0.0;
        unsigned int lidx = 0, ridx = 0;
        lFeat = 0;
        for( it1 = list1.begin(); it1 != list1.end(); ++it1, ++lFeat )
        {
            // STEP 1
            // Loop regarding the scale will be done from 'firstScale' to 'lastScale' (both included)
            int firstScale = max( 0, (int)opts.lowScl1 );
            int lastScale = min( (int)(*it1)->multiScales.size()-1, (int)opts.highScl1 );
            if( opts.useDepthFilter )
                vision::setProperScales( (*it1), (*it2), firstScale, lastScale );

            for( int ii = 0; ii < (int)(*it2)->multiOrientations[0].size(); ++ii )  // Orientations of scale 0
            {
                for( int jj = firstScale; jj <= lastScale; ++jj )
                {
                    for( int kk = 0; kk < (int)(*it1)->multiOrientations[jj].size(); ++kk )
                    {
                        // STEP 2
                        if( opts.useOriFilter && fabs( (*it1)->multiOrientations[jj][kk] - (*it2)->multiOrientations[0][ii] ) > opts.oriThreshold &&
                            fabs( M_2PI - fabs( (*it1)->multiOrientations[jj][kk] - (*it2)->multiOrientations[0][ii] ) ) > opts.oriThreshold )
                                continue;

                        // STEP 3
                        // We have passed the depth and orientation filters, this is a potential match!
                        int dist;
                        dist = 0;
                        for( unsigned int n = 0; n < 128; n++ )
                            dist += square( (*it1)->descriptors.multiSIFTDescriptors[jj][kk][n] - (*it2)->descriptors.multiSIFTDescriptors[0][ii][n] );

                        //logger.enter("saveToFile");
//                        mrpt::system::os::fprintf( f, "%d 1.0 %.3f %d %.1f %.3f %d %.4f %.4f\n",
//                            rFeat, (*it2)->multiOrientations[0][ii],
//                            lFeat, (*it1)->multiScales[jj], (*it1)->multiOrientations[jj][kk],
//                            dist,
//                            (*it1)->depth, (*it2)->depth );
                        //logger.leave("saveToFile");
                        if( dist < min_dist )
                        {
                            min_dist    = dist;                         // distances

                            lidx        = lFeat;                        // idx
                            ridx        = rFeat;

                            lscl        = (*it1)->multiScales[jj];              // scales
                            rscl        = 1.0;

                            lori        = (*it1)->multiOrientations[jj][kk];    // orientations
                            rori        = (*it2)->multiOrientations[0][ii];
                        } // end-if
                        else if( dist < min_dist2 && lidx != lFeat )
                            min_dist2 = dist;
                    } // end-for-kk
                } // end-for-jj
            } // end-for-ii
        } // end-for-it1
        // STEP 4
        if( min_dist < opts.matchingThreshold && min_dist/min_dist2 < opts.matchingRatioThreshold )
        {
            // AVOID COLLISIONS IN A GOOD MANNER
            int right_idx = idx_left_corrs[ lidx ];
            if( right_idx != FEAT_FREE )                                    // if it is not free
            {
                if( dist_corrs[ right_idx ] > min_dist )                    // is this one better than the previous one?
                {
                    dist_corrs[ right_idx ]         = min_dist;         // update distance, scales, and orientations
                    idx_right_corrs[ right_idx ]    = FEAT_FREE;

                    idx_right_corrs[ ridx ] = lidx;
                    idx_left_corrs[ lidx ]  = ridx;
                    lscl_corrs[ lidx ]      = lscl;
                    rscl_corrs[ ridx ]      = rscl;

                    lori_corrs[ lidx ]      = lori;
                    rori_corrs[ ridx ]      = rori;
                } // end-if dist_corrs
            } // end-FEAT_FREE
            else
            {
                dist_corrs[ ridx ]      = min_dist;

                idx_right_corrs[ ridx ] = lidx;
                idx_left_corrs[ lidx ]  = ridx;
                lscl_corrs[ lidx ]      = lscl;
                rscl_corrs[ ridx ]      = rscl;

                lori_corrs[ lidx ]      = lori;
                rori_corrs[ ridx ]      = rori;
            }
            ++nMatchesFound;
        } // end-if-a match has been found
        // TO DO: STEP 5 OF THE ALGORITHM
    } // end-for-it2

    return nMatchesFound;
#else
	THROW_EXCEPTION("This function requires OpenCV 2.1+")
#endif
} // end-vision::matchMultiResolutionFeatures

/*-------------------------------------------------------------
					matchMultiResolutionFeatures
-------------------------------------------------------------*/
int  vision::matchMultiResolutionFeatures(
        CMatchedFeatureList             & mList1,
        CMatchedFeatureList             & mList2,
        const CImage                    & leftImage,
        const CImage                    & rightImage,
        const TMultiResDescMatchOptions & matchOpts,
        const TMultiResDescOptions      & computeOpts )
{
    // "mList1" has a set of matched features with their depths computed
    // "mList2" has a set of matched FAST features detected in the next frame at scale 1.0 (with their depths)
    // We perform a prediction of the "mList1" features and try to find its matches within mList2
    // --------------------------------------------------------
    // Algortihm summary:
    // --------------------------------------------------------
    // For each feature in List1 we find a search region: by now a fixed window e.g. 20x20 pixels
    // TO DO: Non-maximal supression according to the feature score
    // From the remaining set, we compute the main orientation and check its consistency with the List1 feature
    // NEW: compute the corresponding feature in right image and compute its depth
    // From the remaining set, we compute the descriptor at scale 1.0 and determine the set of scales where to look for
    // If the distance between descriptors is below a certain threshold, we've found a match.
    // --------------------------------------------------------

    // General variables
    CTimeLogger logger;
    //logger.disable();

    const int LAST_SEEN_TH = 5;

    ASSERT_( mList1.size() > 0 && mList2.size() > 0 );

    CFeatureList baseList1, baseList2;
    mList1.getBothFeatureLists( baseList1, baseList2 );

    CFeatureList auxList1, auxList2;
    mList2.getBothFeatureLists( auxList1, auxList2 );

    vector<int> leftIdx1, leftIdx2, rightIdx1, rightIdx2;
    vector<int> scales1, scales2;
    // Left image
    //int nM1 =
    matchMultiResolutionFeatures( baseList1, auxList1, leftImage, leftIdx1, leftIdx2, scales1, matchOpts, computeOpts );
    _updateCounter(leftIdx1,baseList1);     //Update counters
    _updateCounter(leftIdx2,auxList1);

    // Right image
    //int nM2 =
    matchMultiResolutionFeatures( baseList2, auxList2, rightImage, rightIdx1, rightIdx2, scales2, matchOpts, computeOpts );
    _updateCounter(rightIdx1,baseList2);    //Update counters
    _updateCounter(rightIdx2,auxList2);

//    cout << "Left matches: " << nM1 << " out of " << baseList1.size() << "," << auxList1.size() << endl;
//    cout << "Right matches: " << nM2 << " out of " << baseList2.size() << "," << auxList2.size() << endl;

    int nScales = baseList1[0]->multiScales.size()-1;

    CMatchedFeatureList::iterator itMatch;
    int m;
    for( m = 0, itMatch = mList1.begin(); itMatch != mList1.end(); ++m )
    {
        if( itMatch->first->nTimesLastSeen > LAST_SEEN_TH || itMatch->second->nTimesLastSeen > LAST_SEEN_TH )
            itMatch = mList1.erase( itMatch );
        else
        {
            // We've found a tracked match
            // We have found the match in both frames! Check out the scales!
            if( scales1[m] == 0 )
            {
                cout << "Left feature " << m << " found in scale 0!" << endl;
                //int res =
                computeMoreDescriptors( (*auxList1[m]), leftImage, (*itMatch->first), true, computeOpts );
            }
            else
            {
                if( scales1[m] == nScales )
                {
                    cout << "Left feature found in scale " << nScales << "!" << endl; //computeMoreDescriptors( auxList1[k1], leftImage, false, itMatch->first );
                    //int res =
                    computeMoreDescriptors( (*auxList1[m]), leftImage, (*itMatch->first), false, computeOpts );
                }
            }
            itMatch->first->dumpToConsole();
            mrpt::system::pause();
            if( scales2[m] == 0 )
                cout << "Right feature " << m << " found in scale 0!" << endl; //computeMoreDescriptors( auxList2[k2], rightImage, true, itMatch->second );
            else
            {
                if( scales2[m] == nScales )
                    cout << "Right feature found in scale " << nScales << "!" << endl; //computeMoreDescriptors( auxList2[k2], rightImage, false, itMatch->second );
            }
            ++itMatch;
        } // end-else
    } // end-for
    return mList1.size();
} // end matchMultiResolutionFeatures

/*-------------------------------------------------------------
					computeMoreDescriptors
-------------------------------------------------------------*/
int vision::computeMoreDescriptors(
                const CFeature              & inputFeat,
                const CImage                & image,
                CFeature                    & outputFeat,
                const bool                  & lowerScales,
                const TMultiResDescOptions  & opts )
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211

    MRPT_START
    //**************************************************************************
    // Pre-smooth the image with sigma = sg1 (typically 0.5)
    //**************************************************************************
    cv::Mat tempImg1;
    IplImage aux1;

    cv::Mat inImg1 = static_cast<IplImage*>(image.getAsIplImage());

    cv::GaussianBlur( inImg1, tempImg1, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
    aux1 = tempImg1;
    CImage smLeftImg( &aux1 );
    //--------------------------------------------------------------------------

    unsigned int a = opts.basePSize;

    int largestSize = round(a*opts.scales[opts.scales.size()-1]);
    largestSize = largestSize%2 ? largestSize : largestSize+1;          // round to the closest odd number
    int hLargestSize = largestSize/2;

    unsigned int npSize;
    unsigned int hpSize;

    // We don't take into account the matching if it is too close to the image border (the largest patch cannot be extracted)
    if( inputFeat.x+hLargestSize > smLeftImg.getWidth()-1 || inputFeat.x-hLargestSize < 0 ||
        inputFeat.y+hLargestSize > smLeftImg.getHeight()-1 || inputFeat.y-hLargestSize < 0 )
            return 0;

    int iniScale = 0, endScale = 0;

    cout << "Smooth done" << endl;

    if( lowerScales )
    {
        cout << "Lower scales" << endl;
        outputFeat.multiScales.push_front( 0.8*outputFeat.multiScales[0] );
        outputFeat.multiScales.push_front( 0.5*outputFeat.multiScales[0] );

        iniScale = 0;
        endScale = 2;

        for( int k = 1; k >= 0; --k )
        {
            npSize = round( a*outputFeat.multiScales[k] );
            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
            hpSize = npSize/2;                              // half size of the patch

            cout << "For scale " << k << endl;

            CImage tPatch;

            // LEFT IMAGE:
            smLeftImg.extract_patch( tPatch, inputFeat.x-hpSize, inputFeat.y-hpSize, npSize, npSize );

            cv::Mat out_mat_patch;
            // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
            // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
            cv::resize( cv::Mat( static_cast<IplImage*>(tPatch.getAsIplImage()) ), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img = IplImage(out_mat_patch);
            CImage rsPatch( &aux_img );

            cout << "Patch extracted and resized" << endl;

            vector<double> auxOriVector;
            vision::computeMainOrientations(
                            rsPatch, a/2+1, a/2+1, a,
                            auxOriVector, opts.sg2 );

            cout << "Orientation computed" << endl;

            vector< vector<int> > auxDescVector;
            auxDescVector.resize( auxOriVector.size() );
            for( unsigned int m = 0; m < auxOriVector.size(); ++m )
            {
                cout << "Descriptor for orientation " << auxOriVector[m];
                computeHistogramOfOrientations(
                            rsPatch,
                            a/2+1, a/2+1, a,
                            auxOriVector[m],
                            auxDescVector[m], opts );
                cout << " ...done" << endl;
            } // end-for
            outputFeat.multiOrientations.push_front( auxOriVector );
            outputFeat.descriptors.multiSIFTDescriptors.push_front( auxDescVector );
        } // end-for
    }
    else
    {
        size_t nCurrScales = outputFeat.multiScales.size();
        outputFeat.multiScales.push_back( 1.2*outputFeat.multiScales[nCurrScales-1] );
        outputFeat.multiScales.push_back( 1.5*outputFeat.multiScales[nCurrScales-1] );
        outputFeat.multiScales.push_back( 1.8*outputFeat.multiScales[nCurrScales-1] );
        outputFeat.multiScales.push_back( 2.0*outputFeat.multiScales[nCurrScales-1] );

        for( int k = nCurrScales; k < (int)outputFeat.multiScales.size(); ++k )
        {
            npSize = round( a*outputFeat.multiScales[k] );
            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
            hpSize = npSize/2;                              // half size of the patch

            CImage tPatch;

            // LEFT IMAGE:
            smLeftImg.extract_patch( tPatch, inputFeat.x-hpSize, inputFeat.y-hpSize, npSize, npSize );

            cv::Mat out_mat_patch;
            // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
            // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
            cv::resize( cv::Mat( static_cast<IplImage*>(tPatch.getAsIplImage()) ), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img = IplImage(out_mat_patch);
            CImage rsPatch( &aux_img );

            vector<double> auxOriVector;
            vision::computeMainOrientations(
                            rsPatch, a/2+1, a/2+1, a,
                            auxOriVector, opts.sg2 );

            vector< vector<int> > auxDescVector;
            auxDescVector.resize( auxOriVector.size() );
            for( unsigned int m = 0; m < auxOriVector.size(); ++m )
            {
                computeHistogramOfOrientations(
                            rsPatch,
                            a/2+1, a/2+1, a,
                            auxOriVector[m],
                            auxDescVector[m], opts );
            } // end-for
            outputFeat.multiOrientations.push_back( auxOriVector );
            outputFeat.descriptors.multiSIFTDescriptors.push_back( auxDescVector );
        } // end-for
    } // end else
    return 1;
    MRPT_END
#else
	THROW_EXCEPTION("This function needs OpenCV 2.1+")
	return 0;
#endif
} // end-computeMoreDescriptors

/*-------------------------------------------------------------
					setProperScales
-------------------------------------------------------------*/
void vision::setProperScales( const CFeaturePtr &feat1, const CFeaturePtr &feat2, int &firstScale, int &lastScale )
{
    size_t numScales    = feat1->multiScales.size();
    ASSERT_( numScales > 1 );

    firstScale = 0;
    lastScale  = numScales-1;

    // Determine the range of scale where to look for in the list1
    double smin = (feat2->depth-0.2*feat1->depth)/feat1->depth;
    double smax = (feat2->depth+0.2*feat1->depth)/feat1->depth;

    if( smin < feat1->multiScales[1] )
        firstScale = 0;
    else
    {
        if( smin > feat1->multiScales[numScales-3] )
            firstScale = numScales-2;
        else    // it is in between the limits
        {
            for( int k = 1; k <= (int)numScales-3; ++k )
                if( smin >= feat1->multiScales[k] )
                {
                    firstScale = k;
                    break;
                } // end if
        } // end else
    } // end else

    if( smax < feat1->multiScales[1] )
        lastScale = 1;
    else
    {
        if( smax > feat1->multiScales[numScales-2] )
            lastScale = numScales-1;
        else
        {
            for( int k = 1; k <= (int)numScales-2; ++k )
            {
                if( smax <= feat1->multiScales[k] )
                {
                    lastScale = k;
                    break;
                } // end if
            }
        } // end else
    } // end else
    ASSERT_( firstScale >= 0 && lastScale < (int)numScales && firstScale < lastScale );
} // end setProperScales

/*-------------------------------------------------------------
					computeMultiResolutionDescriptors
-------------------------------------------------------------*/
//void vision::computeMultiResolutionDescriptors(
//        const CImage &image,
//        const CFeatureList &featList,
//        vector<TMultiDescFeature> &multiFeatsList,
//        const TMultiResDescOptions &opts )
//{
//    MRPT_START
//    CTimeLogger tlogger;
//    tlogger.disable();
//    ASSERT_( featList.size() > 0 );
//
//    //**************************************************************************
//    // Pre-smooth the image with sigma = sg1
//    //**************************************************************************
//    tlogger.enter("smooth");
//    cv::Mat tempImg;
//    IplImage aux;
//
//    cv::Mat inImg = static_cast<IplImage*>(image.getAsIplImage());
//    cv::GaussianBlur( inImg, tempImg, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
//    aux = tempImg;
//    CImage smImg( &aux );
//    tlogger.leave("smooth");
//    //--------------------------------------------------------------------------
//
//    unsigned int a = opts.basePSize;
//
//    multiFeatsList.clear();
//    multiFeatsList.reserve( featList.size() );
//
//    unsigned int feat_counter = 0;
//    unsigned int good_matches = 0;
//
//    int largestSize = round(a*opts.scales[opts.scales.size()-1]);
//    largestSize = largestSize%2 ? largestSize : largestSize+1;          // round to the closest odd number
//    int hLargestSize = largestSize/2;
//
//    unsigned int npSize;
//    unsigned int hpSize;
//
//    for( CFeatureList::const_iterator itList = featList.begin(); itList != featList.end(); ++itList, ++feat_counter )
//    {
//        // We don't take into account the matching if it is too close to the image border (the largest patch cannot be extracted)
//        if( (*itList)->x+hLargestSize > smImg.getWidth()-1 || (*itList)->x-hLargestSize < 0 ||
//            (*itList)->y+hLargestSize > smImg.getHeight()-1 || (*itList)->y-hLargestSize < 0 )
//            continue;
//
//        // We don't know the depth
//        TMultiDescFeature multiFeat;
//        multiFeat.x     = (*itList)->x;
//        multiFeat.y     = (*itList)->y;
//        multiFeat.depth = 0.0;
//
//        // For each of the involved scales
//        for( unsigned int k = opts.comLScl; k < opts.comHScl; ++k )
//        {
//            npSize = round(a*opts.scales[k]);
//            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
//            hpSize = npSize/2;                              // half size of the patch
//
//            CImage tPatch;
//
//            // LEFT IMAGE:
//            tlogger.enter("extract & resize");
//            smImg.extract_patch( tPatch, (*itList)->x-hpSize, (*itList)->y-hpSize, npSize, npSize );
//
//            cv::Mat out_mat_patch;
//            cv::resize( cv::Mat( static_cast<IplImage*>(tPatch.getAsIplImage()) ), out_mat_patch, cv::Size(a+2,a+2) );
//            IplImage aux_img = IplImage(out_mat_patch);
//            CImage rsPatch( &aux_img );
//            tlogger.leave("extract & resize");
//
//            // *****************************************************
//            // f(patch) = main orientations + descriptors
//            // *****************************************************
//            tlogger.enter("main orientations");
//            vision::computeMainOrientations(
//                            rsPatch, a/2+1, a/2+1, a,
//                            multiFeat.orientations[k], opts.sg2 );
//            tlogger.leave("main orientations");
//
//            size_t nMainOris = multiFeat.orientations[k].size();
//            multiFeat.descriptors[k].resize( nMainOris );
//            for( unsigned int m = 0; m < nMainOris; ++m )
//            {
//                tlogger.enter("compute histogram");
//                computeHistogramOfOrientations(
//                            rsPatch,
//                            a/2+1, a/2+1, a,
//                            multiFeat.orientations[k][m],
//                            multiFeat.descriptors[k][m] );
//                tlogger.leave("compute histogram");
//            } // end for
//            // *****************************************************
//            mrpt::system::pause();
//
//        } // end scales for
//        multiFeatsList.push_back( multiFeat );
//        good_matches++;
//    } // end matches for
//    MRPT_END
//} // end-vision::computeMultiResolutionDescriptors

/*-------------------------------------------------------------
					computeMultiResolutionDescriptors
-------------------------------------------------------------*/
void vision::computeMultiResolutionDescriptors(
        const CImage &imageLeft, const CImage &imageRight,
        CMatchedFeatureList &matchedFeats,
        const TMultiResDescOptions &opts )
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211

    MRPT_START
    CTimeLogger tlogger;
    tlogger.disable();
    ASSERT_( matchedFeats.size() > 0 );

    //**************************************************************************
    // Pre-smooth the image with sigma = sg1 (typically 0.5)
    //**************************************************************************
    tlogger.enter("smooth");
    cv::Mat tempImg1, tempImg2;
    IplImage aux1, aux2;

    cv::Mat inImg1 = static_cast<IplImage*>(imageLeft.getAsIplImage());
    cv::Mat inImg2 = static_cast<IplImage*>(imageRight.getAsIplImage());

    cv::GaussianBlur( inImg1, tempImg1, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
    aux1 = tempImg1;
    CImage smLeftImg( &aux1 );

    cv::GaussianBlur( inImg2, tempImg2, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
    aux2 = tempImg2;
    CImage smRightImg( &aux2 );
    tlogger.leave("smooth");
    //--------------------------------------------------------------------------

    unsigned int a              = opts.basePSize;
    unsigned int feat_counter   = 0;
    unsigned int good_matches   = 0;

    int largestSize = round(a*opts.scales[opts.scales.size()-1]);
    largestSize = largestSize%2 ? largestSize : largestSize+1;          // round to the closest odd number
    int hLargestSize = largestSize/2;

    unsigned int npSize;
    unsigned int hpSize;

    for( CMatchedFeatureList::iterator itMatch = matchedFeats.begin(); itMatch != matchedFeats.end(); ++itMatch, ++feat_counter )
    {
        // We don't take into account the matching if it is too close to the image border (the largest patch cannot be extracted)
        if( itMatch->first->x+hLargestSize > smLeftImg.getWidth()-1 || itMatch->first->x-hLargestSize < 0 ||
            itMatch->first->y+hLargestSize > smLeftImg.getHeight()-1 || itMatch->first->y-hLargestSize < 0 ||
            itMatch->second->x+hLargestSize > smRightImg.getWidth()-1 || itMatch->second->x-hLargestSize < 0 ||
            itMatch->second->y+hLargestSize > smRightImg.getHeight()-1 || itMatch->second->y-hLargestSize < 0 )
                continue;

        // We have found a proper match to obtain the multi-descriptor
        // Compute the depth and store the coords:
        tlogger.enter("compute depth");
        if( opts.computeDepth )
        {
            double disp = itMatch->first->x - itMatch->second->x;
            double aux  = opts.baseline/disp;
            double x3D  = (itMatch->first->x-opts.cx)*aux;
            double y3D  = (itMatch->first->y-opts.cy)*aux;
            double z3D  = opts.fx*aux;

            itMatch->first->depth     = sqrt( x3D*x3D + y3D*y3D + z3D*z3D );
            itMatch->second->depth    = sqrt( (x3D-opts.baseline)*(x3D-opts.baseline) + y3D*y3D + z3D*z3D );
        }
        tlogger.leave("compute depth");

        tlogger.enter("cp scales");
        itMatch->first->multiScales.resize( opts.scales.size() );
        itMatch->first->multiOrientations.resize( opts.scales.size() );
        itMatch->first->descriptors.multiSIFTDescriptors.resize( opts.scales.size() );

        itMatch->second->multiScales.resize( opts.scales.size() );
        itMatch->second->multiOrientations.resize( opts.scales.size() );
        itMatch->second->descriptors.multiSIFTDescriptors.resize( opts.scales.size() );

        // Copy the scale values within the feature.
        memcpy( &itMatch->first->multiScales[0], &opts.scales[0], opts.scales.size()*sizeof(double) );
        memcpy( &itMatch->second->multiScales[0], &opts.scales[0], opts.scales.size()*sizeof(double) );
        tlogger.leave("cp scales");

        // For each of the involved scales
        for( unsigned int k = 0; k < opts.scales.size(); ++k )
        {
            npSize = round(a*opts.scales[k]);
            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
            hpSize = npSize/2;                              // half size of the patch

            CImage tPatch;

            // LEFT IMAGE:
            tlogger.enter("extract & resize");
            smLeftImg.extract_patch( tPatch, itMatch->first->x-hpSize, itMatch->first->y-hpSize, npSize, npSize );

            cv::Mat out_mat_patch;
            // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
            // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
            cv::resize( cv::Mat( static_cast<IplImage*>(tPatch.getAsIplImage()) ), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img = IplImage(out_mat_patch);
            CImage rsPatch( &aux_img );
            tlogger.leave("extract & resize");

            tlogger.enter("main orientations");
            // Compute the main orientations for the axa patch, taking into account that the actual patch has a size of a+2xa+2
            // (being the center point the one with coordinates a/2+1,a/2+1).
            // A sigma = opts.sg2 is used to smooth the entries in the orientations histograms
            // Orientation vector will be resize inside the function, so don't reserve space for it
            vision::computeMainOrientations(
                            rsPatch, a/2+1, a/2+1, a,
                            itMatch->first->multiOrientations[k], opts.sg2 );
            tlogger.leave("main orientations");

            size_t nMainOris = itMatch->first->multiOrientations[k].size();
            itMatch->first->descriptors.multiSIFTDescriptors[k].resize( nMainOris );
            for( unsigned int m = 0; m < nMainOris; ++m )
            {
                tlogger.enter("compute histogram");
                computeHistogramOfOrientations(
                            rsPatch,
                            a/2+1, a/2+1, a,
                            itMatch->first->multiOrientations[k][m],
                            itMatch->first->descriptors.multiSIFTDescriptors[k][m], opts );
                tlogger.leave("compute histogram");
            } // end for

            // RIGHT IMAGE:
            tlogger.enter("extract & resize");
            imageRight.extract_patch( tPatch, itMatch->second->x-hpSize, itMatch->second->y-hpSize, npSize, npSize );

            cv::resize( cv::Mat( static_cast<IplImage*>(tPatch.getAsIplImage()) ), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img2 = IplImage(out_mat_patch);
            CImage rsPatch2( &aux_img2 );
            tlogger.leave("extract & resize");

            tlogger.enter("main orientations");
            vision::computeMainOrientations(
                                rsPatch2, a/2+1, a/2+1, a,
                                itMatch->second->multiOrientations[k], opts.sg2 );
            tlogger.leave("main orientations");

            nMainOris = itMatch->second->multiOrientations[k].size();
            itMatch->second->descriptors.multiSIFTDescriptors[k].resize( nMainOris );

            for( unsigned int m = 0; m < nMainOris; ++m )
            {
                tlogger.enter("compute histogram");
                computeHistogramOfOrientations(
                            rsPatch2,
                            a/2+1, a/2+1, a,
                            itMatch->second->multiOrientations[k][m],
                            itMatch->second->descriptors.multiSIFTDescriptors[k][m], opts );
                tlogger.leave("compute histogram");
            } // end for
        } // end scales for
        good_matches++;
    } // end matches for
    MRPT_END
#else
	THROW_EXCEPTION("This function needs OpenCV 2.1+")
#endif
} // end-vision::computeMultiResolutionDescriptors

/*-------------------------------------------------------------
					computeMultiResolutionDescriptors
-------------------------------------------------------------*/
void vision::computeMultiResolutionDescriptors(
        const CImage &image,
        CFeatureList &list,
        const TMultiResDescOptions &opts )
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211
    MRPT_START
    CTimeLogger tlogger;
    tlogger.disable();
    ASSERT_( list.size() > 0 );

    //**************************************************************************
    // Pre-smooth the image with sigma = sg1 (typically 0.5)
    //**************************************************************************
    tlogger.enter("smooth");
    cv::Mat tempImg1;
    IplImage aux1;

    cv::Mat inImg1 = static_cast<IplImage*>(image.getAsIplImage());

    cv::GaussianBlur( inImg1, tempImg1, cvSize(0,0), opts.sg1 /*sigmaX*/, opts.sg1 /*sigmaY*/ );
    aux1 = tempImg1;
    CImage smLeftImg( &aux1 );
    //--------------------------------------------------------------------------

    unsigned int a = opts.basePSize;

    int largestSize = round(a*opts.scales[opts.scales.size()-1]);
    largestSize = largestSize%2 ? largestSize : largestSize+1;          // round to the closest odd number
    int hLargestSize = largestSize/2;

    unsigned int npSize;
    unsigned int hpSize;

    for( CFeatureList::iterator it = list.begin(); it != list.end(); ++it )
    {
        // We don't take into account the matching if it is too close to the image border (the largest patch cannot be extracted)
        if( (*it)->x+hLargestSize > smLeftImg.getWidth()-1 || (*it)->x-hLargestSize < 0 ||
            (*it)->y+hLargestSize > smLeftImg.getHeight()-1 || (*it)->y-hLargestSize < 0 )
                continue;

        // We have found a proper match to obtain the multi-descriptor
        tlogger.enter("cp scales");
        (*it)->multiScales.resize( opts.scales.size() );
        (*it)->multiOrientations.resize( opts.scales.size() );
        (*it)->descriptors.multiSIFTDescriptors.resize( opts.scales.size() );

        // Copy the scale values within the feature.
        memcpy( &(*it)->multiScales[0], &opts.scales[0], opts.scales.size()*sizeof(double) );
        tlogger.leave("cp scales");

        // For each of the involved scales
        for( unsigned int k = 0; k < opts.scales.size(); ++k )
        {
            npSize = round(a*opts.scales[k]);
            npSize = npSize%2 ? npSize : npSize+1;          // round to the closest odd number
            hpSize = npSize/2;                              // half size of the patch

            CImage tPatch;

            // LEFT IMAGE:
            tlogger.enter("extract & resize");
            smLeftImg.extract_patch( tPatch, (*it)->x-hpSize, (*it)->y-hpSize, npSize, npSize );

            cv::Mat out_mat_patch;
            // The size is a+2xa+2 because we have to compute the gradient (magnitude and orientation) in every pixel within the axa patch so we need
            // one more row and column. For instance, for a 23x23 patch we need a 25x25 patch.
            cv::resize( cv::Mat( static_cast<IplImage*>(tPatch.getAsIplImage()) ), out_mat_patch, cv::Size(a+2,a+2) );
            IplImage aux_img = IplImage(out_mat_patch);
            CImage rsPatch( &aux_img );
            tlogger.leave("extract & resize");

            tlogger.enter("main orientations");
            // Compute the main orientations for the axa patch, taking into account that the actual patch has a size of a+2xa+2
            // (being the center point the one with coordinates a/2+1,a/2+1).
            // A sigma = opts.sg2 is used to smooth the entries in the orientations histograms
            // Orientation vector will be resize inside the function, so don't reserve space for it
            vision::computeMainOrientations(
                            rsPatch, a/2+1, a/2+1, a,
                            (*it)->multiOrientations[k], opts.sg2 );
            tlogger.leave("main orientations");

            size_t nMainOris = (*it)->multiOrientations[k].size();
            (*it)->descriptors.multiSIFTDescriptors[k].resize( nMainOris );
            for( unsigned int m = 0; m < nMainOris; ++m )
            {
                tlogger.enter("compute histogram");
                computeHistogramOfOrientations(
                            rsPatch,
                            a/2+1, a/2+1, a,
                            (*it)->multiOrientations[k][m],
                            (*it)->descriptors.multiSIFTDescriptors[k][m], opts );
                tlogger.leave("compute histogram");
            } // end for
        } // end scales for
    } // end matches for
    MRPT_END
#else
	THROW_EXCEPTION("This function needs OpenCV 2.1+")
#endif
} // end computeMultiResolutionDescriptors

/*************************************** END FAMD ********************************************/

/*-------------------------------------------------------------
						matchFeatures
-------------------------------------------------------------*/
size_t vision::matchFeatures( const CFeatureList &list1,
							const CFeatureList &list2,
							CMatchedFeatureList &matches,
							const TMatchingOptions &options )
{
	// Clear the output structure
	MRPT_START;
	matches.clear();

	// Preliminary comprobations
	size_t sz1 = list1.size(), sz2 = list2.size();

	ASSERT_( (sz1 > 0) && (sz2 > 0) );    // Both lists have features within it
	ASSERT_( list1.get_type() == list2.get_type() );	    // Both lists must be of the same type

	CFeatureList::const_iterator	itList1, itList2;	// Iterators for the lists

	// For SIFT & SURF
	float							distDesc;			// EDD or EDSD
	float							minDist1;		    // Minimum EDD or EDSD
	float							minDist2;		    // Second minimum EDD or EDSD
	TFeatureID						minIdx; 			// Index of the closest feature

	// For Harris
	double							maxCC1;			    // Maximum CC
	double							maxCC2;			    // Second maximum CC

	// For SAD
	double							minSAD1, minSAD2;

	vector<int> idxLeftList, idxRightList;
	idxLeftList.resize(sz1, FEAT_FREE);
	idxRightList.resize(sz2, FEAT_FREE);
	vector<double> distCorrs(sz1);
	int lFeat, rFeat;
	int minLeftIdx = 0, minRightIdx;
	int nMatches = 0;

	// For each feature in list1 ...
	for( lFeat = 0, itList1 = list1.begin(); itList1 != list1.end(); ++itList1, ++lFeat )
	{
		// For SIFT & SURF
		minDist1 = 1e5;
		minDist2 = 1e5;

		// For Harris
		maxCC1	= 0;
		maxCC2	= 0;

		// For SAD
		minSAD1 = 1e5;
		minSAD2 = 1e5;

		// For all the cases
		minIdx	    = 0;
		minRightIdx = 0;

		for( rFeat = 0, itList2 = list2.begin(); itList2 != list2.end(); ++itList2, ++rFeat )		// ... compare with all the features in list2.
		{
			// Filter out by epipolar constraint
			double d = 0.0;														// Distance to the epipolar line
			if( options.useEpipolarRestriction )
			{
				if( options.parallelOpticalAxis )
					d = (*itList1)->y - (*itList2)->y;
				else
				{
					ASSERT_( options.hasFundamentalMatrix );

					// Compute epipolar line Ax + By + C = 0
					TLine2D		epiLine;
					TPoint2D	oPoint((*itList2)->x,(*itList2)->y);

					CMatrixDouble31 l, p;
					p(0,0) = (*itList1)->x;
					p(1,0) = (*itList1)->y;
					p(2,0) = 1;

					l = options.F*p;

					epiLine.coefs[0] = l(0,0);
					epiLine.coefs[1] = l(1,0);
					epiLine.coefs[2] = l(2,0);

					d = epiLine.distance( oPoint );
				} // end else
			} // end if

			bool c1 = options.useEpipolarRestriction ? fabs(d) < options.epipolar_TH : true;	// Use epipolar restriction
			bool c2 = options.useXRestriction ? ((*itList1)->x - (*itList2)->x) > 0 : true;		// Use x-coord restriction

			if( c1 && c2 )
			{
				switch( options.matching_method )
				{

				case TMatchingOptions::mmDescriptorSIFT:
				{
					// Ensure that both features have SIFT descriptors
					ASSERT_((*itList1)->descriptors.hasDescriptorSIFT() && (*itList2)->descriptors.hasDescriptorSIFT() );

					// Compute the Euclidean distance between descriptors
					distDesc = (*itList1)->descriptorSIFTDistanceTo( *(*itList2) );

					// Search for the two minimum values
					if( distDesc < minDist1 )
					{
						minDist2 = minDist1;
						minDist1 = distDesc;
						minLeftIdx  = lFeat;
						minRightIdx = rFeat;
					}
					else if ( distDesc < minDist2 )
						minDist2 = distDesc;

					break;
				} // end mmDescriptorSIFT

				case TMatchingOptions::mmCorrelation:
				{
					size_t							u,v;				// Coordinates of the peak
					double							res;				// Value of the peak

					// Ensure that both features have patches
					ASSERT_( (*itList1)->patchSize > 0 && (*itList2)->patchSize > 0 );
					vision::openCV_cross_correlation( (*itList1)->patch, (*itList2)->patch, u, v, res );

					// Search for the two maximum values
					if( res > maxCC1 )
					{

						maxCC2 = maxCC1;
						maxCC1 = res;
						minLeftIdx  = lFeat;
						minRightIdx = rFeat;
					}
					else if( res > maxCC2 )
						maxCC2 = res;

					break;
				} // end mmCorrelation

				case TMatchingOptions::mmDescriptorSURF:
				{
					// Ensure that both features have SURF descriptors
					ASSERT_((*itList1)->descriptors.hasDescriptorSURF() && (*itList2)->descriptors.hasDescriptorSURF() );

					// Compute the Euclidean distance between descriptors
					distDesc = (*itList1)->descriptorSURFDistanceTo( *(*itList2) );

					// Search for the two minimum values
					if( distDesc < minDist1 )
					{
						minDist2 = minDist1;
						minDist1 = distDesc;
						minLeftIdx  = lFeat;
						minRightIdx = rFeat;
					}
					else if ( distDesc < minDist2 )
						minDist2 = distDesc;

					break; // end case featSURF
				} // end mmDescriptorSURF

				case TMatchingOptions::mmSAD:
				{
					// Ensure that both features have patches
					ASSERT_( (*itList1)->patchSize > 0 && (*itList2)->patchSize == (*itList1)->patchSize );
#if !MRPT_HAS_OPENCV
	THROW_EXCEPTION("MRPT has been compiled without OpenCV")
#else
					IplImage *aux1, *aux2;
					if( (*itList1)->patch.isColor() && (*itList2)->patch.isColor() )
					{

						IplImage* preAux1 = (IplImage*)(*itList1)->patch.getAsIplImage();
						IplImage* preAux2 = (IplImage*)(*itList2)->patch.getAsIplImage();

						aux1 = cvCreateImage( cvSize( (*itList1)->patch.getHeight(), (*itList1)->patch.getWidth() ), IPL_DEPTH_8U, 1 );
						aux2 = cvCreateImage( cvSize( (*itList2)->patch.getHeight(), (*itList2)->patch.getWidth() ), IPL_DEPTH_8U, 1 );

						cvCvtColor( preAux1, aux1, CV_BGR2GRAY );
						cvCvtColor( preAux2, aux2, CV_BGR2GRAY );
					}
					else
					{
						aux1 = (IplImage*)(*itList1)->patch.getAsIplImage();
						aux2 = (IplImage*)(*itList2)->patch.getAsIplImage();
					}

                    // OLD CODE (for checking purposes)
//					for( unsigned int ii = 0; ii < (unsigned int)aux1->imageSize; ++ii )
//						m1 += aux1->imageData[ii];
//					m1 /= (double)aux1->imageSize;

//					for( unsigned int ii = 0; ii < (unsigned int)aux2->imageSize; ++ii )
//						m2 += aux2->imageData[ii];
//					m2 /= (double)aux2->imageSize;

//					for( unsigned int ii = 0; ii < (unsigned int)aux1->imageSize; ++ii )
//						res += fabs( fabs((double)aux1->imageData[ii]-m1) - fabs((double)aux2->imageData[ii]-m2) );

					// NEW CODE
					double res = 0;
                    for( unsigned int ii = 0; ii < (unsigned int)aux1->height; ++ii )       // Rows
                        for( unsigned int jj = 0; jj < (unsigned int)aux1->width; ++jj )    // Cols
                            res += fabs((double)(aux1->imageData[ii*aux1->widthStep+jj]) - ((double)(aux2->imageData[ii*aux2->widthStep+jj])) );
					res = res/(255.0f*aux1->width*aux1->height);

					if( res < minSAD1 )
					{
						minSAD2 = minSAD1;
						minSAD1 = res;
						minLeftIdx  = lFeat;
						minRightIdx = rFeat;
					}
					else if ( res < minSAD2 )
						minSAD2 = res;
#endif
					break;
				} // end mmSAD
				} // end switch
			} // end if
		} // end for 'list2' (right features)

		bool cond1 = false, cond2 = false;
		double minVal = 1.0;
		switch(options.matching_method)
		{
			case TMatchingOptions::mmDescriptorSIFT:
				cond1 = minDist1 < options.maxEDD_TH;						// Maximum Euclidean Distance between SIFT descriptors (EDD)
				cond2 = (minDist1/minDist2) < options.EDD_RATIO;			// Ratio between the two lowest EDSD
				minVal = minDist1;
				break;
			case TMatchingOptions::mmCorrelation:
				cond1 = maxCC1 > options.minCC_TH;							// Minimum cross correlation value
				cond2 = (maxCC2/maxCC1) < options.rCC_TH;					// Ratio between the two highest cross correlation values
				minVal = 1-maxCC1;
				break;
			case TMatchingOptions::mmDescriptorSURF:
				cond1 = minDist1 < options.maxEDSD_TH;						// Maximum Euclidean Distance between SURF descriptors (EDSD)
				cond2 = (minDist1/minDist2) < options.EDSD_RATIO;			// Ratio between the two lowest EDSD
				minVal = minDist1;
				break;
			case TMatchingOptions::mmSAD:
				cond1 = minSAD1 < options.maxSAD_TH;
				cond2 = (minSAD1/minSAD2) < options.SAD_RATIO;
				minVal = minSAD1;
				break;
			default:
				THROW_EXCEPTION("Invalid value of 'matching_method'");
		}

		// PROCESS THE RESULTS
		if( cond1 && cond2 )					// The minimum distance must be below a threshold
		{
		    int auxIdx = idxRightList[ minRightIdx ];
		    if( auxIdx != FEAT_FREE )
		    {
		        if( distCorrs[ auxIdx ] > minVal )
                {
                    // We've found a better match
//                    cout << "Better match found: [R] " << idxLeftList[auxIdx] << " was with [L] " << auxIdx << "(" << distCorrs[ auxIdx ] << ")";
//                    cout << " now is with [L] " << minLeftIdx << "(" << minVal << ")" << endl;
                    distCorrs[ minLeftIdx ]     = minVal;
                    idxLeftList[ minLeftIdx ]   = minRightIdx;
                    idxRightList[ minRightIdx ] = minLeftIdx;

                    distCorrs[ auxIdx ]         = 1.0;
                    idxLeftList[ auxIdx ]       = FEAT_FREE;
                } // end-if
//                else
//                    cout << "Conflict but not better match" << endl;
		    } // end-if
		    else
		    {
//		        cout << "New match found: [R] " << minRightIdx << " with [L] " << minLeftIdx << "(" << minVal << ")" << endl;
		        idxRightList[ minRightIdx ] = minLeftIdx;
		        idxLeftList[ minLeftIdx ]   = minRightIdx;
		        distCorrs[ minLeftIdx ]     = minVal;
		        nMatches++;
		    }
		} // end if
	} // end for 'list1' (left features)

    matches.resize( nMatches );
    CMatchedFeatureList::iterator itMatch = matches.begin();
    for( int vCnt = 0; vCnt < (int)idxLeftList.size(); ++vCnt )
    {
        if( idxLeftList[vCnt] != FEAT_FREE )
        {
            itMatch->first    = list1[vCnt];
            itMatch->second   = list2[idxLeftList[vCnt]];

            if( options.estimateDepth && options.parallelOpticalAxis )
            {
                double disp = itMatch->first->x - itMatch->second->x;
                double aux  = options.baseline/disp;
                double x3D  = (itMatch->first->x-options.cx)*aux;
                double y3D  = (itMatch->first->y-options.cy)*aux;
                double z3D  = options.fx*aux;

                itMatch->first->depth     = sqrt( x3D*x3D + y3D*y3D + z3D*z3D );
                itMatch->second->depth    = sqrt( (x3D-options.baseline)*(x3D-options.baseline) + y3D*y3D + z3D*z3D );
            } // end-if
            ++itMatch;
        }
    } // end for matches
	return matches.size();

	MRPT_END;
}

double vision::computeSAD( const CImage &patch1, const CImage &patch2 )
{
    IplImage *im1 = static_cast<IplImage*>(patch1.getAsIplImage());
    IplImage *im2 = static_cast<IplImage*>(patch2.getAsIplImage());

    ASSERT_( im1->width == im2->width &&  im1->height == im2->height );
    double res = 0.0;
    for( unsigned int ii = 0; ii < (unsigned int)im1->height; ++ii )       // Rows
        for( unsigned int jj = 0; jj < (unsigned int)im1->width; ++jj )    // Cols
            res += fabs((double)(im1->imageData[ii*im1->widthStep+jj]) - ((double)(im2->imageData[ii*im2->widthStep+jj])) );
    return res/(255.0f*im1->width*im1->height);

} // end computeSAD

/*-------------------------------------------------------------
			TStereoSystemParams: constructor
-------------------------------------------------------------*/
TStereoSystemParams::TStereoSystemParams() :
	uncPropagation(Prop_Linear),
	baseline ( 0.119f ),	// Bumblebee
	stdPixel ( 1 ),
	stdDisp  ( 1 ),
	maxZ	 ( 20.0f ),	// Indoor
	minZ	 ( 0.5f ),	// Indoor
	maxY	 ( 3.0f ),	// Indoor
	factor_k ( 1.5f ),
	factor_a ( 1e-3f ),
	factor_b ( 2.0f )
{
    K = defaultIntrinsicParamsMatrix(0,640,480);
}

/*-------------------------------------------------------------
			TStereoSystemParams: loadFromConfigFile
-------------------------------------------------------------*/
void  TStereoSystemParams::loadFromConfigFile(
			const mrpt::utils::CConfigFileBase	&iniFile,
			const std::string		&section)
{
	int unc;
	unc				= iniFile.read_int(section.c_str(),"uncPropagation",uncPropagation);
	switch (unc)
	{
		case 0:
			uncPropagation = Prop_Linear;
			break;
		case 1:
			uncPropagation = Prop_UT;
			break;
		case 2:
			uncPropagation = Prop_SUT;
			break;
	} // end switch

	vector_double k_vec(9);
	iniFile.read_vector( section.c_str(), "k_vec", vector_double(), k_vec, false );

	for (unsigned int ii = 0; ii < 3; ++ii )
        for (unsigned int jj = 0; jj < 3; ++jj)
            K(ii,jj) = k_vec[ii*3+jj];

    baseline = iniFile.read_float(section.c_str(),"baseline",baseline);

	// These should be initialized from the first CObservationStereoImages within a certain Rawlog
	//K(0,0)			= iniFile.read_float(section.c_str(),"k00",K(0,0));
	//K(0,1)			= iniFile.read_float(section.c_str(),"k01",K(0,1));
	//K(0,2)			= iniFile.read_float(section.c_str(),"k02",K(0,2));
	//K(1,0)			= iniFile.read_float(section.c_str(),"k10",K(1,0));
	//K(1,1)			= iniFile.read_float(section.c_str(),"k11",K(1,1));
	//K(1,2)			= iniFile.read_float(section.c_str(),"k12",K(1,2));
	//K(2,0)			= iniFile.read_float(section.c_str(),"k20",K(2,0));
	//K(2,1)			= iniFile.read_float(section.c_str(),"k21",K(2,1));
	//K(2,2)			= iniFile.read_float(section.c_str(),"k22",K(2,2));

	//baseline		= iniFile.read_float(section.c_str(),"baseline",baseline);
	stdPixel		= iniFile.read_float(section.c_str(),"stdPixel",stdPixel);
	stdDisp			= iniFile.read_float(section.c_str(),"stdDisp",stdDisp);
	maxZ			= iniFile.read_float(section.c_str(),"maxZ",maxZ);
	minZ			= iniFile.read_float(section.c_str(),"minZ",minZ);
	maxY			= iniFile.read_float(section.c_str(),"maxY",maxY);
	factor_k		= iniFile.read_float(section.c_str(),"factor_k",factor_k);
	factor_a		= iniFile.read_float(section.c_str(),"factor_a",factor_a);
	factor_b		= iniFile.read_float(section.c_str(),"factor_b",factor_b);
} // end of loadFromConfigFile

/*---------------------------------------------------------------
					TStereoSystemParams: dumpToTextStream
  ---------------------------------------------------------------*/
void  TStereoSystemParams::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [vision::TStereoSystemParams] ------------ \n");
	out.printf("Method for 3D Uncert. \t= ");
	switch (uncPropagation)
	{
	case Prop_Linear:
		out.printf("Linear propagation\n");
		break;
	case Prop_UT:
		out.printf("Unscented Transform\n");
		break;
	case Prop_SUT:
		out.printf("Scaled Unscented Transform\n");
		break;
	} // end switch

	out.printf("K\t\t\t= [%f\t%f\t%f]\n", K(0,0), K(0,1), K(0,2));
	out.printf(" \t\t\t  [%f\t%f\t%f]\n", K(1,0), K(1,1), K(1,2));
	out.printf(" \t\t\t  [%f\t%f\t%f]\n", K(2,0), K(2,1), K(2,2));

	out.printf("Baseline \t\t= %f\n", baseline);
	out.printf("Pixel std \t\t= %f\n", stdPixel);
	out.printf("Disparity std\t\t= %f\n", stdDisp);
	out.printf("Z maximum\t\t= %f\n", maxZ);
	out.printf("Z minimum\t\t= %f\n", minZ);
	out.printf("Y maximum\t\t= %f\n", maxY);

	out.printf("k Factor [UT]\t\t= %f\n", factor_k);
	out.printf("a Factor [UT]\t\t= %f\n", factor_a);
	out.printf("b Factor [UT]\t\t= %f\n", factor_b);
	out.printf("-------------------------------------------------------- \n");
}
/*-------------------------------------------------------------
			TMatchingOptions: constructor
-------------------------------------------------------------*/
TMatchingOptions::TMatchingOptions() :
	// General
	useEpipolarRestriction ( true ),	// Whether or not take into account the epipolar restriction for finding correspondences
	hasFundamentalMatrix ( false ),		// Whether or not there is a fundamental matrix
	parallelOpticalAxis ( true ),		// Whether or not take into account the epipolar restriction for finding correspondences
	useXRestriction ( true ),			// Whether or not employ the x-coord restriction for finding correspondences (bumblebee camera, for example)

	matching_method ( mmCorrelation ),	// Matching method
	epipolar_TH	( 1.5f ),				// Epipolar constraint (rows of pixels)

	// SIFT
	maxEDD_TH	( 90 ),					// Maximum Euclidean Distance Between SIFT Descriptors
	EDD_RATIO   ( 0.6 ),				// Boundary Ratio between the two lowest EDD

	// KLT
	minCC_TH	( 0.95f ),				// Minimum Value of the Cross Correlation
	minDCC_TH	( 0.025f ),				// Minimum Difference Between the Maximum Cross Correlation Values
	rCC_TH		( 0.92f ),				// Maximum Ratio Between the two highest CC values

	// SURF
	maxEDSD_TH	( 0.15 ),				// Maximum Euclidean Distance Between SURF Descriptors
	EDSD_RATIO  ( 0.6 ),				// Boundary Ratio between the two lowest SURF EDSD

	// SAD
	maxSAD_TH	( 0.4 ),
	SAD_RATIO	( 0.5 ),

	// For estimating depth
	estimateDepth   ( false ),
	fx          ( 0.0 ),
	cx          ( 0.0 ),
	cy          ( 0.0 ),
	baseline    ( 0.0 )
{
} // end constructor TMatchingOptions

/*-------------------------------------------------------------
			TMatchingOptions: loadFromConfigFile
-------------------------------------------------------------*/
void  TMatchingOptions::loadFromConfigFile(
			const mrpt::utils::CConfigFileBase	&iniFile,
			const std::string		&section)
{
	int mm			= iniFile.read_int(section.c_str(),"matching_method",matching_method);
	switch( mm )
	{
	case 0:
		matching_method = mmCorrelation;
		break;
	case 1:
		matching_method = mmDescriptorSIFT;
		break;
	case 2:
		matching_method = mmDescriptorSURF;
		break;
	case 3:
		matching_method = mmSAD;
		break;
	} // end switch

    useEpipolarRestriction  = iniFile.read_bool(section.c_str(), "useEpipolarRestriction", useEpipolarRestriction );
    hasFundamentalMatrix    = iniFile.read_bool(section.c_str(), "hasFundamentalMatrix", hasFundamentalMatrix );
    parallelOpticalAxis     = iniFile.read_bool(section.c_str(), "parallelOpticalAxis", parallelOpticalAxis );
    useXRestriction         = iniFile.read_bool(section.c_str(), "useXRestriction", useXRestriction );

	epipolar_TH		= iniFile.read_float(section.c_str(),"epipolar_TH",epipolar_TH);
	maxEDD_TH		= iniFile.read_float(section.c_str(),"maxEDD_TH",maxEDD_TH);
	EDD_RATIO		= iniFile.read_float(section.c_str(),"minDIF_TH",EDD_RATIO);
	minCC_TH		= iniFile.read_float(section.c_str(),"minCC_TH",minCC_TH);
	minDCC_TH		= iniFile.read_float(section.c_str(),"minDCC_TH",minDCC_TH);
	rCC_TH			= iniFile.read_float(section.c_str(),"rCC_TH",rCC_TH);
	maxEDSD_TH		= iniFile.read_float(section.c_str(),"maxEDSD_TH",maxEDSD_TH);
	EDSD_RATIO		= iniFile.read_float(section.c_str(),"EDSD_RATIO",EDSD_RATIO);
	maxSAD_TH		= iniFile.read_float(section.c_str(),"maxSAD_TH",maxSAD_TH);
	SAD_RATIO		= iniFile.read_float(section.c_str(),"SAD_RATIO",SAD_RATIO);
	SAD_RATIO		= iniFile.read_float(section.c_str(),"SAD_RATIO",SAD_RATIO);

	estimateDepth   = iniFile.read_bool(section.c_str(), "estimateDepth", estimateDepth );
	fx              = iniFile.read_float(section.c_str(),"fx",fx);
	cx              = iniFile.read_float(section.c_str(),"cx",cx);
	cy              = iniFile.read_float(section.c_str(),"cy",cy);
	baseline        = iniFile.read_float(section.c_str(),"baseline",baseline);
} // end TMatchingOptions::loadFromConfigFile

/*---------------------------------------------------------------
					TMatchingOptions: dumpToTextStream
  ---------------------------------------------------------------*/
void  TMatchingOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [vision::TMatchingOptions] ------------ \n");
	out.printf("Matching method:                ");
	switch( matching_method )
	{
	case mmCorrelation:
		out.printf("Cross Correlation\n");
        out.printf("· Min. CC. Threshold:           %f\n", minCC_TH);
        out.printf("· Min. Dif. CC Threshold:       %f\n", minDCC_TH);
        out.printf("· Max. Ratio CC Threshold:      %f\n", rCC_TH);
		break;
	case mmDescriptorSIFT:
		out.printf("SIFT descriptor\n");
        out.printf("· Max. EDD Threshold:           %f\n", maxEDD_TH);
        out.printf("· EDD Ratio:                    %f\n", EDD_RATIO);
		break;
	case mmDescriptorSURF:
		out.printf("SURF descriptor\n");
        out.printf("· EDD Ratio:                    %f\n", maxEDSD_TH);
        out.printf("· Min. CC Threshold:            %f\n", EDSD_RATIO);
		break;
	case mmSAD:
		out.printf("SAD\n");
        out.printf("· Max. Dif. SAD Threshold:      %f\n", maxSAD_TH);
        out.printf("· Ratio SAD Threshold:          %f\n", SAD_RATIO);
		break;
	} // end switch
	out.printf("Epipolar Thres:                 %.2f px\n", epipolar_TH);
	out.printf("Using epipolar restriction?:    ");
	out.printf( useEpipolarRestriction ? "Yes\n" : "No\n" );
	out.printf("Has Fundamental Matrix?:        ");
	out.printf( hasFundamentalMatrix ? "Yes\n" : "No\n" );
	out.printf("Are camera axis parallel?:      ");
	out.printf( parallelOpticalAxis ? "Yes\n" : "No\n" );
	out.printf("Use X-coord restriction?:       ");
	out.printf( useXRestriction ? "Yes\n" : "No\n" );
	out.printf("Estimate depth?:                ");
	out.printf( estimateDepth ? "Yes\n" : "No\n" );
	if( estimateDepth )
	{
        out.printf("· Focal length:                 %f px\n", fx);
        out.printf("· Principal Point (cx):         %f px\n", cx);
        out.printf("· Principal Point (cy):         %f px\n", cy);
        out.printf("· Baseline:                     %f m\n", baseline);
	}
    out.printf("-------------------------------------------------------- \n");
} // end TMatchingOptions::dumpToTextStream


/* -------------------------------------------------------
				StereoObs2RBObs #1
   ------------------------------------------------------- */
void vision::StereoObs2BRObs(
	const CMatchedFeatureList &inMatches,
	const CMatrixDouble33 &intrinsicParams,
	const double &baseline,
	const CPose3D &sensorPose,
	const std::vector<double> &sg,
	CObservationBearingRange &outObs )
{

	// Compute the range and bearing
	double f		= intrinsicParams(0,0);				// Focal length in pixels
	double x0		= intrinsicParams(0,2);				// Principal point column
	double y0		= intrinsicParams(1,2);				// Principal point row
	double b		= baseline;							// Stereo camera baseline
	double sg_c2	= square( sg[0] );					// Sigma of the column variable
	double sg_r2	= square( sg[1] );					// Sigma of the row variable
	double sg_d2	= square( sg[2] );					// Sigma of the disparity

	for( CMatchedFeatureList::const_iterator itMatchList = inMatches.begin();
			itMatchList != inMatches.end();
			itMatchList++ )
	{
		double x	= itMatchList->first->x;			// Column of the feature
		double y	= itMatchList->first->y;			// Row of the feature

		double d	= itMatchList->first->x - itMatchList->second->x;	// Disparity
		double d2	= square(d);
		double k	= square(b/d);

		// Projection equations according to a standard camera coordinate axis (+Z forward & +Y downwards)
		// -------------------------------------------------------------------------------------------------------
		double X	= ( x - x0 ) * b / d;
		double Y	= ( y - y0 ) * b / d;
		double Z	= f * b / d;

		// Projection equations according to a standard coordinate axis (+X forward & +Z upwards)
		// -------------------------------------------------------------------------------------------------------
		//double X	= f * b / d;
		//double Y	= ( x0 - x ) * b / d;
		//double Z	= ( y0 - y ) * b / d;

		CObservationBearingRange::TMeasurement m;
		m.range = sqrt( square(X) + square(Y) + square(Z) );
		m.yaw	= atan2( Y,X );
		m.pitch = -asin( Z/m.range );
		m.landmarkID	= itMatchList->first->ID;

		// Compute the covariance
		// Formula: S_BR = JG * (JF * diag(sg_c^2, sg_r^2, sg_d^2) * JF') * JG'
		//						\---------------------------------------/
		//											aux

		CMatrixDouble33 aux;

		// Jacobian equations according to a standard CAMERA coordinate axis (+Z forward & +Y downwards)
		// -------------------------------------------------------------------------------------------------------
		 aux.get_unsafe( 0, 0 ) = k*(sg_c2 + sg_d2*square(x-x0)/d2);
		 aux.get_unsafe( 0, 1 ) = aux.get_unsafe( 1, 0 ) = k*(sg_d2*(x-x0)*(y-y0)/d2);
		 aux.get_unsafe( 0, 2 ) = aux.get_unsafe( 2, 0 ) = k*(sg_d2*(x-x0)*f/d2);

		 aux.get_unsafe( 1, 1 ) = k*(sg_r2 + sg_d2*square(y-y0)/d2);
		 aux.get_unsafe( 1, 2 ) = aux.get_unsafe( 2, 1 ) = k*(sg_d2*(y-y0)*f/d2);

		 aux.get_unsafe( 2, 2 ) = k*(sg_d2*square(f)/d2);

		// Jacobian equations according to a standard coordinate axis (+X forward & +Z upwards)
		// -------------------------------------------------------------------------------------------------------
		//aux.get_unsafe( 0, 0 ) = k*(sg_d2*square(f)/d2);
		//aux.get_unsafe( 0, 1 ) = aux.get_unsafe( 1, 0 ) = k*sg_d2*(x0-x)*f/d2;
		//aux.get_unsafe( 0, 2 ) = aux.get_unsafe( 2, 0 ) = k*sg_d2*(y0-y)*f/d2;

		//aux.get_unsafe( 1, 1 ) = k*(sg_c2 + sg_d2*square(x0-x)/d2);
		//aux.get_unsafe( 1, 2 ) = aux.get_unsafe( 2, 1 ) = k*sg_d2*(x0-x)*(y0-y)/d2;

		//aux.get_unsafe( 2, 2 ) = k*(sg_r2 + sg_d2*square(y0-y)/d2);

		//CMatrixDouble33 JF;
		//JF.set_unsafe(0,0) = JF.set_unsafe(1,1) = JF.set_unsafe(2,0) = JF.set_unsafe(2,1) = 0.0f;
		//JF.set_unsafe(0,1) = JF.set_unsafe(1,0) = b/d;
		//JF.set_unsafe(0,2) = -X/d;
		//JF.set_unsafe(1,2) = -Y/d;
		//JF.set_unsafe(2,2) = -Z/d;

		CMatrixDouble33 JG;
		JG.set_unsafe( 0, 0, X/m.range );
		JG.set_unsafe( 0, 1, Y/m.range );
		JG.set_unsafe( 0, 2, Z/m.range );

		JG.set_unsafe( 1, 0, -Y/(square(X)+square(Y)) );
		JG.set_unsafe( 1, 1, X/(square(X)+square(Y)) );
		JG.set_unsafe( 1, 2, 0 );

		JG.set_unsafe( 2, 0, Z*X/(square(m.range)*sqrt(square(X)+square(Y))) );
		JG.set_unsafe( 2, 1, Z*Y/(square(m.range)*sqrt(square(X)+square(Y))) );
		JG.set_unsafe( 2, 2, -sqrt(square(X)+square(Y))/square(m.range) );

		//CMatrixDouble33 aux;
		//CMatrixDouble33 diag;
		//diag.zeros();
		//diag.set_unsafe(0,0) = square( sg_r );
		//diag.set_unsafe(1,1) = square( sg_c );
		//diag.set_unsafe(2,2) = square( sg_d );

		// JF.multiply_HCHt( diag, aux );
		JG.multiply_HCHt( aux, m.covariance );
		// DEBUG:
		// m.covariance = aux; // error covariance in 3D
		//m.landmarkID = itMatchList->first->id;
		outObs.sensedData.push_back( m );

	} // end for

	// Indicate that the covariances have been calculated (for compatibility with earlier versions)
	outObs.validCovariances = true;
	outObs.setSensorPose( sensorPose );
}

/* -------------------------------------------------------
				StereoObs2RBObs #2
   ------------------------------------------------------- */
void vision::StereoObs2BRObs( const CObservationStereoImages &inObs, const std::vector<double> &sg, CObservationBearingRange &outObs )
{
	// Local variables
	CFeatureExtraction	fExt;
	CFeatureList		leftList, rightList;
	CMatchedFeatureList	matchList;
	unsigned int		id = 0;

	// Extract features
	fExt.detectFeatures( inObs.imageLeft, leftList );
	fExt.detectFeatures( inObs.imageRight, rightList );

	// DEBUG:
	// CDisplayWindow		win1, win2;
	// win1.showImageAndPoints( inObs.imageLeft, leftList );
	// win2.showImageAndPoints( inObs.imageRight, rightList );

	// Match features
	size_t nMatches = vision::matchFeatures( leftList, rightList, matchList );
	MRPT_UNUSED_PARAM( nMatches );

	// Compute the range and bearing
	double f		= inObs.leftCamera.fx();			// Focal length in pixels
	double x0		= inObs.leftCamera.cx();			// Principal point column
	double y0		= inObs.leftCamera.cy();			// Principal point row
	double b		= inObs.rightCameraPose.x();		// Stereo camera baseline
	double sg_c2	= square( sg[0] );					// Sigma of the column variable
	double sg_r2	= square( sg[1] );					// Sigma of the row variable
	double sg_d2	= square( sg[2] );					// Sigma of the disparity

	for( CMatchedFeatureList::iterator itMatchList = matchList.begin();
			itMatchList != matchList.end();
			itMatchList++, id++ )
	{
		double x	= itMatchList->first->x;			// Column of the feature
		double y	= itMatchList->first->y;			// Row of the feature

		double d	= itMatchList->first->x - itMatchList->second->x;	// Disparity
		double d2	= square(d);
		double k	= square(b/d);

		// Projection equations according to a standard camera coordinate axis (+Z forward & +Y downwards)
		// -------------------------------------------------------------------------------------------------------
		double X	= ( x - x0 ) * b / d;
		double Y	= ( y - y0 ) * b / d;
		double Z	= f * b / d;

		// Projection equations according to a standard coordinate axis (+X forward & +Z upwards)
		// -------------------------------------------------------------------------------------------------------
		//double X	= f * b / d;
		//double Y	= ( x0 - x ) * b / d;
		//double Z	= ( y0 - y ) * b / d;

		CObservationBearingRange::TMeasurement m;
		m.range = sqrt( square(X) + square(Y) + square(Z) );
		//m.yaw	= atan2( Y,X );
		//m.pitch = -asin( Z/m.range );
		m.yaw	= atan2( X,Z );
		m.pitch = atan2( Y,Z );

		// Compute the covariance
		// Formula: S_BR = JG * (JF * diag(sg_c^2, sg_r^2, sg_d^2) * JF') * JG'
		//						\---------------------------------------/
		//											aux

		CMatrixDouble33 aux;

		// Jacobian equations according to a standard CAMERA coordinate axis (+Z forward & +Y downwards)
		// -------------------------------------------------------------------------------------------------------
		 aux.get_unsafe( 0, 0 ) = k*(sg_c2 + sg_d2*square(x-x0)/d2);
		 aux.get_unsafe( 0, 1 ) = aux.get_unsafe( 1, 0 ) = k*(sg_d2*(x-x0)*(y-y0)/d2);
		 aux.get_unsafe( 0, 2 ) = aux.get_unsafe( 2, 0 ) = k*(sg_d2*(x-x0)*f/d2);

		 aux.get_unsafe( 1, 1 ) = k*(sg_r2 + sg_d2*square(y-y0)/d2);
		 aux.get_unsafe( 1, 2 ) = aux.get_unsafe( 2, 1 ) = k*(sg_d2*(y-y0)*f/d2);

		 aux.get_unsafe( 2, 2 ) = k*(sg_d2*square(f)/d2);

		// Jacobian equations according to a standard coordinate axis (+X forward & +Z upwards)
		// -------------------------------------------------------------------------------------------------------
		//aux.get_unsafe( 0, 0 ) = k*(sg_d2*square(f)/d2);
		//aux.get_unsafe( 0, 1 ) = aux.get_unsafe( 1, 0 ) = k*sg_d2*(x0-x)*f/d2;
		//aux.get_unsafe( 0, 2 ) = aux.get_unsafe( 2, 0 ) = k*sg_d2*(y0-y)*f/d2;

		//aux.get_unsafe( 1, 1 ) = k*(sg_c2 + sg_d2*square(x0-x)/d2);
		//aux.get_unsafe( 1, 2 ) = aux.get_unsafe( 2, 1 ) = k*sg_d2*(x0-x)*(y0-y)/d2;

		//aux.get_unsafe( 2, 2 ) = k*(sg_r2 + sg_d2*square(y0-y)/d2);

		//CMatrixDouble33 JF;
		//JF.set_unsafe(0,0) = JF.set_unsafe(1,1) = JF.set_unsafe(2,0) = JF.set_unsafe(2,1) = 0.0f;
		//JF.set_unsafe(0,1) = JF.set_unsafe(1,0) = b/d;
		//JF.set_unsafe(0,2) = -X/d;
		//JF.set_unsafe(1,2) = -Y/d;
		//JF.set_unsafe(2,2) = -Z/d;

		CMatrixDouble33 JG;
		JG.set_unsafe( 0, 0, X/m.range );
		JG.set_unsafe( 0, 1, Y/m.range );
		JG.set_unsafe( 0, 2, Z/m.range );

		JG.set_unsafe( 1, 0, -Y/(square(X)+square(Y)) );
		JG.set_unsafe( 1, 1, X/(square(X)+square(Y)) );
		JG.set_unsafe( 1, 2, 0 );

		JG.set_unsafe( 2, 0, Z*X/(square(m.range)*sqrt(square(X)+square(Y))) );
		JG.set_unsafe( 2, 1, Z*Y/(square(m.range)*sqrt(square(X)+square(Y))) );
		JG.set_unsafe( 2, 2, -sqrt(square(X)+square(Y))/square(m.range) );

		//CMatrixDouble33 aux;
		//CMatrixDouble33 diag;
		//diag.zeros();
		//diag.set_unsafe(0,0) = square( sg_r );
		//diag.set_unsafe(1,1) = square( sg_c );
		//diag.set_unsafe(2,2) = square( sg_d );

		// JF.multiply_HCHt( diag, aux );
		JG.multiply_HCHt( aux, m.covariance );
		// DEBUG:
		// m.covariance = aux; // error covariance in 3D
		m.landmarkID = id;
		outObs.sensedData.push_back( m );
		outObs.fieldOfView_yaw		= 2*fabs(atan2( -x0, f ));
		outObs.fieldOfView_pitch	= 2*fabs(atan2( -y0, f ));

	} // end for

	// Indicate that the covariances have been calculated (for compatibility with earlier versions)
	outObs.validCovariances = true;
	outObs.setSensorPose( inObs.cameraPose );

} // end StereoObs2BRObs

/* -------------------------------------------------------
				StereoObs2RBObs #3
   ------------------------------------------------------- */
void vision::StereoObs2BRObs( const CObservationVisualLandmarks &inObs, CObservationBearingRange &outObs )
{
	// For each of the 3D landmarks [X,Y,Z] we compute their range and bearing representation.
	// The reference system is assumed to be that typical of cameras: +Z forward and +X to the right.
	CLandmarksMap::TCustomSequenceLandmarks::const_iterator itCloud;
	for( itCloud = inObs.landmarks.landmarks.begin(); itCloud != inObs.landmarks.landmarks.end(); ++itCloud )
	{
		CObservationBearingRange::TMeasurement m;
		m.range			= sqrt( square(itCloud->pose_mean.x) + square(itCloud->pose_mean.y) + square(itCloud->pose_mean.z) );
		//m.yaw			= atan2( itCloud->pose_mean.x, itCloud->pose_mean.z );
		//m.pitch		= atan2( itCloud->pose_mean.y, itCloud->pose_mean.z );
		// The reference system is assumed to be that typical robot operation: +X forward and +Z upwards.
		m.yaw			= atan2( itCloud->pose_mean.y, itCloud->pose_mean.x );
		m.pitch			= -sin( itCloud->pose_mean.z/m.range );
		m.landmarkID	= itCloud->ID;

		outObs.sensedData.push_back( m );
	} // end for
} // end StereoObs2BRObs

/* -------------------------------------------------------
				computeStereoRectificationMaps
   ------------------------------------------------------- */
void vision::computeStereoRectificationMaps(
    const TCamera &cam1, const TCamera &cam2,
    const mrpt::poses::CPose3D &rightCameraPose,
    void *outMap1x, void *outMap1y,
    void *outMap2x, void *outMap2y )
{
    ASSERT_( cam1.ncols == cam2.ncols && cam1.nrows == cam2.nrows );

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x211

    cv::Mat *mapx1, *mapy1, *mapx2, *mapy2;
    mapx1 = static_cast<cv::Mat*>(outMap1x);
    mapy1 = static_cast<cv::Mat*>(outMap1y);
    mapx2 = static_cast<cv::Mat*>(outMap2x);
    mapy2 = static_cast<cv::Mat*>(outMap2y);

    const int resX = cam1.ncols;
    const int resY = cam1.nrows;

    CMatrixDouble44 hMatrix;
    rightCameraPose.getHomogeneousMatrix( hMatrix );

	double rcTrans[3];
	rcTrans[0] = hMatrix(0,3);
	rcTrans[1] = hMatrix(1,3);
	rcTrans[2] = hMatrix(2,3);

    double m1[3][3];
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            m1[i][j] = hMatrix(i,j);

    double ipl[3][3], ipr[3][3], dpl[5], dpr[5];
    for( unsigned int i = 0; i < 3; ++i )
        for( unsigned int j = 0; j < 3; ++j )
        {
            ipl[i][j] = cam1.intrinsicParams(i,j);
            ipr[i][j] = cam2.intrinsicParams(i,j);
        }

    for( unsigned int i = 0; i < 5; ++i )
    {
        dpl[i] = cam1.dist[i];
        dpr[i] = cam2.dist[i];
    }

    // WITH OLD OPENCV VERSION
    // *****************************************************************
    /** /
    CvMat R = cvMat( 3, 3, CV_64F, &m1 );
    CvMat T = cvMat( 3, 1, CV_64F, &rcTrans );

    CvMat K1 = cvMat(3,3,CV_64F,ipl);
    CvMat K2 = cvMat(3,3,CV_64F,ipr);
    CvMat D1 = cvMat(1,5,CV_64F,dpl);
    CvMat D2 = cvMat(1,5,CV_64F,dpr);

    double _R1[3][3], _R2[3][3], _P1[3][4], _P2[3][4];
    CvMat R1 = cvMat(3,3,CV_64F,_R1);
    CvMat R2 = cvMat(3,3,CV_64F,_R2);
    CvMat P1 = cvMat(3,4,CV_64F,_P1);
    CvMat P2 = cvMat(3,4,CV_64F,_P2);

    CvSize imageSize = {resX,resY};
    cvStereoRectify( &K1, &K2, &D1, &D2, imageSize,
        &R, &T,
        &R1, &R2, &P1, &P2, 0, 0 );

    CvMat* mx1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
    CvMat* my1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
    CvMat* mx2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
    CvMat* my2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
    CvMat* img1r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
    CvMat* img2r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );

    cvInitUndistortRectifyMap(&K1,&D1,&R1,&P1,mx1,my1);
    cvInitUndistortRectifyMap(&K2,&D2,&R2,&P2,mx2,my2);

    CImage im1, im2;
    im1.loadFromFile("imgs/leftImage1024.jpg");
    im2.loadFromFile("imgs/rightImage1024.jpg");

    cvRemap( static_cast<IplImage *>( im1.getAsIplImage() ), img1r, mx1, my1 );
    cvRemap( static_cast<IplImage *>( im2.getAsIplImage() ), img2r, mx2, my2 );

    cvSaveImage( "imgs/leftImage1024_rect.jpg", img1r );
    cvSaveImage( "imgs/rightImage1024_rect.jpg", img2r );

    IplImage stub, *dst_img, stub2, *dst_img2;
    dst_img = cvGetImage(img1r, &stub);
    dst_img2 = cvGetImage(img2r, &stub2);

    CImage im1out( dst_img );
    CImage im2out( dst_img2 );
    / * */
    // *****************************************************************

    // WITH NEW OPENCV VERSION
    // *****************************************************************
    /**/
    cv::Mat R( 3, 3, CV_64F, &m1 );
    cv::Mat T( 3, 1, CV_64F, &rcTrans );

    cv::Mat K1(3,3,CV_64F,ipl);
    cv::Mat K2(3,3,CV_64F,ipr);
    cv::Mat D1(1,5,CV_64F,dpl);
    cv::Mat D2(1,5,CV_64F,dpr);

    double _R1[3][3], _R2[3][3], _P1[3][4], _P2[3][4], _Q[4][4];
    cv::Mat R1(3,3,CV_64F,_R1);
    cv::Mat R2(3,3,CV_64F,_R2);
    cv::Mat P1(3,4,CV_64F,_P1);
    cv::Mat P2(3,4,CV_64F,_P2);
    cv::Mat Q(4,4,CV_64F,_Q);

    cv::Size nSize(resX,resY);
    float alpha = 0.0;                  // alpha value: 0.0 = zoom and crop the image so that there's not black areas
    cv::stereoRectify(
        K1, D1,
        K2, D2,
        nSize,
        R, T,
        R1, R2, P1, P2, Q, alpha,
        cv::Size(), 0, 0, 0 );

    cv::Size sz1, sz2;
    cv::initUndistortRectifyMap( K1, D1, R1, P1, cv::Size(resX,resY), CV_32FC1, *mapx1, *mapy1 );
    cv::initUndistortRectifyMap( K2, D2, R2, P2, cv::Size(resX,resY), CV_32FC1, *mapx2, *mapy2 );
    /**/
#else
    THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV = 0 or OpenCV version is < 2.1.1!");
#endif

} // end computeStereoRectificationMaps
