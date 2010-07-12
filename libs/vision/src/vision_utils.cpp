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
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/geometry.h>


#include "do_opencv_includes.h"


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::gui;
using namespace std;


#ifdef MRPT_OS_WINDOWS
    #include <process.h>
    #include <windows.h>		// TODO: This is temporary!!!
#endif

const int NOT_ASIG = 0;
const int ASG_FEAT = 1;
const int AMB_FEAT = 2;

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
	CFeatureList						&leftList,		// The left of matched features (matches must be ordered!)
	CFeatureList						&rightList,		// The right of matched features (matches must be ordered!)
	const vision::TStereoSystemParams	&param,			// Parameters for the stereo system
	mrpt::slam::CLandmarksMap			&landmarks )	// Output map of 3D landmarks
{
	MRPT_START;
	ASSERT_( leftList.size() == rightList.size() );

	landmarks.clear();								// Assert that the output CLandmarksMap is clear

	CFeatureList::iterator		itListL, itListR;
	float								stdPixel2	= square( param.stdPixel );
	float								stdDisp2	= square( param.stdDisp );

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
							L.extractRow( i-1, vAux );						// Extract the proper row
							myPoint[0] = meanA[0] + vAux[0];
							myPoint[1] = meanA[1] + vAux[1];
							myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							L.extractRow( (i-Na)-1, vAux );					// Extract the proper row
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
							L.extractRow( i-1, vAux );						// Extract the proper row
							myPoint = meanA + vAux;
							//myPoint[0] = meanA[0] + vAux[0];
							//myPoint[1] = meanA[1] + vAux[1];
							//myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							L.extractRow( (i-Na)-1, vAux );					// Extract the proper row
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
	CMatchedFeatureList					&mfList,		// The set of matched features
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
							L.extractRow( i-1, vAux );						// Extract the proper row
							myPoint[0] = meanA[0] + vAux[0];
							myPoint[1] = meanA[1] + vAux[1];
							myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							L.extractRow( (i-Na)-1, vAux );					// Extract the proper row
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
							L.extractRow( i-1, vAux );						// Extract the proper row
							myPoint = meanA + vAux;
							//myPoint[0] = meanA[0] + vAux[0];
							//myPoint[1] = meanA[1] + vAux[1];
							//myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							L.extractRow( (i-Na)-1, vAux );					// Extract the proper row
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
	std.clear();
	mean.clear();

	std.resize(2,0);
	mean.resize(2,0);

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
		//CV_TM_CCOEFF_NORMED
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
/*************************************** END FAMD ********************************************/
/*-------------------------------------------------------------
						matchFeatures
-------------------------------------------------------------*/
size_t vision::matchFeatures( const CFeatureList &list1,
							const CFeatureList &list2,
							CMatchedFeatureList &matches,
							const TMatchingOptions &options )
{
	CTicTac				tictac;

	// Clear the output structure
	matches.clear();
	MRPT_START;

	// Preliminary comprobations
	ASSERT_( list1.get_type() == list2.get_type() );		// Both lists must be of the same type

	switch( options.matching_method )
	{
	case TMatchingOptions::mmDescriptorSIFT:
	{
		/***********************************************************************
		   SIFT Features -> Matching by Euclidean distance between descriptors
		************************************************************************/
		CFeatureList::const_iterator	itList1, itList2;	// Iterators for the lists

		float							distDesc;			// EDD
		float							minDist1 = 1e5;		// Minimum EDD
		float							minDist2 = 1e5;		// Second minimum EDD
		TFeatureID						minIdx = 0;			// Index of the closest feature

		map<TFeatureID,unsigned int>	featAssigned;		// Right feature status: 0 (idle) 1 (assigned) 2 (ambiguous)
		//unsigned int					featIdx;			// Counter

		// For each feature in list1 ...
		for( itList1 = list1.begin(); itList1 != list1.end(); itList1++ )
		{
			minDist1 = 1e5;
			minDist2 = 1e5;
			minIdx	 = 0;
			for( itList2 = list2.begin()/*, featIdx = 0*/;
				 itList2 != list2.end();
				 itList2++/*, featIdx++*/ )					// ... compare with all the features in list2.
			{
				// Filter out by epipolar constraint (By now: row checking)
				if( fabs( (*itList1)->y - (*itList2)->y ) < options.epipolar_TH && fabs( (*itList1)->x - (*itList2)->x ) > 0 )
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
						minIdx	 = (*itList2)->ID;
					}
					else if ( distDesc < minDist2 )
						minDist2 = distDesc;
				}
			} // end for 'list2'

			// PROCESS THE RESULTS
			if( minDist1 < options.maxEDD_TH && 					// The minimum distance must be below a threshold
			  ( minDist1/minDist2) < options.EDD_RATIO )			// The difference between it and the second minimum distance must be over a threshold
			{
				switch( featAssigned[ minIdx ] )
				{
				case NOT_ASIG: // Right feature is free
				{
					// Create the match and insert it into the matched list
					std::pair<CFeaturePtr,CFeaturePtr> mPair( *itList1, list2.getByID( minIdx ) );
					matches.push_back( mPair );
					featAssigned[ minIdx ] = ASG_FEAT;						// Set the right feature status to ASSIGNED (1)
					break;
				} // end FREE

				case ASG_FEAT: // Right feature is already assigned
				{
					// Not create the match and remove the old one
					CMatchedFeatureList::iterator			itVec;				// We set an iterator for the vector
					for( itVec = matches.begin(); itVec != matches.end(); )		// ... we search it into the vector
					{
						if( itVec->second->ID == minIdx )
						{
							itVec = matches.erase( itVec );
							break;
						}
						else
							itVec++;
					} // end for
					featAssigned[ minIdx ] = AMB_FEAT;						// Set the right feature status to AMBIGUOUS (2)
					break;
				} // end ASSIGNED

				default:{break;}
				} // end switch
			} // end if
		} // end for
		break;
	} // end case mmDescriptorSIFT

	case TMatchingOptions::mmCorrelation:
	{
		///***********************************************************************
		//   Matching by Cross Correlation
		//************************************************************************/
		CFeatureList::const_iterator	itList1, itList2;	// Iterators for the lists

		size_t							u,v;				// Coordinates of the peak
		double							res;				// Value of the peak

		double							maxCC1 = 0;			// Maximum CC
		double							maxCC2 = 0;			// Second maximum CC
		TFeatureID						minIdx = 0;			// Index of the closest feature

		map<TFeatureID,unsigned int>	featAssigned;		// Right feature status: 0 (idle) 1 (assigned) 2 (ambiguous)
		// unsigned int					featIdx;			// Counter

		// For each feature in list1 ...
		//unsigned int k = 0;

		for( itList1 = list1.begin(); itList1 != list1.end(); itList1++/*, k++*/ )
		{
			maxCC1	= 0;
			maxCC2	= 0;
			minIdx	= 0;
			for( itList2 = list2.begin()/*, featIdx = 0*/;
				 itList2 != list2.end();
				 itList2++/*, featIdx++*/ )			// ... compare with all the features in list2.
			{

				if( fabs( (*itList1)->y - (*itList2)->y ) < options.epipolar_TH && ( (*itList1)->x - (*itList2)->x ) > 0 ) // Quick filtering of bad correspondences
				{
					// Ensure that both features have patches
					ASSERT_(	(*itList1)->patch.getHeight() > 0 && (*itList1)->patch.getWidth() > 0 &&
								(*itList2)->patch.getHeight() > 0 && (*itList2)->patch.getWidth() > 0 );
					vision::openCV_cross_correlation( (*itList1)->patch, (*itList2)->patch, u, v, res );

					// Search for the two maximum values
					if( res > maxCC1 )
					{
						maxCC2 = maxCC1;
						maxCC1 = res;
						minIdx = (*itList2)->ID;
					}
					else if( res > maxCC2 )
						maxCC2 = res;

				} // end if EPIPOLAR CONSTRAINT
			} // end for "2"

			// PROCESS THE RESULTS
			if( maxCC1 > options.minCC_TH && 				// The minimum distance must be below a threshold
			   (maxCC2/maxCC1) < options.rCC_TH )			// The difference between it and the second minimum distance must be over a threshold
			{

				switch( featAssigned[ minIdx ] )
				{
				case NOT_ASIG: // Right feature is free
				{
					// Create the match and insert it into the matched list
					std::pair<CFeaturePtr,CFeaturePtr> mPair( *itList1, list2.getByID( minIdx ) );
					matches.push_back( mPair );
					featAssigned[ minIdx ] = ASG_FEAT;							// Set the right feature status to ASSIGNED
					break;
				} // end FREE

				case ASG_FEAT: // Right feature is already assigned
				{
					// Not create the match and remove the old one
					CMatchedFeatureList::iterator			itVec;				// We set an iterator for the vector
					for( itVec = matches.begin(); itVec != matches.end(); )		// ... we search it into the vector
					{
						if( itVec->second->ID == minIdx )
						{
							itVec = matches.erase( itVec );
							break;
						}
						else
							itVec++;
					} // end for
					featAssigned[ minIdx ] = AMB_FEAT;							// Set the right feature status to AMBIGUOUS
					break;
				} // end ASSIGNED

				default:{break;}
				} // end switch
			} // end if
		} // end for
		break;
		} // end case mmCorrelation

		case TMatchingOptions::mmDescriptorSURF:
		{
			cout << "To do ... [MATCH SURF FEATURES]" << endl;
			break; // end case featSURF
		}
		default:
		{
			THROW_EXCEPTION( "Invalid type of feature when matching" );
		}
	} // end switch

	return matches.size();

	MRPT_END;
}

/*-------------------------------------------------------------
						matchFeatures2
-------------------------------------------------------------*/
size_t vision::matchFeatures2( const CFeatureList &list1,
							const CFeatureList &list2,
							CMatchedFeatureList &matches,
							const TMatchingOptions &options )
{
	// Clear the output structure
	MRPT_START;
	matches.clear();

	// Preliminary comprobations
	ASSERT_( list1.get_type() == list2.get_type() );	// Both lists must be of the same type

	CFeatureList::const_iterator	itList1, itList2;	// Iterators for the lists

	// For SIFT & SURF
	float							distDesc;			// EDD or EDSD
	float							minDist1 = 1e5;		// Minimum EDD or EDSD
	float							minDist2 = 1e5;		// Second minimum EDD or EDSD
	TFeatureID						minIdx = 0;			// Index of the closest feature

	// For Harris
	double							maxCC1 = 0;			// Maximum CC
	double							maxCC2 = 0;			// Second maximum CC

	// For SAD
	double							minSAD1, minSAD2;

	map<TFeatureID,unsigned int>	featAssigned;		// Right feature status: 0 (idle) 1 (assigned) 2 (ambiguous)

	// For each feature in list1 ...
	for( itList1 = list1.begin(); itList1 != list1.end(); itList1++ )
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
		minIdx	 = 0;

		for( itList2 = list2.begin(); itList2 != list2.end(); itList2++ )		// ... compare with all the features in list2.
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

			bool c1 = options.useEpipolarRestriction ? fabs(d) < options.epipolar_TH : true;			// Use epipolar restriction
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
						minIdx	 = (*itList2)->ID;
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
					ASSERT_(	(*itList1)->patch.getHeight() > 0 && (*itList1)->patch.getWidth() > 0 &&
								(*itList2)->patch.getHeight() > 0 && (*itList2)->patch.getWidth() > 0 );

					vision::openCV_cross_correlation( (*itList1)->patch, (*itList2)->patch, u, v, res );

					// Search for the two maximum values
					if( res > maxCC1 )
					{

						maxCC2 = maxCC1;
						maxCC1 = res;
						minIdx = (*itList2)->ID;
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
						minIdx	 = (*itList2)->ID;
					}
					else if ( distDesc < minDist2 )
						minDist2 = distDesc;

					break; // end case featSURF
				} // end mmDescriptorSURF

				case TMatchingOptions::mmSAD:
				{
					// Ensure that both features have patches
					ASSERT_(	(*itList1)->patch.getHeight() > 0 && (*itList1)->patch.getWidth() > 0 &&
								(*itList2)->patch.getHeight() > 0 && (*itList2)->patch.getWidth() > 0 );
#if !MRPT_HAS_OPENCV
	THROW_EXCEPTION("MRPT has been compiled without OpenCV")
#else
					IplImage *aux1, *aux2;

					if( (*itList1)->patch.isColor() && (*itList2)->patch.isColor() )
					{

						IplImage* preAux1 = (IplImage*)(*itList1)->patch.getAsIplImage();
						IplImage* preAux2 = (IplImage*)(*itList2)->patch.getAsIplImage();

						aux1 = cvCreateImage( cvSize( (*itList1)->patch.getHeight(), (*itList1)->patch.getWidth() ), 8, 1 );
						aux2 = cvCreateImage( cvSize( (*itList2)->patch.getHeight(), (*itList2)->patch.getWidth() ), 8, 1 );

						cvCvtColor( preAux1, aux1, CV_BGR2GRAY );
						cvCvtColor( preAux2, aux2, CV_BGR2GRAY );
					}
					else
					{
						aux1 = (IplImage*)(*itList1)->patch.getAsIplImage();
						aux2 = (IplImage*)(*itList2)->patch.getAsIplImage();
					}

					// Substract its mean value
					double m1 = 0, m2 = 0;
					for( unsigned int ii = 0; ii < (unsigned int)aux1->imageSize; ++ii )
						m1 += aux1->imageData[ii];
					m1 /= (double)aux1->imageSize;

					for( unsigned int ii = 0; ii < (unsigned int)aux2->imageSize; ++ii )
						m2 += aux2->imageData[ii];
					m2 /= (double)aux2->imageSize;

					double res = 0;
					for( unsigned int ii = 0; ii < (unsigned int)aux1->imageSize; ++ii )
						res += fabs( fabs((double)aux1->imageData[ii]-m1) - fabs((double)aux2->imageData[ii]-m2) );

					res = res/(255.0f*aux1->imageSize);

					if( res < minSAD1 )
					{
						minSAD2 = minSAD1;
						minSAD1 = res;
						minIdx  = (*itList2)->ID;
					}
					else if ( res < minSAD2 )
						minSAD2 = res;

					//cvReleaseImage( &aux1 );
					//cvReleaseImage( &aux2 );
#endif
					break;
					//cv::Mat out = cv::Mat( (*itList1)->patch.getHeight(), (*itList1)->patch.getWidth(), CV_8UC1 );
					//cv::absdiff(
					//	cv::Mat::Mat( (IplImage*)( (*itList1)->patch.getAsIplImage() ), false ),
					//	cv::Mat::Mat( (IplImage*)( (*itList2)->patch.getAsIplImage() ), false ),
					//	out );
					//for( unsigned
				} // end mmSAD
				} // end switch
			} // end if
		} // end for 'list2' (right features)

		bool cond1, cond2;
		switch(options.matching_method)
		{
			case TMatchingOptions::mmDescriptorSIFT:
				cond1 = minDist1 < options.maxEDD_TH;						// Maximum Euclidean Distance between SIFT descriptors (EDD)
				cond2 = (minDist1/minDist2) < options.EDD_RATIO;			// Ratio between the two lowest EDSD
				break;
			case TMatchingOptions::mmCorrelation:
				cond1 = maxCC1 > options.minCC_TH;							// Minimum cross correlation value
				cond2 = (maxCC2/maxCC1) < options.rCC_TH;					// Ratio between the two highest cross correlation values
				break;
			case TMatchingOptions::mmDescriptorSURF:
				cond1 = minDist1 < options.maxEDSD_TH;						// Maximum Euclidean Distance between SURF descriptors (EDSD)
				cond2 = (minDist1/minDist2) < options.EDSD_RATIO;			// Ratio between the two lowest EDSD
				break;
			case TMatchingOptions::mmSAD:
				cond1 = minSAD1 < options.minSAD_TH;
				cond2 = (minSAD1/minSAD2) < options.SAD_RATIO;
				break;
			default:
				THROW_EXCEPTION("Invalid value of 'matching_method'");
		};

		// PROCESS THE RESULTS
		if( cond1 && cond2 )					// The minimum distance must be below a threshold
		{
			switch( featAssigned[ minIdx ] )
			{
			case NOT_ASIG: // Right feature is free
			{
				// Create the match and insert it into the matched list
				std::pair<CFeaturePtr,CFeaturePtr> mPair( *itList1, list2.getByID( minIdx ) );
				matches.push_back( mPair );
				featAssigned[ minIdx ] = ASG_FEAT;						// Set the right feature status to ASSIGNED (1)
				break;
			} // end FREE

			case ASG_FEAT: // Right feature is already assigned
			{
				// Not create the match and remove the old one
				CMatchedFeatureList::iterator			itVec;				// We set an iterator for the vector
				for( itVec = matches.begin(); itVec != matches.end(); )		// ... we search it into the vector
				{
					if( itVec->second->ID == minIdx )
					{
						itVec = matches.erase( itVec );
						break;
					}
					else
						itVec++;
				} // end for
				featAssigned[ minIdx ] = AMB_FEAT;						// Set the right feature status to AMBIGUOUS (2)
				break;
			} // end ASSIGNED

			default:{break;}
			} // end switch
		} // end if
	} // end for 'list1' (left features)

	return matches.size();

	MRPT_END;
}

/*-------------------------------------------------------------
			TStereoSystemParams: constructor
-------------------------------------------------------------*/
TStereoSystemParams::TStereoSystemParams() :
	uncPropagation(Prop_Linear),
	K(),
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
	out.printf("\n----------- [vision::TStereoSystemParams] ------------ \n\n");

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
	minSAD_TH	( 0.4 ),
	SAD_RATIO	( 0.5 )

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

	epipolar_TH		= iniFile.read_float(section.c_str(),"epipolar_TH",epipolar_TH);
	maxEDD_TH		= iniFile.read_float(section.c_str(),"maxEDD_TH",maxEDD_TH);
	EDD_RATIO		= iniFile.read_float(section.c_str(),"minDIF_TH",EDD_RATIO);
	minCC_TH		= iniFile.read_float(section.c_str(),"minCC_TH",minCC_TH);
	minDCC_TH		= iniFile.read_float(section.c_str(),"minDCC_TH",minDCC_TH);
	rCC_TH			= iniFile.read_float(section.c_str(),"rCC_TH",rCC_TH);
	maxEDSD_TH		= iniFile.read_float(section.c_str(),"maxEDSD_TH",maxEDSD_TH);
	EDSD_RATIO		= iniFile.read_float(section.c_str(),"EDSD_RATIO",EDSD_RATIO);
	minSAD_TH		= iniFile.read_float(section.c_str(),"minSAD_TH",minSAD_TH);
	SAD_RATIO		= iniFile.read_float(section.c_str(),"SAD_RATIO",SAD_RATIO);

} // end TMatchingOptions::loadFromConfigFile

/*---------------------------------------------------------------
					TMatchingOptions: dumpToTextStream
  ---------------------------------------------------------------*/
void  TMatchingOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [vision::TMatchingOptions] ------------ \n\n");

	out.printf("Matching method	\t= ");
	switch( matching_method )
	{
	case mmCorrelation:
		out.printf("Cross Correlation\n");
		break;
	case mmDescriptorSIFT:
		out.printf("SIFT descriptor\n");
		break;
	case mmDescriptorSURF:
		out.printf("SURF descriptor\n");
		break;
	case mmSAD:
		out.printf("SAD\n");
		break;
	} // end switch
	out.printf("Epipolar Thres.\t\t= %f\n", epipolar_TH);
	out.printf("Max. EDD Thres [SIFT].\t\t= %f\n", maxEDD_TH);
	out.printf("EDD Ratio [SIFT]\t\t= %f\n", EDD_RATIO);
	out.printf("Min. CC Thres.\t\t= %f\n", minCC_TH);
	out.printf("Min. Dif. CC Thres.\t= %f\n", minDCC_TH);
	out.printf("Max. Ratio CC Thres.\t= %f\n", rCC_TH);
	out.printf("EDD Ratio [SURF]\t\t= %f\n", maxEDSD_TH);
	out.printf("Min. CC Thres. [SURF]\t\t= %f\n", EDSD_RATIO);
	out.printf("Min. Dif. SAD Thres.\t= %f\n", minSAD_TH);
	out.printf("Max. Ratio SAD Thres.\t= %f\n", SAD_RATIO);
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
