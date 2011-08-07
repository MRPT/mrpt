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
	leastSquareErrorRigidTransformation6D
  ---------------------------------------------------------------*/
bool  scanmatching::leastSquareErrorRigidTransformation6DRANSAC(
	const TMatchingPairList	&in_correspondences,
	CPose3D								&out_transformation,
	double								&out_scale,
	vector_int							&out_inliers_idx,
	const unsigned int					ransac_minSetSize,
	const unsigned int					ransac_nmaxSimulations,
	const double						ransac_maxSetSizePct,
	const bool 							forceScaleToUnity
	)
{
	MRPT_START

	unsigned int N = (unsigned int)in_correspondences.size();

	// -------------------------------------------
	// Thresholds
	// -------------------------------------------
	vector_float	th(7);
	th[0] = 0.05;			// X (meters)
	th[1] = 0.05;			// Y (meters)
	th[2] = 0.05;			// Z (meters)

	th[3] = DEG2RAD(1);		// YAW (degrees)
	th[4] = DEG2RAD(1);		// PITCH (degrees)
	th[5] = DEG2RAD(1);		// ROLL (degrees)

	th[6] = 0.03;			// SCALE

	// -------------------------------------------
	// RANSAC parameters
	// -------------------------------------------
	unsigned int	n, d, max_it, iterations;
	double			min_err = 1e2;						// Minimum error achieved so far
	unsigned int	max_size = 0;						// Maximum size of the consensus set so far
	double			scale;								// Output scale

	n		= ransac_minSetSize;						// Minimum number of points to fit the model
	d		= round( N*0.5/*ransac_maxSetSizePct*/ );			// Minimum number of points to be considered a good set
	max_it	= ransac_nmaxSimulations;					// Maximum number of iterations

	if( d < n )
	{
	    cout << "Minimum number of points to be considered a good set is < Minimum number of points to fit the model" << endl;
	    return false;
	}


	// -------------------------------------------
	// MAIN loop
	// -------------------------------------------
	for( iterations = 0; iterations < max_it; iterations++ )
	{
		//printf("Iteration %2u of %u", iterations+1, max_it );

		// Generate maybe inliers
		vector_int	rub, mbSet, cSet;
		mrpt::math::linspace( (int)0, (int)N-1, (int)N, rub );
		randomGenerator.permuteVector( rub, mbSet );

		// Compute first inliers output
		CPose3D							mbOut;
		vector_float					mbOut_vec(7);
		TMatchingPairList	mbInliers;
		mbInliers.resize( n );
		for( unsigned int i = 0; i < n; i++ )
		{
			mbInliers[i] = in_correspondences[ mbSet[i] ];
			cSet.push_back( mbSet[i] );
		}

		bool res = leastSquareErrorRigidTransformation6D( mbInliers, mbOut, scale, forceScaleToUnity );
		if( !res )
		{
		    cout << "leastSquareErrorRigidTransformation6D returned false" << endl;
		    return false;
		}

		// Maybe inliers Output
		mbOut_vec[0] = mbOut.x();
		mbOut_vec[1] = mbOut.y();
		mbOut_vec[2] = mbOut.z();

		mbOut_vec[3] = mbOut.yaw();
		mbOut_vec[4] = mbOut.pitch();
		mbOut_vec[5] = mbOut.roll();

		mbOut_vec[6] = scale;

		// Inner loop: for each point NOT in the maybe inliers
		for( unsigned int k = n; k < N; k++ )
		{
			CPose3D		csOut;

			// Consensus set: Maybe inliers + new point
			mbInliers.push_back( in_correspondences[ mbSet[k] ] );							// Insert
			bool res = leastSquareErrorRigidTransformation6D( mbInliers, csOut, scale, forceScaleToUnity );	// Compute
			mbInliers.erase( mbInliers.end()-1 );											// Erase
			if( !res )
            {
                cout << "leastSquareErrorRigidTransformation6D returned false" << endl;
                return false;
            }

			// Is this point a supporter of the initial inlier group?
			if( fabs( mbOut_vec[0] - csOut.x() ) < th[0] && fabs( mbOut_vec[1] - csOut.y() ) < th[1] &&
				fabs( mbOut_vec[2] - csOut.z() ) < th[2] && fabs( mbOut_vec[3] - csOut.yaw() ) < th[3] &&
				fabs( mbOut_vec[4] - csOut.pitch() ) < th[4] && fabs( mbOut_vec[5] - csOut.roll() ) < th[5] &&
				fabs( mbOut_vec[6] - scale ) < th[6] )
			{
				// Inlier detected -> add to the inlier list
				cSet.push_back( mbSet[k] );
			} // end if INLIERS
			else
			{
				//cout << " It " << iterations << " - RANSAC Outlier Detected: " << k << endl;
			}
		} // end 'inner' for

		// Test cSet size
		if( cSet.size() > d )
		{
			// Good set of points found
			TMatchingPairList	cSetInliers;
			cSetInliers.resize( cSet.size() );
			for( unsigned int m = 0; m < cSet.size(); m++ )
				cSetInliers[m] = in_correspondences[ cSet[m] ];

			// Compute output: Consensus Set + Initial Inliers Guess
			CPose3D		cIOut;
			bool res = leastSquareErrorRigidTransformation6D( cSetInliers, cIOut, scale, forceScaleToUnity );	// Compute output
			if( !res )
            {
                cout << "leastSquareErrorRigidTransformation6D returned false" << endl;
                return false;
            }

			// Compute error for consensus_set
			double err = sqrt( square( mbOut_vec[0] - cIOut.x() ) + square( mbOut_vec[1] - cIOut.y() ) + square( mbOut_vec[2] - cIOut.z() ) +
							   square( mbOut_vec[3] - cIOut.yaw() ) + square( mbOut_vec[4] - cIOut.pitch() ) + square( mbOut_vec[5] - cIOut.roll() ) +
							   square( mbOut_vec[6] - scale ));

			// Is the best set of points so far?
			if( err < min_err && cSet.size() >= max_size )
			{
				min_err						= err;
				max_size					= cSet.size();
				out_transformation			= cIOut;
				out_scale					= scale;
				out_inliers_idx				= cSet;
			} // end if SCALE ERROR
			//printf(" - Consensus set size: %u - Error: %.6f\n", (unsigned int)cSet.size(), err );
		} // end if cSet.size() > d
		else
		{
			//printf(" - Consensus set size: %u - Not big enough!\n", (unsigned int)cSet.size() );
		}
	} // end 'iterations' for

	if( max_size == 0 )
    {
        cout << "maximum size is == 0" << endl;
        return false;
    }

	MRPT_END

	return true;
} // end leastSquareErrorRigidTransformation6DRANSAC


