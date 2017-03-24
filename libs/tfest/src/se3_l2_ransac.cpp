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
#include <mrpt/random.h>
#include <mrpt/utils/round.h>
#include <mrpt/math/utils.h>  // linspace()
#include <numeric>

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

/*---------------------------------------------------------------
	                     se3_l2_robust
  ---------------------------------------------------------------*/
bool tfest::se3_l2_robust(
	const mrpt::utils::TMatchingPairList & in_correspondences,
	const TSE3RobustParams               & params,
	TSE3RobustResult                     & results )
{
	MRPT_START

	const size_t N = in_correspondences.size();

	// -------------------------------------------
	// Thresholds
	// -------------------------------------------
	Eigen::Matrix<double,7,1> th;
	th[0] =  // X (meters)
	th[1] = // Y (meters)
	th[2] = params.ransac_threshold_lin; // Z (meters)
	th[3] = // YAW (degrees)
	th[4] = // PITCH (degrees)
	th[5] = params.ransac_threshold_ang;// ROLL (degrees)
	th[6] = params.ransac_threshold_scale; // SCALE

	// -------------------------------------------
	// RANSAC parameters
	// -------------------------------------------
	double min_err = std::numeric_limits<double>::max(); // Minimum error achieved so far
	size_t max_size = 0; // Maximum size of the consensus set so far
	double scale; // Output scale

	const size_t n = params.ransac_minSetSize;  // Minimum number of points to fit the model
	const size_t d = mrpt::utils::round( N*params.ransac_maxSetSizePct ); // Minimum number of points to be considered a good set
	const size_t max_it = params.ransac_nmaxSimulations; // Maximum number of iterations

	ASSERTMSG_(d>=n,"Minimum number of points to be considered a good set is < Minimum number of points to fit the model")

	// -------------------------------------------
	// MAIN loop
	// -------------------------------------------
	for(size_t iterations = 0; iterations < max_it; iterations++ )
	{
		//printf("Iteration %2u of %u", iterations+1, max_it );

		// Generate maybe inliers
		vector_int	rub, mbSet, cSet;
		mrpt::math::linspace( (int)0, (int)N-1, (int)N, rub );
		randomGenerator.permuteVector( rub, mbSet );

		// Compute first inliers output
		TMatchingPairList	mbInliers;
		mbInliers.reserve( n );
		for(size_t i = 0; mbInliers.size()<n && i<N; i++ )
		{
			const size_t idx = mbSet[i];

			// User-provided filter: 
			if (params.user_individual_compat_callback)
			{
				mrpt::tfest::TPotentialMatch pm;
				pm.idx_this  = in_correspondences[ idx ].this_idx;
				pm.idx_other = in_correspondences[ idx ].other_idx;
				if (! (*params.user_individual_compat_callback)(pm,params.user_individual_compat_callback_userdata))
					continue; // Skip this one!
			}

			mbInliers.push_back( in_correspondences[ idx ] );
			cSet.push_back( idx );
		}

		// Check minimum number:
		if (cSet.size()<n)
		{
			if (params.verbose)
				std::cerr << "[tfest::se3_l2_robust] Iter " << iterations <<": It was not possible to find the min no of (compatible) matching pairs.\n";
			continue; // Try again
		}

		CPose3DQuat mbOutQuat;
		const bool res = mrpt::tfest::se3_l2( mbInliers, mbOutQuat, scale, params.forceScaleToUnity );
		if (!res) { std::cerr << "[tfest::se3_l2_robust] tfest::se3_l2() returned false for tentative subset during RANSAC iteration!\n"; continue; }

		// Maybe inliers Output
		const CPose3D mbOut = CPose3D(mbOutQuat);
		CVectorFloat mbOut_vec(7);
		mbOut_vec[0] = mbOut.x();
		mbOut_vec[1] = mbOut.y();
		mbOut_vec[2] = mbOut.z();

		mbOut_vec[3] = mbOut.yaw();
		mbOut_vec[4] = mbOut.pitch();
		mbOut_vec[5] = mbOut.roll();

		mbOut_vec[6] = scale;

		// Inner loop: for each point NOT in the maybe inliers
		for(size_t k = n; k < N; k++ )
		{
			const size_t idx =  mbSet[k];

			// User-provided filter: 
			if (params.user_individual_compat_callback)
			{
				mrpt::tfest::TPotentialMatch pm;
				pm.idx_this  = in_correspondences[ idx ].this_idx;
				pm.idx_other = in_correspondences[ idx ].other_idx;
				if (! (*params.user_individual_compat_callback)(pm,params.user_individual_compat_callback_userdata))
					continue; // Skip this one!
			}

			// Consensus set: Maybe inliers + new point
			CPose3DQuat csOutQuat;
			mbInliers.push_back( in_correspondences[ idx ] ); // Insert
			const bool res = mrpt::tfest::se3_l2( mbInliers, csOutQuat, scale, params.forceScaleToUnity );
			mbInliers.erase( mbInliers.end()-1 ); // Erase

			if (!res) { std::cerr << "[tfest::se3_l2_robust] tfest::se3_l2() returned false for tentative subset during RANSAC iteration!\n"; continue; }

			// Is this point a supporter of the initial inlier group?
			const CPose3D csOut = CPose3D(csOutQuat);

			if( fabs( mbOut_vec[0] - csOut.x() ) < th[0] && fabs( mbOut_vec[1] - csOut.y() ) < th[1] &&
				fabs( mbOut_vec[2] - csOut.z() ) < th[2] && fabs( mbOut_vec[3] - csOut.yaw() ) < th[3] &&
				fabs( mbOut_vec[4] - csOut.pitch() ) < th[4] && fabs( mbOut_vec[5] - csOut.roll() ) < th[5] &&
				fabs( mbOut_vec[6] - scale ) < th[6] )
			{
				// Inlier detected -> add to the inlier list
				cSet.push_back( idx );
			} // end if INLIERS
			else
			{
				//cout << " It " << iterations << " - RANSAC Outlier Detected: " << k << endl;
			}
		} // end 'inner' for

		// Test cSet size
		if( cSet.size() >= d )
		{
			// Good set of points found
			TMatchingPairList	cSetInliers;
			cSetInliers.resize( cSet.size() );
			for( unsigned int m = 0; m < cSet.size(); m++ )
				cSetInliers[m] = in_correspondences[ cSet[m] ];

			// Compute output: Consensus Set + Initial Inliers Guess
			CPose3DQuat cIOutQuat;
			const bool res = mrpt::tfest::se3_l2( cSetInliers, cIOutQuat, scale, params.forceScaleToUnity ); // Compute output
			ASSERTMSG_(res, "tfest::se3_l2() returned false for tentative subset during RANSAC iteration!")

			// Compute error for consensus_set
			const CPose3D cIOut = CPose3D(cIOutQuat);
			const double err = std::sqrt( square( mbOut_vec[0] - cIOut.x() ) + square( mbOut_vec[1] - cIOut.y() ) + square( mbOut_vec[2] - cIOut.z() ) +
							   square( mbOut_vec[3] - cIOut.yaw() ) + square( mbOut_vec[4] - cIOut.pitch() ) + square( mbOut_vec[5] - cIOut.roll() ) +
							   square( mbOut_vec[6] - scale ));

			// Is the best set of points so far?
			if( err < min_err && cSet.size() >= max_size )
			{
				min_err                = err;
				max_size               = cSet.size();
				results.transformation = cIOutQuat;
				results.scale          = scale;
				results.inliers_idx    = cSet;
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
		std::cerr << "[se3_l2_robust] maximum size is == 0!\n";
		return false;
	}

	MRPT_END

	return true;

} // end se3_l2_robust()

