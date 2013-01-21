/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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


//#define AVOID_MULTIPLE_CORRESPONDENCES

// mark this pair as "selected" so it won't be picked again:
void markAsPicked(
	const TMatchingPair & c, 
	std::vector<bool> &alreadySelectedThis, 
	std::vector<bool> &alreadySelectedOther
#ifdef  AVOID_MULTIPLE_CORRESPONDENCES
	, const std::vector<vector_int> &listDuplicatedLandmarksThis 
#endif
	)
{
	ASSERTDEB_( c.this_idx < alreadySelectedThis.size() );
	ASSERTDEB_( c.other_idx < alreadySelectedOther.size() );

#ifndef  AVOID_MULTIPLE_CORRESPONDENCES
	alreadySelectedThis[ c.this_idx ]= true;
	alreadySelectedOther[ c.other_idx ] = true;
#else
	for (vector_int::iterator it1 = listDuplicatedLandmarksThis[c.this_idx].begin();it1!=listDuplicatedLandmarksThis[c.this_idx].end();it1++)
		alreadySelectedThis[ *it1 ] = true;
	for (vector_int::iterator it2 = listDuplicatedLandmarksOther[c.other_idx].begin();it2!=listDuplicatedLandmarksOther[c.other_idx].end();it2++)
		alreadySelectedOther[ *it2 ] = true;
#endif
}


/*---------------------------------------------------------------

					robustRigidTransformation

  The technique was described in the paper:
    J.L. Blanco, J. González-Jimenez and J.A. Fernandez-Madrigal. 
	"A robust, multi-hypothesis approach to matching occupancy grid maps". 
	Robotica, available on CJO2013. doi:10.1017/S0263574712000732. 
	http://journals.cambridge.org/action/displayAbstract?aid=8815308

 This works as follows:
	- Repeat "ransac_nSimulations" times:
		- Randomly pick TWO correspondences from the set "in_correspondences".
		- Compute the associated rigid transformation.
		- For "ransac_maxSetSize" randomly selected correspondences, test for "consensus" with the current group:
			- If if is compatible (ransac_maxErrorXY, ransac_maxErrorPHI), grow the "consensus set"
			- If not, do not add it.
  ---------------------------------------------------------------*/
void  scanmatching::robustRigidTransformation(
	TMatchingPairList	&in_correspondences,
	poses::CPosePDFSOG				&out_transformation,
	float							normalizationStd,
	unsigned int					ransac_minSetSize,
	unsigned int					ransac_maxSetSize,
	float						ransac_mahalanobisDistanceThreshold,
	unsigned int					ransac_nSimulations,
	TMatchingPairList	*out_largestSubSet,
	bool						ransac_fuseByCorrsMatch,
	float						ransac_fuseMaxDiffXY,
	float						ransac_fuseMaxDiffPhi,
	bool						ransac_algorithmForLandmarks,
	double 						probability_find_good_model,
	unsigned int				ransac_min_nSimulations,
	const bool                  verbose
	)
{
	const size_t nCorrs = in_correspondences.size();

//#define DEBUG_OUT

	MRPT_START

	// Asserts:
	if( nCorrs < ransac_minSetSize )
	{
		// Nothing to do!
		out_transformation.clear();
		if (out_largestSubSet!=NULL)
		{
			TMatchingPairList		emptySet;
			*out_largestSubSet = emptySet;
		}
		return;
	}

	// Find the max. index of "this" and "other:
	unsigned int maxThis=0, maxOther=0;
	for (TMatchingPairList::iterator matchIt=in_correspondences.begin();matchIt!=in_correspondences.end(); ++matchIt)
	{
		maxThis = max(maxThis , matchIt->this_idx  );
		maxOther= max(maxOther, matchIt->other_idx );
	}

	// Fill out 2 arrays indicating whether each element has a correspondence:
	std::vector<bool>	hasCorrThis(maxThis+1,false);
	std::vector<bool>	hasCorrOther(maxOther+1,false);
	unsigned int		howManyDifCorrs = 0;
	//for (i=0;i<nCorrs;i++)
	for (TMatchingPairList::iterator matchIt=in_correspondences.begin();matchIt!=in_correspondences.end(); ++matchIt)
	{
		if (!hasCorrThis[matchIt->this_idx] &&
			!hasCorrOther[matchIt->other_idx] )
		{
			hasCorrThis[matchIt->this_idx] = true;
			hasCorrOther[matchIt->other_idx] = true;
			howManyDifCorrs++;
		}
	}

	// Clear the set of output particles:
	out_transformation.clear();

	// If there are less different correspondences than the minimum required, quit:
	if ( howManyDifCorrs < ransac_minSetSize )
	{
		// Nothing we can do here!!! :~$
		if (out_largestSubSet!=NULL)
		{
			TMatchingPairList		emptySet;
			*out_largestSubSet = emptySet;
		}

		out_transformation.clear();
		return;
	}


#ifdef  AVOID_MULTIPLE_CORRESPONDENCES
	unsigned 					k;
	// Find duplicated landmarks (from SIFT features with different descriptors,etc...)
	//   this is to avoid establishing multiple correspondences for the same physical point!
	// ------------------------------------------------------------------------------------------------
	std::vector<vector_int>		listDuplicatedLandmarksThis(maxThis+1);
	ASSERT_(nCorrs>=1);
	for (k=0;k<nCorrs-1;k++)
	{
		vector_int		duplis;
		for (unsigned j=k;j<nCorrs-1;j++)
		{
			if ( in_correspondences[k].this_x == in_correspondences[j].this_x &&
				 in_correspondences[k].this_y == in_correspondences[j].this_y &&
				 in_correspondences[k].this_z == in_correspondences[j].this_z )
					duplis.push_back(in_correspondences[j].this_idx);
		}
		listDuplicatedLandmarksThis[in_correspondences[k].this_idx] = duplis;
	}

	std::vector<vector_int>		listDuplicatedLandmarksOther(maxOther+1);
	for (k=0;k<nCorrs-1;k++)
	{
		vector_int		duplis;
		for (unsigned j=k;j<nCorrs-1;j++)
		{
			if ( in_correspondences[k].other_x == in_correspondences[j].other_x &&
				 in_correspondences[k].other_y == in_correspondences[j].other_y &&
				 in_correspondences[k].other_z == in_correspondences[j].other_z )
					duplis.push_back(in_correspondences[j].other_idx);
		}
		listDuplicatedLandmarksOther[in_correspondences[k].other_idx] = duplis;
	}
#endif

	std::deque<TMatchingPairList>	alreadyAddedSubSets;
	std::vector<size_t> 	corrsIdxs( nCorrs), corrsIdxsPermutation;
	for (size_t i=0;i<nCorrs;i++) corrsIdxs[i]= i;

	CPosePDFGaussian    referenceEstimation;
	CPoint2DPDFGaussian pt_this;

	const double ransac_consistency_test_chi2_quantile = 0.99;
	const double chi2_thres_dim1 = mrpt::math::chi2inv(ransac_consistency_test_chi2_quantile, 1);

	// -------------------------
	//		The RANSAC loop
	// -------------------------
	size_t largest_consensus_yet = 0; // Used for dynamic # of steps
	double largestSubSet_sqerr = 0;

	const bool use_dynamic_iter_number = ransac_nSimulations==0;
	if (use_dynamic_iter_number)
	{
		ASSERT_(probability_find_good_model>0 && probability_find_good_model<1);
		// Set an initial # of iterations:
		ransac_nSimulations = 10;  // It doesn't matter actually, since will be changed in the first loop
	}

	std::vector<bool> alreadySelectedThis, alreadySelectedOther;
	
	if (!ransac_algorithmForLandmarks) 
	{
		alreadySelectedThis.assign(maxThis+1,false);
		alreadySelectedOther.assign(maxOther+1, false);
	}
	// else -> It will be done anyway inside the for() below


	for (size_t i = 0;i<ransac_nSimulations; i++) // ransac_nSimulations can be dynamic
	{
		TMatchingPairList subSet;

		// Select a subset of correspondences at random:
		if (ransac_algorithmForLandmarks)
		{
			alreadySelectedThis.assign(maxThis+1,false);
			alreadySelectedOther.assign(maxOther+1, false);
		}
		else
		{
			// For points: Do not repeat the corrs, and take the number of corrs as weights
		}

		// Try to build a subsetof "ransac_maxSetSize" (maximum) elements that achieve consensus:
		// ------------------------------------------------------------------------------------------
		// First: Build a permutation of the correspondences to pick from it sequentially:
		randomGenerator.permuteVector(corrsIdxs,corrsIdxsPermutation );

		for (unsigned int j=0;j<ransac_maxSetSize;j++)
		{
			ASSERTDEB_(j<corrsIdxsPermutation.size())

			const size_t idx = corrsIdxsPermutation[j];

			const TMatchingPair & corr_j = in_correspondences[idx];

			// Don't pick the same features twice!
			if (alreadySelectedThis [corr_j.this_idx] || alreadySelectedOther[corr_j.other_idx]) 
				continue;

			if (subSet.size()<2)
			{
				// ------------------------------------------------------------------------------------------------------
				// If we are within the first two correspondences, just add them to the subset:
				// ------------------------------------------------------------------------------------------------------
				subSet.push_back( corr_j );
				markAsPicked(corr_j, alreadySelectedThis,alreadySelectedOther);

				if (subSet.size()==2)
				{
					// Consistency Test: From 

					// Check the feasibility of this pair "idx1"-"idx2":
					//  The distance between the pair of points in MAP1 must be very close
					//   to that of their correspondences in MAP2:
					const double corrs_dist1 = mrpt::math::distanceBetweenPoints(
						subSet[0].this_x, subSet[0].this_y,
						subSet[1].this_x, subSet[1].this_y );

					const double corrs_dist2 = mrpt::math::distanceBetweenPoints(
						subSet[0].other_x, subSet[0].other_y,
						subSet[1].other_x, subSet[1].other_y );

					// Is is a consistent possibility?
					//  We use a chi2 test (see paper for the derivation)
					const double corrs_dist_chi2 =
						square( square(corrs_dist1)-square(corrs_dist2) ) /
						(8.0* square(normalizationStd) * (square(corrs_dist1)+square(corrs_dist2)) );

					bool is_acceptable = (corrs_dist_chi2 < chi2_thres_dim1  );

					if (is_acceptable)
					{
						// Perform estimation:
						scanmatching::leastSquareErrorRigidTransformation(
							subSet,
							referenceEstimation.mean,
							&referenceEstimation.cov );
						// Normalized covariance: scale!
						referenceEstimation.cov *= square(normalizationStd);

						// Additional filter:
						//  If the correspondences as such the transformation has a high ambiguity, we discard it!
						is_acceptable = ( referenceEstimation.cov(2,2)<square(DEG2RAD(5.0f)) );
					}

					if (!is_acceptable)
					{
						// Remove this correspondence & try again with a different pair:
						subSet.erase( subSet.begin() + (subSet.size() -1) );
					}
					else
					{
						// Only mark as picked if we're really keeping it:
						markAsPicked(corr_j, alreadySelectedThis,alreadySelectedOther);
					}
				}
			}
			else
			{
				// ------------------------------------------------------------------------------------------------------
				// The normal case:
				//  - test for "consensus" with the current group:
				//		- If it is compatible (ransac_maxErrorXY, ransac_maxErrorPHI), grow the "consensus set"
				//		- If not, do not add it.
				// ------------------------------------------------------------------------------------------------------

				// Test for the mahalanobis distance between:
				//  "referenceEstimation (+) point_other" AND "point_this"
				referenceEstimation.composePoint( mrpt::math::TPoint2D(corr_j.other_x,corr_j.other_y), pt_this);

				const double maha_dist = pt_this.mahalanobisDistanceToPoint(corr_j.this_x,corr_j.this_y);

				const bool passTest = maha_dist < ransac_mahalanobisDistanceThreshold;

				if ( passTest )
				{
					// OK, consensus passed:
					subSet.push_back( corr_j );
					markAsPicked(corr_j, alreadySelectedThis,alreadySelectedOther);
				}
				// else -> Test failed

			} // end else "normal case"


		} // end for j

		// Save the estimation result as a "particle", only if the subSet contains
		//  "ransac_minSetSize" elements at least:
		if (subSet.size()>=ransac_minSetSize)
		{
			// If this subset was previously added to the SOG, just increment its weight
			//  and do not add a new mode:
			int		indexFound = -1;

			// JLBC Added DEC-2007: An alternative (optional) method to fuse Gaussian modes:
			if (!ransac_fuseByCorrsMatch)
			{
				// Find matching by approximate match in the X,Y,PHI means
				// -------------------------------------------------------------------
				// Recompute referenceEstimation from all the corrs:
				scanmatching::leastSquareErrorRigidTransformation(
					subSet,
					referenceEstimation.mean,
					&referenceEstimation.cov );
				// Normalized covariance: scale!
				referenceEstimation.cov *= square(normalizationStd);
				for (size_t i=0;i<out_transformation.size();i++)
				{
					double diffXY = out_transformation.get(i).mean.distanceTo( referenceEstimation.mean );
					double diffPhi = fabs( math::wrapToPi( out_transformation.get(i).mean.phi() - referenceEstimation.mean.phi() ) );
					if ( diffXY < ransac_fuseMaxDiffXY && diffPhi < ransac_fuseMaxDiffPhi )
					{
						//printf("Match by distance found: distXY:%f distPhi=%f deg\n",diffXY,RAD2DEG(diffPhi));
						indexFound = i;
						break;
					}
				}
			}
			else
			{
				// Find matching mode by exact match in the list of correspondences:
				// -------------------------------------------------------------------
				// Sort "subSet" in order to compare them easily!
				//std::sort( subSet.begin(), subSet.end() );

				// Try to find matching corrs:
				for (size_t i=0;i<alreadyAddedSubSets.size();i++)
				{
					if ( subSet == alreadyAddedSubSets[i] )
					{
						indexFound = i;
						break;
					}
				}
			}

			if (indexFound!=-1)
			{
				// This is an already added mode:
				if (ransac_algorithmForLandmarks)
						out_transformation.get(indexFound).log_w = log(1+ exp(out_transformation.get(indexFound).log_w));
				else	out_transformation.get(indexFound).log_w = log(subSet.size()+ exp(out_transformation.get(indexFound).log_w));
			}
			else
			{
				// Add a new mode to the SOG:
				alreadyAddedSubSets.push_back( subSet );

				CPosePDFSOG::TGaussianMode	newSOGMode;
				if (ransac_algorithmForLandmarks)
						newSOGMode.log_w = 0; //log(1);
				else	newSOGMode.log_w = log(static_cast<double>(subSet.size()));

				scanmatching::leastSquareErrorRigidTransformation(
					subSet,
					newSOGMode.mean,
					&newSOGMode.cov );

				// Normalized covariance: scale!
				newSOGMode.cov *= square(normalizationStd);

				// Add a new mode to the SOG!
				out_transformation.push_back(newSOGMode);
			}
		} // end if subSet.size()>=ransac_minSetSize

		const size_t ninliers = subSet.size();
		if (largest_consensus_yet<ninliers )
		{
			largest_consensus_yet = ninliers;

			// Dynamic # of steps:
			if (use_dynamic_iter_number)
			{
				// Update estimate of nCorrs, the number of trials to ensure we pick,
				// with probability p, a data set with no outliers.
				const double fracinliers =  ninliers/static_cast<double>(howManyDifCorrs); // corrsIdxs.size());
				double pNoOutliers = 1 -  pow(fracinliers,static_cast<double>(2.0 /*minimumSizeSamplesToFit*/ ));

				pNoOutliers = std::max( std::numeric_limits<double>::epsilon(), pNoOutliers);  // Avoid division by -Inf
				pNoOutliers = std::min(1.0 - std::numeric_limits<double>::epsilon() , pNoOutliers); // Avoid division by 0.
				// Number of
				ransac_nSimulations = log(1-probability_find_good_model)/log(pNoOutliers);

				ransac_nSimulations = std::max(ransac_nSimulations, ransac_min_nSimulations);

				if (verbose)
					cout << "[scanmatching::RANSAC] Iter #" << i << " Estimated number of iters: " << ransac_nSimulations << "  pNoOutliers = " << pNoOutliers << " #inliers: " << ninliers << endl;
			}

		}

		// Save the largest subset:
		if (out_largestSubSet!=NULL)
		{
			CPose2D estPose;
			scanmatching::leastSquareErrorRigidTransformation( subSet, estPose);

			double this_subset_sqerr = 0;
			for (size_t k=0;k<subSet.size();k++)
			{
				double gx,gy;
				estPose.composePoint( 
					subSet[k].other_x, subSet[k].other_y,
					gx,gy);

				this_subset_sqerr += mrpt::math::distanceSqrBetweenPoints<double>( subSet[k].this_x,subSet[k].this_y, gx,gy );
			}

			if (subSet.size()>out_largestSubSet->size() || 
			    ( subSet.size()==out_largestSubSet->size() && this_subset_sqerr<largestSubSet_sqerr) 
			   )
			{
				if (verbose)
					cout << "[scanmatching::RANSAC] Iter #" << i << " Better subset with " << subSet.size() << " inliers found: sqErr=" << this_subset_sqerr << endl;

				*out_largestSubSet = subSet;
				largestSubSet_sqerr = this_subset_sqerr;
			}
		}

#ifdef DEBUG_OUT
		printf("[RANSAC] Sim #%i/%i \t--> |subSet|=%u \n",
			(int)i,
			(int)ransac_nSimulations,
			(unsigned)subSet.size()
			);
#endif
	} // end for i

	// Set the weights of the particles to sum the unity:
	out_transformation.normalizeWeights();

	// Now the estimation is in the particles set!
	// Done!

	MRPT_END_WITH_CLEAN_UP( \
		printf("nCorrs=%u\n",static_cast<unsigned int>(nCorrs)); \
		printf("Saving '_debug_in_correspondences.txt'..."); \
		in_correspondences.dumpToFile("_debug_in_correspondences.txt"); \
		printf("Ok\n"); \
		printf("Saving '_debug_out_transformation.txt'..."); \
		out_transformation.saveToTextFile("_debug_out_transformation.txt"); \
		printf("Ok\n"); );

}
