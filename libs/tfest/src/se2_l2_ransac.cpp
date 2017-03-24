/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "tfest-precomp.h"  // Precompiled headers

#include <mrpt/tfest/se2.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/distributions.h>
#include <mrpt/utils/CTimeLogger.h>

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
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
    J.L. Blanco, J. Gonzalez-Jimenez and J.A. Fernandez-Madrigal.
	"A robust, multi-hypothesis approach to matching occupancy grid maps".
	Robotica, available on CJO2013. doi:10.1017/S0263574712000732.
	http://journals.cambridge.org/action/displayAbstract?aid=8815308

 This works as follows:
	- Repeat "results.ransac_iters" times:
		- Randomly pick TWO correspondences from the set "in_correspondences".
		- Compute the associated rigid transformation.
		- For "ransac_maxSetSize" randomly selected correspondences, test for "consensus" with the current group:
			- If if is compatible (ransac_maxErrorXY, ransac_maxErrorPHI), grow the "consensus set"
			- If not, do not add it.
  ---------------------------------------------------------------*/
bool tfest::se2_l2_robust(
	const mrpt::utils::TMatchingPairList &in_correspondences,
	const double               normalizationStd,
	const TSE2RobustParams   & params,
	TSE2RobustResult         & results
	)
{
//#define DO_PROFILING

#ifdef DO_PROFILING
	CTimeLogger timlog;
#endif

	const size_t nCorrs = in_correspondences.size();

	// Default: 2 * normalizationStd ("noise level")
	const double MAX_RMSE_TO_END = (params.max_rmse_to_end<=0 ? 2*normalizationStd : params.max_rmse_to_end);

	MRPT_START

	// Asserts:
	if( nCorrs < params.ransac_minSetSize )
	{
		// Nothing to do!
		results.transformation.clear();
		results.largestSubSet = TMatchingPairList();
		return false;
	}


#ifdef DO_PROFILING
	timlog.enter("ransac.find_max*");
#endif
	// Find the max. index of "this" and "other:
	unsigned int maxThis=0, maxOther=0;
	for (TMatchingPairList::const_iterator matchIt=in_correspondences.begin();matchIt!=in_correspondences.end(); ++matchIt)
	{
		maxThis = max(maxThis , matchIt->this_idx  );
		maxOther= max(maxOther, matchIt->other_idx );
	}
#ifdef DO_PROFILING
	timlog.leave("ransac.find_max*");
#endif


#ifdef DO_PROFILING
	timlog.enter("ransac.count_unique_corrs");
#endif

	// Fill out 2 arrays indicating whether each element has a correspondence:
	std::vector<bool>	hasCorrThis(maxThis+1,false);
	std::vector<bool>	hasCorrOther(maxOther+1,false);
	unsigned int		howManyDifCorrs = 0;
	for (TMatchingPairList::const_iterator matchIt=in_correspondences.begin();matchIt!=in_correspondences.end(); ++matchIt)
	{
		if (!hasCorrThis[matchIt->this_idx] &&
			!hasCorrOther[matchIt->other_idx] )
		{
			hasCorrThis[matchIt->this_idx] = true;
			hasCorrOther[matchIt->other_idx] = true;
			howManyDifCorrs++;
		}
	}
#ifdef DO_PROFILING
	timlog.leave("ransac.count_unique_corrs");
#endif

	// Clear the set of output particles:
	results.transformation.clear();

	// If there are less different correspondences than the minimum required, quit:
	if ( howManyDifCorrs < params.ransac_minSetSize )
	{
		// Nothing we can do here!!! :~$
		results.transformation.clear();
		results.largestSubSet = TMatchingPairList();
		return false;
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

	CPosePDFGaussian    referenceEstimation;
	CPoint2DPDFGaussian pt_this;

	const double ransac_consistency_test_chi2_quantile = 0.99;
	const double chi2_thres_dim1 = mrpt::math::chi2inv(ransac_consistency_test_chi2_quantile, 1);

	// -------------------------
	//		The RANSAC loop
	// -------------------------
	size_t largest_consensus_yet = 0; // Used for dynamic # of steps
	double largestSubSet_RMSE = std::numeric_limits<double>::max();

	results.ransac_iters = 	params.ransac_nSimulations;
	const bool use_dynamic_iter_number = results.ransac_iters==0;
	if (use_dynamic_iter_number)
	{
		ASSERT_(params.probability_find_good_model>0 && params.probability_find_good_model<1);
		// Set an initial # of iterations:
		results.ransac_iters = 10;  // It doesn't matter actually, since will be changed in the first loop
	}

	std::vector<bool> alreadySelectedThis, alreadySelectedOther;

	if (!params.ransac_algorithmForLandmarks)
	{
		alreadySelectedThis.assign(maxThis+1,false);
		alreadySelectedOther.assign(maxOther+1, false);
	}
	// else -> It will be done anyway inside the for() below
	
	// First: Build a permutation of the correspondences to pick from it sequentially:
	std::vector<size_t> corrsIdxs(nCorrs), corrsIdxsPermutation;
	for (size_t i=0;i<nCorrs;i++) corrsIdxs[i]= i;

	size_t iter_idx;
	for (iter_idx = 0;iter_idx<results.ransac_iters; iter_idx++) // results.ransac_iters can be dynamic
	{
#ifdef DO_PROFILING
		CTimeLoggerEntry tle(timlog,"ransac.iter");
#endif

#ifdef DO_PROFILING
		timlog.enter("ransac.permute");
#endif
		randomGenerator.permuteVector(corrsIdxs,corrsIdxsPermutation );

#ifdef DO_PROFILING
		timlog.leave("ransac.permute");
#endif

		TMatchingPairList subSet;

		// Select a subset of correspondences at random:
		if (params.ransac_algorithmForLandmarks)
		{
#ifdef DO_PROFILING
			timlog.enter("ransac.reset_selection_marks");
#endif
			alreadySelectedThis.assign(maxThis+1,false);
			alreadySelectedOther.assign(maxOther+1, false);
#ifdef DO_PROFILING
			timlog.leave("ransac.reset_selection_marks");
#endif
		}
		else
		{
			// For points: Do not repeat the corrs, and take the number of corrs as weights
		}

		// Try to build a subsetof "ransac_maxSetSize" (maximum) elements that achieve consensus:
		// ------------------------------------------------------------------------------------------
#ifdef DO_PROFILING
		timlog.enter("ransac.inner_loops");
#endif
		for (unsigned int j=0;j<nCorrs && subSet.size()<params.ransac_maxSetSize;j++)
		{
			const size_t idx = corrsIdxsPermutation[j];

			const TMatchingPair & corr_j = in_correspondences[idx];

			// Don't pick the same features twice!
			if (alreadySelectedThis [corr_j.this_idx] || alreadySelectedOther[corr_j.other_idx])
				continue;

			// Additional user-provided filter: 
			if (params.user_individual_compat_callback)
			{
				mrpt::tfest::TPotentialMatch pm;
				pm.idx_this  = corr_j.this_idx;
				pm.idx_other = corr_j.other_idx;
				if (! (*params.user_individual_compat_callback)(pm,params.user_individual_compat_callback_userdata))
					continue; // Skip this one!
			}

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
						tfest::se2_l2( subSet, referenceEstimation );
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
#ifdef DO_PROFILING
				timlog.enter("ransac.test_consistency");
#endif

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

				const bool passTest = maha_dist < params.ransac_mahalanobisDistanceThreshold;

				if ( passTest )
				{
					// OK, consensus passed:
					subSet.push_back( corr_j );
					markAsPicked(corr_j, alreadySelectedThis,alreadySelectedOther);
				}
				// else -> Test failed

#ifdef DO_PROFILING
				timlog.leave("ransac.test_consistency");
#endif
			} // end else "normal case"

		} // end for j
#ifdef DO_PROFILING
		timlog.leave("ransac.inner_loops");
#endif


		const bool has_to_eval_RMSE = (subSet.size()>=params.ransac_minSetSize);

		// Compute the RMSE of this matching and the corresponding transformation (only if we'll use this value below)
		double this_subset_RMSE = 0;
		if (has_to_eval_RMSE)
		{
#ifdef DO_PROFILING
			CTimeLoggerEntry tle(timlog,"ransac.comp_rmse");
#endif

			// Recompute referenceEstimation from all the corrs:
			tfest::se2_l2(subSet,referenceEstimation);
			// Normalized covariance: scale!
			referenceEstimation.cov *= square(normalizationStd);

			for (size_t k=0;k<subSet.size();k++)
			{
				double gx,gy;
				referenceEstimation.mean.composePoint(
					subSet[k].other_x, subSet[k].other_y,
					gx,gy);

				this_subset_RMSE += mrpt::math::distanceSqrBetweenPoints<double>( subSet[k].this_x,subSet[k].this_y, gx,gy );
			}
			this_subset_RMSE /= std::max( static_cast<size_t>(1), subSet.size() );
		}
		else
		{
			this_subset_RMSE = std::numeric_limits<double>::max();
		}

		// Save the estimation result as a "particle", only if the subSet contains
		//  "ransac_minSetSize" elements at least:
		if (subSet.size()>=params.ransac_minSetSize)
		{
			// If this subset was previously added to the SOG, just increment its weight
			//  and do not add a new mode:
			int		indexFound = -1;

			// JLBC Added DEC-2007: An alternative (optional) method to fuse Gaussian modes:
			if (!params.ransac_fuseByCorrsMatch)
			{
				// Find matching by approximate match in the X,Y,PHI means
				// -------------------------------------------------------------------
				for (size_t i=0;i<results.transformation.size();i++)
				{
					double diffXY = results.transformation.get(i).mean.distanceTo( referenceEstimation.mean );
					double diffPhi = fabs( math::wrapToPi( results.transformation.get(i).mean.phi() - referenceEstimation.mean.phi() ) );
					if ( diffXY < params.ransac_fuseMaxDiffXY && diffPhi < params.ransac_fuseMaxDiffPhi )
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
				if (params.ransac_algorithmForLandmarks)
						results.transformation.get(indexFound).log_w = log(1+ exp(results.transformation.get(indexFound).log_w));
				else	results.transformation.get(indexFound).log_w = log(subSet.size()+ exp(results.transformation.get(indexFound).log_w));
			}
			else
			{
				// Add a new mode to the SOG:
				alreadyAddedSubSets.push_back( subSet );

				CPosePDFSOG::TGaussianMode	newSOGMode;
				if (params.ransac_algorithmForLandmarks)
						newSOGMode.log_w = 0; //log(1);
				else	newSOGMode.log_w = log(static_cast<double>(subSet.size()));

				newSOGMode.mean = referenceEstimation.mean;
				newSOGMode.cov  = referenceEstimation.cov;

				// Add a new mode to the SOG!
				results.transformation.push_back(newSOGMode);
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
				results.ransac_iters = log(1-params.probability_find_good_model)/log(pNoOutliers);

				results.ransac_iters = std::max(results.ransac_iters, params.ransac_min_nSimulations);

				if (params.verbose)
					cout << "[tfest::RANSAC] Iter #" << iter_idx << ":est. # iters=" << results.ransac_iters << " pNoOutliers=" << pNoOutliers << " #inliers: " << ninliers << endl;
			}

		}

		// Save the largest subset:
		if (subSet.size()>=params.ransac_minSetSize && this_subset_RMSE<largestSubSet_RMSE )
		{
			if (params.verbose)
				cout << "[tfest::RANSAC] Iter #" << iter_idx << " Better subset: " << subSet.size() << " inliers, RMSE=" << this_subset_RMSE << endl;

			results.largestSubSet = subSet;
			largestSubSet_RMSE = this_subset_RMSE;
		}

		// Is the found subset good enough?
		if (subSet.size()>=params.ransac_minSetSize &&
			this_subset_RMSE<MAX_RMSE_TO_END)
		{
				break; // end RANSAC iterations.
		}

#ifdef DO_PROFILING
	timlog.leave("ransac.iter");
#endif
	} // end for each iteration

	if (params.verbose)
		cout << "[tfest::RANSAC] Finished after " << iter_idx << " iterations.\n";


	// Set the weights of the particles to sum the unity:
	results.transformation.normalizeWeights();

	// Done!

	MRPT_END_WITH_CLEAN_UP( \
		printf("nCorrs=%u\n",static_cast<unsigned int>(nCorrs)); \
		printf("Saving '_debug_in_correspondences.txt'..."); \
		in_correspondences.dumpToFile("_debug_in_correspondences.txt"); \
		printf("Ok\n"); \
		printf("Saving '_debug_results.transformation.txt'..."); \
		results.transformation.saveToTextFile("_debug_results.transformation.txt"); \
		printf("Ok\n"); );

	return true;
}
