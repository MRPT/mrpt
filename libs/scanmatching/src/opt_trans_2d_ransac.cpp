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

					robustRigidTransformation

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
	unsigned int				ransac_min_nSimulations
	)
{
	size_t								i,N = in_correspondences.size();
	unsigned int						maxThis=0, maxOther=0;
	CPosePDFGaussian					temptativeEstimation, referenceEstimation;
	TMatchingPairList::iterator		matchIt;
	std::vector<bool>					alreadySelectedThis;
	std::vector<bool>					alreadySelectedOther;

//#define DEBUG_OUT

	MRPT_START

	// Asserts:
	if( N < ransac_minSetSize )
	{
		// Nothing to do!
		out_transformation.clear();
		return;
	}

	// Find the max. index of "this" and "other:
	for (matchIt=in_correspondences.begin();matchIt!=in_correspondences.end(); matchIt++)
	{
		maxThis = max(maxThis , matchIt->this_idx  );
		maxOther= max(maxOther, matchIt->other_idx );
	}

	// Fill out 2 arrays indicating whether each element has a correspondence:
	std::vector<bool>	hasCorrThis(maxThis+1,false);
	std::vector<bool>	hasCorrOther(maxOther+1,false);
	unsigned int		howManyDifCorrs = 0;
	//for (i=0;i<N;i++)
	for (matchIt=in_correspondences.begin();matchIt!=in_correspondences.end(); matchIt++)
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

	// The max. number of corrs!
	//ransac_maxSetSize = min(ransac_maxSetSize, max(2,(howManyDifCorrs-1)));
	ransac_maxSetSize = min(ransac_maxSetSize, max((unsigned int)2,howManyDifCorrs) );

	//printf("howManyDifCorrs=%u  ransac_maxSetSize=%u\n",howManyDifCorrs,ransac_maxSetSize);

	//ASSERT_( ransac_maxSetSize>=ransac_minSetSize );
	if ( ransac_maxSetSize < ransac_minSetSize )
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

//#define AVOID_MULTIPLE_CORRESPONDENCES

#ifdef  AVOID_MULTIPLE_CORRESPONDENCES
	unsigned 					k;
	// Find duplicated landmarks (from SIFT features with different descriptors,etc...)
	//   this is to avoid establishing multiple correspondences for the same physical point!
	// ------------------------------------------------------------------------------------------------
	std::vector<vector_int>		listDuplicatedLandmarksThis(maxThis+1);
	ASSERT_(N>=1);
	for (k=0;k<N-1;k++)
	{
		vector_int		duplis;
		for (unsigned j=k;j<N-1;j++)
		{
			if ( in_correspondences[k].this_x == in_correspondences[j].this_x &&
				 in_correspondences[k].this_y == in_correspondences[j].this_y &&
				 in_correspondences[k].this_z == in_correspondences[j].this_z )
					duplis.push_back(in_correspondences[j].this_idx);
		}
		listDuplicatedLandmarksThis[in_correspondences[k].this_idx] = duplis;
	}

	std::vector<vector_int>		listDuplicatedLandmarksOther(maxOther+1);
	for (k=0;k<N-1;k++)
	{
		vector_int		duplis;
		for (unsigned j=k;j<N-1;j++)
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
	std::vector<size_t> 	corrsIdxs( N), corrsIdxsPermutation;
	for (i=0;i<N;i++) corrsIdxs[i]= i;

	// If we put this out of the loop, each correspondence will be used just ONCE!
	/**/
	alreadySelectedThis.clear();
	alreadySelectedThis.resize(maxThis+1,false);
	alreadySelectedOther.clear();
	alreadySelectedOther.resize(maxOther+1, false);
	/**/

	//~ CPosePDFGaussian	temptativeEstimation;

	// -------------------------
	//		The RANSAC loop
	// -------------------------
	size_t largest_consensus_yet = 0; // Used for dynamic # of steps

	const bool use_dynamic_iter_number = ransac_nSimulations==0;
	if (use_dynamic_iter_number)
	{
		ASSERT_(probability_find_good_model>0 && probability_find_good_model<1);
		// Set an initial # of iterations:
		ransac_nSimulations = 10;  // It doesn't matter actually, since will be changed in the first loop
	}


	i = 0;
	while (i<ransac_nSimulations)  // ransac_nSimulations can be dynamic
	{
		i++;

		TMatchingPairList		subSet,temptativeSubSet;

		// Select a subset of correspondences at random:
		if (ransac_algorithmForLandmarks)
		{
			alreadySelectedThis.clear();
			alreadySelectedThis.resize(maxThis+1,false);
			alreadySelectedOther.clear();
			alreadySelectedOther.resize(maxOther+1, false);
		}
		else
		{
			// For points: Do not repeat the corrs, and take the numer of corrs as weights
		}

		// Try to build a subsetof "ransac_maxSetSize" (maximum) elements that achieve consensus:
		// ------------------------------------------------------------------------------------------
		// First: Build a permutation of the correspondences to pick from it sequentially:
		randomGenerator.permuteVector(corrsIdxs,corrsIdxsPermutation );

		for (unsigned int j=0;j<ransac_maxSetSize;j++)
		{
			ASSERT_(j<corrsIdxsPermutation.size())

			size_t	idx = corrsIdxsPermutation[j];

			matchIt = in_correspondences.begin() + idx;

			ASSERT_( matchIt->this_idx < alreadySelectedThis.size() );
			ASSERT_( matchIt->other_idx < alreadySelectedOther.size() );

			if ( !(alreadySelectedThis [ matchIt->this_idx ] &&
					alreadySelectedOther[ matchIt->other_idx]) )
//			if ( !alreadySelectedThis [ matchIt->this_idx ] &&
//			     !alreadySelectedOther[ matchIt->other_idx]  )
			{
				// mark as "selected" for this pair not to be selected again:
				//  ***NOTE***: That the expresion of the "if" above requires the
				//  same PAIR not to be selected again, but one of the elements
				//  may be selected again with a diferent matching! This improves the
				//  robustness and posibilities of the algorithm! (JLBC - NOV/2006)

#ifndef  AVOID_MULTIPLE_CORRESPONDENCES
				alreadySelectedThis[ matchIt->this_idx ]= true;
				alreadySelectedOther[ matchIt->other_idx ] = true;
#else
				for (vector_int::iterator it1 = listDuplicatedLandmarksThis[matchIt->this_idx].begin();it1!=listDuplicatedLandmarksThis[matchIt->this_idx].end();it1++)
					alreadySelectedThis[ *it1 ] = true;
				for (vector_int::iterator it2 = listDuplicatedLandmarksOther[matchIt->other_idx].begin();it2!=listDuplicatedLandmarksOther[matchIt->other_idx].end();it2++)
					alreadySelectedOther[ *it2 ] = true;
#endif
				if (subSet.size()<2)
				{
					// ------------------------------------------------------------------------------------------------------
					// If we are within the first two correspondences, just add them to the subset:
					// ------------------------------------------------------------------------------------------------------
					subSet.push_back( *matchIt );

					if (subSet.size()==2)
					{
						temptativeSubSet = subSet;
						// JLBC: Modification DEC/2007: If we leave only ONE correspondence in the ref. set
						//  the algorithm will be pretty much sensible to reject bad correspondences:
						temptativeSubSet.erase( temptativeSubSet.begin() + (temptativeSubSet.size() -1) );

						// Perform estimation:
						scanmatching::leastSquareErrorRigidTransformation(
							subSet,
							referenceEstimation.mean,
							&referenceEstimation.cov );
						// Normalized covariance: scale!
						referenceEstimation.cov *= square(normalizationStd);

						// Additional filter:
						//  If the correspondences as such the transformation has a high ambiguity, we discard it!
						if ( referenceEstimation.cov(2,2)>=square(DEG2RAD(5.0f)) )
						{
						 	// Remove this correspondence & try again with a different pair:
						 	subSet.erase( subSet.begin() + (subSet.size() -1) );
						}
						else
						{
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

					// Compute the temptative new estimation (matchIt will be removed after the test!):
					temptativeSubSet.push_back( *matchIt );

					scanmatching::leastSquareErrorRigidTransformation(
						temptativeSubSet,
						temptativeEstimation.mean,
						&temptativeEstimation.cov );
					// Normalized covariance: scale!
					temptativeEstimation.cov *= square(normalizationStd);

					// Additional filter:
					//  If the correspondences as such the transformation has a high ambiguity, we discard it!
					if ( temptativeEstimation.cov(2,2)<square(DEG2RAD(5.0f)) )
					{
						// ASSERT minimum covariance!!
						/*temptativeEstimation.cov(0,0) = max( temptativeEstimation.cov(0,0), square( 0.03f ) );
						temptativeEstimation.cov(1,1) = max( temptativeEstimation.cov(1,1), square( 0.03f ) );

						referenceEstimation.cov(0,0) = max( referenceEstimation.cov(0,0), square( 0.03f ) );
						referenceEstimation.cov(1,1) = max( referenceEstimation.cov(1,1), square( 0.03f ) ); */

						temptativeEstimation.cov(2,2) = max( temptativeEstimation.cov(2,2), square( DEG2RAD(0.2) ) );
						referenceEstimation.cov(2,2) = max( referenceEstimation.cov(2,2), square( DEG2RAD(0.2) ) );

						// Test for compatibility:
						bool passTest;

						if (ransac_algorithmForLandmarks)
						{
							// Compatibility test: Mahalanobis distance between Gaussians:
							double	mahaDist = temptativeEstimation.mahalanobisDistanceTo( referenceEstimation );
							passTest = mahaDist < ransac_mahalanobisDistanceThreshold;
						}
						else
						{
							// Compatibility test: Euclidean distances
							double diffXY = referenceEstimation.mean.distanceTo( temptativeEstimation.mean );
							double diffPhi = fabs( math::wrapToPi( referenceEstimation.mean.phi() - temptativeEstimation.mean.phi() ) );
							passTest  = diffXY < 0.02f && diffPhi < DEG2RAD(2.0f);

							//FILE *f=os::fopen("hist.txt","at");
							//fprintf(f,"%f %f\n",diffXY, RAD2DEG(diffPhi) );
							//fclose(f);
						}

						if ( passTest )
						{
							// OK, consensus passed!!
							subSet.push_back( *matchIt );
							referenceEstimation = temptativeEstimation;
						}
						else
						{
							// Test failed!
							//printf("Discarded!:\n");
							//std::cout << "temptativeEstimation:" << temptativeEstimation << " referenceEstimation:" << referenceEstimation << " mahaDist:" << mahaDist << "\n";
						}
					}
					else
					{
						// Test failed!
						//printf("Discarded! stdPhi=%f\n",RAD2DEG(sqrt(temptativeEstimation.cov(2,2))));
					}

					// Remove the temporaryy added last correspondence:
					temptativeSubSet.pop_back();

				} // end else "normal case"

			} // end "if" the randomly selected item is new

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
				// This is an alrady added mode:
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

		// Dynamic # of steps:
		if (use_dynamic_iter_number)
		{
			const size_t ninliers = subSet.size();
			if (largest_consensus_yet<ninliers )
			{
				largest_consensus_yet = ninliers;

				// Update estimate of N, the number of trials to ensure we pick,
				// with probability p, a data set with no outliers.
				const double fracinliers =  ninliers/static_cast<double>(howManyDifCorrs); // corrsIdxs.size());
				double pNoOutliers = 1 -  pow(fracinliers,static_cast<double>(2.0 /*minimumSizeSamplesToFit*/ ));

				pNoOutliers = std::max( std::numeric_limits<double>::epsilon(), pNoOutliers);  // Avoid division by -Inf
				pNoOutliers = std::min(1.0 - std::numeric_limits<double>::epsilon() , pNoOutliers); // Avoid division by 0.
				// Number of
				ransac_nSimulations = log(1-probability_find_good_model)/log(pNoOutliers);

				if (ransac_nSimulations<ransac_min_nSimulations)
					ransac_nSimulations = ransac_min_nSimulations;

				//if (verbose)
					cout << "[scanmatching::RANSAC] Iter #" << i << " Estimated number of iters: " << ransac_nSimulations << "  pNoOutliers = " << pNoOutliers << " #inliers: " << ninliers << endl;

			}
		}

		// Save the largest subset:
		if (out_largestSubSet!=NULL)
		{
			if (subSet.size()>out_largestSubSet->size())
			{
				*out_largestSubSet = subSet;
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
		printf("maxThis=%u, maxOther=%u\n",static_cast<unsigned int>(maxThis), static_cast<unsigned int>(maxOther)); \
		printf("N=%u\n",static_cast<unsigned int>(N)); \
		printf("Saving '_debug_in_correspondences.txt'..."); \
		in_correspondences.dumpToFile("_debug_in_correspondences.txt"); \
		printf("Ok\n"); \
		printf("Saving '_debug_out_transformation.txt'..."); \
		out_transformation.saveToTextFile("_debug_out_transformation.txt"); \
		printf("Ok\n"); );

}
