/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_vision_descriptor_pairing_H
#define mrpt_vision_descriptor_pairing_H

#include <mrpt/vision/types.h>

namespace mrpt
{
	namespace vision
	{
		/** \addtogroup  mrptvision_features
		    @{ */

		/** Search for pairings between two sets of visual descriptors (for now, only SURF 
		  *   and SIFT features are considered).
		  *  Pairings are returned in one of two formats (or both of them simultaneously), 
		  *   depending on the non-NULL output containers present in the call.
		  *
		  * \code
		  *  CFeatureList  feats1, feats2;
		  *  // Populate feature lists [...]
		  *  
		  *  // Create kd-tree for SIFT features of "feats2":
		  *  TSIFTDescriptorsKDTreeIndex<double>  feats2_kdtree(feats2);
		  *  
		  *  // Search correspondences:
		  *  std::vector<vector_size_t>             pairings_1_to_multi_2;
		  *  std::vector<std::pair<size_t,size_t> > pairings_1_to_2;
		  *  mrpt::vision::find_descriptor_pairings(
		  *     &pairings_1_to_multi_2,   // Can be set to NULL if not needed
		  *     &pairings_1_to_2,         // Can be set to NULL if not needed
		  *     feats1, feats2_kdtree,    // The two sets of features
		  *     mrpt::vision::descSIFT    // Select descriptor to use
		  *     // [further optional params]
		  *     );
		  * \endcode
		  * 
		  * \sa TSIFTDescriptorsKDTreeIndex, TSURFDescriptorsKDTreeIndex
		  */
		template <class DESCRIPTOR_KDTREE>
		size_t find_descriptor_pairings(
			std::vector<vector_size_t>             * pairings_1_to_multi_2, 
			std::vector<std::pair<size_t,size_t> > * pairings_1_to_2,
			const CFeatureList                     & feats_img1, 
			const DESCRIPTOR_KDTREE                & feats_img2_kdtree,
			const mrpt::vision::TDescriptorType      descriptor = descSIFT,
			const size_t                             max_neighbors = 4, 
			const double                             max_relative_distance = 1.2,
			const typename DESCRIPTOR_KDTREE::kdtree_t::DistanceType max_distance = std::numeric_limits<typename DESCRIPTOR_KDTREE::kdtree_t::DistanceType>::max()
			)
		{
			MRPT_START
			ASSERT_ABOVEEQ_(max_neighbors,1)
			ASSERT_(pairings_1_to_multi_2!=NULL || pairings_1_to_2!=NULL)

			typedef typename DESCRIPTOR_KDTREE::kdtree_t::ElementType   KDTreeElementType; // The expected data type of elements for the kd-tree
			typedef typename DESCRIPTOR_KDTREE::kdtree_t::DistanceType  KDTreeDistanceType; 

			const size_t N=feats_img1.size();
			if (pairings_1_to_multi_2) pairings_1_to_multi_2->assign(N, vector_size_t()); // Reset output container
			if (pairings_1_to_2) { pairings_1_to_2->clear(); pairings_1_to_2->reserve(N); }

			size_t overall_pairs = 0;

			if (!N) return overall_pairs; // No features -> nothing to do

			if (descriptor==descSIFT) {
				ASSERTMSG_(feats_img1[0]->descriptors.hasDescriptorSIFT(), "Request to match SIFT features but feats_img1 has no SIFT descriptors!")
				ASSERTMSG_(sizeof(KDTreeElementType)==sizeof(feats_img1[0]->descriptors.SIFT[0]),"Incorrect data type kd_tree::ElementType for SIFT (should be uint8_t)")
			}
			else if (descriptor==descSURF) {
				ASSERTMSG_(feats_img1[0]->descriptors.hasDescriptorSURF(), "Request to match SURF features but feats_img1 has no SURF descriptors!")
				ASSERTMSG_(sizeof(KDTreeElementType)==sizeof(feats_img1[0]->descriptors.SURF[0]),"Incorrect data type kd_tree::ElementType for SURF (should be float)")
			}
			else { THROW_EXCEPTION("This function only supports SIFT or SURFT descriptors") }

			std::vector<size_t> indices(max_neighbors);
			std::vector<double> distances(max_neighbors);

			for (size_t i=0;i<N;i++)
			{
				const CFeature::TDescriptors &descs = feats_img1[i]->descriptors;

				const void * ptr_query;
				if (descriptor==descSIFT)      ptr_query = &descs.SIFT[0];
				else if (descriptor==descSURF) ptr_query = &descs.SURF[0];

				feats_img2_kdtree.get_kdtree().knnSearch(
					static_cast<const KDTreeElementType*>( ptr_query ), // Query point
					max_neighbors, // Number of neigbors
					&indices[0], &distances[0]  // Output
					);

				// Include all correspondences below the absolute and the relative threshold (indices comes ordered by distances):
				const KDTreeDistanceType this_thresh = std::min( max_relative_distance*distances[0], max_distance);
				for (size_t j=0;j<max_neighbors;j++)
				{
					if (distances[j]<=this_thresh) {
						overall_pairs++;
						if (pairings_1_to_multi_2) (*pairings_1_to_multi_2)[i].push_back(indices[j]);
						if (pairings_1_to_2) pairings_1_to_2->push_back(std::make_pair(i,indices[j]));
					}
					else break;
				}
			}
			return overall_pairs;
			MRPT_END
		}

		/** @} */

	}
}
#endif

