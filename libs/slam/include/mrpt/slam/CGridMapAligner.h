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
#ifndef CGridMapAligner_H
#define CGridMapAligner_H

#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/slam/CLandmarksMap.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>

namespace mrpt
{
	namespace slam
	{
		using namespace poses;
		using namespace utils;

		/** A class for aligning two multi-metric maps (with an occupancy grid maps and a points map, at least) based on features extraction and matching.
		 * The matching pose is returned as a Sum of Gaussians (poses::CPosePDFSOG).
		 *
		 *  This method was reported in the paper: <br>
		 *
		 * See CGridMapAligner::Align for more instructions.
		 *
		 * \sa CMetricMapsAlignmentAlgorithm
		 * \ingroup mrpt_slam_grp
		 */
		class SLAM_IMPEXP  CGridMapAligner : public CMetricMapsAlignmentAlgorithm
		{
		private:
			/** Private member, implements one the algorithms.
			  */
			CPosePDFPtr AlignPDF_correlation(
					const CMetricMap		*m1,
					const CMetricMap		*m2,
					const CPosePDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,
					void					*info = NULL );

			/** Private member, implements both, the "robustMatch" and the newer "modifiedRANSAC" algorithms.
			  */
			CPosePDFPtr AlignPDF_robustMatch(
					const CMetricMap		*m1,
					const CMetricMap		*m2,
					const CPosePDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,
					void					*info = NULL );

			COccupancyGridMapFeatureExtractor	m_grid_feat_extr; //!< Grid map features extractor
		public:

			CGridMapAligner() :
				options()
			{}

			/** The type for selecting the grid-map alignment algorithm.
			  */
			enum TAlignerMethod
			{
				amRobustMatch = 0,
				amCorrelation,
				amModifiedRANSAC
			};

			/** The ICP algorithm configuration data
			 */
			class SLAM_IMPEXP TConfigParams : public utils::CLoadableOptions
			{
			public:
				/** Initializer for default values:
				  */
				TConfigParams();

				/** See utils::CLoadableOptions
				  */
				void  loadFromConfigFile(
					const mrpt::utils::CConfigFileBase  &source,
					const std::string &section);

				/** See utils::CLoadableOptions
				  */
				void  dumpToTextStream(CStream	&out) const;


				TAlignerMethod		methodSelection;		//!< The aligner method:

				/** The feature descriptor to use: 0=detector already has descriptor, 1= SIFT, 2=SURF, 4=Spin images, 8=Polar images, 16=log-polar images */
				mrpt::vision::TDescriptorType	feature_descriptor;

				/** All the parameters for the feature detector. */
				mrpt::vision::CFeatureExtraction::TOptions	feature_detector_options;

				/** RANSAC-step options:
				  * \sa CICP::robustRigidTransformation
				  */
				float	ransac_minSetSizeRatio;  //!< The ratio of landmarks that must be inliers to accepto an hypotheses (typ: 0.20)
				float	ransac_SOG_sigma_m;	//!< The square root of the uncertainty normalization variance var_m (see papers...)

				/** [amRobustMatch method only] RANSAC-step options:
				  * \sa CICP::robustRigidTransformation
				  */
				float	ransac_mahalanobisDistanceThreshold;

				double  ransac_chi2_quantile;	//!< [amModifiedRANSAC method only] The quantile used for chi-square thresholding (default=0.99)

				double	ransac_prob_good_inliers; //!< Probability of having a good inliers (def:0,9999), used for automatic number of iterations
				float	featsPerSquareMeter;	//!< Features extraction from grid map: How many features to extract
				float	threshold_max;		//!< Correspondences are considered if their distances are below this threshold (in the range [0,1]) (default=0.15).
				float	threshold_delta;	//!< Correspondences are considered if their distances to the best match are below this threshold (in the range [0,1]) (default=0.15).

				float   min_ICP_goodness;	//!< The minimum goodness (0-1) of the post-matching ICP to accept a hypothesis as good (default=0.25)
				double  max_ICP_mahadist;	//!< The maximum Mahalanobis distance between the initial and final poses in the ICP not to discard the hypothesis (default=10)
				double  maxKLd_for_merge;	//!< Maximum KL-divergence for merging modes of the SOG (default=0.9)

				bool	save_feat_coors;	//!< DEBUG - Dump all feature correspondences in a directory "grid_feats"
				bool	debug_show_corrs;	//!< DEBUG - Show graphs with the details of each feature correspondences
				bool	debug_save_map_pairs;	//!< DEBUG - Save the pair of maps with all the pairings.

			} options;

			/** The ICP algorithm return information.
			 */
			struct SLAM_IMPEXP TReturnInfo
			{
				/** Initialization
				  */
				TReturnInfo() :
					cbSize(sizeof(TReturnInfo)),
					goodness(0),
					noRobustEstimation()
				{
				}

				size_t		cbSize;	//!< Size of the structure, do not change, it's set automatically

				/** A goodness measure for the alignment, it is a [0,1] range indicator of percentage of correspondences.
				 */
				float			goodness;

				/** The "brute" estimation from using all the available correspondences, provided just for comparison purposes (it is not the robust estimation, available as the result of the Align method).
				  */
				CPose2D			noRobustEstimation;

				/** The different SOG densities at different steps of the algorithm:
				  *   - sog1 : Directly from the matching of features
				  *   - sog2 : Merged of sog1
				  *   - sog3 : sog2 refined with ICP
				  *
				  *   - The final sog is the merge of sog3.
				  *
				  */
				CPosePDFSOGPtr	sog1,sog2,sog3;

				/** The landmarks of each map (the indices of these landmarks correspond to those in "correspondences")  */
				CLandmarksMapPtr	landmarks_map1, landmarks_map2;

				/** All the found correspondences (not consistent) */
				mrpt::utils::TMatchingPairList	correspondences;

				struct TPairPlusDistance
				{
					TPairPlusDistance(size_t i1, size_t i2, float d) :
						idx_this(i1), idx_other(i2), dist(d)
					{ }
					size_t		idx_this,idx_other;
					float		dist;
				};

				std::vector<TPairPlusDistance>  correspondences_dists_maha;	//!< Mahalanobis distance for each potential correspondence

				std::vector<double>	icp_goodness_all_sog_modes;	//!< The ICP goodness of all potential SOG modes at the stage "sog2", thus before the removing of "bad" ICP matches.
			};

			/** The method for aligning a pair of 2D points map.
			 *   The meaning of some parameters are implementation dependant,
			 *    so look for derived classes for instructions.
			 *  The target is to find a PDF for the pose displacement between
			 *   maps, thus <b>the pose of m2 relative to m1</b>. This pose
			 *   is returned as a PDF rather than a single value.
			 *
			 *  NOTE: This method can be configurated with "options"
			 *
			 * \param m1			[IN] The first map (Must be a mrpt::slam::CMultiMetricMap class)
			 * \param m2			[IN] The second map (Must be a mrpt::slam::CMultiMetricMap class)
			 * \param initialEstimationPDF	[IN] (IGNORED IN THIS ALGORITHM!)
			 * \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
			 * \param info			[OUT] A pointer to a CAlignerFromMotionDraws::TReturnInfo struct, or NULL if result information are not required.
			 *
			 * \note The returned PDF depends on the selected alignment method:
			 *		- "amRobustMatch" --> A "poses::CPosePDFSOG" object.
			 *		- "amCorrelation" --> A "poses::CPosePDFGrid" object.
			 *
			 * \return A smart pointer to the output estimated pose PDF.
			 * \sa CPointsMapAlignmentAlgorithm, options
			 */
			CPosePDFPtr AlignPDF(
					const CMetricMap		*m1,
					const CMetricMap		*m2,
					const CPosePDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,

					void					*info = NULL );


			/** The virtual method for aligning a pair of metric maps, aligning the full 6D pose.
			 *   The meaning of some parameters are implementation dependant,
			 *    so look at the derived classes for more details.
			 *  The goal is to find a PDF for the pose displacement between
			 *   maps, that is, <b>the pose of m2 relative to m1</b>. This pose
			 *   is returned as a PDF rather than a single value.
			 *
			 *  \note This method can be configurated with a "options" structure in the implementation classes.
			 *
			 * \param m1			[IN] The first map (MUST BE A COccupancyGridMap2D  derived class)
			 * \param m2			[IN] The second map. (MUST BE A CPointsMap derived class) The pose of this map respect to m1 is to be estimated.
			 * \param initialEstimationPDF	[IN] An initial gross estimation for the displacement.
			 * \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
			 * \param info			[OUT] See derived classes for details, or NULL if it isn't needed.
			 *
			 * \return A smart pointer to the output estimated pose PDF.
			 * \sa CICP
			 */
			CPose3DPDFPtr Align3DPDF(
					const CMetricMap		*m1,
					const CMetricMap		*m2,
					const CPose3DPDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,
					void					*info = NULL )
			{
				THROW_EXCEPTION("Align3D method not applicable to gridmap-aligner");
			}

		};

	} // End of namespace
} // End of namespace

#endif
