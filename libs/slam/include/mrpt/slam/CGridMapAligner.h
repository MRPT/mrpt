/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CGridMapAligner_H
#define CGridMapAligner_H

#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/utils/poly_ptr_ptr.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>

namespace mrpt
{
	namespace slam
	{
		/** A class for aligning two multi-metric maps (with an occupancy grid maps and a points map, at least) based on features extraction and matching.
		 * The matching pose is returned as a Sum of Gaussians (poses::CPosePDFSOG).
		 *
		 *  This class can use three methods (see options.methodSelection):
		 *   - amCorrelation: "Brute-force" correlation of the two maps over a 2D+orientation grid of possible 2D poses.
		 *   - amRobustMatch: Detection of features + RANSAC matching
		 *   - amModifiedRANSAC: Detection of features + modified multi-hypothesis RANSAC matching as described in was reported in the paper http://www.mrpt.org/Paper%3AOccupancy_Grid_Matching
		 *
		 * See CGridMapAligner::Align for more instructions.
		 *
		 * \sa CMetricMapsAlignmentAlgorithm
		 * \ingroup mrpt_slam_grp
		 */
		class SLAM_IMPEXP  CGridMapAligner : public mrpt::slam::CMetricMapsAlignmentAlgorithm
		{
		private:
			/** Private member, implements one the algorithms.
			  */
			mrpt::poses::CPosePDFPtr AlignPDF_correlation(
					const mrpt::maps::CMetricMap		*m1,
					const mrpt::maps::CMetricMap		*m2,
					const mrpt::poses::CPosePDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,
					void					*info = NULL );

			/** Private member, implements both, the "robustMatch" and the newer "modifiedRANSAC" algorithms.
			  */
			mrpt::poses::CPosePDFPtr AlignPDF_robustMatch(
					const mrpt::maps::CMetricMap		*m1,
					const mrpt::maps::CMetricMap		*m2,
					const mrpt::poses::CPosePDFGaussian	&initialEstimationPDF,
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

				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

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
				TReturnInfo() :
					goodness(0),
					noRobustEstimation()
				{
				}

				/** A goodness measure for the alignment, it is a [0,1] range indicator of percentage of correspondences.
				 */
				float			goodness;

				/** The "brute" estimation from using all the available correspondences, provided just for comparison purposes (it is not the robust estimation, available as the result of the Align method).
				  */
				mrpt::poses::CPose2D			noRobustEstimation;

				/** The different SOG densities at different steps of the algorithm:
				  *   - sog1 : Directly from the matching of features
				  *   - sog2 : Merged of sog1
				  *   - sog3 : sog2 refined with ICP
				  *
				  *   - The final sog is the merge of sog3.
				  *
				  */
				mrpt::utils::poly_ptr_ptr<mrpt::poses::CPosePDFSOGPtr> sog1,sog2,sog3;

				/** The landmarks of each map (the indices of these landmarks correspond to those in "correspondences")  */
				mrpt::maps::CLandmarksMapPtr	landmarks_map1, landmarks_map2;

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
			 * \param m1			[IN] The first map (Must be a mrpt::maps::CMultiMetricMap class)
			 * \param m2			[IN] The second map (Must be a mrpt::maps::CMultiMetricMap class)
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
			mrpt::poses::CPosePDFPtr AlignPDF(
					const mrpt::maps::CMetricMap		*m1,
					const mrpt::maps::CMetricMap		*m2,
					const mrpt::poses::CPosePDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,

					void					*info = NULL );


			/** Not applicable in this class, will launch an exception. */
			mrpt::poses::CPose3DPDFPtr Align3DPDF(
					const mrpt::maps::CMetricMap		*m1,
					const mrpt::maps::CMetricMap		*m2,
					const mrpt::poses::CPose3DPDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,
					void					*info = NULL );
			
		};

	} // End of namespace

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<slam::CGridMapAligner::TAlignerMethod>
		{
			typedef slam::CGridMapAligner::TAlignerMethod enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(slam::CGridMapAligner::amRobustMatch,    "amRobustMatch");
				m_map.insert(slam::CGridMapAligner::amCorrelation,    "amCorrelation");
				m_map.insert(slam::CGridMapAligner::amModifiedRANSAC, "amModifiedRANSAC");
			}
		};
	} // End of namespace

} // End of namespace

#endif
