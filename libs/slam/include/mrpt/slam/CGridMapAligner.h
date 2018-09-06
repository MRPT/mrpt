/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>

namespace mrpt::slam
{
/** A class for aligning two multi-metric maps (with an occupancy grid maps and
 * a points map, at least) based on features extraction and matching.
 * The matching pose is returned as a Sum of Gaussians (poses::CPosePDFSOG).
 *
 *  This class can use three methods (see options.methodSelection):
 *   - amCorrelation: "Brute-force" correlation of the two maps over a
 * 2D+orientation grid of possible 2D poses.
 *   - amRobustMatch: Detection of features + RANSAC matching
 *   - amModifiedRANSAC: Detection of features + modified multi-hypothesis
 * RANSAC matching as described in was reported in the paper
 * http://www.mrpt.org/Paper%3AOccupancy_Grid_Matching
 *
 * See CGridMapAligner::Align for more instructions.
 *
 * \sa CMetricMapsAlignmentAlgorithm
 * \ingroup mrpt_slam_grp
 */
class CGridMapAligner : public mrpt::slam::CMetricMapsAlignmentAlgorithm
{
   private:
	/** Private member, implements one the algorithms.
	 */
	mrpt::poses::CPosePDF::Ptr AlignPDF_correlation(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPosePDFGaussian& initialEstimationPDF,
		float* runningTime = nullptr, void* info = nullptr);

	/** Private member, implements both, the "robustMatch" and the newer
	 * "modifiedRANSAC" algorithms.
	 */
	mrpt::poses::CPosePDF::Ptr AlignPDF_robustMatch(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPosePDFGaussian& initialEstimationPDF,
		float* runningTime = nullptr, void* info = nullptr);

	/** Grid map features extractor */
	COccupancyGridMapFeatureExtractor m_grid_feat_extr;

   public:
	CGridMapAligner() : options() {}
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
	class TConfigParams : public mrpt::config::CLoadableOptions
	{
	   public:
		/** Initializer for default values:
		 */
		TConfigParams();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** The aligner method: */
		TAlignerMethod methodSelection{CGridMapAligner::amModifiedRANSAC};

		/** The feature descriptor to use: 0=detector already has descriptor, 1=
		 * SIFT, 2=SURF, 4=Spin images, 8=Polar images, 16=log-polar images */
		mrpt::vision::TDescriptorType feature_descriptor{
			mrpt::vision::descPolarImages};

		/** All the parameters for the feature detector. */
		mrpt::vision::CFeatureExtraction::TOptions feature_detector_options;

		/** RANSAC-step options:
		 * \sa CICP::robustRigidTransformation
		 */
		/** The ratio of landmarks that must be inliers to accepto an hypotheses
		 * (typ: 0.20) */
		float ransac_minSetSizeRatio{0.20f};
		/** The square root of the uncertainty normalization variance var_m (see
		 * papers...) */
		float ransac_SOG_sigma_m{0.10f};

		/** [amRobustMatch method only] RANSAC-step options:
		 * \sa CICP::robustRigidTransformation
		 */
		float ransac_mahalanobisDistanceThreshold{6.0f};

		/** [amModifiedRANSAC method only] The quantile used for chi-square
		 * thresholding (default=0.99) */
		double ransac_chi2_quantile{0.99};

		/** Probability of having a good inliers (def:0,9999), used for
		 * automatic number of iterations */
		double ransac_prob_good_inliers{0.9999};
		/** Features extraction from grid map: How many features to extract */
		float featsPerSquareMeter{0.015f};
		/** Correspondences are considered if their distances are below this
		 * threshold (in the range [0,1]) (default=0.15). */
		float threshold_max{0.15f};
		/** Correspondences are considered if their distances to the best match
		 * are below this threshold (in the range [0,1]) (default=0.15). */
		float threshold_delta{0.10f};

		/** The minimum goodness (0-1) of the post-matching ICP to accept a
		 * hypothesis as good (default=0.30) */
		float min_ICP_goodness{0.30f};
		/** The maximum Mahalanobis distance between the initial and final poses
		 * in the ICP not to discard the hypothesis (default=10) */
		double max_ICP_mahadist{10.0};
		/** Maximum KL-divergence for merging modes of the SOG (default=0.9) */
		double maxKLd_for_merge{0.9};

		/** DEBUG - Dump all feature correspondences in a directory "grid_feats"
		 */
		bool save_feat_coors{false};
		/** DEBUG - Show graphs with the details of each feature correspondences
		 */
		bool debug_show_corrs{false};
		/** DEBUG - Save the pair of maps with all the pairings. */
		bool debug_save_map_pairs{false};

	} options;

	/** The ICP algorithm return information.
	 */
	struct TReturnInfo
	{
		TReturnInfo() : noRobustEstimation() {}
		/** A goodness measure for the alignment, it is a [0,1] range indicator
		 * of percentage of correspondences.
		 */
		float goodness{0};

		/** The "brute" estimation from using all the available correspondences,
		 * provided just for comparison purposes (it is not the robust
		 * estimation, available as the result of the Align method).
		 */
		mrpt::poses::CPose2D noRobustEstimation;

		/** The different SOG densities at different steps of the algorithm:
		 *   - sog1 : Directly from the matching of features
		 *   - sog2 : Merged of sog1
		 *   - sog3 : sog2 refined with ICP
		 *
		 *   - The final sog is the merge of sog3.
		 *
		 */
		mrpt::containers::deepcopy_poly_ptr<mrpt::poses::CPosePDFSOG::Ptr> sog1,
			sog2, sog3;

		/** The landmarks of each map (the indices of these landmarks correspond
		 * to those in "correspondences")  */
		mrpt::maps::CLandmarksMap::Ptr landmarks_map1, landmarks_map2;

		/** All the found correspondences (not consistent) */
		mrpt::tfest::TMatchingPairList correspondences;

		struct TPairPlusDistance
		{
			TPairPlusDistance(size_t i1, size_t i2, float d)
				: idx_this(i1), idx_other(i2), dist(d)
			{
			}
			size_t idx_this, idx_other;
			float dist;
		};

		/** Mahalanobis distance for each potential correspondence */
		std::vector<TPairPlusDistance> correspondences_dists_maha;

		/** The ICP goodness of all potential SOG modes at the stage "sog2",
		 * thus before the removing of "bad" ICP matches. */
		std::vector<double> icp_goodness_all_sog_modes;
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
	 * \param m1			[IN] The first map (Must be a
	 *mrpt::maps::CMultiMetricMap
	 *class)
	 * \param m2			[IN] The second map (Must be a
	 *mrpt::maps::CMultiMetricMap
	 *class)
	 * \param initialEstimationPDF	[IN] (IGNORED IN THIS ALGORITHM!)
	 * \param runningTime	[OUT] A pointer to a container for obtaining the
	 *algorithm running time in seconds, or NULL if you don't need it.
	 * \param info			[OUT] A pointer to a TReturnInfo struct, or NULL if
	 *result information are not required.
	 *
	 * \note The returned PDF depends on the selected alignment method:
	 *		- "amRobustMatch" --> A "poses::CPosePDFSOG" object.
	 *		- "amCorrelation" --> A "poses::CPosePDFGrid" object.
	 *
	 * \return A smart pointer to the output estimated pose PDF.
	 * \sa CPointsMapAlignmentAlgorithm, options
	 */
	mrpt::poses::CPosePDF::Ptr AlignPDF(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPosePDFGaussian& initialEstimationPDF,
		float* runningTime = nullptr,

		void* info = nullptr) override;

	/** Not applicable in this class, will launch an exception. */
	mrpt::poses::CPose3DPDF::Ptr Align3DPDF(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPose3DPDFGaussian& initialEstimationPDF,
		float* runningTime = nullptr, void* info = nullptr) override;
};

}  // namespace mrpt::slam
MRPT_ENUM_TYPE_BEGIN(mrpt::slam::CGridMapAligner::TAlignerMethod)
using namespace mrpt::slam;
MRPT_FILL_ENUM_MEMBER(CGridMapAligner, amRobustMatch);
MRPT_FILL_ENUM_MEMBER(CGridMapAligner, amCorrelation);
MRPT_FILL_ENUM_MEMBER(CGridMapAligner, amModifiedRANSAC);
MRPT_ENUM_TYPE_END()
