/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt::slam
{
/** The ICP algorithm selection, used in mrpt::slam::CICP::options  \ingroup
 * mrpt_slam_grp  */
enum TICPAlgorithm
{
	icpClassic = 0,
	icpLevenbergMarquardt
};

/** ICP covariance estimation methods, used in mrpt::slam::CICP::options
 * \ingroup mrpt_slam_grp  */
enum TICPCovarianceMethod
{
	/** Use the covariance of the optimal registration, disregarding uncertainty
	   in data association */
	icpCovLinealMSE = 0,
	/** Covariance of a least-squares optimizer (includes data association
	   uncertainty) */
	icpCovFiniteDifferences
};

/** Several implementations of ICP (Iterative closest point) algorithms for
 * aligning two point maps or a point map wrt a grid map.
 *
 *  CICP::AlignPDF() or CICP::Align() are the two main entry points of the
 * algorithm.
 *
 *  To choose among existing ICP algorithms or customizing their parameters, see
 * CICP::TConfigParams and the member \a options.
 *
 *  There exists an extension of the original ICP algorithm that provides
 * multihypotheses-support for the correspondences, and which generates a
 * Sum-of-Gaussians (SOG)
 *    PDF as output. See mrpt::tfest::se2_l2_robust()
 *
 * For further details on the implemented methods, check the web:
 *   https://www.mrpt.org/Iterative_Closest_Point_(ICP)_and_other_matching_algorithms
 *
 *  For a paper explaining some of the basic equations, see for example:
 *   J. Martinez, J. Gonzalez, J. Morales, A. Mandow, A. Garcia-Cerezo,
 *   "Mobile robot motion estimation by 2D scan matching with genetic and
 * iterative closest point algorithms",
 *    Journal of Field Robotics, vol. 23, no. 1, pp. 21-34, 2006. (
 * http://babel.isa.uma.es/~jlblanco/papers/martinez2006gil.pdf )
 *
 * \sa CMetricMapsAlignmentAlgorithm
 * \ingroup mrpt_slam_grp
 */
class CICP : public mrpt::slam::CMetricMapsAlignmentAlgorithm
{
   public:
	/** The ICP algorithm configuration data
	 */
	class TConfigParams : public mrpt::config::CLoadableOptions
	{
	   public:
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c,
			const std::string& s) const override;

		/** @name Algorithms selection
			@{ */
		/** The algorithm to use (default: icpClassic). See
		 * https://www.mrpt.org/tutorials/programming/scan-matching-and-icp/ for
		 * details */
		TICPAlgorithm ICP_algorithm{icpClassic};
		/** The method to use for covariance estimation (Default:
		 * icpCovFiniteDifferences) */
		TICPCovarianceMethod ICP_covariance_method{icpCovFiniteDifferences};
		/** @} */

		/** @name Correspondence-finding criteria
			@{ */
		bool onlyUniqueRobust{false};
		//! if this option is enabled only the closest
		//! correspondence for each reference point will
		//! be kept (default=false).
		/** @} */

		/** @name Termination criteria
			@{ */
		/** Maximum number of iterations to run. */
		unsigned int maxIterations{40};
		/** If the correction in all translation coordinates (X,Y,Z) is below
		 * this threshold (in meters), iterations are terminated (Default:1e-6)
		 */
		double minAbsStep_trans{1e-6};
		/** If the correction in all rotation coordinates (yaw,pitch,roll) is
		 * below this threshold (in radians), iterations are terminated
		 * (Default:1e-6) */
		double minAbsStep_rot{1e-6};
		/** @} */

		/** Initial threshold distance for two points to become a
		 * correspondence. */
		double thresholdDist{0.75}, thresholdAng{0.15 * M_PI / 180.0};
		/** The scale factor for thresholds every time convergence is achieved.
		 */
		double ALFA{0.5};
		/** The size for threshold such that iterations will stop, since it is
		 * considered precise enough. */
		double smallestThresholdDist{0.1};

		/** This is the normalization constant \f$ \sigma^2_p \f$ that is used
		 * to scale the whole 3x3 covariance.
		 *  This has a default value of \f$ (0.02)^2 \f$, that is, a 2cm sigma.
		 *  See paper: J.L. Blanco, J. Gonzalez-Jimenez, J.A.
		 * Fernandez-Madrigal, "A Robust, Multi-Hypothesis Approach to Matching
		 * Occupancy Grid Maps", Robotica, vol. 31, no. 5, pp. 687-701, 2013.
		 */
		double covariance_varPoints{0.02 * 0.02};

		/** Perform a RANSAC step, mrpt::tfest::se2_l2_robust(), after the ICP
		 * convergence, to obtain a better estimation of the pose PDF. */
		bool doRANSAC{false};

		/** @name RANSAC-step options for mrpt::tfest::se2_l2_robust() if \a
		 * doRANSAC=true
		 * @{ */
		unsigned int ransac_minSetSize{3}, ransac_maxSetSize{20},
			ransac_nSimulations{100};
		double ransac_mahalanobisDistanceThreshold{3.0};
		/** RANSAC-step option: The standard deviation in X,Y of
		 * landmarks/points which are being matched (used to compute covariances
		 * in the SoG) */
		double normalizationStd{0.02};
		bool ransac_fuseByCorrsMatch{true};
		double ransac_fuseMaxDiffXY{0.01},
			ransac_fuseMaxDiffPhi{0.1 * M_PI / 180.0};
		/** @} */

		/** Cauchy kernel rho, for estimating the optimal transformation
		 * covariance, in meters (default = 0.07m) */
		double kernel_rho{0.07};
		/** Whether to use kernel_rho to smooth distances, or use distances
		 * directly (default=true) */
		bool use_kernel{true};
		/** [LM method only] The size of the perturbance in x & y used to
		 * estimate the Jacobians of the square error (default=0.05) */
		double Axy_aprox_derivatives{0.05};
		/** [LM method only] The initial value of the lambda parameter in the LM
		 * method (default=1e-4) */
		double LM_initial_lambda{1e-4};

		/** Skip the computation of the covariance (saves some time)
		 * (default=false) */
		bool skip_cov_calculation{false};
		/** Skip the (sometimes) expensive evaluation of the term 'quality' at
		 * ICP output (Default=true) */
		bool skip_quality_calculation{true};

		/** Decimation of the point cloud being registered against the reference
		 * one (default=5) - set to 1 to have the older (MRPT <0.9.5) behavior
		 *  of not approximating ICP by ignoring the correspondence of some
		 * points. The speed-up comes from a decimation of the number of KD-tree
		 * queries,
		 *  the most expensive step in ICP */
		uint32_t corresponding_points_decimation{5};
	};

	/** The options employed by the ICP align. */
	TConfigParams options;

	/** Constructor with the default options */
	CICP() : options() {}
	/** Constructor that directly set the ICP params from a given struct \sa
	 * options */
	CICP(const TConfigParams& icpParams) : options(icpParams) {}
	/** Destructor */
	~CICP() override = default;
	/** The ICP algorithm return information*/
	struct TReturnInfo : public TMetricMapAlignmentResult
	{
		TReturnInfo() = default;
		virtual ~TReturnInfo() override = default;

		/** The number of executed iterations until convergence */
		unsigned int nIterations = 0;

		/** A goodness measure for the alignment, it is a [0,1] range indicator
		 * of percentage of correspondences. */
		double goodness = 0;

		/** A measure of the 'quality' of the local minimum of the sqr. error
		 * found by the method. Higher values are better. Low values will be
		 * found in ill-conditioned situations (e.g. a corridor) */
		double quality = 0;
	};

	/** See base method docs.
	 * This class offers an implementation for the case of "m1" being a point
	 * map and "m2" either an occupancy grid or a point map.
	 *
	 *  \note This method can be configurated with "CICP::options"
	 *  \note The output PDF is a CPosePDFGaussian if "doRANSAC=false", or a
	 * CPosePDFSOG otherwise.
	 *
	 * \sa CMetricMapsAlignmentAlgorithm, CICP::options, CICP::TReturnInfo
	 */
	mrpt::poses::CPosePDF::Ptr AlignPDF(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPosePDFGaussian& initialEstimationPDF,
		mrpt::optional_ref<TMetricMapAlignmentResult> outInfo =
			std::nullopt) override;

	// See base class for docs
	mrpt::poses::CPose3DPDF::Ptr Align3DPDF(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPose3DPDFGaussian& initialEstimationPDF,
		mrpt::optional_ref<TMetricMapAlignmentResult> outInfo =
			std::nullopt) override;

   protected:
	/** Computes:
	 *  \f[ K(x^2) = \frac{x^2}{x^2+\rho^2}  \f]
	 *  or just return the input if options.useKernel = false.
	 */
	float kernel(float x2, float rho2);

	mrpt::poses::CPosePDF::Ptr ICP_Method_Classic(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPosePDFGaussian& initialEstimationPDF,
		TReturnInfo& outInfo);
	mrpt::poses::CPosePDF::Ptr ICP_Method_LM(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPosePDFGaussian& initialEstimationPDF,
		TReturnInfo& outInfo);
	mrpt::poses::CPose3DPDF::Ptr ICP3D_Method_Classic(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPose3DPDFGaussian& initialEstimationPDF,
		TReturnInfo& outInfo);
};
}  // namespace mrpt::slam
MRPT_ENUM_TYPE_BEGIN(mrpt::slam::TICPAlgorithm)
using namespace mrpt::slam;
MRPT_FILL_ENUM(icpClassic);
MRPT_FILL_ENUM(icpLevenbergMarquardt);
MRPT_ENUM_TYPE_END()

MRPT_ENUM_TYPE_BEGIN(mrpt::slam::TICPCovarianceMethod)
using namespace mrpt::slam;
MRPT_FILL_ENUM(icpCovLinealMSE);
MRPT_FILL_ENUM(icpCovFiniteDifferences);
MRPT_ENUM_TYPE_END()
