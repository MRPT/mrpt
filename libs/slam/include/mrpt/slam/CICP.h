/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CICP_H
#define CICP_H

#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TEnumType.h>

namespace mrpt
{
	namespace slam
	{
		/** The ICP algorithm selection, used in mrpt::slam::CICP::options  \ingroup mrpt_slam_grp  */
		enum TICPAlgorithm {
			icpClassic = 0,
			icpLevenbergMarquardt
		};

		/** ICP covariance estimation methods, used in mrpt::slam::CICP::options  \ingroup mrpt_slam_grp  */
		enum TICPCovarianceMethod {
			icpCovLinealMSE = 0,         //!< Use the covariance of the optimal registration, disregarding uncertainty in data association
			icpCovFiniteDifferences      //!< Covariance of a least-squares optimizer (includes data association uncertainty)
		};

		/** Several implementations of ICP (Iterative closest point) algorithms for aligning two point maps or a point map wrt a grid map.
		 *
		 *  CICP::AlignPDF() or CICP::Align() are the two main entry points of the algorithm.
		 *
		 *  To choose among existing ICP algorithms or customizing their parameters, see CICP::TConfigParams and the member \a options.
		 *
		 *  There exists an extension of the original ICP algorithm that provides multihypotheses-support for the correspondences, and which generates a Sum-of-Gaussians (SOG)
		 *    PDF as output. See mrpt::tfest::se2_l2_robust()
		 *
		 * For further details on the implemented methods, check the web:
		 *   http://www.mrpt.org/Iterative_Closest_Point_(ICP)_and_other_matching_algorithms
		 *
		 *  For a paper explaining some of the basic equations, see for example:
		 *   J. Martinez, J. Gonzalez, J. Morales, A. Mandow, A. Garcia-Cerezo,
		 *   "Mobile robot motion estimation by 2D scan matching with genetic and iterative closest point algorithms",
		 *    Journal of Field Robotics, vol. 23, no. 1, pp. 21-34, 2006. ( http://babel.isa.uma.es/~jlblanco/papers/martinez2006gil.pdf )
		 *
		 * \sa CMetricMapsAlignmentAlgorithm
		 * \ingroup mrpt_slam_grp
		 */
		class SLAM_IMPEXP  CICP : public mrpt::slam::CMetricMapsAlignmentAlgorithm
		{
		public:
			/** The ICP algorithm configuration data
			 */
			class SLAM_IMPEXP TConfigParams : public utils::CLoadableOptions
			{
			public:
				TConfigParams();	//!< Initializer for default values:

				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

				/** @name Algorithms selection
				    @{ */
				TICPAlgorithm         ICP_algorithm; //!< The algorithm to use (default: icpClassic). See http://www.mrpt.org/tutorials/programming/scan-matching-and-icp/ for details
				TICPCovarianceMethod  ICP_covariance_method; //!< The method to use for covariance estimation (Default: icpCovFiniteDifferences)
				/** @} */

				/** @name Correspondence-finding criteria
				    @{ */
				bool  onlyClosestCorrespondences;  //!< The usual approach: to consider only the closest correspondence for each local point (Default to true)
				bool  onlyUniqueRobust;            //! Apart of "onlyClosestCorrespondences=true", if this option is enabled only the closest correspondence for each reference point will be kept (default=false).
				/** @} */

				/** @name Termination criteria
				    @{ */
				unsigned int	maxIterations;  //!< Maximum number of iterations to run.
				float           minAbsStep_trans; //!< If the correction in all translation coordinates (X,Y,Z) is below this threshold (in meters), iterations are terminated (Default:1e-6)
				float           minAbsStep_rot;   //!< If the correction in all rotation coordinates (yaw,pitch,roll) is below this threshold (in radians), iterations are terminated (Default:1e-6)
				/** @} */

				float	thresholdDist,thresholdAng; //!< Initial threshold distance for two points to become a correspondence.
				float	ALFA;  //!< The scale factor for threshold everytime convergence is achieved.
				float	smallestThresholdDist;  //!< The size for threshold such that iterations will stop, since it is considered precise enough.

				/** This is the normalization constant \f$ \sigma^2_p \f$ that is used to scale the whole 3x3 covariance.
				  *  This has a default value of \f$ (0.02)^2 \f$, that is, a 2cm sigma.
				  *  See paper: J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, "A Robust, Multi-Hypothesis Approach to Matching Occupancy Grid Maps", Robotica, vol. 31, no. 5, pp. 687-701, 2013.
				  */
				float	covariance_varPoints;

				bool	doRANSAC;  //!< Perform a RANSAC step, mrpt::tfest::se2_l2_robust(), after the ICP convergence, to obtain a better estimation of the pose PDF.

				/** @name RANSAC-step options for mrpt::tfest::se2_l2_robust() if \a doRANSAC=true
				  * @{ */
				unsigned int ransac_minSetSize,ransac_maxSetSize,ransac_nSimulations;
				float        ransac_mahalanobisDistanceThreshold;
				float        normalizationStd; //!< RANSAC-step option: The standard deviation in X,Y of landmarks/points which are being matched (used to compute covariances in the SoG)
				bool         ransac_fuseByCorrsMatch;
				float        ransac_fuseMaxDiffXY, ransac_fuseMaxDiffPhi;
				/** @} */

				float   kernel_rho; //!< Cauchy kernel rho, for estimating the optimal transformation covariance, in meters (default = 0.07m)
				bool    use_kernel; //!< Whether to use kernel_rho to smooth distances, or use distances directly (default=true)
				float   Axy_aprox_derivatives; //!< [LM method only] The size of the perturbance in x & y used to estimate the Jacobians of the square error (default=0.05)
				float   LM_initial_lambda; //!< [LM method only] The initial value of the lambda parameter in the LM method (default=1e-4)

				bool    skip_cov_calculation; //!< Skip the computation of the covariance (saves some time) (default=false)
				bool    skip_quality_calculation;  //!< Skip the (sometimes) expensive evaluation of the term 'quality' at ICP output (Default=true)

				/** Decimation of the point cloud being registered against the reference one (default=5) - set to 1 to have the older (MRPT <0.9.5) behavior
				  *  of not approximating ICP by ignoring the correspondence of some points. The speed-up comes from a decimation of the number of KD-tree queries,
				  *  the most expensive step in ICP */
				uint32_t        corresponding_points_decimation;
			};

			TConfigParams  options; //!< The options employed by the ICP align.

			/** Constructor with the default options */
			CICP() : options() { }
			/** Constructor that directly set the ICP params from a given struct \sa options */
			CICP(const TConfigParams &icpParams) : options(icpParams) { }
			virtual ~CICP() { }	//!< Destructor

			/** The ICP algorithm return information*/
			struct SLAM_IMPEXP TReturnInfo
			{
				TReturnInfo() :
					nIterations(0),
					goodness(0),
					quality(0)
				{
				}
				unsigned short nIterations; //!< The number of executed iterations until convergence
				float          goodness;    //!< A goodness measure for the alignment, it is a [0,1] range indicator of percentage of correspondences.
				float          quality;     //!< A measure of the 'quality' of the local minimum of the sqr. error found by the method. Higher values are better. Low values will be found in ill-conditioned situations (e.g. a corridor)
			};

			/** An implementation of CMetricMapsAlignmentAlgorithm for the case of a point maps and a occupancy grid/point map.
			 *
			 *  This method computes the PDF of the displacement (relative pose) between
			 *   two maps: <b>the relative pose of m2 with respect to m1</b>. This pose
			 *   is returned as a PDF rather than a single value.
			 *
			 *  \note This method can be configurated with "CICP::options"
			 *  \note The output PDF is a CPosePDFGaussian if "doRANSAC=false", or a CPosePDFSOG otherwise.
			 *
			 * \param m1			[IN] The first map (CAN BE A mrpt::poses::CPointsMap derived class or a mrpt::slam::COccupancyGrid2D class)
			 * \param m2			[IN] The second map. (MUST BE A mrpt::poses::CPointsMap derived class)The pose of this map respect to m1 is to be estimated.
			 * \param initialEstimationPDF	[IN] An initial gross estimation for the displacement.
			 * \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
			 * \param info			[OUT] A pointer to a CICP::TReturnInfo, or NULL if it isn't needed.
			 *
			 * \return A smart pointer to the output estimated pose PDF.
			 *
			 * \sa CMetricMapsAlignmentAlgorithm, CICP::options, CICP::TReturnInfo
			 */
			mrpt::poses::CPosePDFPtr AlignPDF(
					const mrpt::maps::CMetricMap		*m1,
					const mrpt::maps::CMetricMap		*m2,
					const mrpt::poses::CPosePDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,
					void					*info = NULL );

			// See base class for docs
			mrpt::poses::CPose3DPDFPtr Align3DPDF(
					const mrpt::maps::CMetricMap		*m1,
					const mrpt::maps::CMetricMap		*m2,
					const mrpt::poses::CPose3DPDFGaussian	&initialEstimationPDF,
					float					*runningTime = NULL,
					void					*info = NULL );


		protected:
			/** Computes:
			  *  \f[ K(x^2) = \frac{x^2}{x^2+\rho^2}  \f]
			  *  or just return the input if options.useKernel = false.
			  */
			float kernel(const float &x2, const float &rho2);

			mrpt::poses::CPosePDFPtr ICP_Method_Classic(
				const mrpt::maps::CMetricMap		*m1,
				const mrpt::maps::CMetricMap		*m2,
				const mrpt::poses::CPosePDFGaussian	&initialEstimationPDF,
				TReturnInfo				&outInfo );
			mrpt::poses::CPosePDFPtr ICP_Method_LM(
				const mrpt::maps::CMetricMap		*m1,
				const mrpt::maps::CMetricMap		*m2,
				const mrpt::poses::CPosePDFGaussian	&initialEstimationPDF,
				TReturnInfo				&outInfo );
			mrpt::poses::CPose3DPDFPtr ICP3D_Method_Classic(
				const mrpt::maps::CMetricMap		*m1,
				const mrpt::maps::CMetricMap		*m2,
				const mrpt::poses::CPose3DPDFGaussian &initialEstimationPDF,
				TReturnInfo				&outInfo );
		};
	} // End of namespace

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<slam::TICPAlgorithm>
		{
			typedef slam::TICPAlgorithm enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(slam::icpClassic, "icpClassic");
				m_map.insert(slam::icpLevenbergMarquardt, "icpLevenbergMarquardt");
			}
		};
		template <>
		struct TEnumTypeFiller<slam::TICPCovarianceMethod>
		{
			typedef slam::TICPCovarianceMethod enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(slam::icpCovLinealMSE, "icpCovLinealMSE");
				m_map.insert(slam::icpCovFiniteDifferences, "icpCovFiniteDifferences");
			}
		};
	} // End of namespace
} // End of namespace

#endif
