/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CPosePDFParticlesExtended_H
#define CPosePDFParticlesExtended_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CSensoryFrame.h>

namespace mrpt
{
	namespace poses
	{

		class TExtendedCPose2D
		{
		public:
			CPose2D			pose;
			mrpt::math::CVectorDouble	state;
		};

#define DUMMY_LINKAGE

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CPosePDFParticlesExtended, mrpt::poses::CPosePDF, DUMMY_LINKAGE )

		/** Declares a class that represents a Probability Distribution
		 *    function (PDF) of a 2D pose (x,y,phi).
		 *   This class implements that PDF using a set of particles
		 *    (for using particle filter methods), where M weighted
		 *    particles represent the PDF.
		 *
		 * \sa CPose2D, CPosePDF, CPoseGaussianPDF
		 */
		class CPosePDFParticlesExtended :
			public CPosePDF,
			public mrpt::bayes::CParticleFilterData<TExtendedCPose2D>,
			public mrpt::bayes::CParticleFilterDataImpl<CPosePDFParticlesExtended,mrpt::bayes::CParticleFilterData<TExtendedCPose2D>::CParticleList>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPosePDFParticlesExtended )

		public:

			/** The struct for passing extra simulation parameters to the prediction stage
			 *    when running a particle filter.
			 * \sa prediction
			 */
			struct TPredictionParams
			{
				/** Default settings method.
				  */
				TPredictionParams();

				/** [update stage] Must be set to a metric map used to estimate the likelihood of observations
				  */
				mrpt::maps::CMetricMap			*metricMap;

				/** [update stage] Alternative way (if metricMap==NULL): A metric map is supplied for each particle: There must be the same maps here as pose particles.
				  */
				mrpt::maps::TMetricMapList		metricMaps;

				/** Parameters for the KLD adaptive sample size algorithm (see Dieter Fox's papers), which is used only if the CParticleFilter is created with the "adaptiveSampleSize" flag set to true.
				  */
				float			KLD_binSize_XY, KLD_binSize_PHI,
								KLD_delta, KLD_epsilon;

				/** Parameters for the KLD adaptive sample size algorithm (see Dieter Fox's papers), which is used only if the CParticleFilter is created with the "adaptiveSampleSize" flag set to true.
				  */
				unsigned int	KLD_minSampleSize, KLD_maxSampleSize;

				/** In the algorithm "CParticleFilter::pfAuxiliaryPFOptimal", the number of samples for searching the maximum likelihood value (see papers!)
				  */
				unsigned int	pfAuxFilterOptimal_MaximumSearchSamples;

				/** The probability [0,1] of changing the UWB bias (individual for each beacon's bias).
				  */
				float			probabilityChangingBias;

				/** The bias of each beacon will be added a uniform random variable [-changingBiasUnifRange/2,changingBiasUnifRange/2] with a probability "probabilityChangingBias".
				  */
				float			changingBiasUnifRange;

				/** A number between 0 and 1 (0=standard proposal).
				  */
				float			mixtureProposalRatio;

			} options;

			/** Free all the memory associated to particles, and set the number of parts = 0 */
			void  clear( );

		 public:

			/** Constructor
			  * \param M The number of particles.
			  */
			CPosePDFParticlesExtended( size_t M = 1 );

			/** Copy constructor:
			  */
			CPosePDFParticlesExtended( const CPosePDFParticlesExtended& obj )
			{
				copyFrom( obj );
			}

			/** Destructor
			 */
			virtual ~CPosePDFParticlesExtended();


			/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
			  */
			void  copyFrom(const CPosePDF &o) MRPT_OVERRIDE;

			 /** Reset the PDF to a single point: All particles will be set exactly to the supplied pose.
			  * \param location The location to set all the particles.
			  * \param particlesCount If this is set to -1 the number of particles remains unchanged.
			  *  \sa resetUniform, resetUniformFreeSpace
			  */
			void  resetDeterministic( TExtendedCPose2D	&location,
												int					particlesCount = -1);

			/** Reset the PDF to an uniformly distributed one, inside of the defined cube.
			  * If particlesCount is set to -1 the number of particles remains unchanged.
			  *  \sa resetDeterministic, resetUniformFreeSpace
			  */
			void  resetUniform( float x_min,
										  float x_max,
										  float y_min,
										  float y_max,
										  mrpt::math::CVectorFloat	state_min,
										  mrpt::math::CVectorFloat	state_max,
										  float phi_min = -M_PI,
										  float phi_max = M_PI,
										  int	particlesCount = -1);


			 /** Returns an estimate of the pose, i.e. a "mean value", computed
			  *   as a weighted average over all particles.
			  */
			void getMean(CPose2D  &p) const MRPT_OVERRIDE;

			 /** Returns an estimate of the pose-state, i.e. a "mean value", computed as a weighted average over all particles.
			  */
			TExtendedCPose2D  getEstimatedPoseState() const;

			/** Returns an estimate of the pose covariance matrix (3x3 cov.matrix  for x,y,phi variables)
			  */
			void getCovarianceAndMean(mrpt::math::CMatrixDouble33 &C, CPose2D &p) const MRPT_OVERRIDE;

			/** Returns the pose of the i'th particle.
			  */
			CPose2D	 getParticlePose(int i) const;


			 /** Update the particles, predicting the posterior of robot pose and map after a movement command.
			  *  This method has additional configuration parameters in "options".
			  *  Performs the update stage of the RBPF, using the sensed sensory Frame:
			  *
			  *   \param action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
			  *   \param observation This must be a pointer to a CsensoryFrame object, with robot sensed observations.
			  *
			  * \sa options
			  */
			void  prediction_and_update_pfStandardProposal(
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) MRPT_OVERRIDE;

			 /** Update the particles, predicting the posterior of robot pose and map after a movement command.
			  *  This method has additional configuration parameters in "options".
			  *  Performs the update stage of the RBPF, using the sensed sensory Frame:
			  *
			  *   \param action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
			  *   \param observation This must be a pointer to a CsensoryFrame object, with robot sensed observations.
			  *
			  * \sa options
			  */
			void  prediction_and_update_pfAuxiliaryPFOptimal(
				const mrpt::obs::CActionCollection	* actions,
				const mrpt::obs::CSensoryFrame		* sf,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options ) MRPT_OVERRIDE;

			/** Save PDF's particles to a text file. In each line it will go: "x y phi weight"
			 */
			void  saveToTextFile(const std::string &file) const MRPT_OVERRIDE;

			/** Get the particles count (equivalent to "particlesCount")
			 */
			size_t  size() const { return m_particles.size(); }

			/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
			  *   "to proyect" the current pdf. Result PDF substituted the currently stored one in the object.
			  */
			void  changeCoordinatesReference( const mrpt::poses::CPose3D &newReferenceBase ) MRPT_OVERRIDE;

			/** Draws a single sample from the distribution (WARNING: weights are assumed to be normalized!)
			  */
			void  drawSingleSample(mrpt::poses::CPose2D &outPart ) const MRPT_OVERRIDE;

			/** Draws a number of samples from the distribution, and saves as a list of 1x3 vectors, where each row contains a (x,y,phi) datum.
			  */
			void  drawManySamples( size_t N, std::vector<mrpt::math::CVectorDouble> & outSamples ) const MRPT_OVERRIDE;

			/** Appends (pose-composition) a given pose "p" to each particle
			  */
			void  operator += ( const mrpt::poses::CPose2D &Ap);

			/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
			  */
			void	 inverse(mrpt::poses::CPosePDF &o) const MRPT_OVERRIDE;

			/** Returns the particle with the highest weight.
			  */
			CPose2D	 getMostLikelyParticle() const;

			/** Bayesian fusion.
			  */
			void  bayesianFusion( const mrpt::poses::CPosePDF &p1, const mrpt::poses::CPosePDF &p2, const double&minMahalanobisDistToDrop = 0 ) MRPT_OVERRIDE;

			/** Evaluates the PDF at a given arbitrary point as reconstructed by a Parzen window.
			  * \sa saveParzenPDFToTextFile
			  */
			double  evaluatePDF_parzen(
				float	x,
				float	y,
				float	phi,
				float	stdXY,
				float	stdPhi ) const;

			/** Save a text file (compatible with matlab) representing the 2D evaluation of the PDF as reconstructed by a Parzen window.
			  * \sa evaluatePDF_parzen
			  */
			void  saveParzenPDFToTextFile(
				const char		*fileName,
				float			x_min,
				float			x_max,
				float			y_min,
				float			y_max,
				float			phi,
				float			stepSizeXY,
				float			stdXY,
				float			stdPhi ) const;

			void  bayesianFusion( mrpt::poses::CPosePDF &p1, mrpt::poses::CPosePDF &p2, const double&minMahalanobisDistToDrop = 0)
			{ THROW_EXCEPTION("Not implemented"); }

		private:

			void  offsetTransitionModel( double &val );

			/** Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm.
			  */
			mutable mrpt::math::CVectorDouble				m_pfAuxiliaryPFOptimal_estimatedProb;

			/** Auxiliary function that evaluates the likelihood of an observation, given a robot pose, and according to the options in "CPosePDFParticlesExtended::options".
			  */
			static double  auxiliarComputeObservationLikelihood(
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const bayes::CParticleFilterCapable	*obj,
				size_t			particleIndexForMap,
				const mrpt::obs::CSensoryFrame	*observation,
				const TExtendedCPose2D			*x );

			/** Auxiliary function used in "prediction_and_update_pfAuxiliaryPFOptimal"
				*/
			static double  particlesEvaluator_AuxPFOptimal(
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const bayes::CParticleFilterCapable	*obj,
				size_t		index,
				const void	* action,
				const void	* observation );


		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CPosePDFParticlesExtended, mrpt::poses::CPosePDF, DUMMY_LINKAGE )

			/** Auxiliary structure
			  */
			struct TPoseBin
			{
				int	x,y,phi;
			};
			/** Auxiliary structure
			  */
			struct lt_TPoseBin
			{
				bool operator()(const TPoseBin& s1, const TPoseBin& s2) const
				{
				return s1.x < s2.x;
				}
			};

	} // End of namespace
} // End of namespace

#endif
