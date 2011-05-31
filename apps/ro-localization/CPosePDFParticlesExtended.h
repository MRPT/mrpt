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

#ifndef CPosePDFParticlesExtended_H
#define CPosePDFParticlesExtended_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/bayes/CParticleFilterCapable.h>

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CSensoryFrame.h>

namespace mrpt
{
	namespace poses
	{

		class TExtendedCPose2D
		{
		public:
			CPose2D			pose;
			vector<double>	state;
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
			public mrpt::bayes::CParticleFilterCapable,
			public mrpt::bayes::CParticleFilterData<TExtendedCPose2D>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPosePDFParticlesExtended )

			// This uses CParticleFilterData to implement some methods required for CParticleFilterCapable:
			IMPLEMENT_PARTICLE_FILTER_CAPABLE(TExtendedCPose2D);

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
				CMetricMap			*metricMap;

				/** [update stage] Alternative way (if metricMap==NULL): A metric map is supplied for each particle: There must be the same maps here as pose particles.
				  */
				TMetricMapList		metricMaps;

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

	//	 private:
			/** The particles
			 */
			//std::deque<CPose2DParticleExtended>		particles;

			/** Free all the memory associated to particles, and set the number of parts = 0
			  */
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
			void  copyFrom(const CPosePDF &o);

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
										  vector_float	state_min,
										  vector_float	state_max,
										  float phi_min = -M_PI,
										  float phi_max = M_PI,
										  int	particlesCount = -1);


			 /** Returns an estimate of the pose, i.e. a "mean value", computed
			  *   as a weighted average over all particles.
			  */
			void getMean(CPose2D  &p) const;

			 /** Returns an estimate of the pose-state, i.e. a "mean value", computed as a weighted average over all particles.
			  */
			TExtendedCPose2D  getEstimatedPoseState() const;

			/** Returns an estimate of the pose covariance matrix (3x3 cov.matrix  for x,y,phi variables)
			  */
			void getCovarianceAndMean(CMatrixDouble33 &C, CPose2D &p) const;

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
				const mrpt::slam::CActionCollection	* action,
				const mrpt::slam::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

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
				const mrpt::slam::CActionCollection	* actions,
				const mrpt::slam::CSensoryFrame		* sf,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

			/** Save PDF's particles to a text file. In each line it will go: "x y phi weight"
			 */
			void  saveToTextFile(const std::string &file) const;

			/** Get the particles count (equivalent to "particlesCount")
			 */
			size_t  size() const { return m_particles.size(); }

			/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
			  *   "to proyect" the current pdf. Result PDF substituted the currently stored one in the object.
			  */
			void  changeCoordinatesReference( const CPose3D &newReferenceBase );

			/** Draws a single sample from the distribution (WARNING: weights are assumed to be normalized!)
			  */
			void  drawSingleSample(CPose2D &outPart ) const;

			/** Draws a number of samples from the distribution, and saves as a list of 1x3 vectors, where each row contains a (x,y,phi) datum.
			  */
			void  drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const;

			/** Appends (pose-composition) a given pose "p" to each particle
			  */
			void  operator += ( const CPose2D &Ap);

			/** Appends (add to the list) a set of particles to the existing ones, and then normalize weights.
			  */
			//void  append( CPosePDFParticlesExtended &o );

			/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
			  */
			void	 inverse(CPosePDF &o) const;

			/** Returns the particle with the highest weight.
			  */
			CPose2D	 getMostLikelyParticle() const;

			/** Bayesian fusion.
			  */
			void  bayesianFusion( const CPosePDF &p1, const CPosePDF &p2, const double&minMahalanobisDistToDrop = 0 );

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

			void  bayesianFusion( CPosePDF &p1, CPosePDF &p2, const double&minMahalanobisDistToDrop = 0)
			{ THROW_EXCEPTION("Not implemented"); }

		private:

			void  offsetTransitionModel( double &val );

			/** Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm.
			  */
			mutable vector_double				m_pfAuxiliaryPFOptimal_estimatedProb;

			/** Auxiliary function that evaluates the likelihood of an observation, given a robot pose, and according to the options in "CPosePDFParticlesExtended::options".
			  */
			static double  auxiliarComputeObservationLikelihood(
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const CParticleFilterCapable	*obj,
				size_t			particleIndexForMap,
				const CSensoryFrame	*observation,
				const TExtendedCPose2D			*x );

			/** Auxiliary function used in "prediction_and_update_pfAuxiliaryPFOptimal"
				*/
			static double  particlesEvaluator_AuxPFOptimal(
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const CParticleFilterCapable	*obj,
				size_t		index,
				const void	* action,
				const void	* observation );


		}; // End of class def.

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
