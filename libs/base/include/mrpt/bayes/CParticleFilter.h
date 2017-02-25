/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPARTICLEFILTER_H
#define CPARTICLEFILTER_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CLoadableOptions.h>

namespace mrpt
{
	namespace obs { class CSensoryFrame; class CActionCollection; }

	/** The namespace for Bayesian filtering algorithm: different particle filters and Kalman filter algorithms. \ingroup mrpt_base_grp
	  */
	namespace bayes
	{
		class CParticleFilterCapable;

		/** This class acts as a common interface to the different interfaces (see CParticleFilter::TParticleFilterAlgorithm) any bayes::CParticleFilterCapable class can implement: it is the invoker of particle filter algorithms.
		 *   The particle filter is executed on a probability density function (PDF) described by a CParticleFilterCapable object, passed in the constructor or alternatively through the CParticleFilter::executeOn method.<br>
		 *
		 * For a complete example and further details, see the <a href="http://www.mrpt.org/Particle_Filter_Tutorial" >Particle Filter tutorial</a>.
		 *
		 *   The basic SIR algorithm (pfStandardProposal) consists of:
		 *		- Execute a prediction with the given "action".
		 *		- Update the weights of the particles using the likelihood of the "observation".
		 *		- Normalize weights.
		 *		- Perform resampling if the ESS is below the threshold options.BETA.
		 *
		 * \ingroup mrpt_base_grp
		 * \sa mrpt::poses::CPoseParticlesPDF
		 */
		class BASE_IMPEXP CParticleFilter : public mrpt::utils::COutputLogger
		{
		public:

			/** Defines different types of particle filter algorithms.
			  *  The defined SIR implementations are:
			  *		- pfStandardProposal: Standard proposal distribution + weights according to likelihood function.
			  *		- pfAuxiliaryPFStandard: An auxiliary PF using the standard proposal distribution.
			  *		- pfOptimalProposal: Use the optimal proposal distribution (where available!, usually this will perform approximations)
			  *		- pfAuxiliaryPFOptimal: Use the optimal proposal and a auxiliary particle filter (see <a href="http://www.mrpt.org/Paper:An_Optimal_Filtering_Algorithm_for_Non-Parametric_Observation_Models_in_Robot_Localization_(ICRA_2008)" >paper</a>).
			  *
			  * See the theoretical discussion in <a href="http://www.mrpt.org/Resampling_Schemes" >resampling schemes</a>.
			  */
			enum TParticleFilterAlgorithm
			{
				pfStandardProposal = 0,
				pfAuxiliaryPFStandard,
				pfOptimalProposal,
				pfAuxiliaryPFOptimal
			};

			/** Defines the different resampling algorithms.
			  *  The implemented resampling methods are:
			  *		- prMultinomial (Default): Uses standard select with replacement (draws M random uniform numbers)
			  *		- prResidual: The residual or "remainder" method.
			  *		- prStratified: The stratified resampling, where a uniform sample is drawn for each of M subdivisions of the range (0,1].
			  *		- prSystematic: A single uniform sample is drawn in the range (0,1/M].
			  *
			  * See the theoretical discussion in <a href="http://www.mrpt.org/Resampling_Schemes" >resampling schemes</a>.
			  */
			enum TParticleResamplingAlgorithm
			{
				prMultinomial = 0,
				prResidual,
				prStratified,
				prSystematic
			};

			/** The configuration of a particle filter.
			  */
			struct BASE_IMPEXP TParticleFilterOptions : public mrpt::utils::CLoadableOptions
			{
			public:
				TParticleFilterOptions(); //!< Initilization of default parameters
				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

				bool         adaptiveSampleSize; //!< A flag that indicates whether the CParticleFilterCapable object should perform adative sample size (default=false).
				double       BETA; //!< The resampling of particles will be performed when ESS (in range [0,1]) < BETA (default is 0.5)
				unsigned int sampleSize; //!< The initial number of particles in the filter (it can change only if adaptiveSampleSize=true) (default=1)

				/** In the algorithm "CParticleFilter::pfAuxiliaryPFOptimal" (and in "CParticleFilter::pfAuxiliaryPFStandard" only if pfAuxFilterStandard_FirstStageWeightsMonteCarlo = true) the number of samples for searching the maximum likelihood value and also to estimate the "first stage weights" (see papers!) (default=100)
				  */
				unsigned int pfAuxFilterOptimal_MaximumSearchSamples;
				double		powFactor; //!< An optional step to "smooth" dramatic changes in the observation model to affect the variance of the particle weights, eg weight*=likelihood^powFactor (default=1 = no effects).
				TParticleFilterAlgorithm		PF_algorithm; //!< The PF algorithm to use (default=pfStandardProposal) See TParticleFilterAlgorithm for the posibilities.
				TParticleResamplingAlgorithm	resamplingMethod; //!< The resampling algorithm to use (default=prMultinomial).

				/** Only for PF_algorithm=pfAuxiliaryPFOptimal: If a given particle has a max_likelihood (from the a-priori estimate) below the maximum from all the samples - max_loglikelihood_dyn_range, then the particle is directly discarded.
				  *  This is done to assure that the rejection sampling doesn't get stuck in an infinite loop trying to get an acceptable sample.
				  *  Default = 15 (in logarithmic likelihood)
				  */
				double max_loglikelihood_dyn_range;

				/** Only for PF_algorithm==pfAuxiliaryPFStandard:
				  * If false, the APF will predict the first stage weights just at the mean of the prior of the next time step.
				  * If true, these weights will be estimated as described in the papers for the "pfAuxiliaryPFOptimal" method, i.e. through a monte carlo simulation.
				  *  In that case, "pfAuxFilterOptimal_MaximumSearchSamples" is the number of MC samples used.
				  */
				bool pfAuxFilterStandard_FirstStageWeightsMonteCarlo;

				bool pfAuxFilterOptimal_MLE; //!< (Default=false) In the algorithm "CParticleFilter::pfAuxiliaryPFOptimal", if set to true, do not perform rejection sampling, but just the most-likely (ML) particle found in the preliminary weight-determination stage.
			};

			/** Statistics for being returned from the "execute" method. */
			struct BASE_IMPEXP TParticleFilterStats
			{
				TParticleFilterStats() : ESS_beforeResample(0), weightsVariance_beforeResample (0) {  }
				double		ESS_beforeResample;
				double		weightsVariance_beforeResample;
			};

			/** Default constructor.
			 *   After creating the PF object, set the options in CParticleFilter::m_options, then execute steps through CParticleFilter::executeOn.
			 */
			CParticleFilter();

			virtual ~CParticleFilter() {}

			/** Executes a complete prediction + update step of the selected particle filtering algorithm.
			 *    The member CParticleFilter::m_options must be set before calling this to settle the algorithm parameters.
			 *
			 * \param obj           The object representing the probability distribution function (PDF) which apply the particle filter algorithm to.
			 * \param action		A pointer to an action in the form of a CActionCollection, or NULL if there is no action.
			 * \param observation	A pointer to observations in the form of a CSensoryFrame, or NULL if there is no observation.
			 * \param stats An output structure for gathering statistics of the particle filter execution, or set to NULL if you do not need it (see CParticleFilter::TParticleFilterStats).
			 *
			 * \sa CParticleFilterCapable, executeOn
			 */
			void  executeOn(
				CParticleFilterCapable 			&obj,
				const mrpt::obs::CActionCollection   *action,
				const mrpt::obs::CSensoryFrame	    *observation,
				TParticleFilterStats	        *stats = NULL);


			/** The options to be used in the PF, must be set before executing any step of the particle filter.
			  */
			CParticleFilter::TParticleFilterOptions		m_options;

		}; // End of class def.

	} // end namespace
} // end namespace
#endif
