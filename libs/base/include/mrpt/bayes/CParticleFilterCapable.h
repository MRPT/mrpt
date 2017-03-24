/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPARTICLEFILTERCAPABLE_H
#define CPARTICLEFILTERCAPABLE_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/bayes/CParticleFilter.h>

namespace mrpt
{
namespace bayes
{
	#define INVALID_LIKELIHOOD_VALUE  (-1e300)   // An invalid log-likelihood value, used to signal non-initialized likelihood variables.

	/** This virtual class defines the interface that any particles based PDF class must implement in order to be executed by a mrpt::bayes::CParticleFilter.
	 *
	 * See the <a href="http://www.mrpt.org/Particle_Filter_Tutorial" >Particle Filter tutorial</a> explaining how to use the particle filter-related classes.
	 * \sa CParticleFilter, CParticleFilterData
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CParticleFilterCapable
	{
		friend class CParticleFilter;

	private:
		static const unsigned PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS;

	public:

		CParticleFilterCapable() : m_fastDrawAuxiliary()
		{ }


        /** Virtual destructor
          */
        virtual ~CParticleFilterCapable()
		{
		}

		/** A callback function type for evaluating the probability of m_particles of being selected, used in "fastDrawSample".
		  *  The default evaluator function "defaultEvaluator" simply returns the particle weight.
		  * \param index This is the index of the particle its probability is being computed.
		  * \param action The value of this is the parameter passed to "prepareFastDrawSample"
		  * \param observation The value of this is the parameter passed to "prepareFastDrawSample"
		  *  The action and the observation are declared as "void*" for a greater flexibility.
		  * \sa prepareFastDrawSample
		  */
		typedef double ( *TParticleProbabilityEvaluator) (
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
			const CParticleFilterCapable	*obj,
			size_t					index,
			const void	* action,
			const void	* observation );

		/** The default evaluator function, which simply returns the particle weight.
		  *  The action and the observation are declared as "void*" for a greater flexibility.
		  * \sa prepareFastDrawSample
		  */
		static double  defaultEvaluator(
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
			const CParticleFilterCapable	*obj,
			size_t				index,
			const void	* action,
			const void	* observation )
		{
			MRPT_UNUSED_PARAM(PF_options); MRPT_UNUSED_PARAM(action); MRPT_UNUSED_PARAM(observation);
			return obj->getW(index);
		}

		/** Prepares data structures for calling fastDrawSample method next.
		  *  This method must be called once before using "fastDrawSample" (calling this more than once has no effect, but it takes time for nothing!)
		  *  The behavior depends on the configuration of the PF (see CParticleFilter::TParticleFilterOptions):
		  *		- <b>DYNAMIC SAMPLE SIZE=NO</b>: In this case this method fills out an internal array (m_fastDrawAuxiliary.alreadyDrawnIndexes) with
		  *			the random indexes generated according to the selected resample scheme in TParticleFilterOptions. Those indexes are
		  *			read sequentially by subsequent calls to fastDrawSample.
		  *		- <b>DYNAMIC SAMPLE SIZE=YES</b>: Then:
		  *			- If TParticleFilterOptions.resamplingMethod = prMultinomial, the internal buffers will be filled out (m_fastDrawAuxiliary.CDF, CDF_indexes & PDF) and
		  *				then fastDrawSample can be called an arbitrary number of times to generate random indexes.
		  *			- For the rest of resampling algorithms, an exception will be raised since they are not appropriate for a dynamic (unknown in advance) number of particles.
		  *
		  * The function pointed by "partEvaluator" should take into account the particle filter algorithm selected in "m_PFAlgorithm".
		  * If called without arguments (defaultEvaluator), the default behavior is to draw samples with a probability proportional to their current weights.
		  *  The action and the observation are declared as "void*" for a greater flexibility.
		  *  For a more detailed information see the <a href="http://www.mrpt.org/Particle_Filters" >Particle Filter tutorial</a>.
		  *  Custom supplied "partEvaluator" functions must take into account the previous particle weight, i.e. multiplying the current observation likelihood by the weights.
		  * \sa fastDrawSample
		  */
		void  prepareFastDrawSample(
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
			TParticleProbabilityEvaluator partEvaluator = defaultEvaluator,
			const void	* action = NULL,
			const void	* observation = NULL
			) const;

		/** Draws a random sample from the particle filter, in such a way that each particle has a probability proportional to its weight (in the standard PF algorithm).
		  *   This method can be used to generate a variable number of m_particles when resampling: to vary the number of m_particles in the filter.
		  *   See prepareFastDrawSample for more information, or the <a href="http://www.mrpt.org/Particle_Filters" >Particle Filter tutorial</a>.
		  *
		  * NOTES:
		  *		- You MUST call "prepareFastDrawSample" ONCE before calling this method. That method must be called after modifying the particle filter (executing one step, resampling, etc...)
		  *		- This method returns ONE index for the selected ("drawn") particle, in the range [0,M-1]
		  *		- You do not need to call "normalizeWeights" before calling this.
		  * \sa prepareFastDrawSample
		  */
		size_t  fastDrawSample( const bayes::CParticleFilter::TParticleFilterOptions &PF_options  ) const;

		/** Access to i'th particle (logarithm) weight, where first one is index 0.
		 */
		virtual double  getW(size_t i) const = 0;

		/** Modifies i'th particle (logarithm) weight, where first one is index 0.
		 */
		virtual void  setW(size_t i, double w) = 0;

		/** Get the m_particles count.
		 */
		virtual size_t particlesCount() const = 0;

		/** Performs the prediction stage of the Particle Filter.
		 *  This method simply selects the appropiate protected method according to the particle filter algorithm to run.
		 * \sa prediction_and_update_pfStandardProposal,prediction_and_update_pfAuxiliaryPFStandard,prediction_and_update_pfOptimalProposal,prediction_and_update_pfAuxiliaryPFOptimal
		 */
		void  prediction_and_update(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options
			 );

		/**  Performs the substitution for internal use of resample in particle filter algorithm, don't call it directly.
		 *  \param indx The indices of current m_particles to be saved as the new m_particles set.
		 */
		virtual void  performSubstitution( const std::vector<size_t> &indx) = 0;

		/** Normalize the (logarithmic) weights, such as the maximum weight is zero.
		 * \param out_max_log_w If provided, will return with the maximum log_w before normalizing, such as new_weights = old_weights - max_log_w.
		 * \return The max/min ratio of weights ("dynamic range")
		 */
		virtual double  normalizeWeights( double *out_max_log_w = NULL ) =0;

		/** Returns the normalized ESS (Estimated Sample Size), in the range [0,1].
		  *  Note that you do NOT need to normalize the weights before calling this.
		 */
		virtual double ESS() const = 0;

		/** Performs a resample of the m_particles, using the method selected in the constructor.
		  * After computing the surviving samples, this method internally calls "performSubstitution" to actually perform the particle replacement.
		  * This method is called automatically by CParticleFilter::execute, andshould not be invoked manually normally.
		  * To just obtaining the sequence of resampled indexes from a sequence of weights, use "resample"
		  * \param[in] out_particle_count The desired number of output particles after resampling; 0 means don't modify the current number.
		  * \sa resample
		  */
		void  performResampling( const bayes::CParticleFilter::TParticleFilterOptions &PF_options,size_t out_particle_count = 0 );

		/** A static method to perform the computation of the samples resulting from resampling a given set of particles, given their logarithmic weights, and a resampling method.
		  * It returns the sequence of indexes from the resampling. The number of output samples is the same than the input population.
		  *  This generic method just computes these indexes, to actually perform a resampling in a particle filter object, call performResampling
		  * \param[in] out_particle_count The desired number of output particles after resampling; 0 means don't modify the current number.
		  * \sa performResampling
		  */
		static void computeResampling(
			CParticleFilter::TParticleResamplingAlgorithm	method,
			const std::vector<double>	&in_logWeights,
			std::vector<size_t>			&out_indexes,
			size_t          out_particle_count = 0
			);

		/** A static method to compute the linear, normalized (the sum the unity) weights from log-weights.
		  * \sa performResampling
		  */
		static void log2linearWeights(
			const std::vector<double>	&in_logWeights,
			std::vector<double>		&out_linWeights );


	protected:
		/** Performs the particle filter prediction/update stages for the algorithm "pfStandardProposal" (if not implemented in heritated class, it will raise a 'non-implemented' exception).
		 * \sa prediction_and_update
		 */
		virtual void  prediction_and_update_pfStandardProposal(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options );
		/** Performs the particle filter prediction/update stages for the algorithm "pfAuxiliaryPFStandard" (if not implemented in heritated class, it will raise a 'non-implemented' exception).
		 * \sa prediction_and_update
		 */
		virtual void  prediction_and_update_pfAuxiliaryPFStandard(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options );
		/** Performs the particle filter prediction/update stages for the algorithm "pfOptimalProposal" (if not implemented in heritated class, it will raise a 'non-implemented' exception).
		 * \sa prediction_and_update
		 */
		virtual void  prediction_and_update_pfOptimalProposal(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options );
		/** Performs the particle filter prediction/update stages for the algorithm "pfAuxiliaryPFOptimal" (if not implemented in heritated class, it will raise a 'non-implemented' exception).
		 * \sa prediction_and_update
		 */
		virtual void  prediction_and_update_pfAuxiliaryPFOptimal(
			const mrpt::obs::CActionCollection	* action,
			const mrpt::obs::CSensoryFrame		* observation,
			const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

		/** Auxiliary vectors, see CParticleFilterCapable::prepareFastDrawSample for more information
		  */
		struct BASE_IMPEXP TFastDrawAuxVars
		{
			TFastDrawAuxVars() :
				CDF(),
				CDF_indexes(),
				PDF(),
				alreadyDrawnIndexes(),
				alreadyDrawnNextOne(0)
			{ }

			std::vector<double>	CDF;
			vector_uint		CDF_indexes;
			std::vector<double>	PDF;

			vector_uint		alreadyDrawnIndexes;
			size_t			alreadyDrawnNextOne;
		};

		/** Auxiliary vectors, see CParticleFilterCapable::prepareFastDrawSample for more information
		  */
		mutable TFastDrawAuxVars	m_fastDrawAuxiliary;

	}; // End of class def.

	} // end namespace
} // end namespace
#endif
