/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef PF_implementations_data_H
#define PF_implementations_data_H

#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/slam/TKLDParams.h>
#include <mrpt/utils/COutputLogger.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		// Frwd decl:
		template <class PARTICLETYPE, class BINTYPE>
		void KLF_loadBinFromParticle(
			BINTYPE				&outBin,
			const TKLDParams  	&opts,
			const PARTICLETYPE 	*currentParticleValue = NULL,
			const mrpt::math::TPose3D		*newPoseToBeInserted = NULL );


		/** A set of common data shared by PF implementations for both SLAM and localization
		  *   \ingroup mrpt_slam_grp
		  */
		template <class PARTICLE_TYPE, class MYSELF>
		class PF_implementation :
			public mrpt::utils::COutputLogger
		{
		public:
			PF_implementation() :
				mrpt::utils::COutputLogger("PF_implementation"),
				m_accumRobotMovement2DIsValid(false),
				m_accumRobotMovement3DIsValid(false)
			{
			}
		protected:
			/** \name Data members and methods used by generic PF implementations
			    @{ */

			mrpt::obs::CActionRobotMovement2D	m_accumRobotMovement2D;
			bool					m_accumRobotMovement2DIsValid;
			mrpt::poses::CPose3DPDFGaussian		m_accumRobotMovement3D;
			bool					m_accumRobotMovement3DIsValid;

			mrpt::poses::CPoseRandomSampler				m_movementDrawer;						//!< Used in al PF implementations. \sa PF_SLAM_implementation_gatherActionsCheckBothActObs
			mutable mrpt::math::CVectorDouble			m_pfAuxiliaryPFOptimal_estimatedProb;	//!< Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm.
			mutable mrpt::math::CVectorDouble			m_pfAuxiliaryPFStandard_estimatedProb;	//!< Auxiliary variable used in the "pfAuxiliaryPFStandard" algorithm.
			mutable mrpt::math::CVectorDouble			m_pfAuxiliaryPFOptimal_maxLikelihood;						//!< Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm.
			mutable std::vector<mrpt::math::TPose3D>	m_pfAuxiliaryPFOptimal_maxLikDrawnMovement;		//!< Auxiliary variable used in the "pfAuxiliaryPFOptimal" algorithm.
			std::vector<bool>				m_pfAuxiliaryPFOptimal_maxLikMovementDrawHasBeenUsed;

			/**  Compute w[i]*p(z_t | mu_t^i), with mu_t^i being
			  *    the mean of the new robot pose
			  *
			  * \param action MUST be a "const CPose3D*"
			  * \param observation MUST be a "const CSensoryFrame*"
			  */
			template <class BINTYPE> // Template arg. actually not used, just to allow giving the definition in another file later on
			static double PF_SLAM_particlesEvaluator_AuxPFStandard(
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const mrpt::bayes::CParticleFilterCapable	*obj,
				size_t index,
				const void * action,
				const void * observation );

			template <class BINTYPE> // Template arg. actually not used, just to allow giving the definition in another file later on
			static double  PF_SLAM_particlesEvaluator_AuxPFOptimal(
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const mrpt::bayes::CParticleFilterCapable	*obj,
				size_t					index,
				const void				*action,
				const void				*observation );

			/** @} */

			/** \name The generic PF implementations for localization & SLAM.
			    @{ */

			/** A generic implementation of the PF method "prediction_and_update_pfAuxiliaryPFOptimal" (optimal sampling with rejection sampling approximation),
			  *  common to both localization and mapping.
			  *
			  * - BINTYPE: TPoseBin or whatever to discretize the sample space for KLD-sampling.
			  *
			  *  This method implements optimal sampling with a rejection sampling-based approximation of the true posterior.
			  *  For details, see the papers:
			  *
			  *  J.L. Blanco, J. Gonzalez, and J.-A. Fernandez-Madrigal,
			  *    "An Optimal Filtering Algorithm for Non-Parametric Observation Models in
			  *     Robot Localization," in Proc. IEEE International Conference on Robotics
			  *     and Automation (ICRA'08), 2008, pp. 461-466.
			  */
			template <class BINTYPE>
			void PF_SLAM_implementation_pfAuxiliaryPFOptimal(
				const mrpt::obs::CActionCollection	* actions,
				const mrpt::obs::CSensoryFrame		* sf,
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const TKLDParams &KLD_options);

			/** A generic implementation of the PF method "prediction_and_update_pfAuxiliaryPFStandard" (Auxiliary particle filter with the standard proposal),
			  *  common to both localization and mapping.
			  *
			  * - BINTYPE: TPoseBin or whatever to discretize the sample space for KLD-sampling.
			  *
			  *  This method is described in the paper:
			  *   Pitt, M.K.; Shephard, N. (1999). "Filtering Via Simulation: Auxiliary Particle Filters".
			  *    Journal of the American Statistical Association 94 (446): 590-591. doi:10.2307/2670179.
			  *
			  */
			template <class BINTYPE>
			void PF_SLAM_implementation_pfAuxiliaryPFStandard(
				const mrpt::obs::CActionCollection	* actions,
				const mrpt::obs::CSensoryFrame		* sf,
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const TKLDParams &KLD_options);


			/** A generic implementation of the PF method "pfStandardProposal" (standard proposal distribution, that is, a simple SIS particle filter),
			  *  common to both localization and mapping.
			  *
			  * - BINTYPE: TPoseBin or whatever to discretize the sample space for KLD-sampling.
			  */
			template <class BINTYPE>
			void PF_SLAM_implementation_pfStandardProposal(
				const mrpt::obs::CActionCollection	* actions,
				const mrpt::obs::CSensoryFrame		* sf,
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const TKLDParams &KLD_options);

			/** @} */


		public:
			/** \name Virtual methods that the PF_implementations assume exist.
			    @{ */

			/** Return a pointer to the last robot pose in the i'th particle (or NULL if it's a path and it's empty). */
			virtual const  mrpt::math::TPose3D * getLastPose(const size_t i) const = 0;

			virtual void PF_SLAM_implementation_custom_update_particle_with_new_pose(
				PARTICLE_TYPE *particleData,
				const mrpt::math::TPose3D &newPose) const = 0;

			/** This is the default algorithm to efficiently replace one old set of samples by another new set.
			  *  The method uses pointers to make fast copies the first time each particle is duplicated, then
			  *   makes real copies for the next ones.
			  *
			  *  Note that more efficient specializations might exist for specific particle data structs.
			  */
			virtual void PF_SLAM_implementation_replaceByNewParticleSet(
				typename mrpt::bayes::CParticleFilterData<PARTICLE_TYPE>::CParticleList	 &old_particles,
				const std::vector<mrpt::math::TPose3D>		&newParticles,
				const std::vector<double>		&newParticlesWeight,
				const std::vector<size_t>		&newParticlesDerivedFromIdx ) const
			{
				// ---------------------------------------------------------------------------------
				// Substitute old by new particle set:
				//   Old are in "m_particles"
				//   New are in "newParticles", "newParticlesWeight","newParticlesDerivedFromIdx"
				// ---------------------------------------------------------------------------------
				const size_t N = newParticles.size();
				typename MYSELF::CParticleList newParticlesArray(N);

				// For efficiency, just copy the "CParticleData" from the old particle into the
				//  new one, but this can be done only once:
				std::vector<bool>	oldParticleAlreadyCopied(old_particles.size(),false);

				size_t i;
				typename MYSELF::CParticleList::iterator	newPartIt;
				for (newPartIt=newParticlesArray.begin(),i=0;newPartIt!=newParticlesArray.end();++newPartIt,++i)
				{
					// The weight:
					newPartIt->log_w = newParticlesWeight[i];

					// The data (CParticleData):
					PARTICLE_TYPE *newPartData;
					if (!oldParticleAlreadyCopied[newParticlesDerivedFromIdx[i]])
					{
						// The first copy of this old particle:
						newPartData = old_particles[newParticlesDerivedFromIdx[i]].d.release();
						oldParticleAlreadyCopied[newParticlesDerivedFromIdx[i]] = true;
					}
					else
					{
						// Make a copy:
						newPartData = new PARTICLE_TYPE( *old_particles[ newParticlesDerivedFromIdx[i] ].d );
					}

					newPartIt->d.reset(newPartData);
				} // end for "newPartIt"

				// Now add the new robot pose to the paths:
				//  (this MUST be done after the above loop, separately):
				// Update the particle with the new pose: this part is caller-dependant and must be implemented there:
				for (newPartIt=newParticlesArray.begin(),i=0;i<N;++newPartIt,++i)
					PF_SLAM_implementation_custom_update_particle_with_new_pose( newPartIt->d.get(), newParticles[i] );

				// Free those old m_particles not being copied into the new ones: not needed since use of smart ptr.

				// Copy into "m_particles"
				old_particles.resize( newParticlesArray.size() );
				typename MYSELF::CParticleList::iterator	trgPartIt;
				for (newPartIt=newParticlesArray.begin(),trgPartIt=old_particles.begin(); newPartIt!=newParticlesArray.end(); ++newPartIt, ++trgPartIt )
				{
					trgPartIt->log_w = newPartIt->log_w;
					trgPartIt->d.move_from( newPartIt->d );
				}
			} // end of PF_SLAM_implementation_replaceByNewParticleSet



			virtual bool PF_SLAM_implementation_doWeHaveValidObservations(
				const typename mrpt::bayes::CParticleFilterData<PARTICLE_TYPE>::CParticleList	&particles,
				const mrpt::obs::CSensoryFrame *sf) const
			{
				MRPT_UNUSED_PARAM(particles); MRPT_UNUSED_PARAM(sf);
				return true;	// By default, always process the SFs.
			}

			/** Make a specialization if needed, eg. in the first step in SLAM.  */
			virtual bool PF_SLAM_implementation_skipRobotMovement() const
			{
				return false; // By default, always allow the robot to move!
			}

			/** Evaluate the observation likelihood for one particle at a given location */
			virtual double PF_SLAM_computeObservationLikelihoodForParticle(
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions	&PF_options,
				const size_t			particleIndexForMap,
				const mrpt::obs::CSensoryFrame		&observation,
				const mrpt::poses::CPose3D			&x )  const = 0;

			/** @} */


		/** Auxiliary method called by PF implementations: return true if we have both action & observation,
		  *   otherwise, return false AND accumulate the odometry so when we have an observation we didn't lose a thing.
		  *   On return=true, the "m_movementDrawer" member is loaded and ready to draw samples of the increment of pose since last step.
		  *  This method is smart enough to accumulate CActionRobotMovement2D or CActionRobotMovement3D, whatever comes in.
		  */
		template <class BINTYPE> // Template arg. actually not used, just to allow giving the definition in another file later on
		bool PF_SLAM_implementation_gatherActionsCheckBothActObs(
			const mrpt::obs::CActionCollection	* actions,
			const mrpt::obs::CSensoryFrame		* sf );

		private:
			/** The shared implementation body of two PF methods: APF and Optimal-APF, depending on USE_OPTIMAL_SAMPLING */
			template <class BINTYPE>
			void PF_SLAM_implementation_pfAuxiliaryPFStandardAndOptimal(
				const mrpt::obs::CActionCollection	* actions,
				const mrpt::obs::CSensoryFrame		* sf,
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				const TKLDParams &KLD_options,
				const bool USE_OPTIMAL_SAMPLING  );

			template <class BINTYPE>
			void PF_SLAM_aux_perform_one_rejection_sampling_step(
				const bool		USE_OPTIMAL_SAMPLING,
				const bool		doResample,
				const double	maxMeanLik,
				size_t    k, // The particle from the old set "m_particles[]"
				const mrpt::obs::CSensoryFrame		* sf,
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions &PF_options,
				mrpt::poses::CPose3D			& out_newPose,
				double			& out_newParticleLogWeight);


		}; // end PF_implementation
	}
}

#endif
