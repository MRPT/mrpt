/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/random.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/slam/TKLDParams.h>

#include <mrpt/math/distributions.h>  // chi2inv
#include <mrpt/math/data_utils.h>  // averageLogLikelihood()

#include <mrpt/slam/PF_implementations_data.h>

/** \file PF_implementations.h
 *  This file contains the implementations of the template members declared in
 * mrpt::slam::PF_implementation
 */

namespace mrpt::slam
{
/** Auxiliary method called by PF implementations: return true if we have both
 * action & observation,
 *   otherwise, return false AND accumulate the odometry so when we have an
 * observation we didn't lose a thing.
 *   On return=true, the "m_movementDrawer" member is loaded and ready to draw
 * samples of the increment of pose since last step.
 *  This method is smart enough to accumulate CActionRobotMovement2D or
 * CActionRobotMovement3D, whatever comes in.
 *   \ingroup mrpt_slam_grp
 */
template <
	class PARTICLE_TYPE, class MYSELF,
	mrpt::bayes::particle_storage_mode STORAGE>
template <class BINTYPE>
bool PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>::
	PF_SLAM_implementation_gatherActionsCheckBothActObs(
		const mrpt::obs::CActionCollection* actions,
		const mrpt::obs::CSensoryFrame* sf)
{
	auto* me = static_cast<MYSELF*>(this);

	if (actions != nullptr)  // A valid action?
	{
		mrpt::obs::CActionRobotMovement2D::Ptr robotMovement2D =
			actions->getBestMovementEstimation();
		if (robotMovement2D)
		{
			if (m_accumRobotMovement3DIsValid)
				THROW_EXCEPTION("Mixing 2D and 3D actions is not allowed.");

			ASSERT_(robotMovement2D->poseChange);
			if (!m_accumRobotMovement2DIsValid)
			{  // First time:
				robotMovement2D->poseChange->getMean(
					m_accumRobotMovement2D.rawOdometryIncrementReading);
				m_accumRobotMovement2D.motionModelConfiguration =
					robotMovement2D->motionModelConfiguration;
			}
			else
				m_accumRobotMovement2D.rawOdometryIncrementReading +=
					robotMovement2D->poseChange->getMeanVal();

			m_accumRobotMovement2DIsValid = true;
		}
		else  // If there is no 2D action, look for a 3D action:
		{
			mrpt::obs::CActionRobotMovement3D::Ptr robotMovement3D =
				actions->getActionByClass<mrpt::obs::CActionRobotMovement3D>();
			if (robotMovement3D)
			{
				if (m_accumRobotMovement2DIsValid)
					THROW_EXCEPTION("Mixing 2D and 3D actions is not allowed.");

				if (!m_accumRobotMovement3DIsValid)
					m_accumRobotMovement3D = robotMovement3D->poseChange;
				else
					m_accumRobotMovement3D += robotMovement3D->poseChange;
				// This "+=" takes care of all the Jacobians, etc... You
				// MUST love C++!!! ;-)

				m_accumRobotMovement3DIsValid = true;
			}
			else
				return false;  // We have no actions...
		}
	}

	const bool SFhasValidObservations =
		(sf == nullptr) ? false
						: PF_SLAM_implementation_doWeHaveValidObservations(
							  me->m_particles, sf);

	// All the things we need?
	if (!((m_accumRobotMovement2DIsValid || m_accumRobotMovement3DIsValid) &&
		  SFhasValidObservations))
		return false;

	// Since we're gonna return true, load the pose-drawer:
	// Take the accum. actions as input:
	if (m_accumRobotMovement3DIsValid)
	{
		m_movementDrawer.setPosePDF(
			m_accumRobotMovement3D);  // <--- Set mov. drawer
		m_accumRobotMovement3DIsValid =
			false;  // Reset odometry for next iteration
	}
	else
	{
		mrpt::obs::CActionRobotMovement2D theResultingRobotMov;
		theResultingRobotMov.computeFromOdometry(
			m_accumRobotMovement2D.rawOdometryIncrementReading,
			m_accumRobotMovement2D.motionModelConfiguration);

		ASSERT_(theResultingRobotMov.poseChange);
		m_movementDrawer.setPosePDF(
			theResultingRobotMov.poseChange.get_ptr());  // <--- Set mov. drawer
		m_accumRobotMovement2DIsValid =
			false;  // Reset odometry for next iteration
	}
	return true;
}  // end of PF_SLAM_implementation_gatherActionsCheckBothActObs

/** A generic implementation of the PF method
 * "prediction_and_update_pfAuxiliaryPFOptimal" (optimal sampling with rejection
 * sampling approximation),
 *  common to both localization and mapping.
 *
 * - BINTYPE: TPoseBin or whatever to discretize the sample space for
 * KLD-sampling.
 *
 *  This method implements optimal sampling with a rejection sampling-based
 * approximation of the true posterior.
 *  For details, see the papers:
 *
 *  J.-L. Blanco, J. Gonzalez, and J.-A. Fernandez-Madrigal,
 *    "An Optimal Filtering Algorithm for Non-Parametric Observation Models in
 *     Robot Localization," in Proc. IEEE International Conference on Robotics
 *     and Automation (ICRA'08), 2008, pp. 461466.
 */
template <
	class PARTICLE_TYPE, class MYSELF,
	mrpt::bayes::particle_storage_mode STORAGE>
template <class BINTYPE>
void PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>::
	PF_SLAM_implementation_pfAuxiliaryPFOptimal(
		const mrpt::obs::CActionCollection* actions,
		const mrpt::obs::CSensoryFrame* sf,
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const TKLDParams& KLD_options)
{
	// Standard and Optimal AuxiliaryPF actually have a shared implementation
	// body:
	PF_SLAM_implementation_pfAuxiliaryPFStandardAndOptimal<BINTYPE>(
		actions, sf, PF_options, KLD_options, true /*Optimal PF*/);
}

/** A generic implementation of the PF method "pfStandardProposal" (standard
 * proposal distribution, that is, a simple SIS particle filter),
 *  common to both localization and mapping.
 *
 * - BINTYPE: TPoseBin or whatever to discretize the sample space for
 * KLD-sampling.
 */
template <
	class PARTICLE_TYPE, class MYSELF,
	mrpt::bayes::particle_storage_mode STORAGE>
template <class BINTYPE>
void PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>::
	PF_SLAM_implementation_pfStandardProposal(
		const mrpt::obs::CActionCollection* actions,
		const mrpt::obs::CSensoryFrame* sf,
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const TKLDParams& KLD_options)
{
	MRPT_START
	using TSetStateSpaceBins = std::set<BINTYPE, typename BINTYPE::lt_operator>;

	auto* me = static_cast<MYSELF*>(this);

	// In this method we don't need the
	// "PF_SLAM_implementation_gatherActionsCheckBothActObs" machinery,
	//  since prediction & update are two independent stages well separated for
	//  this algorithm.

	// --------------------------------------------------------------------------------------
	//  Prediction: Simply draw samples from the motion model
	// --------------------------------------------------------------------------------------
	if (actions)
	{
		// Find a robot movement estimation:
		mrpt::poses::CPose3D motionModelMeanIncr;
		{
			mrpt::obs::CActionRobotMovement2D::Ptr robotMovement2D =
				actions->getBestMovementEstimation();
			// If there is no 2D action, look for a 3D action:
			if (robotMovement2D)
			{
				ASSERT_(robotMovement2D->poseChange);
				m_movementDrawer.setPosePDF(
					robotMovement2D->poseChange.get_ptr());
				motionModelMeanIncr = mrpt::poses::CPose3D(
					robotMovement2D->poseChange->getMeanVal());
			}
			else
			{
				mrpt::obs::CActionRobotMovement3D::Ptr robotMovement3D =
					actions
						->getActionByClass<mrpt::obs::CActionRobotMovement3D>();
				if (robotMovement3D)
				{
					m_movementDrawer.setPosePDF(robotMovement3D->poseChange);
					robotMovement3D->poseChange.getMean(motionModelMeanIncr);
				}
				else
				{
					THROW_EXCEPTION(
						"Action list does not contain any "
						"CActionRobotMovement2D or CActionRobotMovement3D "
						"object!");
				}
			}
		}

		// Update particle poses:
		if (!PF_options.adaptiveSampleSize)
		{
			const size_t M = me->m_particles.size();
			// -------------------------------------------------------------
			// FIXED SAMPLE SIZE
			// -------------------------------------------------------------
			mrpt::poses::CPose3D incrPose;
			for (size_t i = 0; i < M; i++)
			{
				// Generate gaussian-distributed 2D-pose increments according to
				// mean-cov:
				m_movementDrawer.drawSample(incrPose);
				bool pose_is_valid;
				const mrpt::poses::CPose3D finalPose =
					mrpt::poses::CPose3D(getLastPose(i, pose_is_valid)) +
					incrPose;

				// Update the particle with the new pose: this part is
				// caller-dependant and must be implemented there:
				if constexpr (
					STORAGE == mrpt::bayes::particle_storage_mode::POINTER)
				{
					PF_SLAM_implementation_custom_update_particle_with_new_pose(
						me->m_particles[i].d.get(), finalPose.asTPose());
				}
				else
				{
					PF_SLAM_implementation_custom_update_particle_with_new_pose(
						&me->m_particles[i].d, finalPose.asTPose());
				}
			}
		}
		else
		{
			// -------------------------------------------------------------
			//   ADAPTIVE SAMPLE SIZE
			// Implementation of Dieter Fox's KLD algorithm
			//  31-Oct-2006 (JLBC): First version
			//  19-Jan-2009 (JLBC): Rewriten within a generic template
			// -------------------------------------------------------------
			TSetStateSpaceBins stateSpaceBins;

			size_t Nx = KLD_options.KLD_minSampleSize;
			const double delta_1 = 1.0 - KLD_options.KLD_delta;
			const double epsilon_1 = 0.5 / KLD_options.KLD_epsilon;

			// Prepare data for executing "fastDrawSample"
			me->prepareFastDrawSample(PF_options);

			// The new particle set:
			std::vector<mrpt::math::TPose3D> newParticles;
			std::vector<double> newParticlesWeight;
			std::vector<size_t> newParticlesDerivedFromIdx;

			mrpt::poses::CPose3D increment_i;
			size_t N = 1;

			do  // THE MAIN DRAW SAMPLING LOOP
			{
				// Draw a robot movement increment:
				m_movementDrawer.drawSample(increment_i);

				// generate the new particle:
				const size_t drawn_idx = me->fastDrawSample(PF_options);

				bool pose_is_valid;
				const mrpt::poses::CPose3D newPose =
					mrpt::poses::CPose3D(
						getLastPose(drawn_idx, pose_is_valid)) +
					increment_i;
				const mrpt::math::TPose3D newPose_s = newPose.asTPose();

				// Add to the new particles list:
				newParticles.push_back(newPose_s);
				newParticlesWeight.push_back(0);
				newParticlesDerivedFromIdx.push_back(drawn_idx);

				// Now, look if the particle falls in a new bin or not:
				// --------------------------------------------------------
				const PARTICLE_TYPE* part;
				if constexpr (
					STORAGE == mrpt::bayes::particle_storage_mode::POINTER)
					part = me->m_particles[drawn_idx].d.get();
				else
					part = &me->m_particles[drawn_idx].d;

				BINTYPE p;
				KLF_loadBinFromParticle<PARTICLE_TYPE, BINTYPE>(
					p, KLD_options, part, &newPose_s);

				if (stateSpaceBins.find(p) == stateSpaceBins.end())
				{
					// It falls into a new bin:
					// Add to the stateSpaceBins:
					stateSpaceBins.insert(p);

					// K = K + 1
					size_t K = stateSpaceBins.size();
					if (K > 1)  //&& newParticles.size() >
					// options.KLD_minSampleSize )
					{
						// Update the number of m_particles!!
						Nx = round(epsilon_1 * math::chi2inv(delta_1, K - 1));
						// printf("k=%u \tn=%u \tNx:%u\n", k,
						// newParticles.size(), Nx);
					}
				}
				N = newParticles.size();
			} while (N < std::max(Nx, (size_t)KLD_options.KLD_minSampleSize) &&
					 N < KLD_options.KLD_maxSampleSize);

			// ---------------------------------------------------------------------------------
			// Substitute old by new particle set:
			//   Old are in "m_particles"
			//   New are in "newParticles",
			//   "newParticlesWeight","newParticlesDerivedFromIdx"
			// ---------------------------------------------------------------------------------
			this->PF_SLAM_implementation_replaceByNewParticleSet(
				me->m_particles, newParticles, newParticlesWeight,
				newParticlesDerivedFromIdx);

		}  // end adaptive sample size
	}

	if (sf)
	{
		const size_t M = me->m_particles.size();
		//	UPDATE STAGE
		// ----------------------------------------------------------------------
		// Compute all the likelihood values & update particles weight:
		for (size_t i = 0; i < M; i++)
		{
			bool pose_is_valid;
			const mrpt::math::TPose3D partPose =
				getLastPose(i, pose_is_valid);  // Take the particle data:
			auto partPose2 = mrpt::poses::CPose3D(partPose);
			const double obs_log_likelihood =
				PF_SLAM_computeObservationLikelihoodForParticle(
					PF_options, i, *sf, partPose2);
			me->m_particles[i].log_w +=
				obs_log_likelihood * PF_options.powFactor;
		}  // for each particle "i"

		// Normalization of weights is done outside of this method
		// automatically.
	}

	MRPT_END
}  // end of PF_SLAM_implementation_pfStandardProposal

/** A generic implementation of the PF method
 * "prediction_and_update_pfAuxiliaryPFStandard" (Auxiliary particle filter with
 * the standard proposal),
 *  common to both localization and mapping.
 *
 * - BINTYPE: TPoseBin or whatever to discretize the sample space for
 * KLD-sampling.
 *
 *  This method is described in the paper:
 *   Pitt, M.K.; Shephard, N. (1999). "Filtering Via Simulation: Auxiliary
 * Particle Filters".
 *    Journal of the American Statistical Association 94 (446): 590-591.
 * doi:10.2307/2670179.
 *
 */
template <
	class PARTICLE_TYPE, class MYSELF,
	mrpt::bayes::particle_storage_mode STORAGE>
template <class BINTYPE>
void PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>::
	PF_SLAM_implementation_pfAuxiliaryPFStandard(
		const mrpt::obs::CActionCollection* actions,
		const mrpt::obs::CSensoryFrame* sf,
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const TKLDParams& KLD_options)
{
	// Standard and Optimal AuxiliaryPF actually have a shared implementation
	// body:
	PF_SLAM_implementation_pfAuxiliaryPFStandardAndOptimal<BINTYPE>(
		actions, sf, PF_options, KLD_options, false /*APF*/);
}

/*---------------------------------------------------------------
			PF_SLAM_particlesEvaluator_AuxPFOptimal
 ---------------------------------------------------------------*/
template <
	class PARTICLE_TYPE, class MYSELF,
	mrpt::bayes::particle_storage_mode STORAGE>
template <class BINTYPE>
double PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>::
	PF_SLAM_particlesEvaluator_AuxPFOptimal(
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const mrpt::bayes::CParticleFilterCapable* obj, size_t index,
		const void* action, const void* observation)
{
	MRPT_UNUSED_PARAM(action);
	MRPT_START

	// const PF_implementation<PARTICLE_TYPE,MYSELF> *myObj =
	// reinterpret_cast<const PF_implementation<PARTICLE_TYPE,MYSELF>*>( obj );
	const auto* me = static_cast<const MYSELF*>(obj);

	// Compute the quantity:
	//     w[i]*p(zt|z^{t-1},x^{[i],t-1})
	// As the Monte-Carlo approximation of the integral over all posible $x_t$.
	// --------------------------------------------
	double indivLik, maxLik = -1e300;
	mrpt::poses::CPose3D maxLikDraw;
	size_t N = PF_options.pfAuxFilterOptimal_MaximumSearchSamples;
	ASSERT_(N > 1);

	bool pose_is_valid;
	const mrpt::poses::CPose3D oldPose =
		mrpt::poses::CPose3D(me->getLastPose(index, pose_is_valid));
	mrpt::math::CVectorDouble vectLiks(
		N, 0);  // The vector with the individual log-likelihoods.
	mrpt::poses::CPose3D drawnSample;
	for (size_t q = 0; q < N; q++)
	{
		me->m_movementDrawer.drawSample(drawnSample);
		mrpt::poses::CPose3D x_predict = oldPose + drawnSample;

		// Estimate the mean...
		indivLik = me->PF_SLAM_computeObservationLikelihoodForParticle(
			PF_options, index,
			*static_cast<const mrpt::obs::CSensoryFrame*>(observation),
			x_predict);

		MRPT_CHECK_NORMAL_NUMBER(indivLik);
		vectLiks[q] = indivLik;
		if (indivLik > maxLik)
		{  // Keep the maximum value:
			maxLikDraw = drawnSample;
			maxLik = indivLik;
		}
	}

	// This is done to avoid floating point overflow!!
	//      average_lik    =      \sum(e^liks)   * e^maxLik  /     N
	// log( average_lik  ) = log( \sum(e^liks) ) + maxLik   - log( N )
	double avrgLogLik = math::averageLogLikelihood(vectLiks);

	// Save into the object:
	me->m_pfAuxiliaryPFOptimal_estimatedProb[index] =
		avrgLogLik;  // log( accum / N );
	me->m_pfAuxiliaryPFOptimal_maxLikelihood[index] = maxLik;

	if (PF_options.pfAuxFilterOptimal_MLE)
		me->m_pfAuxiliaryPFOptimal_maxLikDrawnMovement[index] =
			maxLikDraw.asTPose();

	// and compute the resulting probability of this particle:
	// ------------------------------------------------------------
	return me->m_particles[index].log_w +
		   me->m_pfAuxiliaryPFOptimal_estimatedProb[index];

	MRPT_END
}  // end of PF_SLAM_particlesEvaluator_AuxPFOptimal

/**  Compute w[i]*p(z_t | mu_t^i), with mu_t^i being
 *    the mean of the new robot pose
 *
 * \param action MUST be a "const mrpt::poses::CPose3D*"
 * \param observation MUST be a "const CSensoryFrame*"
 */
template <
	class PARTICLE_TYPE, class MYSELF,
	mrpt::bayes::particle_storage_mode STORAGE>
template <class BINTYPE>
double PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>::
	PF_SLAM_particlesEvaluator_AuxPFStandard(
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const mrpt::bayes::CParticleFilterCapable* obj, size_t index,
		const void* action, const void* observation)
{
	MRPT_START

	// const PF_implementation<PARTICLE_TYPE,MYSELF> *myObj =
	// reinterpret_cast<const PF_implementation<PARTICLE_TYPE,MYSELF>*>( obj );
	const auto* myObj = static_cast<const MYSELF*>(obj);

	// Take the previous particle weight:
	const double cur_logweight = myObj->m_particles[index].log_w;
	bool pose_is_valid;
	const mrpt::poses::CPose3D oldPose =
		mrpt::poses::CPose3D(myObj->getLastPose(index, pose_is_valid));

	if (!PF_options.pfAuxFilterStandard_FirstStageWeightsMonteCarlo)
	{
		// Just use the mean:
		// , take the mean of the posterior density:
		mrpt::poses::CPose3D x_predict;
		x_predict.composeFrom(
			oldPose, *static_cast<const mrpt::poses::CPose3D*>(action));

		// and compute the obs. likelihood:
		// --------------------------------------------
		myObj->m_pfAuxiliaryPFStandard_estimatedProb[index] =
			myObj->PF_SLAM_computeObservationLikelihoodForParticle(
				PF_options, index,
				*static_cast<const mrpt::obs::CSensoryFrame*>(observation),
				x_predict);

		// Combined log_likelihood: Previous weight * obs_likelihood:
		return cur_logweight +
			   myObj->m_pfAuxiliaryPFStandard_estimatedProb[index];
	}
	else
	{
		// Do something similar to in Optimal sampling:
		// Compute the quantity:
		//     w[i]*p(zt|z^{t-1},x^{[i],t-1})
		// As the Monte-Carlo approximation of the integral over all posible
		// $x_t$.
		// --------------------------------------------
		double indivLik, maxLik = -1e300;
		mrpt::poses::CPose3D maxLikDraw;
		size_t N = PF_options.pfAuxFilterOptimal_MaximumSearchSamples;
		ASSERT_(N > 1);

		mrpt::math::CVectorDouble vectLiks(
			N, 0);  // The vector with the individual log-likelihoods.
		mrpt::poses::CPose3D drawnSample;
		for (size_t q = 0; q < N; q++)
		{
			myObj->m_movementDrawer.drawSample(drawnSample);
			mrpt::poses::CPose3D x_predict = oldPose + drawnSample;

			// Estimate the mean...
			indivLik = myObj->PF_SLAM_computeObservationLikelihoodForParticle(
				PF_options, index,
				*static_cast<const mrpt::obs::CSensoryFrame*>(observation),
				x_predict);

			MRPT_CHECK_NORMAL_NUMBER(indivLik);
			vectLiks[q] = indivLik;
			if (indivLik > maxLik)
			{  // Keep the maximum value:
				maxLikDraw = drawnSample;
				maxLik = indivLik;
			}
		}

		// This is done to avoid floating point overflow!!
		//      average_lik    =      \sum(e^liks)   * e^maxLik  /     N
		// log( average_lik  ) = log( \sum(e^liks) ) + maxLik   - log( N )
		double avrgLogLik = math::averageLogLikelihood(vectLiks);

		// Save into the object:
		myObj->m_pfAuxiliaryPFStandard_estimatedProb[index] =
			avrgLogLik;  // log( accum / N );

		myObj->m_pfAuxiliaryPFOptimal_maxLikelihood[index] = maxLik;
		if (PF_options.pfAuxFilterOptimal_MLE)
			myObj->m_pfAuxiliaryPFOptimal_maxLikDrawnMovement[index] =
				maxLikDraw.asTPose();

		// and compute the resulting probability of this particle:
		// ------------------------------------------------------------
		return cur_logweight +
			   myObj->m_pfAuxiliaryPFOptimal_estimatedProb[index];
	}
	MRPT_END
}

// USE_OPTIMAL_SAMPLING:
//   true -> PF_SLAM_implementation_pfAuxiliaryPFOptimal
//  false -> PF_SLAM_implementation_pfAuxiliaryPFStandard
template <
	class PARTICLE_TYPE, class MYSELF,
	mrpt::bayes::particle_storage_mode STORAGE>
template <class BINTYPE>
void PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>::
	PF_SLAM_implementation_pfAuxiliaryPFStandardAndOptimal(
		const mrpt::obs::CActionCollection* actions,
		const mrpt::obs::CSensoryFrame* sf,
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const TKLDParams& KLD_options, const bool USE_OPTIMAL_SAMPLING)
{
	MRPT_START
	using TSetStateSpaceBins = std::set<BINTYPE, typename BINTYPE::lt_operator>;

	auto* me = static_cast<MYSELF*>(this);

	const size_t M = me->m_particles.size();

	// ----------------------------------------------------------------------
	//	  We can execute optimal PF only when we have both, an action, and
	//     a valid observation from which to compute the likelihood:
	//   Accumulate odometry/actions until we have a valid observation, then
	//    process them simultaneously.
	// ----------------------------------------------------------------------
	if (!PF_SLAM_implementation_gatherActionsCheckBothActObs<BINTYPE>(
			actions, sf))
		return;  // Nothing we can do here...
	// OK, we have m_movementDrawer loaded and observations...let's roll!

	// -------------------------------------------------------------------------------
	//		0) Common part:  Prepare m_particles "draw" and compute
	//"fastDrawSample"
	// -------------------------------------------------------------------------------
	// We need the (aproximate) maximum likelihood value for each
	//  previous particle [i]:
	//     max{ p( z^t | data^[i], x_(t-1)^[i], u_(t) ) }
	//

	m_pfAuxiliaryPFOptimal_maxLikelihood.assign(M, INVALID_LIKELIHOOD_VALUE);
	m_pfAuxiliaryPFOptimal_maxLikDrawnMovement.resize(M);
	m_pfAuxiliaryPFOptimal_estimatedProb.resize(M);
	m_pfAuxiliaryPFStandard_estimatedProb.resize(M);

	// Pass the "mean" robot movement to the "weights" computing function:
	mrpt::poses::CPose3D meanRobotMovement;
	m_movementDrawer.getSamplingMean3D(meanRobotMovement);

	// Prepare data for executing "fastDrawSample"
	using TMyClass = PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>;
	auto funcOpt =
		&TMyClass::template PF_SLAM_particlesEvaluator_AuxPFOptimal<BINTYPE>;
	auto funcStd =
		&TMyClass::template PF_SLAM_particlesEvaluator_AuxPFStandard<BINTYPE>;

	me->prepareFastDrawSample(
		PF_options, USE_OPTIMAL_SAMPLING ? funcOpt : funcStd,
		&meanRobotMovement, sf);

	// For USE_OPTIMAL_SAMPLING=1,  m_pfAuxiliaryPFOptimal_maxLikelihood is now
	// computed.

	if (USE_OPTIMAL_SAMPLING &&
		me->isLoggingLevelVisible(mrpt::system::LVL_DEBUG))
	{
		me->logStr(
			mrpt::system::LVL_DEBUG,
			mrpt::format(
				"[prepareFastDrawSample] max      (log) = %10.06f\n",
				math::maximum(m_pfAuxiliaryPFOptimal_estimatedProb)));
		me->logStr(
			mrpt::system::LVL_DEBUG,
			mrpt::format(
				"[prepareFastDrawSample] max-mean (log) = %10.06f\n",
				-math::mean(m_pfAuxiliaryPFOptimal_estimatedProb) +
					math::maximum(m_pfAuxiliaryPFOptimal_estimatedProb)));
		me->logStr(
			mrpt::system::LVL_DEBUG,
			mrpt::format(
				"[prepareFastDrawSample] max-min  (log) = %10.06f\n",
				-math::minimum(m_pfAuxiliaryPFOptimal_estimatedProb) +
					math::maximum(m_pfAuxiliaryPFOptimal_estimatedProb)));
	}

	// Now we have the vector "m_fastDrawProbability" filled out with:
	//               w[i]*p(zt|z^{t-1},x^{[i],t-1},X)
	//  where,
	//
	//  =========== For USE_OPTIMAL_SAMPLING = true ====================
	//  X is the robot pose prior (as implemented in
	//  the aux. function "PF_SLAM_particlesEvaluator_AuxPFOptimal"),
	//  and also the "m_pfAuxiliaryPFOptimal_maxLikelihood" filled with the
	//  maximum lik. values.
	//
	//  =========== For USE_OPTIMAL_SAMPLING = false ====================
	//  X is a single point close to the mean of the robot pose prior (as
	//  implemented in
	//  the aux. function "PF_SLAM_particlesEvaluator_AuxPFStandard").
	//
	std::vector<mrpt::math::TPose3D> newParticles;
	std::vector<double> newParticlesWeight;
	std::vector<size_t> newParticlesDerivedFromIdx;

	// We need the (aproximate) maximum likelihood value for each
	//  previous particle [i]:
	//
	//     max{ p( z^t | data^[i], x_(t-1)^[i], u_(t) ) }
	//
	if (PF_options.pfAuxFilterOptimal_MLE)
		m_pfAuxiliaryPFOptimal_maxLikMovementDrawHasBeenUsed.assign(M, false);

	const double maxMeanLik = math::maximum(
		USE_OPTIMAL_SAMPLING ? m_pfAuxiliaryPFOptimal_estimatedProb
							 : m_pfAuxiliaryPFStandard_estimatedProb);

	if (!PF_options.adaptiveSampleSize)
	{
		// ----------------------------------------------------------------------
		//						1) FIXED SAMPLE SIZE VERSION
		// ----------------------------------------------------------------------
		newParticles.resize(M);
		newParticlesWeight.resize(M);
		newParticlesDerivedFromIdx.resize(M);

		const bool doResample = me->ESS() < PF_options.BETA;

		for (size_t i = 0; i < M; i++)
		{
			size_t k;

			// Generate a new particle:
			//   (a) Draw a "t-1" m_particles' index:
			// ----------------------------------------------------------------
			if (doResample)
				k = me->fastDrawSample(
					PF_options);  // Based on weights of last step only!
			else
				k = i;

			// Do one rejection sampling step:
			// ---------------------------------------------
			mrpt::poses::CPose3D newPose;
			double newParticleLogWeight;
			PF_SLAM_aux_perform_one_rejection_sampling_step<BINTYPE>(
				USE_OPTIMAL_SAMPLING, doResample, maxMeanLik, k, sf, PF_options,
				newPose, newParticleLogWeight);

			// Insert the new particle
			newParticles[i] = newPose.asTPose();
			newParticlesDerivedFromIdx[i] = k;
			newParticlesWeight[i] = newParticleLogWeight;

		}  // for i
	}  // end fixed sample size
	else
	{
		// -------------------------------------------------------------------------------------------------
		//				 				2) ADAPTIVE SAMPLE SIZE VERSION
		//
		//	Implementation of Dieter Fox's KLD algorithm
		//		JLBC (3/OCT/2006)
		// -------------------------------------------------------------------------------------------------
		// The new particle set:
		newParticles.clear();
		newParticlesWeight.resize(0);
		newParticlesDerivedFromIdx.clear();

		// ------------------------------------------------------------------------------
		// 2.1) PRELIMINARY STAGE: Build a list of
		// pairs<TPathBin,std::vector<uint32_t>>
		// with the
		//      indexes of m_particles that fall into each
		//      multi-dimensional-path bins
		//      //The bins will be saved into "stateSpaceBinsLastTimestep", and
		//      the list
		//      //of corresponding m_particles (in the last timestep), in
		//      "stateSpaceBinsLastTimestepParticles"
		//  - Added JLBC (01/DEC/2006)
		// ------------------------------------------------------------------------------
		TSetStateSpaceBins stateSpaceBinsLastTimestep;
		std::vector<std::vector<uint32_t>> stateSpaceBinsLastTimestepParticles;
		typename MYSELF::CParticleList::iterator partIt;
		unsigned int partIndex;

		me->logStr(mrpt::system::LVL_DEBUG, "[FIXED_SAMPLING] Computing...");
		for (partIt = me->m_particles.begin(), partIndex = 0;
			 partIt != me->m_particles.end(); ++partIt, ++partIndex)
		{
			// Load the bin from the path data:
			const PARTICLE_TYPE* part;
			if constexpr (
				STORAGE == mrpt::bayes::particle_storage_mode::POINTER)
				part = partIt->d.get();
			else
				part = &partIt->d;

			BINTYPE p;
			KLF_loadBinFromParticle<PARTICLE_TYPE, BINTYPE>(
				p, KLD_options, part);

			// Is it a new bin?
			auto posFound = stateSpaceBinsLastTimestep.find(p);
			if (posFound == stateSpaceBinsLastTimestep.end())
			{  // Yes, create a new pair <bin,index_list> in the list:
				stateSpaceBinsLastTimestep.insert(p);
				stateSpaceBinsLastTimestepParticles.emplace_back(1, partIndex);
			}
			else
			{  // No, add the particle's index to the existing entry:
				const size_t idx =
					std::distance(stateSpaceBinsLastTimestep.begin(), posFound);
				stateSpaceBinsLastTimestepParticles[idx].push_back(partIndex);
			}
		}
		me->logStr(
			mrpt::system::LVL_DEBUG,
			mrpt::format(
				"[FIXED_SAMPLING] done (%u bins in t-1)\n",
				(unsigned int)stateSpaceBinsLastTimestep.size()));

		// ------------------------------------------------------------------------------
		// 2.2)    THE MAIN KLD-BASED DRAW SAMPLING LOOP
		// ------------------------------------------------------------------------------
		double delta_1 = 1.0 - KLD_options.KLD_delta;
		double epsilon_1 = 0.5 / KLD_options.KLD_epsilon;
		bool doResample = me->ESS() < PF_options.BETA;
		// double	maxLik =
		// math::maximum(m_pfAuxiliaryPFOptimal_maxLikelihood);
		// // For normalization purposes only

		// The desired dynamic number of m_particles (to be modified dynamically
		// below):
		const size_t minNumSamples_KLD = std::max(
			(size_t)KLD_options.KLD_minSampleSize,
			(size_t)round(
				KLD_options.KLD_minSamplesPerBin *
				stateSpaceBinsLastTimestep.size()));
		size_t Nx = minNumSamples_KLD;

		const size_t Np1 = me->m_particles.size();
		std::vector<size_t> oldPartIdxsStillNotPropragated(
			Np1);  // Use a list since we'll use "erase" a lot here.
		for (size_t k = 0; k < Np1; k++)
			oldPartIdxsStillNotPropragated[k] = k;  //.push_back(k);

		const size_t Np = stateSpaceBinsLastTimestepParticles.size();
		std::vector<size_t> permutationPathsAuxVector(Np);
		for (size_t k = 0; k < Np; k++) permutationPathsAuxVector[k] = k;

		// Instead of picking randomly from "permutationPathsAuxVector", we can
		// shuffle it now just once,
		// then pick in sequence from the tail and resize the container:
		mrpt::random::shuffle(
			permutationPathsAuxVector.begin(), permutationPathsAuxVector.end());

		size_t k = 0;
		size_t N = 0;

		TSetStateSpaceBins stateSpaceBins;

		do  // "N" is the index of the current "new particle":
		{
			// Generate a new particle:
			//
			//   (a) Propagate the last set of m_particles, and only if the
			//       desired number of m_particles in this step is larger,
			//       perform a UNIFORM sampling from the last set. In this way
			//       the new weights can be computed in the same way for all
			//       m_particles.
			// ---------------------------------------------------------------------------
			if (doResample)
			{
				k = me->fastDrawSample(
					PF_options);  // Based on weights of last step only!
			}
			else
			{
				// Assure that at least one particle per "discrete-path" is
				// taken (if the
				//  number of samples allows it)
				if (permutationPathsAuxVector.size())
				{
					const size_t idxBinSpacePath =
						*permutationPathsAuxVector.rbegin();
					permutationPathsAuxVector.resize(
						permutationPathsAuxVector.size() - 1);

					const size_t idx =
						mrpt::random::getRandomGenerator().drawUniform32bit() %
						stateSpaceBinsLastTimestepParticles[idxBinSpacePath]
							.size();
					k = stateSpaceBinsLastTimestepParticles[idxBinSpacePath]
														   [idx];
					ASSERT_(k < me->m_particles.size());

					// Also erase it from the other permutation vector list:
					oldPartIdxsStillNotPropragated.erase(std::find(
						oldPartIdxsStillNotPropragated.begin(),
						oldPartIdxsStillNotPropragated.end(), k));
				}
				else
				{
					// Select a particle from the previous set with a UNIFORM
					// distribution,
					// in such a way we will assign each particle the updated
					// weight accounting
					// for its last weight.
					// The first "old_N" m_particles will be drawn using a
					// uniform random selection
					// without repetitions:
					//
					// Select a index from "oldPartIdxsStillNotPropragated" and
					// remove it from the list:
					if (oldPartIdxsStillNotPropragated.size())
					{
						const size_t idx =
							mrpt::random::getRandomGenerator()
								.drawUniform32bit() %
							oldPartIdxsStillNotPropragated.size();
						auto it = oldPartIdxsStillNotPropragated.begin() +
								  idx;  // advance(it,idx);
						k = *it;
						oldPartIdxsStillNotPropragated.erase(it);
					}
					else
					{
						// N>N_old -> Uniformly draw index:
						k = mrpt::random::getRandomGenerator()
								.drawUniform32bit() %
							me->m_particles.size();
					}
				}
			}

			// Do one rejection sampling step:
			// ---------------------------------------------
			mrpt::poses::CPose3D newPose;
			double newParticleLogWeight;
			PF_SLAM_aux_perform_one_rejection_sampling_step<BINTYPE>(
				USE_OPTIMAL_SAMPLING, doResample, maxMeanLik, k, sf, PF_options,
				newPose, newParticleLogWeight);

			// Insert the new particle
			newParticles.push_back(newPose.asTPose());
			newParticlesDerivedFromIdx.push_back(k);
			newParticlesWeight.push_back(newParticleLogWeight);

			// ----------------------------------------------------------------
			// Now, the KLD-sampling dynamic sample size stuff:
			//  look if the particle's PATH falls into a new bin or not:
			// ----------------------------------------------------------------
			const PARTICLE_TYPE* part;
			if constexpr (
				STORAGE == mrpt::bayes::particle_storage_mode::POINTER)
				part = me->m_particles[k].d.get();
			else
				part = &me->m_particles[k].d;

			BINTYPE p;
			const mrpt::math::TPose3D newPose_s = newPose.asTPose();
			KLF_loadBinFromParticle<PARTICLE_TYPE, BINTYPE>(
				p, KLD_options, part, &newPose_s);

			// -----------------------------------------------------------------------------
			// Look for the bin "p" into "stateSpaceBins": If it is not yet into
			// the set,
			//  then we may increase the desired particle number:
			// -----------------------------------------------------------------------------

			// Found?
			if (stateSpaceBins.find(p) == stateSpaceBins.end())
			{
				// It falls into a new bin: add to the stateSpaceBins:
				stateSpaceBins.insert(p);

				// K = K + 1
				int K = stateSpaceBins.size();
				if (K > 1)
				{
					// Update the number of m_particles!!
					Nx = (size_t)(epsilon_1 * math::chi2inv(delta_1, K - 1));
					// printf("k=%u \tn=%u \tNx:%u\n", k, newParticles.size(),
					// Nx);
				}
			}

			N = newParticles.size();

		} while ((N < KLD_options.KLD_maxSampleSize &&
				  N < std::max(Nx, minNumSamples_KLD)) ||
				 (permutationPathsAuxVector.size() && !doResample));

		me->logStr(
			mrpt::system::LVL_DEBUG,
			mrpt::format(
				"[ADAPTIVE SAMPLE SIZE]  #Bins: %u \t #Particles: %u \t "
				"Nx=%u\n",
				static_cast<unsigned>(stateSpaceBins.size()),
				static_cast<unsigned>(N), (unsigned)Nx));
	}  // end adaptive sample size

	// ---------------------------------------------------------------------------------
	// Substitute old by new particle set:
	//   Old are in "m_particles"
	//   New are in "newParticles",
	//   "newParticlesWeight","newParticlesDerivedFromIdx"
	// ---------------------------------------------------------------------------------
	this->PF_SLAM_implementation_replaceByNewParticleSet(
		me->m_particles, newParticles, newParticlesWeight,
		newParticlesDerivedFromIdx);

	// In this PF_algorithm, we must do weight normalization by ourselves:
	me->normalizeWeights();

	MRPT_END
}  // end of PF_SLAM_implementation_pfAuxiliaryPFStandardAndOptimal

/* ------------------------------------------------------------------------
					PF_SLAM_aux_perform_one_rejection_sampling_step
   ------------------------------------------------------------------------ */
template <
	class PARTICLE_TYPE, class MYSELF,
	mrpt::bayes::particle_storage_mode STORAGE>
template <class BINTYPE>
void PF_implementation<PARTICLE_TYPE, MYSELF, STORAGE>::
	PF_SLAM_aux_perform_one_rejection_sampling_step(
		const bool USE_OPTIMAL_SAMPLING, const bool doResample,
		const double maxMeanLik,
		size_t k,  // The particle from the old set "m_particles[]"
		const mrpt::obs::CSensoryFrame* sf,
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		mrpt::poses::CPose3D& out_newPose, double& out_newParticleLogWeight)
{
	auto* me = static_cast<MYSELF*>(this);

	// ADD-ON: If the 'm_pfAuxiliaryPFOptimal_estimatedProb[k]' is
	// **extremelly** low relative to the other m_particles,
	//  resample only this particle with a copy of another one, uniformly:
	while (((USE_OPTIMAL_SAMPLING ? m_pfAuxiliaryPFOptimal_estimatedProb[k]
								  : m_pfAuxiliaryPFStandard_estimatedProb[k]) -
			maxMeanLik) < -PF_options.max_loglikelihood_dyn_range)
	{
		// Select another 'k' uniformly:
		k = mrpt::random::getRandomGenerator().drawUniform32bit() %
			me->m_particles.size();
		me->logStr(
			mrpt::system::LVL_DEBUG,
			"[PF_SLAM_aux_perform_one_rejection_sampling_step] Warning: "
			"Discarding very unlikely particle.");
	}

	bool pose_is_valid;
	const mrpt::poses::CPose3D oldPose = mrpt::poses::CPose3D(
		getLastPose(k, pose_is_valid));  // current pose of the k'th particle
	// ASSERT_(pose_is_valid); Use the default (0,0,0) if path is empty.

	//   (b) Rejection-sampling: Draw a new robot pose from x[k],
	//       and accept it with probability p(zk|x) / maxLikelihood:
	// ----------------------------------------------------------------
	double poseLogLik;
	if (PF_SLAM_implementation_skipRobotMovement())
	{
		// The first robot pose in the SLAM execution: All m_particles start
		// at the same point (this is the lowest bound of subsequent
		// uncertainty):
		out_newPose = oldPose;
		poseLogLik = 0;
	}
	else
	{
		mrpt::poses::CPose3D movementDraw;
		if (!USE_OPTIMAL_SAMPLING)
		{  // APF:
			m_movementDrawer.drawSample(movementDraw);
			out_newPose.composeFrom(
				oldPose, movementDraw);  // newPose = oldPose + movementDraw;
			// Compute likelihood:
			poseLogLik = PF_SLAM_computeObservationLikelihoodForParticle(
				PF_options, k, *sf, out_newPose);
		}
		else
		{  // Optimal APF with rejection sampling:
			// Rejection-sampling:
			double acceptanceProb;
			int timeout = 0;
			const int maxTries = 10000;
			double bestTryByNow_loglik = -std::numeric_limits<double>::max();
			mrpt::math::TPose3D bestTryByNow_pose;
			do
			{
				// Draw new robot pose:
				if (PF_options.pfAuxFilterOptimal_MLE &&
					!m_pfAuxiliaryPFOptimal_maxLikMovementDrawHasBeenUsed[k])
				{  // No! first take advantage of a good drawn value, but only
					// once!!
					m_pfAuxiliaryPFOptimal_maxLikMovementDrawHasBeenUsed[k] =
						true;
					movementDraw = mrpt::poses::CPose3D(
						m_pfAuxiliaryPFOptimal_maxLikDrawnMovement[k]);
				}
				else
				{
					// Draw new robot pose:
					m_movementDrawer.drawSample(movementDraw);
				}

				out_newPose.composeFrom(
					oldPose,
					movementDraw);  // out_newPose = oldPose + movementDraw;

				// Compute acceptance probability:
				poseLogLik = PF_SLAM_computeObservationLikelihoodForParticle(
					PF_options, k, *sf, out_newPose);

				if (poseLogLik > bestTryByNow_loglik)
				{
					bestTryByNow_loglik = poseLogLik;
					bestTryByNow_pose = out_newPose.asTPose();
				}

				double ratioLikLik = std::exp(
					poseLogLik - m_pfAuxiliaryPFOptimal_maxLikelihood[k]);
				acceptanceProb = std::min(1.0, ratioLikLik);

				if (ratioLikLik > 1)
				{
					m_pfAuxiliaryPFOptimal_maxLikelihood[k] =
						poseLogLik;  //  :'-( !!!
					// acceptanceProb = 0;		// Keep searching or keep this
					// one?
				}
			} while (
				++timeout < maxTries &&
				acceptanceProb <
					mrpt::random::getRandomGenerator().drawUniform(0.0, 0.999));

			if (timeout >= maxTries)
			{
				out_newPose = mrpt::poses::CPose3D(bestTryByNow_pose);
				poseLogLik = bestTryByNow_loglik;
				me->logStr(
					mrpt::system::LVL_WARN,
					"[PF_implementation] Warning: timeout in rejection "
					"sampling.");
			}
		}

		// And its weight:
		if (USE_OPTIMAL_SAMPLING)
		{  // Optimal PF
			if (doResample)
				out_newParticleLogWeight = 0;  // By definition of our optimal
			// PF, all samples have identical
			// weights.
			else
			{
				const double weightFact =
					m_pfAuxiliaryPFOptimal_estimatedProb[k] *
					PF_options.powFactor;
				out_newParticleLogWeight =
					me->m_particles[k].log_w + weightFact;
			}
		}
		else
		{  // APF:
			const double weightFact =
				(poseLogLik - m_pfAuxiliaryPFStandard_estimatedProb[k]) *
				PF_options.powFactor;
			if (doResample)
				out_newParticleLogWeight = weightFact;
			else
				out_newParticleLogWeight =
					weightFact + me->m_particles[k].log_w;
		}
	}
	// Done.
}
}  // namespace mrpt::slam
