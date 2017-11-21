/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/hmtslam/CRobotPosesGraph.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/slam/CICP.h>

#include <mrpt/random.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileStream.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilter_impl.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CTicTac.h>

#include <functional>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::maps;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace std;

namespace mrpt
{
namespace hmtslam
{
class LSLAMAuxiliaryPFOptimal
{
public:
	static constexpr bool DoesResampling = false;
	MRPT_TODO("MOVE predicition_and_update code here");
};

class LSLAMOptimalProposal
{
   public:
	static constexpr bool DoesResampling = true;
	MRPT_TODO("MOVE prediction_and_update here");
};

}


namespace bayes
{
template <>
void CParticleFilter::executeOn<CLocalMetricHypothesis>(
	CLocalMetricHypothesis& obj, const mrpt::obs::CActionCollection* action,
	const mrpt::obs::CSensoryFrame* observation, TParticleFilterStats* stats)
{
	switch (m_options.PF_algorithm)
	{
		case CParticleFilter::pfOptimalProposal:
			executeOn<
				CLocalMetricHypothesis, mrpt::hmtslam::LSLAMOptimalProposal>(
				obj, action, observation, stats);
			break;
		case CParticleFilter::pfAuxiliaryPFOptimal:
			executeOn<
				CLocalMetricHypothesis, mrpt::hmtslam::LSLAMAuxiliaryPFOptimal>(
				obj, action, observation, stats);
			break;
		default:
		{
			THROW_EXCEPTION("Invalid particle filter algorithm selection!");
		}
		break;
	}
}
}  // namespace bayes

namespace hmtslam
{
// Constructor
CLSLAM_RBPF_2DLASER::CLSLAM_RBPF_2DLASER(CHMTSLAM* parent)
	: CLSLAMAlgorithmBase(parent)
{
}
// Destructor
CLSLAM_RBPF_2DLASER::~CLSLAM_RBPF_2DLASER() {}
/*---------------------------------------------------------------

					CLSLAM_RBPF_2DLASER

	Implements a 2D local SLAM method based on a RBPF
		over an occupancy grid map. A part of HMT-SLAM.

\param LMH   The local metric hypothesis which must be updated by this SLAM
algorithm.
\param act   The action to process (or nullptr).
\param sf    The observations to process (or nullptr).

 WE ALREADY HAVE CONTROL OVER THE CRITICAL SECTION OF THE LMHs!

--------------------------------------------------------------- */
void CLSLAM_RBPF_2DLASER::processOneLMH(
	CLocalMetricHypothesis* LMH, const CActionCollection::Ptr& actions,
	const CSensoryFrame::Ptr& sf)
{
	MRPT_START

	// Get the current robot pose estimation:
	TPoseID currentPoseID = LMH->m_currentRobotPose;

	// If this is the first iteration, just create a new robot pose at the
	// origin:
	if (currentPoseID == POSEID_INVALID)
	{
		currentPoseID = CHMTSLAM::generatePoseID();
		LMH->m_currentRobotPose = currentPoseID;  // Change it in the LMH

		// Create a new robot pose:
		CPose3D initPose(0, 0, 0);

		ASSERT_(LMH->m_poseParticles.m_particles.size() > 0);
		for (CLocalMetricHypothesis::CParticleList::iterator it =
				 LMH->m_poseParticles.m_particles.begin();
			 it != LMH->m_poseParticles.m_particles.end(); ++it)
			it->d->robotPoses[currentPoseID] = initPose;

		ASSERT_(m_parent->m_map.nodeCount() == 1);

		m_parent->m_map_cs.lock();
		CHMHMapNode::Ptr firstArea = m_parent->m_map.getFirstNode();
		ASSERT_(firstArea);
		LMH->m_nodeIDmemberships[currentPoseID] = firstArea->getID();

		// Set annotation for the reference pose:
		firstArea->m_annotations.setElemental(
			NODE_ANNOTATION_REF_POSEID, currentPoseID, LMH->m_ID);
		m_parent->m_map_cs.unlock();
	}

	bool insertNewRobotPose = false;
	if (sf)
	{
		if (LMH->m_nodeIDmemberships.size() < 2)  // If there is just 1 node
		// (the current robot pose),
		// then there is no
		// observation in the map yet!
		{  // Update map if this is the first observation!
			insertNewRobotPose = true;
		}
		else
		{
			// Check minimum distance from current robot pose to those in the
			// neighborhood:
			TMapPoseID2Pose3D lstRobotPoses;
			LMH->getMeans(lstRobotPoses);

			CPose3D* currentRobotPose = &lstRobotPoses[currentPoseID];
			float minDistLin = 1e6f;
			float minDistAng = 1e6f;

			// printf("Poses in graph:\n");
			for (TMapPoseID2Pose3D::iterator it = lstRobotPoses.begin();
				 it != lstRobotPoses.end(); ++it)
			{
				if (it->first != currentPoseID)
				{
					float linDist = it->second.distanceTo(*currentRobotPose);
					float angDist = fabs(math::wrapToPi(
						it->second.yaw() - currentRobotPose->yaw()));

					minDistLin = min(minDistLin, linDist);

					if (linDist < m_parent->m_options.SLAM_MIN_DIST_BETWEEN_OBS)
						minDistAng = min(minDistAng, angDist);
				}
			}

			// time to insert a new node??
			insertNewRobotPose =
				(minDistLin > m_parent->m_options.SLAM_MIN_DIST_BETWEEN_OBS) ||
				(minDistAng > m_parent->m_options.SLAM_MIN_HEADING_BETWEEN_OBS);
		}

	}  // end if there are SF

	// Save data in members so PF callback "prediction_and_update_pfXXXX" have
	// access to them:
	m_insertNewRobotPose = insertNewRobotPose;

	// ------------------------------------------------
	//  Execute RBPF method:
	// 	1) PROCESS ACTION
	// 	2) PROCESS OBSERVATIONS
	// ------------------------------------------------
	CParticleFilter pf;
	pf.m_options = m_parent->m_options.pf_options;
	pf.executeOn(*LMH, actions.get(), sf.get());

	// 3) The appearance observation: update the log likelihood
	// ...

	// -----------------------------------------------------------
	//			4) UPDATE THE MAP
	// -----------------------------------------------------------
	if (insertNewRobotPose)
	{
		m_parent->logStr(
			mrpt::utils::LVL_INFO,
			"[CLSLAM_RBPF_2DLASER] Adding new pose...\n");

		//	Leave the up-to-now "current pose" in the map, insert the SF in it,
		// and...
		// ----------------------------------------------------------------------------
		TPoseID newCurrentPoseID = CHMTSLAM::generatePoseID();

		//	...and create a new "current pose" making a copy of the previous
		// one:
		//     and insert the observations into the metric maps:
		// ----------------------------------------------------------------------------
		for (CLocalMetricHypothesis::CParticleList::iterator partIt =
				 LMH->m_poseParticles.m_particles.begin();
			 partIt != LMH->m_poseParticles.m_particles.end(); partIt++)
		{
			const CPose3D* curRobotPose = &partIt->d->robotPoses[currentPoseID];
			partIt->d->robotPoses[newCurrentPoseID] = *curRobotPose;
			sf->insertObservationsInto(&partIt->d->metricMaps, curRobotPose);
		}

		// Add node membership: for now, a copy of the current pose:
		LMH->m_nodeIDmemberships[newCurrentPoseID] =
			LMH->m_nodeIDmemberships[currentPoseID];

		// Now, insert the SF in the just added robot pose:
		// -----------------------------------------------------
		LMH->m_SFs[currentPoseID] = *sf;

		// Sets the new poseID as current robot pose:
		// ----------------------------------------------------
		TPoseID newlyAddedPose = currentPoseID;
		currentPoseID = LMH->m_currentRobotPose = newCurrentPoseID;

		// Mark the "newlyAddedPose" as pending to reconsider in the graph-cut
		// method
		//  (Done in the main loop @ LSLAM thread)
		// --------------------------------------------------------------------------------
		LMH->m_posesPendingAddPartitioner.push_back(newlyAddedPose);

		m_parent->logFmt(
			mrpt::utils::LVL_INFO, "[CLSLAM_RBPF_2DLASER] Added pose %i.\n",
			(int)newlyAddedPose);

		// Notice LC detectors:
		// ------------------------------
		{
			std::lock_guard<std::mutex> lock(m_parent->m_topLCdets_cs);

			for (std::deque<CTopLCDetectorBase*>::iterator it =
					 m_parent->m_topLCdets.begin();
				 it != m_parent->m_topLCdets.end(); ++it)
				(*it)->OnNewPose(newlyAddedPose, sf.get());
		}

	}  // end of insertNewRobotPose

	MRPT_END
}

/** The PF algorithm implementation for "optimal sampling for non-parametric
 * observation models"
 */
template <>
void CLocalMetricHypothesis::prediction_and_update<LSLAMAuxiliaryPFOptimal>(
	const mrpt::obs::CActionCollection* actions,
	const mrpt::obs::CSensoryFrame* sf,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	MRPT_START

	// Get the current robot pose estimation:
	TPoseID currentPoseID = m_currentRobotPose;

	size_t i, k, N, M = m_poseParticles.m_particles.size();

	// ----------------------------------------------------------------------
	//	  We can execute optimal PF only when we have both, an action, and
	//     a valid observation from which to compute the likelihood:
	//   Accumulate odometry/actions until we have a valid observation, then
	//    process them simultaneously.
	// ----------------------------------------------------------------------
	//	static CActionRobotMovement2D	accumRobotMovement;
	//	static bool						accumRobotMovementIsValid = false;
	bool SFhasValidObservations = false;
	// A valid action?
	if (actions != nullptr)
	{
		CActionRobotMovement2D::Ptr act =
			actions->getBestMovementEstimation();  // Find a robot movement
		// estimation:
		if (!act)
			THROW_EXCEPTION(
				"Action list does not contain any CActionRobotMovement2D "
				"derived object!");

		if (!m_accumRobotMovementIsValid)  // Reset accum.
		{
			act->poseChange->getMean(
				m_accumRobotMovement.rawOdometryIncrementReading);
			m_accumRobotMovement.motionModelConfiguration =
				act->motionModelConfiguration;
		}
		else
			m_accumRobotMovement.rawOdometryIncrementReading =
				m_accumRobotMovement.rawOdometryIncrementReading +
				act->poseChange->getMeanVal();

		m_accumRobotMovementIsValid = true;
	}

	if (sf != nullptr)
	{
		ASSERT_(m_poseParticles.m_particles.size() > 0);
		SFhasValidObservations =
			(*m_poseParticles.m_particles.begin())
				.d->metricMaps.canComputeObservationsLikelihood(*sf);
	}

	// All the needed things?
	if (!m_accumRobotMovementIsValid || !SFhasValidObservations)
		return;  // Nothing we can do here...

	// OK, we have all we need, let's start!

	// Take the accum. actions as input:
	CActionRobotMovement2D theResultingRobotMov;

	// Over
	keep_max(
		m_accumRobotMovement.motionModelConfiguration.gaussianModel
			.minStdXY,
		m_parent->m_options.MIN_ODOMETRY_STD_XY);
	keep_max(
		m_accumRobotMovement.motionModelConfiguration.gaussianModel
			.minStdPHI,
		m_parent->m_options.MIN_ODOMETRY_STD_PHI);

	theResultingRobotMov.computeFromOdometry(
		m_accumRobotMovement.rawOdometryIncrementReading,
		m_accumRobotMovement.motionModelConfiguration);

	const CActionRobotMovement2D* robotMovement = &theResultingRobotMov;

	m_accumRobotMovementIsValid =
		false;  // To reset odometry at next iteration!

	// ----------------------------------------------------------------------
	//		0) Common part:  Prepare m_poseParticles.m_particles "draw" and compute
	// ----------------------------------------------------------------------
	// Precompute a list of "random" samples from the movement model:
	m_movementDraws.clear();

	// Fast pseudorandom generator of poses...
	// m_movementDraws.resize(
	// max(2000,(int)(PF_options.pfAuxFilterOptimal_MaximumSearchSamples *
	// 5.6574) ) );
	m_movementDraws.resize(
		PF_options.pfAuxFilterOptimal_MaximumSearchSamples * M);
	size_t size_movementDraws = m_movementDraws.size();
	m_movementDrawsIdx = (unsigned int)floor(
		getRandomGenerator().drawUniform(0.0f, ((float)size_movementDraws) - 0.01f));

	robotMovement->prepareFastDrawSingleSamples();
	for (size_t i = 0; i < m_movementDraws.size(); i++)
		robotMovement->fastDrawSingleSample(m_movementDraws[i]);

	m_pfAuxiliaryPFOptimal_estimatedProb.resize(M);
	m_maxLikelihood.clear();
	m_maxLikelihood.resize(M, 0);
	m_movementDrawMaximumLikelihood.resize(M);

	// Prepare data for executing "fastDrawSample"
	CTicTac tictac;
	tictac.Tic();
	using namespace std::placeholders;
	m_poseParticles.prepareFastDrawSample(
		PF_options, [&](size_t i)
		{
			return this->particlesEvaluator_AuxPFOptimal(PF_options, i, sf);
		});
	printf("[prepareFastDrawSample] Done in %.06f ms\n", tictac.Tac() * 1e3f);

#if 0
	printf("[prepareFastDrawSample] max      (log) = %10.06f\n",  math::maximum(m_poseParticles.m_pfAuxiliaryPFOptimal_estimatedProb) );
	printf("[prepareFastDrawSample] max-mean (log) = %10.06f\n", -math::mean(m_poseParticles.m_pfAuxiliaryPFOptimal_estimatedProb) + math::maximum(m_poseParticles.m_pfAuxiliaryPFOptimal_estimatedProb) );
	printf("[prepareFastDrawSample] max-min  (log) = %10.06f\n", -math::minimum(m_poseParticles.m_pfAuxiliaryPFOptimal_estimatedProb) + math::maximum(m_poseParticles.m_pfAuxiliaryPFOptimal_estimatedProb) );
#endif

	// Now we have the vector "m_fastDrawProbability" filled out with:
	//     w[i]p(zt|z^{t-1},x^{[i],t-1},X)
	//  where X is the robot pose prior (as implemented in
	//  the aux. function "particlesEvaluator_AuxPFOptimal"),
	//  and also the "m_maxLikelihood" filled with the maximum lik. values.
	StdVector_CPose2D newParticles;
	vector<double> newParticlesWeight;
	vector<size_t> newParticlesDerivedFromIdx;

	// We need the (aproximate) maximum likelihood value for each
	//  previous particle [i]:
	//
	//     max{ p( z^t | data^[i], x_(t-1)^[i], u_(t) ) }
	//
	// CVectorDouble					maxLikelihood(M, -1 );

	float MIN_ACCEPT_UNIF_DISTRIB = 0.00f;

	CPose2D movementDraw, newPose, oldPose;
	double acceptanceProb, newPoseLikelihood, ratioLikLik;
	unsigned int statsTrialsCount = 0, statsTrialsSuccess = 0;
	std::vector<bool> maxLikMovementDrawHasBeenUsed(M, false);
	unsigned int statsWarningsAccProbAboveOne = 0;
	// double							maxMeanLik = math::maximum(
	// m_poseParticles.m_pfAuxiliaryPFOptimal_estimatedProb ); // For normalization
	// purposes only

	ASSERT_(!PF_options.adaptiveSampleSize);

	// ----------------------------------------------------------------------
	//						1) FIXED SAMPLE SIZE VERSION
	// ----------------------------------------------------------------------
	newParticles.resize(M);
	newParticlesWeight.resize(M);
	newParticlesDerivedFromIdx.resize(M);

	bool doResample = m_poseParticles.ESS() < 0.5;

	for (i = 0; i < M; i++)
	{
		// Generate a new particle:
		//   (a) Draw a "t-1" m_poseParticles.m_particles' index:
		// ----------------------------------------------------------------
		if (doResample)
			k = m_poseParticles.fastDrawSample(
				PF_options);  // Based on weights of last step only!
		else
			k = i;

		oldPose = CPose2D(*getCurrentPose(k));

		//   (b) Rejection-sampling: Draw a new robot pose from x[k],
		//       and accept it with probability p(zk|x) / maxLikelihood:
		// ----------------------------------------------------------------
		if (m_SFs.empty())
		{
			// The first robot pose in the SLAM execution: All m_poseParticles.m_particles start
			// at the same point (this is the lowest bound of subsequent
			// uncertainty):
			movementDraw = CPose2D(0, 0, 0);
			newPose = oldPose;  // + movementDraw;
		}
		else
		{
			// Rejection-sampling:
			do
			{
				// Draw new robot pose:
				if (!maxLikMovementDrawHasBeenUsed[k])
				{
					// No! first take advantage of a good drawn value, but only
					// once!!
					maxLikMovementDrawHasBeenUsed[k] = true;
					movementDraw = m_movementDrawMaximumLikelihood[k];
#if 0
					cout << "Drawn pose (max. lik): " << movementDraw << endl;
#endif
				}
				else
				{
					// Draw new robot pose:
					// robotMovement->drawSingleSample( movementDraw );
					// robotMovement->fastDrawSingleSample( movementDraw );
					movementDraw =
						m_movementDraws[m_movementDrawsIdx++ % size_movementDraws];
				}

				newPose.composeFrom(
					oldPose,
					movementDraw);  // newPose = oldPose + movementDraw;

				// Compute acceptance probability:
				newPoseLikelihood = auxiliarComputeObservationLikelihood(
					PF_options, k, sf, &newPose);
				ratioLikLik = exp(newPoseLikelihood - m_maxLikelihood[k]);
				acceptanceProb = min(1.0, ratioLikLik);

				if (ratioLikLik > 1)
				{
					if (ratioLikLik > 1.5)
					{
						statsWarningsAccProbAboveOne++;
						// DEBUG
						// printf("[pfAuxiliaryPFOptimal] Warning!!
						// p(z|x)/p(z|x*)=%f\n",ratioLikLik);
					}
					m_maxLikelihood[k] = newPoseLikelihood;  //  :'-( !!!
					acceptanceProb = 0;  // Keep searching!!
				}

				statsTrialsCount++;  // Stats

			} while (acceptanceProb < getRandomGenerator().drawUniform(
										  MIN_ACCEPT_UNIF_DISTRIB, 0.999f));

			statsTrialsSuccess++;  // Stats:
		}

		// Insert the new particle!:
		newParticles[i] = newPose;

		// And its weight:
		double weightFact =
			m_pfAuxiliaryPFOptimal_estimatedProb[k] * PF_options.powFactor;

		// Add to historic record of log_w weights:
		m_log_w_metric_history.resize(M);
		m_log_w_metric_history[i][currentPoseID] += weightFact;

		if (doResample)
			newParticlesWeight[i] = 0;
		else
			newParticlesWeight[i] = m_poseParticles.m_particles[k].log_w + weightFact;

		// and its heritance:
		newParticlesDerivedFromIdx[i] = (unsigned int)k;

	}  // for i

	// ---------------------------------------------------------------------------------
	// Substitute old by new particle set:
	//   Old are in "m_poseParticles.m_particles"
	//   New are in "newParticles",
	//   "newParticlesWeight","newParticlesDerivedFromIdx"
	// ---------------------------------------------------------------------------------
	N = newParticles.size();
	CLocalMetricHypothesis::CParticleList newParticlesArray(N);
	CLocalMetricHypothesis::CParticleList::iterator newPartIt, trgPartIt;

	// For efficiency, just copy the "CRBPFParticleData" from the old particle
	// into the
	//  new one, but this can be done only once:
	std::vector<bool> oldParticleAlreadyCopied(m_poseParticles.m_particles.size(), false);
	CLSLAMParticleData* newPartData;

	for (newPartIt = newParticlesArray.begin(), i = 0;
		 newPartIt != newParticlesArray.end(); newPartIt++, i++)
	{
		// The weight:
		newPartIt->log_w = newParticlesWeight[i];

		// The data (CRBPFParticleData):
		if (!oldParticleAlreadyCopied[newParticlesDerivedFromIdx[i]])
		{
			// The first copy of this old particle:
			newPartData =
				m_poseParticles.m_particles[newParticlesDerivedFromIdx[i]].d.release();
			oldParticleAlreadyCopied[newParticlesDerivedFromIdx[i]] = true;
		}
		else
		{
			// Make a copy:
			newPartData = new CLSLAMParticleData(
				*m_poseParticles.m_particles[newParticlesDerivedFromIdx[i]].d);
		}

		newPartIt->d.reset(newPartData);
	}  // end for "newPartIt"

	// Now add the new robot pose to the paths: (this MUST be done after the
	// above loop, separately):
	for (newPartIt = newParticlesArray.begin(), i = 0;
		 newPartIt != newParticlesArray.end(); newPartIt++, i++)
		newPartIt->d->robotPoses[m_currentRobotPose] =
			CPose3D(newParticles[i]);

	// Free those old m_poseParticles.m_particles not being copied into the new ones:
	for (i = 0; i < m_poseParticles.m_particles.size(); i++)
	{
		m_poseParticles.m_particles[i].d.reset();
	}

	// Copy into "m_poseParticles.m_particles":
	m_poseParticles.m_particles.resize(newParticlesArray.size());
	for (newPartIt = newParticlesArray.begin(),
		trgPartIt = m_poseParticles.m_particles.begin();
		 newPartIt != newParticlesArray.end(); newPartIt++, trgPartIt++)
	{
		trgPartIt->log_w = newPartIt->log_w;
		trgPartIt->d.move_from(newPartIt->d);
	}

	// Free buffers:
	newParticles.clear();
	newParticlesArray.clear();
	newParticlesWeight.clear();
	newParticlesDerivedFromIdx.clear();

	double out_max_log_w;
	m_poseParticles.normalizeWeights(&out_max_log_w);  // Normalize weights:
	m_log_w += out_max_log_w;

#if 0
	printf("[REJ-SAMP.RATIO: \t%.03f%% \t %u out of %u with P(acep)>1]\n\n",
		100.0f*statsTrialsSuccess / (float)(max(1u,statsTrialsCount)),
		statsWarningsAccProbAboveOne,
		statsTrialsCount
		);
#endif

	MRPT_END
}

/*---------------------------------------------------------------
			particlesEvaluator_AuxPFOptimal
 ---------------------------------------------------------------*/
double CLocalMetricHypothesis::particlesEvaluator_AuxPFOptimal(
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options, size_t index,
	const CSensoryFrame* observation)
{
	MRPT_START

	// Compute the quantity:
	//     w[i]p(zt|z^{t-1},x^{[i],t-1})
	// As the Monte-Carlo approximation of the
	//  integral over all posible $x_t$.

	// Take the previous particle weight:
	// --------------------------------------------
	double indivLik, maxLik = -std::numeric_limits<double>::max();
	size_t maxLikDraw = 0;
	size_t N;
	size_t nDraws = m_movementDraws.size();

	ASSERT_(nDraws > 1);

	// , perform a Monte-Carlo approximation:
	// --------------------------------------------
	N = PF_options.pfAuxFilterOptimal_MaximumSearchSamples;
	ASSERT_(N > 1);

	CPose2D oldPose(*getCurrentPose(index));
	//	CPose2D			drawnSample;
	mrpt::math::CVectorDouble vectLiks(
		N, 0);  // The vector with the individual log-likelihoods.

	for (size_t q = 0; q < N; q++)
	{
		CPose2D x_predict(
			oldPose +
			m_movementDraws[(++m_movementDrawsIdx) % nDraws]);

		// Estimate the mean...
		indivLik = auxiliarComputeObservationLikelihood(
			PF_options, index, observation, &x_predict);

		MRPT_CHECK_NORMAL_NUMBER(indivLik);
		vectLiks[q] = indivLik;

		// and the maximum value:
		if (indivLik > maxLik)
		{
			maxLikDraw = m_movementDrawsIdx % nDraws;
			maxLik = indivLik;
		}
	}

	// This is done to avoid floating point overflow!!
	double maxLogLik = math::maximum(vectLiks);
	vectLiks.array() -=
		maxLogLik;  // Maximum log-lik = 0 (max linear likelihood=1)

	//      average_lik    =      \sum(e^liks)   * e^maxLik  /     N
	// log( average_lik  ) = log( \sum(e^liks) ) + maxLik   - log( N )
	double avrgLogLik =
		log(vectLiks.array().exp().sum()) + maxLogLik - log((double)N);

	// Save into the object:
	m_pfAuxiliaryPFOptimal_estimatedProb[index] =
		avrgLogLik;  // log( accum / N );
	m_maxLikelihood[index] = maxLik;
	m_movementDrawMaximumLikelihood[index] =
		m_movementDraws[maxLikDraw];

	// and compute the resulting probability of this particle:
	// ------------------------------------------------------------
	//	if (myObj->m_adaptiveSampleSize)
	//			return myObj->m_poseParticles.m_particles[index].w *
	// myObj->m_pfAuxiliaryPFOptimal_estimatedProb[index];
	//	else	return myObj->m_poseParticles.m_particles[index].w;

	double ret = m_poseParticles.m_particles[index].log_w +
				 m_pfAuxiliaryPFOptimal_estimatedProb[index];

	MRPT_CHECK_NORMAL_NUMBER(ret);

	return ret;

	MRPT_END
}

/*---------------------------------------------------------------
				auxiliarComputeObservationLikelihood
 ---------------------------------------------------------------*/
double CLocalMetricHypothesis::auxiliarComputeObservationLikelihood(
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options,
	size_t particleIndexForMap,
	const CSensoryFrame* observation, const CPose2D* x)
{
	MRPT_UNUSED_PARAM(PF_options);
	CMultiMetricMap* map = const_cast<CMultiMetricMap*>(
		&m_poseParticles.m_particles[particleIndexForMap].d->metricMaps);

	return map->computeObservationsLikelihood(*observation, *x);
}

/*---------------------------------------------------------------
				dumpToStdOut
 ---------------------------------------------------------------*/
void CLSLAM_RBPF_2DLASER::TPathBin::dumpToStdOut() const
{
	vector_int::const_iterator it;

	std::cout << "x   = [";
	for (it = x.begin(); it != x.end(); it++) std::cout << *it << " ";
	std::cout << "]" << std::endl;

	std::cout << "y   = [";
	for (it = y.begin(); it != y.end(); it++) std::cout << *it << " ";
	std::cout << "]" << std::endl;

	std::cout << "Phi = [";
	for (it = phi.begin(); it != phi.end(); it++) std::cout << *it << " ";
	std::cout << "]" << std::endl;
}

/*---------------------------------------------------------------
					loadTPathBinFromPath
 ---------------------------------------------------------------*/
void CLSLAM_RBPF_2DLASER::loadTPathBinFromPath(
	CLSLAM_RBPF_2DLASER::TPathBin& outBin, TMapPoseID2Pose3D* path,
	CPose2D* newPose)
{
	size_t lenBinPath;

	if (path != nullptr)
		lenBinPath = path->size();
	else
		lenBinPath = 0;

	TMapPoseID2Pose3D::const_iterator itSrc;
	vector_int::iterator itX, itY, itPHI;

	// Set the output bin dimensionality:
	outBin.x.resize(lenBinPath + (newPose != nullptr ? 1 : 0));
	outBin.y.resize(lenBinPath + (newPose != nullptr ? 1 : 0));
	outBin.phi.resize(lenBinPath + (newPose != nullptr ? 1 : 0));

	// Is a path provided??
	if (path != nullptr)
	{
		// Fill the bin data:
		for (itSrc = path->begin(), itX = outBin.x.begin(),
			itY = outBin.y.begin(), itPHI = outBin.phi.begin();
			 itSrc != path->end(); itSrc++, itX++, itY++, itPHI++)
		{
			*itX = (int)round(
				itSrc->second.x() /
				m_parent->m_options.KLD_params.KLD_binSize_XY);
			*itY = (int)round(
				itSrc->second.y() /
				m_parent->m_options.KLD_params.KLD_binSize_XY);
			*itPHI = (int)round(
				itSrc->second.yaw() /
				m_parent->m_options.KLD_params.KLD_binSize_PHI);
		}  // end-for build path bin
	}

	// Is a newPose provided??
	if (newPose != nullptr)
	{
		// And append the last pose: the new one:
		outBin.x[lenBinPath] = (int)round(
			newPose->x() / m_parent->m_options.KLD_params.KLD_binSize_XY);
		outBin.y[lenBinPath] = (int)round(
			newPose->y() / m_parent->m_options.KLD_params.KLD_binSize_XY);
		outBin.phi[lenBinPath] = (int)round(
			newPose->phi() / m_parent->m_options.KLD_params.KLD_binSize_PHI);
	}
}

/*---------------------------------------------------------------
					findTPathBinIntoSet
 ---------------------------------------------------------------*/
int CLSLAM_RBPF_2DLASER::findTPathBinIntoSet(
	CLSLAM_RBPF_2DLASER::TPathBin& desiredBin,
	std::deque<CLSLAM_RBPF_2DLASER::TPathBin>& theSet)
{
	// it = pathBins.find( p ); <---- This doesn't work!!!
	//  TODO: A more efficient search algorithm here!!
	std::deque<CLSLAM_RBPF_2DLASER::TPathBin>::iterator it;
	int ret;

	for (it = theSet.begin(), ret = 0; it != theSet.end(); it++, ret++)
		if ((it->x == desiredBin.x) && (it->y == desiredBin.y) &&
			(it->phi == desiredBin.phi))
			return ret;

	// Not found!
	return -1;
}

/** The PF algorithm implementation for "optimal sampling" approximated with
 * scan matching (Stachniss method)
 */
template <>
void CLocalMetricHypothesis::prediction_and_update<LSLAMOptimalProposal>(
	const mrpt::obs::CActionCollection* actions,
	const mrpt::obs::CSensoryFrame* sf,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	MRPT_START

	CTicTac tictac;

	// Get the current robot pose estimation:
	TPoseID currentPoseID = m_currentRobotPose;

	// ----------------------------------------------------------------------
	//	  We can execute optimal PF only when we have both, an action, and
	//     a valid observation from which to compute the likelihood:
	//   Accumulate odometry/actions until we have a valid observation, then
	//    process them simultaneously.
	// ----------------------------------------------------------------------
	bool SFhasValidObservations = false;
	// A valid action?
	if (actions != nullptr)
	{
		CActionRobotMovement2D::Ptr act =
			actions->getBestMovementEstimation();  // Find a robot movement
		// estimation:
		if (!act)
			THROW_EXCEPTION(
				"Action list does not contain any CActionRobotMovement2D "
				"derived object!");

		if (!m_accumRobotMovementIsValid)  // Reset accum.
		{
			act->poseChange->getMean(
				m_accumRobotMovement.rawOdometryIncrementReading);
			m_accumRobotMovement.motionModelConfiguration =
				act->motionModelConfiguration;
		}
		else
			m_accumRobotMovement.rawOdometryIncrementReading =
				m_accumRobotMovement.rawOdometryIncrementReading +
				act->poseChange->getMeanVal();

		m_accumRobotMovementIsValid = true;
	}

	if (sf != nullptr)
	{
		ASSERT_(m_poseParticles.m_particles.size() > 0);
		SFhasValidObservations =
			(*m_poseParticles.m_particles.begin())
				.d->metricMaps.canComputeObservationsLikelihood(*sf);
	}

	// All the needed things?
	if (!m_accumRobotMovementIsValid || !SFhasValidObservations)
		return;  // Nothing we can do here...
	ASSERT_(sf != nullptr);
	ASSERT_(!PF_options.adaptiveSampleSize);

	// OK, we have all we need, let's start!

	// The odometry-based increment since last step is
	// in:   m_accumRobotMovement.rawOdometryIncrementReading
	const CPose2D initialPoseEstimation =
		m_accumRobotMovement.rawOdometryIncrementReading;
	m_accumRobotMovementIsValid =
		false;  // To reset odometry at next iteration!

	// ----------------------------------------------------------------------
	//						1) FIXED SAMPLE SIZE VERSION
	// ----------------------------------------------------------------------

	// ICP used if "pfOptimalProposal_mapSelection" = 0 or 1
	CICP icp;
	CICP::TReturnInfo icpInfo;

	// ICP options
	// ------------------------------
	icp.options.maxIterations = 80;
	icp.options.thresholdDist = 0.50f;
	icp.options.thresholdAng = DEG2RAD(20);
	icp.options.smallestThresholdDist = 0.05f;
	icp.options.ALFA = 0.5f;
	icp.options.onlyClosestCorrespondences = true;
	icp.options.doRANSAC = false;

	// Build the map of points to align:
	CSimplePointsMap localMapPoints;

	ASSERT_(m_poseParticles.m_particles[0].d->metricMaps.m_gridMaps.size() > 0);
	// float	minDistBetweenPointsInLocalMaps = 0.02f; //3.0f *
	// m_poseParticles.m_particles[0].d->metricMaps.m_gridMaps[0]->getResolution();

	// Build local map:
	localMapPoints.clear();
	localMapPoints.insertionOptions.minDistBetweenLaserPoints = 0.02f;
	sf->insertObservationsInto(&localMapPoints);

	// Process the particles
	const size_t M = m_poseParticles.m_particles.size();
	m_log_w_metric_history.resize(M);

	for (size_t i = 0; i < M; i++)
	{
		CLocalMetricHypothesis::CParticleData& part = m_poseParticles.m_particles[i];
		CPose3D* part_pose = getCurrentPose(i);

		if (m_SFs.empty())
		{
			// The first robot pose in the SLAM execution: All m_poseParticles.m_particles start
			// at the same point (this is the lowest bound of subsequent
			// uncertainty):
			// New pose = old pose.
			// part_pose: Unmodified
		}
		else
		{
			// ICP and add noise:
			CPosePDFGaussian icpEstimation;

			// Select map to use with ICP:
			CMetricMap* mapalign;

			if (!part.d->metricMaps.m_pointsMaps.empty())
				mapalign = part.d->metricMaps.m_pointsMaps[0].get();
			else if (!part.d->metricMaps.m_gridMaps.empty())
				mapalign = part.d->metricMaps.m_gridMaps[0].get();
			else
				THROW_EXCEPTION(
					"There is no point or grid map. At least one needed for "
					"ICP.");

			// Use ICP to align to each particle's map:
			CPosePDF::Ptr alignEst = icp.Align(
				mapalign, &localMapPoints, initialPoseEstimation, nullptr,
				&icpInfo);

			icpEstimation.copyFrom(*alignEst);

#if 0
			// HACK:
			CPose3D Ap = finalEstimatedPoseGauss.mean - ith_last_pose;
			double  Ap_dist = Ap.norm();
			finalEstimatedPoseGauss.cov.zeros();
			finalEstimatedPoseGauss.cov(0,0) = square( fabs(Ap_dist)*0.01 );
			finalEstimatedPoseGauss.cov(1,1) = square( fabs(Ap_dist)*0.01 );
			finalEstimatedPoseGauss.cov(2,2) = square( fabs(Ap.yaw())*0.02 );
#endif

			// Generate gaussian-distributed 2D-pose increments according to
			// "finalEstimatedPoseGauss":
			// -------------------------------------------------------------------------------------------
			// Set the gaussian pose:
			CPose3DPDFGaussian finalEstimatedPoseGauss(icpEstimation);

			CPose3D noisy_increment;
			finalEstimatedPoseGauss.drawSingleSample(noisy_increment);

			CPose3D new_pose;
			new_pose.composeFrom(*part_pose, noisy_increment);

			CPose2D new_pose2d = CPose2D(new_pose);

			// Add the pose to the path:
			part.d->robotPoses[m_currentRobotPose] = new_pose;

			// Update the weight:
			// ---------------------------------------------------------------------------
			const double log_lik =
				PF_options.powFactor * auxiliarComputeObservationLikelihood(
										   PF_options, i, sf, &new_pose2d);

			part.log_w += log_lik;

			// Add to historic record of log_w weights:
			m_log_w_metric_history[i][currentPoseID] += log_lik;

		}  // end else we can do ICP

	}  // end for each particle i

	// Accumulate the log likelihood of this LMH as a whole:
	double out_max_log_w;
	m_poseParticles.normalizeWeights(&out_max_log_w);  // Normalize weights:
	m_log_w += out_max_log_w;

	printf("[CLSLAM_RBPF_2DLASER] Overall likelihood = %.2e\n", out_max_log_w);
	printf("[CLSLAM_RBPF_2DLASER] Done in %.03fms\n", 1e3 * tictac.Tac());

	MRPT_END
}
}  // namespace hmtslam
}  // namespace mrpt
