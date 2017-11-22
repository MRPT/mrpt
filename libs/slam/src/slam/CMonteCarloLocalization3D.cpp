/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headerss

#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/obs/CSensoryFrame.h>

#include <mrpt/math/utils.h>
#include <mrpt/utils/round.h>
#include <mrpt/slam/PF_aux_structs.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::maps;

#include <mrpt/slam/PF_implementations_data.h>
#include <mrpt/slam/PF_implementations.h>

#include <mrpt/bayes/CParticleFilter_impl.h>

namespace mrpt
{
namespace slam
{
/** Fills out a "TPoseBin3D" variable, given a path hypotesis and (if not set to
 * nullptr) a new pose appended at the end, using the KLD params in "options".
 */
template <>
void KLF_loadBinFromParticle(
	mrpt::slam::detail::TPoseBin3D& outBin, const TKLDParams& opts,
	const CMonteCarloLocalization3D::CParticleDataContent* currentParticleValue,
	const TPose3D* newPoseToBeInserted)
{
	// 3D pose approx: Use the latest pose only:
	if (newPoseToBeInserted)
	{
		outBin.x = round(newPoseToBeInserted->x / opts.KLD_binSize_XY);
		outBin.y = round(newPoseToBeInserted->y / opts.KLD_binSize_XY);
		outBin.z = round(newPoseToBeInserted->z / opts.KLD_binSize_XY);

		outBin.yaw = round(newPoseToBeInserted->yaw / opts.KLD_binSize_PHI);
		outBin.pitch = round(newPoseToBeInserted->pitch / opts.KLD_binSize_PHI);
		outBin.roll = round(newPoseToBeInserted->roll / opts.KLD_binSize_PHI);
	}
	else
	{
		ASSERT_(currentParticleValue)
		outBin.x = round(currentParticleValue->x() / opts.KLD_binSize_XY);
		outBin.y = round(currentParticleValue->y() / opts.KLD_binSize_XY);
		outBin.z = round(currentParticleValue->z() / opts.KLD_binSize_XY);

		outBin.yaw = round(currentParticleValue->yaw() / opts.KLD_binSize_PHI);
		outBin.pitch =
			round(currentParticleValue->pitch() / opts.KLD_binSize_PHI);
		outBin.roll =
			round(currentParticleValue->roll() / opts.KLD_binSize_PHI);
	}
}

/*---------------------------------------------------------------
				ctor
 ---------------------------------------------------------------*/
// Passing a "this" pointer at this moment is not a problem since it will be NOT
// access until the object is fully initialized
CMonteCarloLocalization3D::CMonteCarloLocalization3D(size_t M)
	: m_poseParticles(M)
{
	this->setLoggerName("CMonteCarloLocalization3D");
}

/*---------------------------------------------------------------
				Dtor
 ---------------------------------------------------------------*/
CMonteCarloLocalization3D::~CMonteCarloLocalization3D() {}
TPose3D CMonteCarloLocalization3D::getLastPose(
	const size_t i, bool& is_valid_pose) const
{
	if (i >= m_poseParticles.m_particles.size())
		THROW_EXCEPTION("Particle index out of bounds!");
	is_valid_pose = true;
	ASSERTDEB_(m_poseParticles.m_particles[i].d != NULL);
	return TPose3D(*m_poseParticles.m_particles[i].d);
}

/*---------------------------------------------------------------

			prediction_and_update

 ---------------------------------------------------------------*/
template <typename T>
void CMonteCarloLocalization3D::prediction_and_update(
	const mrpt::obs::CActionCollection* actions,
	const mrpt::obs::CSensoryFrame* sf,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	MRPT_START

	if (sf)
	{  // A map MUST be supplied!
		ASSERT_(options.metricMap || options.metricMaps.size() > 0)
		if (!options.metricMap)
			ASSERT_(
				options.metricMaps.size() == m_poseParticles.m_particles.size())
	}

	T::template PF_SLAM_implementation<
		mrpt::poses::CPose3D, CMonteCarloLocalization3D,
		mrpt::slam::detail::TPoseBin3D>(
		actions, sf, PF_options, options.KLD_params, *this);

	MRPT_END
}

/*---------------------------------------------------------------
			PF_SLAM_computeObservationLikelihoodForParticle
 ---------------------------------------------------------------*/
double
	CMonteCarloLocalization3D::PF_SLAM_computeObservationLikelihoodForParticle(
		const CParticleFilter::TParticleFilterOptions& PF_options,
		const size_t particleIndexForMap, const CSensoryFrame& observation,
		const CPose3D& x) const
{
	MRPT_UNUSED_PARAM(PF_options);
	ASSERT_(
		options.metricMap || particleIndexForMap < options.metricMaps.size())

	CMetricMap* map =
		(options.metricMap) ? options.metricMap :  // All particles, one map
			options.metricMaps[particleIndexForMap];  // One map per particle

	// For each observation:
	double ret = 1;
	for (CSensoryFrame::const_iterator it = observation.begin();
		 it != observation.end(); ++it)
		ret += map->computeObservationLikelihood(
			it->get(), x);  // Compute the likelihood:

	// Done!
	return ret;
}

// Specialization for my kind of particles:
void CMonteCarloLocalization3D::
	PF_SLAM_implementation_custom_update_particle_with_new_pose(
		CPose3D* particleData, const TPose3D& newPose) const
{
	*particleData = CPose3D(newPose);
}

void CMonteCarloLocalization3D::PF_SLAM_implementation_replaceByNewParticleSet(
	CParticleList& old_particles, const vector<TPose3D>& newParticles,
	const vector<double>& newParticlesWeight,
	const vector<size_t>& newParticlesDerivedFromIdx) const
{
	MRPT_UNUSED_PARAM(newParticlesDerivedFromIdx);
	ASSERT_(size_t(newParticlesWeight.size()) == newParticles.size())

	// ---------------------------------------------------------------------------------
	// Substitute old by new particle set:
	//   Old are in "m_particles"
	//   New are in "newParticles",
	//   "newParticlesWeight","newParticlesDerivedFromIdx"
	// ---------------------------------------------------------------------------------
	// Free old m_particles (automatically done via smart ptr)

	// Copy into "m_particles"
	const size_t N = newParticles.size();
	old_particles.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		old_particles[i].log_w = newParticlesWeight[i];
		old_particles[i].d.reset(new CPose3D(newParticles[i]));
	}
}

void CMonteCarloLocalization3D::executeOn(
	mrpt::bayes::CParticleFilter& pf, const mrpt::obs::CActionCollection* action,
	const mrpt::obs::CSensoryFrame* observation,
	mrpt::bayes::CParticleFilter::TParticleFilterStats* stats,
	mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm PF_algorithm)
{
	switch (PF_algorithm)
	{
		case CParticleFilter::pfStandardProposal:
			pf.executeOn<CMonteCarloLocalization3D, mrpt::slam::StandardProposal>(
				*this, action, observation, stats);
			break;
		case CParticleFilter::pfAuxiliaryPFStandard:
			pf.executeOn<CMonteCarloLocalization3D, mrpt::slam::AuxiliaryPFStandard>(
				*this, action, observation, stats);
			break;
		case CParticleFilter::pfAuxiliaryPFOptimal:
			pf.executeOn<CMonteCarloLocalization3D, mrpt::slam::AuxiliaryPFOptimal>(
				*this, action, observation, stats);
			break;
		default:
		{
			THROW_EXCEPTION("Invalid particle filter algorithm selection!");
		}
		break;
	}

}

}
}
