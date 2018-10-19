/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headerss

#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/obs/CSensoryFrame.h>

#include <mrpt/math/utils.h>
#include <mrpt/core/round.h>
#include <mrpt/slam/PF_aux_structs.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::maps;

#include <mrpt/slam/PF_implementations_data.h>

namespace mrpt::slam
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
		ASSERT_(currentParticleValue);
		outBin.x = round(currentParticleValue->x / opts.KLD_binSize_XY);
		outBin.y = round(currentParticleValue->y / opts.KLD_binSize_XY);
		outBin.z = round(currentParticleValue->z / opts.KLD_binSize_XY);

		outBin.yaw = round(currentParticleValue->yaw / opts.KLD_binSize_PHI);
		outBin.pitch =
			round(currentParticleValue->pitch / opts.KLD_binSize_PHI);
		outBin.roll = round(currentParticleValue->roll / opts.KLD_binSize_PHI);
	}
}
}  // namespace mrpt::slam

#include <mrpt/slam/PF_implementations.h>

using namespace mrpt::slam;

/*---------------------------------------------------------------
				ctor
 ---------------------------------------------------------------*/
// Passing a "this" pointer at this moment is not a problem since it will be NOT
// access until the object is fully initialized
CMonteCarloLocalization3D::CMonteCarloLocalization3D(size_t M)
	: CPose3DPDFParticles(M)
{
	this->setLoggerName("CMonteCarloLocalization3D");
}

TPose3D CMonteCarloLocalization3D::getLastPose(
	const size_t i, bool& is_valid_pose) const
{
	if (i >= m_particles.size())
		THROW_EXCEPTION("Particle index out of bounds!");
	is_valid_pose = true;
	return m_particles[i].d;
}

/*---------------------------------------------------------------

			prediction_and_update_pfStandardProposal

 ---------------------------------------------------------------*/
void CMonteCarloLocalization3D::prediction_and_update_pfStandardProposal(
	const mrpt::obs::CActionCollection* actions,
	const mrpt::obs::CSensoryFrame* sf,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	MRPT_START

	if (sf)
	{  // A map MUST be supplied!
		ASSERT_(options.metricMap || options.metricMaps.size() > 0);
		if (!options.metricMap)
			ASSERT_(options.metricMaps.size() == m_particles.size());
	}

	PF_SLAM_implementation_pfStandardProposal<mrpt::slam::detail::TPoseBin3D>(
		actions, sf, PF_options, options.KLD_params);

	MRPT_END
}

/*---------------------------------------------------------------

			prediction_and_update_pfAuxiliaryPFStandard

 ---------------------------------------------------------------*/
void CMonteCarloLocalization3D::prediction_and_update_pfAuxiliaryPFStandard(
	const mrpt::obs::CActionCollection* actions,
	const mrpt::obs::CSensoryFrame* sf,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	MRPT_START

	if (sf)
	{  // A map MUST be supplied!
		ASSERT_(options.metricMap || options.metricMaps.size() > 0);
		if (!options.metricMap)
			ASSERT_(options.metricMaps.size() == m_particles.size());
	}

	PF_SLAM_implementation_pfAuxiliaryPFStandard<
		mrpt::slam::detail::TPoseBin3D>(
		actions, sf, PF_options, options.KLD_params);

	MRPT_END
}

/*---------------------------------------------------------------

			prediction_and_update_pfAuxiliaryPFOptimal

 ---------------------------------------------------------------*/
void CMonteCarloLocalization3D::prediction_and_update_pfAuxiliaryPFOptimal(
	const mrpt::obs::CActionCollection* actions,
	const mrpt::obs::CSensoryFrame* sf,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	MRPT_START

	if (sf)
	{  // A map MUST be supplied!
		ASSERT_(options.metricMap || options.metricMaps.size() > 0);
		if (!options.metricMap)
			ASSERT_(options.metricMaps.size() == m_particles.size());
	}

	PF_SLAM_implementation_pfAuxiliaryPFOptimal<mrpt::slam::detail::TPoseBin3D>(
		actions, sf, PF_options, options.KLD_params);

	MRPT_END
}

/*---------------------------------------------------------------
			PF_SLAM_computeObservationLikelihoodForParticle
 ---------------------------------------------------------------*/
double
	CMonteCarloLocalization3D::PF_SLAM_computeObservationLikelihoodForParticle(
		[[maybe_unused]] const CParticleFilter::TParticleFilterOptions&
			PF_options,
		const size_t particleIndexForMap, const CSensoryFrame& observation,
		const CPose3D& x) const
{
	ASSERT_(
		options.metricMap || particleIndexForMap < options.metricMaps.size());

	CMetricMap* map =
		(options.metricMap) ? options.metricMap :  // All particles, one map
			options.metricMaps[particleIndexForMap];  // One map per particle

	// For each observation:
	double ret = 1;
	for (const auto& it : observation)
		ret += map->computeObservationLikelihood(
			it.get(), x);  // Compute the likelihood:

	// Done!
	return ret;
}

// Specialization for my kind of particles:
void CMonteCarloLocalization3D::
	PF_SLAM_implementation_custom_update_particle_with_new_pose(
		TPose3D* particleData, const TPose3D& newPose) const
{
	*particleData = newPose;
}

void CMonteCarloLocalization3D::PF_SLAM_implementation_replaceByNewParticleSet(
	CParticleList& old_particles, const vector<TPose3D>& newParticles,
	const vector<double>& newParticlesWeight,
	[[maybe_unused]] const vector<size_t>& newParticlesDerivedFromIdx) const
{
	ASSERT_(size_t(newParticlesWeight.size()) == newParticles.size());
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
		old_particles[i].d = newParticles[i];
	}
}
