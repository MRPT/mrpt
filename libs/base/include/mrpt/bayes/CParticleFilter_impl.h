/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CPARTICLEFILTER_IMPL_H
#define CPARTICLEFILTER_IMPL_H

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/slam/PF_implementations.h>
#include <mrpt/utils/bits.h>  // for format, square

namespace mrpt
{
namespace bayes
{
using namespace mrpt::utils;

template <class PARTICLEFILTERCAPABLE, class PF_ALGORITHM>
void CParticleFilter::executeOn(
	PARTICLEFILTERCAPABLE& obj, const mrpt::obs::CActionCollection* action,
	const mrpt::obs::CSensoryFrame* observation, TParticleFilterStats* stats)
{
	MRPT_START

	// 1,2) Prediction & Update stages:
	// ---------------------------------------------------
	obj.template prediction_and_update<PF_ALGORITHM>(
		action, observation, m_options);

	// 3) Normalize weights:
	// ---------------------------------------------------
	obj.m_poseParticles.normalizeWeights();

	// Save weights statistics?
	// ---------------------------------------------------
	if (stats)
	{
		const size_t M = obj.m_poseParticles.particlesCount();

		// ESS:
		stats->ESS_beforeResample = obj.m_poseParticles.ESS();

		// Variance:
		if (M > 1)
		{
			double weightsMean = 0, var = 0;
			for (size_t i = 0; i < M; i++)
				weightsMean += exp(obj.m_poseParticles.getW(i));
			weightsMean /= M;
			for (size_t i = 0; i < M; i++)
				var += square(exp(obj.m_poseParticles.getW(i)) - weightsMean);

			var /= (M - 1);
			stats->weightsVariance_beforeResample = var;
		}
	}

	// 4) Particles resampling stage
	// ---------------------------------------------------
	if (!m_options.adaptiveSampleSize &&
		(PF_ALGORITHM::DoesResampling))
	{
		if (obj.m_poseParticles.ESS() < m_options.BETA)
		{
			MRPT_LOG_DEBUG(mrpt::format(
				"Resampling particles (ESS was %.02f)\n",
				obj.m_poseParticles.ESS()));
			obj.m_poseParticles.performResampling(m_options);  // Resample
		}
	}

	MRPT_END
}
}  // namespace bayes
}  // namespace mrpt
#endif
