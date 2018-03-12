/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "bayes-precomp.h"  // Precompiled headers

#include <cmath>  // for exp
#include <mrpt/bayes/CParticleFilter.h>  // for CParticleFilter::TPar...
#include <mrpt/bayes/CParticleFilterCapable.h>  // for CParticleFilterCapable
#include <mrpt/config/CConfigFileBase.h>  // for CConfigFileBase, MRPT...
#include <cstddef>  // for size_t
#include <exception>  // for exception
#include <string>  // for string, allocator
#include <mrpt/system/COutputLogger.h>  // for COutputLogger, MRPT_L...
#include <mrpt/core/bits_math.h>  // square()

namespace mrpt
{
namespace obs
{
class CActionCollection;
}
}  // namespace mrpt
namespace mrpt
{
namespace obs
{
class CSensoryFrame;
}
}  // namespace mrpt

using namespace mrpt::bayes;
using mrpt::square;

CParticleFilter::CParticleFilter()
	: mrpt::system::COutputLogger("CParticleFilter"), m_options()
{
}

void CParticleFilter::executeOn(
	CParticleFilterCapable& obj, const mrpt::obs::CActionCollection* action,
	const mrpt::obs::CSensoryFrame* observation, TParticleFilterStats* stats)
{
	MRPT_START

	// 1,2) Prediction & Update stages:
	// ---------------------------------------------------
	obj.prediction_and_update(action, observation, m_options);

	// 3) Normalize weights:
	// ---------------------------------------------------
	obj.normalizeWeights();

	// Save weights statistics?
	// ---------------------------------------------------
	if (stats)
	{
		const size_t M = obj.particlesCount();

		// ESS:
		stats->ESS_beforeResample = obj.ESS();

		// Variance:
		if (M > 1)
		{
			double weightsMean = 0, var = 0;
			for (size_t i = 0; i < M; i++) weightsMean += exp(obj.getW(i));
			weightsMean /= M;
			for (size_t i = 0; i < M; i++)
				var += square(exp(obj.getW(i)) - weightsMean);

			var /= (M - 1);
			stats->weightsVariance_beforeResample = var;
		}
	}

	// 4) Particles resampling stage
	// ---------------------------------------------------
	if (!m_options.adaptiveSampleSize &&
		(m_options.PF_algorithm == CParticleFilter::pfStandardProposal ||
		 m_options.PF_algorithm == CParticleFilter::pfOptimalProposal))
	{
		if (obj.ESS() < m_options.BETA)
		{
			MRPT_LOG_DEBUG(mrpt::format(
				"Resampling particles (ESS was %.02f)\n", obj.ESS()));
			obj.performResampling(m_options);  // Resample
		}
	}

	MRPT_END
}

/*---------------------------------------------------------------
					TParticleFilterOptions
  ---------------------------------------------------------------*/
void CParticleFilter::TParticleFilterOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		PF_algorithm, "The PF algorithm to use. See TParticleFilterAlgorithm");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		resamplingMethod,
		"The resampling algorithm to use. See TParticleResamplingAlgorithm");

	MRPT_SAVE_CONFIG_VAR_COMMENT(
		adaptiveSampleSize,
		"A flag that indicates whether the CParticleFilterCapable object "
		"should perform adative sample size (default=false)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		BETA,
		"The resampling of particles will be performed when ESS (in range "
		"[0,1]) < BETA (default is 0.5)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		sampleSize,
		"The initial number of particles in the filter (it can change only if "
		"adaptiveSampleSize=true) (default=1)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		pfAuxFilterOptimal_MaximumSearchSamples, "See Doxygen docs");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		powFactor,
		"An optional step to smooth dramatic changes in the observation model "
		"to affect the variance of the particle weights (default=1)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		max_loglikelihood_dyn_range,
		"Only for PF_algorithm=pfAuxiliaryPFOptimal");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		pfAuxFilterStandard_FirstStageWeightsMonteCarlo,
		"Only for PF_algorithm==pfAuxiliaryPFStandard");
	MRPT_SAVE_CONFIG_VAR_COMMENT(pfAuxFilterOptimal_MLE, "See doxygen docs.");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CParticleFilter::TParticleFilterOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	MRPT_START

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(
		adaptiveSampleSize, bool, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(BETA, double, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(sampleSize, int, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(powFactor, double, iniFile, section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		max_loglikelihood_dyn_range, double, iniFile, section.c_str());
	ASSERT_(max_loglikelihood_dyn_range >= 0);

	PF_algorithm = iniFile.read_enum<TParticleFilterAlgorithm>(
		section, "PF_algorithm", PF_algorithm, true);
	resamplingMethod = iniFile.read_enum<TParticleResamplingAlgorithm>(
		section, "resamplingMethod", resamplingMethod, true);

	if (PF_algorithm == pfAuxiliaryPFOptimal)
	{
		MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(
			pfAuxFilterOptimal_MaximumSearchSamples, int, iniFile,
			section.c_str());
	}
	else
	{
		MRPT_LOAD_CONFIG_VAR(
			pfAuxFilterOptimal_MaximumSearchSamples, int, iniFile,
			section.c_str());
	}

	MRPT_LOAD_CONFIG_VAR(
		pfAuxFilterStandard_FirstStageWeightsMonteCarlo, bool, iniFile,
		section.c_str());
	MRPT_LOAD_CONFIG_VAR(
		pfAuxFilterOptimal_MLE, bool, iniFile, section.c_str());

	MRPT_END
}
