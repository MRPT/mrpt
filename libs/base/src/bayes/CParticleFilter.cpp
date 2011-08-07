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

#include <mrpt/base.h>  // Precompiled headers



#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>

//#include <mrpt/slam/CSensoryFrame.h>
//#include <mrpt/slam/CActionCollection.h>

using namespace mrpt::bayes;
using namespace mrpt::utils;

/*---------------------------------------------------------------
						Default Constructor
  ---------------------------------------------------------------*/
CParticleFilter::CParticleFilter() : m_options()
{
}


/*---------------------------------------------------------------
					executeOn
Executes a prediction-update stage of particle filtering. This includes:
	- Check if ESS is below a given threshold, and if it is true resample particles.
	- Executes a prediction with the given "action".
	- Executes an update stage with:
		- The PDF class update member, if updateFunctor is NULL.
		- Using updateFunctor if one is provided.
	- Normalization of weights.
  ---------------------------------------------------------------*/
void  CParticleFilter::executeOn(
	CParticleFilterCapable 			&obj,
	const mrpt::slam::CActionCollection   *action,
	const mrpt::slam::CSensoryFrame	    *observation,
	TParticleFilterStats	        *stats )
{
	MRPT_START

	// 1,2) Prediction & Update stages:
	// ---------------------------------------------------
	obj.prediction_and_update( action, observation, m_options );

	// 3) Normalize weights:
	// ---------------------------------------------------
	obj.normalizeWeights();

	// Save weights statistics?
	// ---------------------------------------------------
	if (stats)
	{
		size_t i,M = obj.particlesCount();
		double	weightsMean = 0, var = 0;

		// ESS:
		stats->ESS_beforeResample = obj.ESS();

		// Variance:
		if (M>1)
		{
			for (i=0;i<M;i++) weightsMean+=exp(obj.getW(i));
			weightsMean /= M;
			for (i=0;i<M;i++) var+=square(exp(obj.getW(i))-weightsMean);

			var/= (M-1);
			stats->weightsVariance_beforeResample = var;
		}
	}

	// 4) Particles resampling stage
	// ---------------------------------------------------
	if (!m_options.adaptiveSampleSize &&
		( m_options.PF_algorithm == CParticleFilter::pfStandardProposal ||
		  m_options.PF_algorithm == CParticleFilter::pfOptimalProposal ) )
	{
		if (obj.ESS() < m_options.BETA)
		{
			if (m_options.verbose)
				printf_debug("[PF] Resampling particles (ESS was %.02f)\n", obj.ESS());
			obj.performResampling( m_options );	// Resample
		}
	}

	MRPT_END
}


/*---------------------------------------------------------------
					TParticleFilterOptions
  ---------------------------------------------------------------*/
CParticleFilter::TParticleFilterOptions::TParticleFilterOptions() :
	adaptiveSampleSize		( false ),
	BETA					( 0.5 ),
	sampleSize				( 1 ),
	pfAuxFilterOptimal_MaximumSearchSamples ( 100 ),
	powFactor				( 1 ),
	PF_algorithm			( pfStandardProposal ),
	resamplingMethod		( prMultinomial ),
	max_loglikelihood_dyn_range ( 15 ),
	pfAuxFilterStandard_FirstStageWeightsMonteCarlo ( false ),
	verbose	(false),
	pfAuxFilterOptimal_MLE(false)
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CParticleFilter::TParticleFilterOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CParticleFilter::TParticleFilterOptions] ------------ \n\n");

	out.printf("PF_algorithm                            = ");
	switch (PF_algorithm)
	{
	case CParticleFilter::pfStandardProposal: out.printf("pfStandardProposal\n"); break;
	case CParticleFilter::pfAuxiliaryPFStandard: out.printf("pfAuxiliaryPFStandard\n"); break;
	case CParticleFilter::pfOptimalProposal: out.printf("pfOptimalProposal\n"); break;
	case CParticleFilter::pfAuxiliaryPFOptimal: out.printf("pfAuxiliaryPFOptimal\n"); break;
	default:
		out.printf("UNKNOWN!!\n"); break;
	};

	out.printf("m_resamplingMethod                      = ");
	switch (resamplingMethod)
	{
	case CParticleFilter::prMultinomial:	out.printf("prMultinomial\n"); break;
	case CParticleFilter::prResidual:		out.printf("prResidual\n"); break;
	case CParticleFilter::prStratified:		out.printf("prStratified\n"); break;
	case CParticleFilter::prSystematic:		out.printf("prSystematic\n"); break;
	default:
		out.printf("UNKNOWN!!\n"); break;
	};

	out.printf("adaptiveSampleSize                      = %c\n", adaptiveSampleSize ? 'Y':'N' );
	out.printf("sampleSize                              = %i\n", sampleSize );
	out.printf("BETA                                    = %f\n", BETA );
	out.printf("pfAuxFilterOptimal_MaximumSearchSamples = %i\n", pfAuxFilterOptimal_MaximumSearchSamples );
	out.printf("powFactor                               = %f\n", powFactor);
	out.printf("max_loglikelihood_dyn_range             = %f\n", max_loglikelihood_dyn_range);
	out.printf("pfAuxFilterStandard_FirstStageWeightsMonteCarlo = %c\n", pfAuxFilterStandard_FirstStageWeightsMonteCarlo ? 'Y':'N');
	out.printf("pfAuxFilterOptimal_MLE                  = %c\n", pfAuxFilterOptimal_MLE? 'Y':'N');

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CParticleFilter::TParticleFilterOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_START

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(adaptiveSampleSize,bool,	iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(BETA,double,				iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(sampleSize,int,				iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(powFactor,double,						iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(max_loglikelihood_dyn_range,double,	iniFile,section.c_str());
	ASSERT_(max_loglikelihood_dyn_range>=0)
	MRPT_LOAD_CONFIG_VAR(verbose,bool,	iniFile,section.c_str());


	MRPT_LOAD_CONFIG_VAR_CAST_NO_DEFAULT(PF_algorithm,int,TParticleFilterAlgorithm,			iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR_CAST_NO_DEFAULT(resamplingMethod,int,TParticleResamplingAlgorithm,	iniFile,section.c_str());

	if ( PF_algorithm == pfAuxiliaryPFOptimal )
	{
		MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(pfAuxFilterOptimal_MaximumSearchSamples,int,			iniFile,section.c_str());
	}
	else
	{
		MRPT_LOAD_CONFIG_VAR(pfAuxFilterOptimal_MaximumSearchSamples,int,			iniFile,section.c_str());
	}

	MRPT_LOAD_CONFIG_VAR(pfAuxFilterStandard_FirstStageWeightsMonteCarlo,bool,	iniFile,section.c_str());
	MRPT_LOAD_CONFIG_VAR(pfAuxFilterOptimal_MLE,bool,	iniFile,section.c_str());


	MRPT_END
}
