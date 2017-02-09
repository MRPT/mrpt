/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CMultiObjMotionOpt_WeightedScores.h>

using namespace mrpt::nav;
using namespace mrpt::utils;

IMPLEMENTS_MRPT_OBJECT(CMultiObjMotionOpt_WeightedScores, CMultiObjectiveMotionOptimizerBase, mrpt::nav)


CMultiObjMotionOpt_WeightedScores::CMultiObjMotionOpt_WeightedScores() :
	CMultiObjectiveMotionOptimizerBase(parameters)
{
}

void CMultiObjMotionOpt_WeightedScores::loadConfigFile(const mrpt::utils::CConfigFileBase & c)
{
	parameters.loadFromConfigFile(c, "CMultiObjMotionOpt_WeightedScores");
}

void CMultiObjMotionOpt_WeightedScores::saveConfigFile(mrpt::utils::CConfigFileBase & c) const
{
	parameters.saveToConfigFile(c, "CMultiObjMotionOpt_WeightedScores");
}

int CMultiObjMotionOpt_WeightedScores::impl_decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info)
{
	return -1;
}

CMultiObjMotionOpt_WeightedScores::TParams::TParams()
{

}

void CMultiObjMotionOpt_WeightedScores::TParams::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source, const std::string &section)
{
	// TParamsBase::formula_score

}

void CMultiObjMotionOpt_WeightedScores::TParams::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg, const std::string &section) const
{
	// TParamsBase::formula_score
}

