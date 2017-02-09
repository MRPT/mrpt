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

void CMultiObjMotionOpt_WeightedScores::TParams::loadFromConfigFile(const mrpt::utils::CConfigFileBase &c, const std::string &s)
{
	// Load: TParamsBase::formula_score
	{
		TParamsBase::formula_score.clear();
		int idx = 1;
		for (;;)
		{
			const std::string sKeyName = mrpt::format("score%i_name"), sKeyValue = mrpt::format("score%i_formula");
			const std::string sName  = c.read_string(s, sKeyName, "");
			const std::string sValue = c.read_string(s, sKeyValue,"");

			const bool none = (sName.empty() && sValue.empty());
			const bool both = (!sName.empty() && !sValue.empty());

			if (none && idx==1)
				THROW_EXCEPTION_FMT("Expect at least a first `%s` and `%s` pair defining one score in section `[%s]`", sKeyName.c_str(), sKeyValue.c_str(),s.c_str());

			if (none)
				break;

			if (!both) {
				THROW_EXCEPTION_FMT("Both `%s` and `%s` must be provided in section `[%s]`", sKeyName.c_str(), sKeyValue.c_str(), s.c_str());
			}

			formula_score[sName] = sValue;
		}
	}

}

void CMultiObjMotionOpt_WeightedScores::TParams::saveToConfigFile(mrpt::utils::CConfigFileBase &c, const std::string &s) const
{
	// TParamsBase::formula_score
}

