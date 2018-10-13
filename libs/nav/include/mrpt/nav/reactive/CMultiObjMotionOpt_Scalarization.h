/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h>

namespace mrpt::nav
{
/** Implementation of multi-objective motion chooser using scalarization: a
 * user-given formula is used to
 * collapse all the scores into a single scalar score. The candidate with the
 * highest positive score is selected.
 * Note that assert expressions are honored via the base class
 * CMultiObjectiveMotionOptimizerBase
 *
 * \sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
 * \ingroup nav_reactive
 */
class CMultiObjMotionOpt_Scalarization
	: public mrpt::nav::CMultiObjectiveMotionOptimizerBase
{
	DEFINE_MRPT_OBJECT(CMultiObjMotionOpt_Scalarization)

   public:
	CMultiObjMotionOpt_Scalarization();

	void loadConfigFile(const mrpt::config::CConfigFileBase& c) override;
	void saveConfigFile(mrpt::config::CConfigFileBase& c) const override;

	struct TParams
		: public mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase
	{
		/** A formula that takes all/a subset of scores and generates a scalar
		 * global score. */
		std::string scalar_score_formula;

		TParams();
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& cfg,
			const std::string& section) const override;  // See base docs
	};

	TParams parameters;

	void clear() override;

   protected:
	mrpt::expr::CRuntimeCompiledExpression m_expr_scalar_formula;
	std::map<std::string, double> m_expr_scalar_vars;

	// This virtual method is called by decide().
	int impl_decide(
		const std::vector<mrpt::nav::TCandidateMovementPTG>& movs,
		TResultInfo& extra_info) override;
};
}  // namespace mrpt::nav
