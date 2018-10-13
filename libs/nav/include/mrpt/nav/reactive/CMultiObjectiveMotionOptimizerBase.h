/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/TCandidateMovementPTG.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>

namespace mrpt::nav
{
/** Virtual base class for multi-objective motion choosers, as used for reactive
 *navigation engines.
 *\sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
 *  \ingroup nav_reactive
 */
class CMultiObjectiveMotionOptimizerBase : public mrpt::rtti::CObject
{
	DEFINE_VIRTUAL_MRPT_OBJECT(CMultiObjectiveMotionOptimizerBase)
   public:
	/** Class factory from C++ class name */
	static CMultiObjectiveMotionOptimizerBase::Ptr Factory(
		const std::string& className) noexcept;

	struct TResultInfo
	{
		/** For each candidate (vector indices), the numerical evaluation of all
		 * scores defined in TParamsBase::formula_score.
		 * A value of 0 in all scores, or an empty map, means unsuitable
		 * candidate. */
		std::vector<std::map<std::string, double>> score_values;

		/** The final evaluation score for each candidate */
		std::vector<double> final_evaluation;
		/** Optionally, debug logging info will be stored here by the
		 * implementor classes */
		std::vector<std::string> log_entries;
	};

	/** The main entry point for the class: returns the 0-based index of the
	 * best of the N motion candidates in `movs`.
	 * If no valid one is found, `-1` will be returned.
	 */
	int decide(
		const std::vector<mrpt::nav::TCandidateMovementPTG>& movs,
		TResultInfo& extra_info);

	virtual void loadConfigFile(const mrpt::config::CConfigFileBase& c) = 0;
	virtual void saveConfigFile(mrpt::config::CConfigFileBase& c) const = 0;

	/** Common params for all children */
	struct TParamsBase : public mrpt::config::CLoadableOptions
	{
		TParamsBase();

		/** A list of `name` -> mathematical expression (in the format of the
		 * exprtk library) for
		 * the list of "score" factors to evaluate.
		 */
		std::map<std::string, std::string> formula_score;

		/** A list of exprtk expressions for conditions that any candidate
		 * movement must
		 * fulfill in order to get through the evaluation process. *All* assert
		 * conditions must be satisfied.
		 */
		std::vector<std::string> movement_assert;

		/** List of score names (as defined in the key of `formula_score`) that
		 * must be normalized
		 * across all candidates, such that the maximum value is 1. */
		std::vector<std::string> scores_to_normalize;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& cfg,
			const std::string& section) const override;  // See base docs
	};

	/** Resets the object state; use if the parameters change, so they are
	 * re-read and applied. */
	virtual void clear();

   protected:
	CMultiObjectiveMotionOptimizerBase(TParamsBase& params);

   private:
	// This virtual method is called by decide().
	virtual int impl_decide(
		const std::vector<mrpt::nav::TCandidateMovementPTG>& movs,
		TResultInfo& extra_info) = 0;

	TParamsBase& m_params_base;

	/** score names -> score compiled expressions */
	std::map<std::string, mrpt::expr::CRuntimeCompiledExpression> m_score_exprs;
	std::vector<mrpt::expr::CRuntimeCompiledExpression> m_movement_assert_exprs;
	std::map<std::string, double> m_expr_vars;
};
}  // namespace mrpt::nav
