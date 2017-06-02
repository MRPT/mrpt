/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/TCandidateMovementPTG.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CObject.h>
#include <mrpt/math/CRuntimeCompiledExpression.h>
#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
		DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE(CMultiObjectiveMotionOptimizerBase, NAV_IMPEXP)

		/** Virtual base class for multi-objective motion choosers, as used for reactive navigation engines.
		  *\sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
		  *  \ingroup nav_reactive
		  */
		class NAV_IMPEXP CMultiObjectiveMotionOptimizerBase :
			public mrpt::utils::CObject
		{
			// This must be added to any CSerializable derived class:
			DEFINE_VIRTUAL_MRPT_OBJECT(CMultiObjectiveMotionOptimizerBase)
		public:
			static CMultiObjectiveMotionOptimizerBase * Create(const std::string &className) MRPT_NO_THROWS; //!< Class factory from C++ class name

			struct NAV_IMPEXP TResultInfo
			{
				/** For each candidate (vector indices), the numerical evaluation of all scores defined in TParamsBase::formula_score.
				  * A value of 0 in all scores, or an empty map, means unsuitable candidate. */
				std::vector<std::map<std::string, double> > score_values;

				std::vector<double>      final_evaluation;  //!< The final evaluation score for each candidate
				std::vector<std::string> log_entries;       //!< Optionally, debug logging info will be stored here by the implementor classes
			};

			/** The main entry point for the class: returns the 0-based index of the best of the N motion candidates in `movs`. 
			  * If no valid one is found, `-1` will be returned.
			  */
			int decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info);

			virtual void loadConfigFile(const mrpt::utils::CConfigFileBase & c) = 0;
			virtual void saveConfigFile(mrpt::utils::CConfigFileBase & c) const = 0;

			/** Common params for all children */
			struct NAV_IMPEXP TParamsBase : public mrpt::utils::CLoadableOptions
			{
				TParamsBase();

				/** A list of `name` -> mathematical expression (in the format of the exprtk library) for 
				  * the list of "score" factors to evaluate.
				  */
				std::map<std::string, std::string>  formula_score;

				/** A list of exprtk expressions for conditions that any candidate movement must 
				  * fulfill in order to get through the evaluation process. *All* assert conditions must be satisfied.
				  */
				std::vector<std::string>  movement_assert;

				/** List of score names (as defined in the key of `formula_score`) that must be normalized 
				  * across all candidates, such that the maximum value is 1. */
				std::vector<std::string> scores_to_normalize;

				virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source, const std::string &section) MRPT_OVERRIDE; // See base docs
				virtual void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg, const std::string &section) const MRPT_OVERRIDE; // See base docs
			};

			virtual void clear();  //!< Resets the object state; use if the parameters change, so they are re-read and applied.

		protected:
			CMultiObjectiveMotionOptimizerBase(TParamsBase & params);

		private:
			// This virtual method is called by decide().
			virtual int impl_decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info) = 0;

			TParamsBase & m_params_base;

			std::map<std::string, mrpt::math::CRuntimeCompiledExpression> m_score_exprs;  //!< score names -> score compiled expressions
			std::vector<mrpt::math::CRuntimeCompiledExpression>  m_movement_assert_exprs;
			std::map<std::string, double>     m_expr_vars;

		};
		DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE(CMultiObjectiveMotionOptimizerBase, NAV_IMPEXP)

	}
}

