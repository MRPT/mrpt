/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h>

#define exprtk_disable_string_capabilities   // Workaround a bug in Ubuntu precise's GCC+libstdc++
#include <mrpt/otherlibs/exprtk.hpp>

using namespace mrpt::nav;
using namespace mrpt::utils;

IMPLEMENTS_MRPT_OBJECT(CMultiObjMotionOpt_Scalarization, CMultiObjectiveMotionOptimizerBase, mrpt::nav)


CMultiObjMotionOpt_Scalarization::CMultiObjMotionOpt_Scalarization() :
	CMultiObjectiveMotionOptimizerBase(parameters)
{
}

void CMultiObjMotionOpt_Scalarization::clear()
{
	CMultiObjectiveMotionOptimizerBase::clear();
	m_expr_scalar_formula = TCompiledFormulaWrapper();
	m_expr_scalar_vars.clear();
}

void CMultiObjMotionOpt_Scalarization::loadConfigFile(const mrpt::utils::CConfigFileBase & c)
{
	parameters.CMultiObjectiveMotionOptimizerBase::TParamsBase::loadFromConfigFile(c,"CMultiObjectiveMotionOptimizerBase"); // call base-member with special argument
	parameters.loadFromConfigFile(c, "CMultiObjMotionOpt_Scalarization");
}

void CMultiObjMotionOpt_Scalarization::saveConfigFile(mrpt::utils::CConfigFileBase & c) const
{
	parameters.CMultiObjectiveMotionOptimizerBase::TParamsBase::saveToConfigFile(c, "CMultiObjectiveMotionOptimizerBase"); // call base-member with special argument
	parameters.saveToConfigFile(c, "CMultiObjMotionOpt_Scalarization");
}

int CMultiObjMotionOpt_Scalarization::impl_decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info)
{
	std::vector<double> & final_evaluation = extra_info.final_evaluation;
	final_evaluation.clear();

	if (extra_info.score_values.empty())
		return -1; // No valid candidate

	// compile expression upon first use:
	if (m_expr_scalar_vars.empty())
	{
		// vars:
		exprtk::symbol_table<double> symbol_table;
		for (const auto &c : extra_info.score_values) {
			for (const auto &score : c) {
				double & var = m_expr_scalar_vars[score.first]; // create or reuse placeholder
				symbol_table.add_variable(score.first, var);
			}
		}
		symbol_table.add_constant("M_PI", M_PI);
		symbol_table.add_constants();

		// formula:
		m_expr_scalar_formula = TCompiledFormulaWrapper();
		PIMPL_GET_REF(exprtk::expression<double>, m_expr_scalar_formula.compiled_formula).register_symbol_table(symbol_table);
		// Compile user-given expressions:
		exprtk::parser<double> parser;
		if (!parser.compile(parameters.scalar_score_formula, PIMPL_GET_REF(exprtk::expression<double>, m_expr_scalar_formula.compiled_formula)))
			THROW_EXCEPTION_FMT("Error compiling `scalar_score_formula` expression: `%s`. Error: `%s`", parameters.scalar_score_formula.c_str(), parser.error().c_str());
	}

	// Evaluate the formula for all candidates:
	const size_t N = extra_info.score_values.size();
	final_evaluation.assign(N, .0);
	int    best_idx = -1;
	double best_val = .0;
	for (size_t i = 0; i < N; i++)
	{
		if (extra_info.score_values[i].empty())
			continue; // this candidate is non-eligible.

		// Update variables & evaluate formula:
		for (const auto &score : extra_info.score_values[i]) {
			auto & it = m_expr_scalar_vars.find(score.first); // this should never *create* a new entry...
			if (it == m_expr_scalar_vars.cend()) {
				THROW_EXCEPTION_FMT("Error: found unexpected (unregistered) score named `%s`.", score.first.c_str());
			}
			double & var = it->second;
			var = score.second;
		}
		const double val = PIMPL_GET_CONSTREF(exprtk::expression<double>, m_expr_scalar_formula.compiled_formula).value();
		extra_info.final_evaluation[i] = val;

		if (val > 0 && (best_idx == -1 || val > best_val))
		{
			best_idx = i;
			best_val = val;
		}
	}

	return best_idx;
}

CMultiObjMotionOpt_Scalarization::TParams::TParams()
{
}

void CMultiObjMotionOpt_Scalarization::TParams::loadFromConfigFile(const mrpt::utils::CConfigFileBase &c, const std::string &s)
{
	// TParamsBase members are already dealt elsewhere.
	MRPT_LOAD_CONFIG_VAR_REQUIRED_CS(scalar_score_formula, string);
}

void CMultiObjMotionOpt_Scalarization::TParams::saveToConfigFile(mrpt::utils::CConfigFileBase &c, const std::string &s) const
{
	// TParamsBase members are already dealt elsewhere.
	MRPT_SAVE_CONFIG_VAR_COMMENT(scalar_score_formula, "A formula that takes all/a subset of scores and generates a scalar global score.");
}

