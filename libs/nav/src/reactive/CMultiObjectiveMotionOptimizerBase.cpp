/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h>

#define exprtk_disable_string_capabilities   // Workaround a bug in Ubuntu precise's GCC+libstdc++
#include <mrpt/otherlibs/exprtk.hpp>

using namespace mrpt::nav;
using namespace mrpt::utils;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(CMultiObjectiveMotionOptimizerBase, CObject, mrpt::nav)

CMultiObjectiveMotionOptimizerBase::CMultiObjectiveMotionOptimizerBase(TParamsBase & params) :
	m_params_base(params)
{
}

int CMultiObjectiveMotionOptimizerBase::decide(const std::vector<mrpt::nav::TCandidateMovementPTG>& movs, TResultInfo & extra_info)
{
	auto & score_values = extra_info.score_values;

	// Mark all vars as NaN so we detect uninitialized values:
	for (auto &p : m_expr_vars) {
		p.second = std::numeric_limits<double>::quiet_NaN();
	}

	// Register / update variable names so we can evaluate the score formulas for each movement:
	// For each movement:
	for (const auto &m : movs)
	{
		exprtk::symbol_table<double> symbol_table;
		for (const auto &prop : m.props) {
			double & var = m_expr_vars[prop.first];
			var = prop.second;
			symbol_table.add_variable(prop.first, var);
		}
		symbol_table.add_constant("M_PI", M_PI);
		symbol_table.add_constants();

		// Upon first iteration: compile expression:
		if (m_score_exprs.size() != m_params_base.formula_score.size())
		{
			m_score_exprs.clear();

			for (const auto &f : m_params_base.formula_score)
			{
				auto &se = m_score_exprs[f.first];
		
				// Compile user-given expressions:
				exprtk::parser<double> parser;
				if (!parser.compile(f.second, PIMPL_GET_REF(exprtk::expression<double>, se.compiled_formula)))
					THROW_EXCEPTION(mrpt::format("Error compiling score `%s` expression: `%s`. Error: `%s`", f.first.c_str(), f.second.c_str(), parser.error().c_str()));
			}
		}

		// For each score: evaluate it
		for (auto &sc : m_score_exprs)
		{
			PIMPL_GET_REF(exprtk::expression<double>, sc.second.compiled_formula).clear_symbol_tables();
			PIMPL_GET_REF(exprtk::expression<double>, sc.second.compiled_formula).register_symbol_table(symbol_table);
			
			// Evaluate:
			const double val = PIMPL_GET_CONSTREF(exprtk::expression<double>, sc.second.compiled_formula).value();

			if (val != val /* NaN */)
			{
				THROW_EXCEPTION(mrpt::format("Undefined value evaluating score `%s`!", sc.first.c_str()));
			}

			// Store:
			score_values[sc.first] = val;
		}
	}

	// Run algorithm:
	return impl_decide(movs, extra_info);
}

void CMultiObjectiveMotionOptimizerBase::clear()
{
	m_score_exprs.clear();
}

CMultiObjectiveMotionOptimizerBase * CMultiObjectiveMotionOptimizerBase::Create(const std::string &className) MRPT_NO_THROWS
{
	try 
	{
		mrpt::utils::registerAllPendingClasses();

		// Factory:
		const mrpt::utils::TRuntimeClassId *classId = mrpt::utils::findRegisteredClass(className);
		if (!classId) return nullptr;

		CMultiObjectiveMotionOptimizerBase *holo = dynamic_cast<CMultiObjectiveMotionOptimizerBase*>(classId->createObject());
		return holo;
	}
	catch (...)
	{
		return nullptr;
	}
}

CMultiObjectiveMotionOptimizerBase::TScoreData::TScoreData()
{
	PIMPL_CONSTRUCT(exprtk::expression<double>, compiled_formula);
}
