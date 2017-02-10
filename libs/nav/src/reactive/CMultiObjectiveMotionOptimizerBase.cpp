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
#include <mrpt/system/string_utils.h>

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
	score_values.resize(movs.size());

	// For each movement:
	for (unsigned int mov_idx = 0; mov_idx< movs.size();++mov_idx)
	{
		const auto &m = movs[mov_idx];

		if (m.speed <= 0) // Invalid candidate:
		{
			score_values[mov_idx].clear();
			continue;
		}

		// Mark all vars as NaN so we detect uninitialized values:
		for (auto &p : m_expr_vars) {
			p.second = std::numeric_limits<double>::quiet_NaN();
		}

		exprtk::symbol_table<double> symbol_table;
		for (const auto &prop : m.props) {
			double & var = m_expr_vars[prop.first];
			var = prop.second;
			symbol_table.add_variable(prop.first, var);
		}
		symbol_table.add_constant("M_PI", M_PI);
		symbol_table.add_constants();

		// Upon first iteration: compile expressions
		if (m_score_exprs.size() != m_params_base.formula_score.size())
		{
			m_score_exprs.clear();

			for (const auto &f : m_params_base.formula_score)
			{
				auto &se = m_score_exprs[f.first];
		
				PIMPL_GET_REF(exprtk::expression<double>, se.compiled_formula).register_symbol_table(symbol_table);
				// Compile user-given expressions:
				exprtk::parser<double> parser;
				if (!parser.compile(f.second, PIMPL_GET_REF(exprtk::expression<double>, se.compiled_formula)))
					THROW_EXCEPTION_FMT("Error compiling score `%s` expression: `%s`. Error: `%s`", f.first.c_str(), f.second.c_str(), parser.error().c_str());
			}
		}

		// Upon first iteration: compile expressions
		if (m_movement_assert_exprs.size() != m_params_base.movement_assert.size())
		{
			const size_t N = m_params_base.movement_assert.size();
			m_movement_assert_exprs.clear();
			m_movement_assert_exprs.resize(N);
			for (size_t i=0;i<N;i++)
			{
				const auto &str = m_params_base.movement_assert[i];
				auto &ce = m_movement_assert_exprs[i];

				PIMPL_GET_REF(exprtk::expression<double>, ce.compiled_formula).register_symbol_table(symbol_table);

				// Compile user-given expressions:
				exprtk::parser<double> parser;
				if (!parser.compile(str, PIMPL_GET_REF(exprtk::expression<double>, ce.compiled_formula)))
					THROW_EXCEPTION_FMT("Error compiling assert expression: `%s`. Error: `%s`", str.c_str(), parser.error().c_str());
			}
		}

		// For each assert, evaluate it:
		{
			bool assert_failed = false;
			for (auto &ma : m_movement_assert_exprs)
			{
				const double val = PIMPL_GET_CONSTREF(exprtk::expression<double>, ma.compiled_formula).value();
				if (val == 0) {
					assert_failed = true;
					break;
				}
			}
			if (assert_failed) {
				score_values[mov_idx].clear();
				continue;
			}
		}

		// For each score: evaluate it
		for (auto &sc : m_score_exprs)
		{
			// Evaluate:
			const double val = PIMPL_GET_CONSTREF(exprtk::expression<double>, sc.second.compiled_formula).value();

			if (val != val /* NaN */)
			{
				THROW_EXCEPTION_FMT("Undefined value evaluating score `%s` for mov_idx=%u!", sc.first.c_str(), mov_idx);
			}

			// Store:
			score_values[mov_idx][sc.first] = val;
		}
	}

	// Optional score post-processing: normalize highest value to 1.0
	for (const auto& sScoreName : m_params_base.scores_to_normalize)
	{
		// Find max:
		double maxScore = .0;
		for (const auto &s : score_values)
		{
			const auto it = s.find(sScoreName);
			if (it != s.cend())
				mrpt::utils::keep_max(maxScore, it->second);
		}

		// Normalize:
		if (maxScore > 0 && maxScore!=1.0 /* already normalized! */)
		{
			double K = 1.0 / maxScore;
			for (auto &s : score_values)
			{
				auto it = s.find(sScoreName);
				if (it != s.end()) {
					it->second *= K;
				}
			}
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

CMultiObjectiveMotionOptimizerBase::TCompiledFormulaWrapper::TCompiledFormulaWrapper()
{
	PIMPL_CONSTRUCT(exprtk::expression<double>, compiled_formula);
}

CMultiObjectiveMotionOptimizerBase::TParamsBase::TParamsBase()
{
	// Default scores:
	formula_score["colision_free_distance"] = "colision_free_distance";
	formula_score["path_index_near_target"] = "var dif:=abs(target_k-move_k); if (dif>(num_paths/2)) { dif:=num_paths-dif; }; exp(-abs(dif / (num_paths/10.0)));";
	formula_score["euclidean_nearness"] = "(ref_dist - dist_eucl_final) / ref_dist";
	formula_score["hysteresis"] = "hysteresis";
	formula_score["clearance"] = "clearance";

	// Default:
	scores_to_normalize.push_back("clearance");
}

void CMultiObjectiveMotionOptimizerBase::TParamsBase::loadFromConfigFile(const mrpt::utils::CConfigFileBase &c, const std::string &s)
{
	// Load: formula_score
	{
		formula_score.clear();
		int idx = 1;
		for (;;idx++)
		{
			const std::string sKeyName = mrpt::format("score%i_name",idx), sKeyValue = mrpt::format("score%i_formula", idx);
			const std::string sName = c.read_string(s, sKeyName, "");
			const std::string sValue = c.read_string(s, sKeyValue, "");

			const bool none = (sName.empty() && sValue.empty());
			const bool both = (!sName.empty() && !sValue.empty());

			if (none && idx == 1)
				THROW_EXCEPTION_FMT("Expect at least a first `%s` and `%s` pair defining one score in section `[%s]`", sKeyName.c_str(), sKeyValue.c_str(), s.c_str());

			if (none)
				break;

			if (!both) {
				THROW_EXCEPTION_FMT("Both `%s` and `%s` must be provided in section `[%s]`", sKeyName.c_str(), sKeyValue.c_str(), s.c_str());
			}

			formula_score[sName] = sValue;
		}
	}

	// Load: movement_assert
	{
		movement_assert.clear();
		int idx = 1;
		for (;; idx++)
		{
			const std::string sKey = mrpt::format("movement_assert%i", idx);
			const std::string sValue = c.read_string(s, sKey, "");
			if (sValue.empty())
				break;
			movement_assert.push_back(sValue);
		}
	}

	{
		scores_to_normalize.clear();
		std::string sLst = c.read_string(s, "scores_to_normalize", "");
		if (!sLst.empty()) {
			mrpt::system::tokenize(sLst, ", \t", scores_to_normalize);
		}
	}
	
}

void CMultiObjectiveMotionOptimizerBase::TParamsBase::saveToConfigFile(mrpt::utils::CConfigFileBase &c, const std::string &s) const
{
	// Save: formula_score

	{
		const std::string sComment = "\n"
			"# Next follows a list of `score%i_{name,formula}` pairs for i=1,...,N\n"
			"# Each one defines one of the scores that will be evaluated for each candidate movement.\n"
			"# Multiobjective optimizers will then use those scores to select the best candidate, \n"
			"# possibly using more parameters that follow below.\n"
			;
		c.write(s, "dummy", "", mrpt::utils::MRPT_SAVE_NAME_PADDING, mrpt::utils::MRPT_SAVE_VALUE_PADDING, sComment);

		int idx = 0;
		for (const auto &p : this->formula_score)
		{
			++idx;
			const std::string sKeyName = mrpt::format("score%i_name", idx), sKeyValue = mrpt::format("score%i_formula", idx);
			c.write(s, sKeyName, p.first, mrpt::utils::MRPT_SAVE_NAME_PADDING, mrpt::utils::MRPT_SAVE_VALUE_PADDING);
			c.write(s, sKeyValue, p.second, mrpt::utils::MRPT_SAVE_NAME_PADDING, mrpt::utils::MRPT_SAVE_VALUE_PADDING);
		}
	}

	// Load: movement_assert
	{
		const std::string sComment = "\n"
			"# Next follows a list of `movement_assert%i` exprtk expressions for i=1,...,N\n"
			"# defining expressions for conditions that any candidate movement must fulfill\n"
			"# in order to get through the evaluation process. *All* assert conditions must be satisfied.\n"
			;
		c.write(s, "dummy2", "", mrpt::utils::MRPT_SAVE_NAME_PADDING, mrpt::utils::MRPT_SAVE_VALUE_PADDING, sComment);

		for (unsigned int idx=0;idx<movement_assert.size();idx++)
		{
			const std::string sKey = mrpt::format("movement_assert%i", idx+1);
			c.write(s, sKey, movement_assert[idx], mrpt::utils::MRPT_SAVE_NAME_PADDING, mrpt::utils::MRPT_SAVE_VALUE_PADDING);
		}
	}

	{
		std::string sLst;
		for (const auto& s : scores_to_normalize) {
			sLst += s;
			sLst += std::string(",");
		}
		c.write(s, "scores_to_normalize", sLst);
	}

}
