/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h>
#include <mrpt/system/string_utils.h>
#include <limits>

using namespace mrpt::nav;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(
	CMultiObjectiveMotionOptimizerBase, CObject, mrpt::nav)

CMultiObjectiveMotionOptimizerBase::CMultiObjectiveMotionOptimizerBase(
	TParamsBase& params)
	: m_params_base(params)
{
}

int CMultiObjectiveMotionOptimizerBase::decide(
	const std::vector<mrpt::nav::TCandidateMovementPTG>& movs,
	TResultInfo& extra_info)
{
	auto& score_values = extra_info.score_values;
	score_values.resize(movs.size());

	// For each movement:
	for (unsigned int mov_idx = 0; mov_idx < movs.size(); ++mov_idx)
	{
		const auto& m = movs[mov_idx];

		// Mark all vars as NaN so we detect uninitialized values:
		for (auto& p : m_expr_vars)
		{
			p.second = std::numeric_limits<double>::quiet_NaN();
		}

		for (const auto& prop : m.props)
		{
			double& var = m_expr_vars[prop.first];
			var = prop.second;
		}

		// Upon first iteration: compile expressions
		if (m_score_exprs.size() != m_params_base.formula_score.size())
		{
			m_score_exprs.clear();

			for (const auto& f : m_params_base.formula_score)
			{
				auto& se = m_score_exprs[f.first];
				try
				{
					se.compile(
						f.second, m_expr_vars,
						std::string("score: ") + f.first);
				}
				catch (std::exception&)
				{
					m_score_exprs.clear();
					throw;  // rethrow
				}

				// Register formulas also as variables, usable by the assert()
				// expressions:
				{
					auto it = m_expr_vars.find(f.first);
					if (it != m_expr_vars.end())
					{
						THROW_EXCEPTION_FMT(
							"Error: Expression name `%s` already exists as an "
							"input variable.",
							f.first.c_str());
					}
					// Add it:
					m_expr_vars[f.first] =
						std::numeric_limits<double>::quiet_NaN();
				}
			}
		}  // end for each score expr

		// Upon first iteration: compile expressions
		if (m_movement_assert_exprs.size() !=
			m_params_base.movement_assert.size())
		{
			const size_t N = m_params_base.movement_assert.size();
			m_movement_assert_exprs.clear();
			m_movement_assert_exprs.resize(N);
			for (size_t i = 0; i < N; i++)
			{
				const auto& str = m_params_base.movement_assert[i];
				auto& ce = m_movement_assert_exprs[i];

				try
				{
					ce.compile(str, m_expr_vars, "assert");
				}
				catch (std::exception&)
				{
					m_movement_assert_exprs.clear();
					throw;  // rethrow
				}
			}
		}

		// For each score: evaluate it
		for (auto& sc : m_score_exprs)
		{
			// Evaluate:
			double val;
			if (m.speed <= 0)  // Invalid candidate
			{
				val = .0;
			}
			else
			{
				val = sc.second.eval();
			}

			if (val != val /* NaN */)
			{
				THROW_EXCEPTION_FMT(
					"Undefined value evaluating score `%s` for mov_idx=%u!",
					sc.first.c_str(), mov_idx);
			}

			// Store:
			score_values[mov_idx][sc.first] = val;
		}
	}  // end for mov_idx

	// Optional score post-processing: normalize highest value to 1.0
	for (const auto& sScoreName : m_params_base.scores_to_normalize)
	{
		// Find max:
		double maxScore = .0;
		for (const auto& s : score_values)
		{
			const auto it = s.find(sScoreName);
			if (it != s.cend()) mrpt::keep_max(maxScore, it->second);
		}

		// Normalize:
		if (maxScore <= 0)  // all scores=0... let's decide that all are equal,
		// so normalized to "1"
		{
			for (auto& s : score_values)
			{
				auto it = s.find(sScoreName);
				if (it != s.end())
				{
					it->second = 1.0;
				}
			}
		}
		else if (maxScore > 0 && maxScore != 1.0 /* already normalized! */)
		{
			double K = 1.0 / maxScore;
			for (auto& s : score_values)
			{
				auto it = s.find(sScoreName);
				if (it != s.end())
				{
					it->second *= K;
				}
			}
		}
	}

	// For each assert, evaluate it (*after* score normalization)
	for (unsigned int mov_idx = 0; mov_idx < movs.size(); ++mov_idx)
	{
		const auto& m = movs[mov_idx];
		// Mark all vars as NaN so we detect uninitialized values:
		for (auto& p : m_expr_vars)
		{
			p.second = std::numeric_limits<double>::quiet_NaN();
		}
		for (const auto& prop : m.props)
		{
			double& var = m_expr_vars[prop.first];
			var = prop.second;
		}

		bool assert_failed = false;
		{
			for (auto& ma : m_movement_assert_exprs)
			{
				const double val = ma.eval();
				if (val == 0)
				{
					assert_failed = true;
					extra_info.log_entries.emplace_back(mrpt::format(
						"[CMultiObjectiveMotionOptimizerBase] "
						"mov_idx=%u ASSERT failed: `%s`",
						mov_idx, ma.get_original_expression().c_str()));
					break;
				}
			}
		}
		if (assert_failed)
		{
			for (auto& e : score_values[mov_idx])
			{
				e.second = .0;
			}
		}
	}  // end mov_idx

	// Run algorithm:
	return impl_decide(movs, extra_info);
}

void CMultiObjectiveMotionOptimizerBase::clear() { m_score_exprs.clear(); }
CMultiObjectiveMotionOptimizerBase::Ptr
	CMultiObjectiveMotionOptimizerBase::Factory(
		const std::string& className) noexcept
{
	try
	{
		mrpt::rtti::registerAllPendingClasses();

		// Factory:
		const mrpt::rtti::TRuntimeClassId* classId =
			mrpt::rtti::findRegisteredClass(className);
		if (!classId) return nullptr;

		return CMultiObjectiveMotionOptimizerBase::Ptr(
			dynamic_cast<CMultiObjectiveMotionOptimizerBase*>(
				classId->createObject()));
	}
	catch (...)
	{
		return nullptr;
	}
}

CMultiObjectiveMotionOptimizerBase::TParamsBase::TParamsBase()
{
	// Default scores:
	formula_score["collision_free_distance"] = "collision_free_distance";
	formula_score["path_index_near_target"] =
		"var dif:=abs(target_k-move_k); if (dif>(num_paths/2)) { "
		"dif:=num_paths-dif; }; exp(-abs(dif / (num_paths/10.0)));";
	formula_score["euclidean_nearness"] =
		"(ref_dist - dist_eucl_final) / ref_dist";
	formula_score["hysteresis"] = "hysteresis";
	formula_score["clearance"] = "clearance";

	// Default:
	scores_to_normalize.emplace_back("clearance");
}

void CMultiObjectiveMotionOptimizerBase::TParamsBase::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	// Load: formula_score
	{
		formula_score.clear();
		int idx = 1;
		for (;; idx++)
		{
			const std::string sKeyName = mrpt::format("score%i_name", idx),
							  sKeyValue = mrpt::format("score%i_formula", idx);
			const std::string sName = c.read_string(s, sKeyName, "");
			const std::string sValue = c.read_string(s, sKeyValue, "");

			const bool none = (sName.empty() && sValue.empty());
			const bool both = (!sName.empty() && !sValue.empty());

			if (none && idx == 1)
				THROW_EXCEPTION_FMT(
					"Expect at least a first `%s` and `%s` pair defining one "
					"score in section `[%s]`",
					sKeyName.c_str(), sKeyValue.c_str(), s.c_str());

			if (none) break;

			if (!both)
			{
				THROW_EXCEPTION_FMT(
					"Both `%s` and `%s` must be provided in section `[%s]`",
					sKeyName.c_str(), sKeyValue.c_str(), s.c_str());
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
			if (sValue.empty()) break;
			movement_assert.push_back(sValue);
		}
	}

	{
		scores_to_normalize.clear();
		std::string sLst = c.read_string(s, "scores_to_normalize", "");
		if (!sLst.empty())
		{
			mrpt::system::tokenize(sLst, ", \t", scores_to_normalize);
		}
	}
}

void CMultiObjectiveMotionOptimizerBase::TParamsBase::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	// Save: formula_score
	const int WN = mrpt::config::MRPT_SAVE_NAME_PADDING(),
			  WV = mrpt::config::MRPT_SAVE_VALUE_PADDING();

	{
		const std::string sComment =
			"\n"
			"# Next follows a list of `score%i_{name,formula}` pairs for "
			"i=1,...,N\n"
			"# Each one defines one of the scores that will be evaluated for "
			"each candidate movement.\n"
			"# Multiobjective optimizers will then use those scores to select "
			"the best candidate, \n"
			"# possibly using more parameters that follow below.\n";
		c.write(s, "dummy", "", WN, WV, sComment);

		int idx = 0;
		for (const auto& p : this->formula_score)
		{
			++idx;
			const std::string sKeyName = mrpt::format("score%i_name", idx),
							  sKeyValue = mrpt::format("score%i_formula", idx);
			c.write(s, sKeyName, p.first, WN, WV);
			c.write(s, sKeyValue, p.second, WN, WV);
		}
	}

	// Load: movement_assert
	{
		const std::string sComment =
			"\n"
			"# Next follows a list of `movement_assert%i` exprtk expressions "
			"for i=1,...,N\n"
			"# defining expressions for conditions that any candidate movement "
			"must fulfill\n"
			"# in order to get through the evaluation process. *All* assert "
			"conditions must be satisfied.\n";
		c.write(s, "dummy2", "", WN, WV, sComment);

		for (unsigned int idx = 0; idx < movement_assert.size(); idx++)
		{
			const std::string sKey = mrpt::format("movement_assert%i", idx + 1);
			c.write(s, sKey, movement_assert[idx], WN, WV);
		}
	}

	{
		std::string sLst;
		for (const auto& sc : scores_to_normalize)
		{
			sLst += sc;
			sLst += std::string(",");
		}
		c.write(s, "scores_to_normalize", sLst);
	}
}
