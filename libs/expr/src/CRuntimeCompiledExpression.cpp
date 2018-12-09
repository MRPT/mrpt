/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "expr-precomp.h"  // Precompiled headers

#include <mrpt/expr/CRuntimeCompiledExpression.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/core/exceptions.h>
#include <cmath>  // M_PI
#include <cstdlib>
#include <iostream>

#define exprtk_disable_string_capabilities  // Workaround a bug in Ubuntu
// precise's GCC+libstdc++

// Disable some parts of exprtk to build faster and reduce lib size:
#define exprtk_disable_enhanced_features
#define exprtk_disable_superscalar_unroll
#define exprtk_disable_rtl_vecops
#define exprtk_disable_rtl_io_file
#include <mrpt/otherlibs/exprtk.hpp>

using namespace mrpt;
using namespace mrpt::expr;

struct CRuntimeCompiledExpression::ExprVerbose
{
	static CRuntimeCompiledExpression::ExprVerbose& Instance()
	{
		static CRuntimeCompiledExpression::ExprVerbose obj;
		return obj;
	}
	void process(const CRuntimeCompiledExpression& rce, const double ret);

   private:
	bool m_verbose_always_enabled{false};
	std::vector<std::string> m_verbose_matches;
	ExprVerbose()
	{
		const char* sp = ::getenv("MRPT_EXPR_VERBOSE");
		if (nullptr == sp) return;
		const std::string s = mrpt::system::trim(std::string(sp));

		if (s == std::string("1"))
		{
			m_verbose_always_enabled = true;
			return;
		}
		mrpt::system::tokenize(s, "|", m_verbose_matches);
	}
};

// We only need this to be on this translation unit, hence the advantage of
// using our MRPT wrapper instead of the original exprtk sources.
struct CRuntimeCompiledExpression::Impl
{
	exprtk::expression<double> m_compiled_formula;
	std::string m_original_expr_str;
};

CRuntimeCompiledExpression::CRuntimeCompiledExpression()
	: m_impl(mrpt::make_impl<CRuntimeCompiledExpression::Impl>())
{
}

CRuntimeCompiledExpression::~CRuntimeCompiledExpression() = default;

void CRuntimeCompiledExpression::compile(
	/** [in] The expression to be compiled. */
	const std::string& expression,
	/** [in] Map of variables/constants by `name` ->  `value`. The references to
	   the values in this map **must** be ensured to be valid thoughout all the
	   life of the compiled expression. */
	const std::map<std::string, double>& variables,
	/** A descriptive name of this formula, to be used when generating error
	   reports via an  exception, if needed */
	const std::string& expr_name_for_error_reporting)
{
	m_impl->m_original_expr_str = expression;

	exprtk::symbol_table<double> symbol_table;
	for (const auto& v : variables)
	{
		auto& var = const_cast<double&>(v.second);
		symbol_table.add_variable(v.first, var);
	}
	symbol_table.add_constant("M_PI", M_PI);
	symbol_table.add_constants();

	m_impl->m_compiled_formula.register_symbol_table(symbol_table);

	// Compile user-given expressions:
	exprtk::parser<double> parser;
	if (!parser.compile(expression, m_impl->m_compiled_formula))
		THROW_EXCEPTION_FMT(
			"Error compiling expression (name=`%s`): `%s`. Error: `%s`",
			expr_name_for_error_reporting.c_str(), expression.c_str(),
			parser.error().c_str());
}

double CRuntimeCompiledExpression::eval() const
{
	ASSERT_(m_impl);
	double ret = m_impl->m_compiled_formula.value();
	ExprVerbose::Instance().process(*this, ret);
	return ret;
}

void CRuntimeCompiledExpression::register_symbol_table(
	/** [in] Map of variables/constants by `name` ->  `value`. The
	   references to the values in this map **must** be ensured to be valid
	   thoughout all the life of the compiled expression. */
	const std::map<std::string, double*>& variables)
{
	exprtk::symbol_table<double> symbol_table;
	for (const auto& v : variables)
	{
		auto* var = const_cast<double*>(v.second);
		symbol_table.add_variable(v.first, *var);
	}
	m_impl->m_compiled_formula.register_symbol_table(symbol_table);
}

exprtk::expression<double>& CRuntimeCompiledExpression::get_raw_exprtk_expr()
{
	ASSERT_(m_impl);
	return m_impl->m_compiled_formula;
}
const exprtk::expression<double>&
	CRuntimeCompiledExpression::get_raw_exprtk_expr() const
{
	ASSERT_(m_impl);
	return m_impl->m_compiled_formula;
}

bool CRuntimeCompiledExpression::is_compiled() const
{
	ASSERT_(m_impl);
	return m_impl->m_compiled_formula;
}
const std::string& CRuntimeCompiledExpression::get_original_expression() const
{
	return m_impl->m_original_expr_str;
}

void CRuntimeCompiledExpression::ExprVerbose::process(
	const CRuntimeCompiledExpression& rce, const double ret)
{
	if (!m_verbose_always_enabled && m_verbose_matches.empty()) return;

	const auto& exp = *rce.m_impl.get();

	if (!m_verbose_matches.empty())
	{
		bool matched = false;
		for (const auto& s : m_verbose_matches)
		{
			if (exp.m_original_expr_str.find(s) != std::string::npos)
			{
				matched = true;
				break;
			}
		}
		if (!matched) return;
	}

	std::vector<std::pair<std::string, double>> lst;
	exp.m_compiled_formula.get_symbol_table().get_variable_list(lst);
	// clang-format off
    std::cout << "[CRuntimeCompiledExpression::eval()] DEBUG:\n"
                 "* Expression: "
              << exp.m_original_expr_str << "\n"
                 "* Final value: " << ret << "\n"
                 "* Using these symbols:\n";
	// clang-format on
	for (const auto& v : lst)
		std::cout << " * " << v.first << " = " << v.second << "\n";
}
