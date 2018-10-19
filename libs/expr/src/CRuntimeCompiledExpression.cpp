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
#include <mrpt/core/exceptions.h>
#include <cmath>  // M_PI

#define exprtk_disable_string_capabilities  // Workaround a bug in Ubuntu
// precise's GCC+libstdc++

// Disable some parts of exprtk to build faster and reduce lib size:
#define exprtk_disable_enhanced_features
#define exprtk_disable_superscalar_unroll
#define exprtk_disable_rtl_vecops
#define exprtk_disable_rtl_io_file
#include <mrpt/otherlibs/exprtk.hpp>

// We only need this to be on this translation unit, hence the advantage of
// using our MRPT wrapper instead
// of the original exprtk sources.
// PIMPL_IMPLEMENT(exprtk::expression<double>

using namespace mrpt;
using namespace mrpt::expr;

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
	return m_impl->m_compiled_formula.value();
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
