/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/CRuntimeCompiledExpression.h>
#include <mrpt/system/string_utils.h>
#include <cstdlib>
#include <iostream>

#define exprtk_disable_string_capabilities   // Workaround a bug in Ubuntu precise's GCC+libstdc++
#include <mrpt/otherlibs/exprtk.hpp>

// We only need this to be on this translation unit, hence the advantage of using our MRPT wrapper instead 
// of the original exprtk sources.
PIMPL_IMPLEMENT(exprtk::expression<double>);

using namespace mrpt;
using namespace mrpt::math;

struct CRuntimeCompiledExpression::ExprVerbose
{
	static CRuntimeCompiledExpression::ExprVerbose & Instance()
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
		if (nullptr==sp) return;
		const std::string s = mrpt::system::trim(std::string(sp));

		if (s==std::string("1"))
		{
			m_verbose_always_enabled=true;
			return;
		}
		mrpt::system::tokenize(s,"|",m_verbose_matches);
	}
};

CRuntimeCompiledExpression::CRuntimeCompiledExpression()
{
	PIMPL_CONSTRUCT(exprtk::expression<double>, m_compiled_formula);
}

void CRuntimeCompiledExpression::compile(
	const std::string &expression,                    //!< [in] The expression to be compiled.
	const std::map<std::string, double> &variables,   //!< [in] Map of variables/constants by `name` ->  `value`. The references to the values in this map **must** be ensured to be valid thoughout all the life of the compiled expression.
	const std::string &expr_name_for_error_reporting  //!< A descriptive name of this formula, to be used when generating error reports via an  exception, if needed
)
{
	m_original_expr_str = expression;

	exprtk::symbol_table<double> symbol_table;
	for (const auto &v : variables) {
		double & var = const_cast<double&>(v.second);
		symbol_table.add_variable(v.first, var);
	}
	symbol_table.add_constant("M_PI", M_PI);
	symbol_table.add_constants();

	PIMPL_GET_REF(exprtk::expression<double>, m_compiled_formula).register_symbol_table(symbol_table);
	// Compile user-given expressions:
	exprtk::parser<double> parser;
	if (!parser.compile(expression, PIMPL_GET_REF(exprtk::expression<double>, m_compiled_formula)))
		THROW_EXCEPTION_FMT("Error compiling expression (name=`%s`): `%s`. Error: `%s`", expr_name_for_error_reporting.c_str(), expression.c_str(), parser.error().c_str());
}

double CRuntimeCompiledExpression::eval() const
{
	ASSERT_(m_compiled_formula.ptr.get() != nullptr);
	double ret = PIMPL_GET_CONSTREF(exprtk::expression<double>, m_compiled_formula).value();
	CRuntimeCompiledExpression::ExprVerbose::Instance().process(*this, ret);
	return ret;
}

void CRuntimeCompiledExpression::ExprVerbose::process(const CRuntimeCompiledExpression& rce, const double ret)
{
	if (!m_verbose_always_enabled && m_verbose_matches.empty()) return;

	if (!m_verbose_matches.empty())
	{
		bool matched=false;
		for (const auto &s:m_verbose_matches)
		{
			if (rce.m_original_expr_str.find(s)!=std::string::npos)
			{
				matched = true;
				break;
			}
		}
		if (!matched) return;
	}

	std::vector<std::pair<std::string,double> > lst;
	PIMPL_GET_REF(exprtk::expression<double>, rce.m_compiled_formula).get_symbol_table().get_variable_list(lst);
	std::cout << "[CRuntimeCompiledExpression::eval()] DEBUG:\n"
	             "* Expression: "<< rce.m_original_expr_str << "\n"
	             "* Final value: " << ret << "\n"
	             "* Using these symbols:\n";
	for (const auto &v : lst)
		std::cout << " * " <<  v.first << " = " << v.second << "\n";
}

void CRuntimeCompiledExpression::register_symbol_table(
	const std::map<std::string, double *> &variables  //!< [in] Map of variables/constants by `name` ->  `value`. The references to the values in this map **must** be ensured to be valid thoughout all the life of the compiled expression.
)
{
	exprtk::symbol_table<double> symbol_table;
	for (const auto &v : variables) {
		double * var = const_cast<double*>(v.second);
		symbol_table.add_variable(v.first, *var);
	}
	PIMPL_GET_REF(exprtk::expression<double>, m_compiled_formula).register_symbol_table(symbol_table);
}

exprtk::expression<double> & CRuntimeCompiledExpression::get_raw_exprtk_expr() {
	ASSERT_(m_compiled_formula.ptr.get() != nullptr);
	return PIMPL_GET_REF(exprtk::expression<double>, m_compiled_formula);
}
const exprtk::expression<double> & CRuntimeCompiledExpression::get_raw_exprtk_expr() const {
	ASSERT_(m_compiled_formula.ptr.get() != nullptr);
	return PIMPL_GET_CONSTREF(exprtk::expression<double>, m_compiled_formula);
}

bool CRuntimeCompiledExpression::is_compiled() const
{
	return m_compiled_formula.ptr.get() != nullptr;
}
const std::string & CRuntimeCompiledExpression::get_original_expression() const
{
	return m_original_expr_str;
}
