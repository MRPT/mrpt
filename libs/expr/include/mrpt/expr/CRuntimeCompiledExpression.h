/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/pimpl.h>

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "mrpt-expr_export.h"

namespace exprtk
{
template <typename T>
class expression;
}

namespace mrpt
{
namespace expr
{
/** A wrapper of `exprtk` runtime expression compiler: it takes a string
 * representing an expression (from a simple mathematical formula to a complete
 * program), compiles it and evaluates its result as many times as required. The
 * result will change as the "variables" appearing in the expression (hold and
 * managed by the user of this object) change.
 *
 * Refer to [exprtk documentation](https://github.com/ArashPartow/exprtk) for
 * reference on supported formulas, control flow instructions, etc.
 *
 * This wrapper is provided to reduce the (very large) compilation time and
 * memory required by the original library, at the cost of only exposing the
 * most commonly used part of its API:
 *  - Only expressions returning `double` are supported.
 *  - Variables must be provided via a `std::map` container **or** pointers to
 * user-stored variables.
 *  - Custom user-defined functions taking 0-3 arguments (New in MRPT 2.5.8).
 *
 * See examples of usage in the [unit test
 * file](https://github.com/MRPT/mrpt/blob/master/libs/base/src/math/CRuntimeCompiledExpression_unittest.cpp).
 *
 * If the environment variable `MRPT_EXPR_VERBOSE=1` is defined, debug
 * information will be dumped to std::cout explaining the values of **all** the
 * involved variables upon each call to `eval()`.
 * Alternatively, the env var `MRPT_EXPR_VERBOSE` can be set to a list of
 * terms split by `|`, and only those formulas that match (i.e. contain as
 * substrings) any of the terms will be traced.
 * Example: `MRPT_EXPR_VERBOSE="cos|sin|speed|if (x>0)"`.
 *
 * \note (New in MRPT 1.5.0)
 * \note (`MRPT_EXPR_VERBOSE` new in MRPT 1.5.7)
 * \ingroup mrpt_expr_grp
 */
// Note: Leave the MRPT_EXPR_EXPORT macro in mrpt-expr, to avoid exporting ALL
// symbols which would increase the lib size too much.
class MRPT_EXPR_EXPORT CRuntimeCompiledExpression
{
   public:
	/** Default ctor */
	CRuntimeCompiledExpression();
	~CRuntimeCompiledExpression();

	/** Initializes the object by compiling an expression.
	 * \exception std::runtime_error On any syntax error or undefined symbol
	 * while compiling the expression. The `e.what()` message describes what is
	 * exactly the problem.
	 * \sa register_symbol_table()
	 */
	void compile(/** [in] The expression to be compiled. */
				 const std::string& expression,
				 /** [in] Map of variables/constants by `name` ->  `value`. The
					references to the values in this map **must** be ensured to
					be
					valid thoughout all the life of the compiled expression. */
				 const std::map<std::string, double>& variables = {},
				 /** [in] A descriptive name of this formula, to be used when
					generating error reports via an  exception, if needed */
				 const std::string& expr_name_for_error_reporting = {});

	/** Can be used **before** calling compile() to register additional
	 * variables by means of **pointers** instead of a std::map  */
	void register_symbol_table(
		/** [in] Map of variables/constants by `name` ->  `value`. The
		   references to the values in this map **must** be ensured to be
		   valid thoughout all the life of the compiled expression. */
		const std::map<std::string, double*>& variables);

	/** Register a user-defined nullary function. (New in MRPT 2.5.8) */
	void register_function(
		const std::string& name, const std::function<double()>& func)
	{
		m_funcs_0[name] = func;
	}

	/** Register a user-defined unary function. (New in MRPT 2.5.8) */
	void register_function(
		const std::string& name, const std::function<double(double)>& func)
	{
		m_funcs_1[name] = func;
	}

	/** Register a user-defined binary function. (New in MRPT 2.5.8) */
	void register_function(
		const std::string& name,
		const std::function<double(double, double)>& func)
	{
		m_funcs_2[name] = func;
	}

	/** Register a user-defined ternary function. (New in MRPT 2.5.8) */
	void register_function(
		const std::string& name,
		const std::function<double(double, double, double)>& func)
	{
		m_funcs_3[name] = func;
	}

	/** Evaluates the current value of the precompiled formula.
	 * \exception std::runtime_error If the formula has not been compiled yet.
	 */
	double eval() const;

	/** Returns true if compile() was called and ended without errors. */
	bool is_compiled() const;
	/** Returns the original formula passed to compile(), or an empty string if
	 * still not compiled. */
	const std::string& get_original_expression() const;

	/** Access raw exprtk expression object. */
	exprtk::expression<double>& get_raw_exprtk_expr();
	/** Access raw exprtk expression object. */
	const exprtk::expression<double>& get_raw_exprtk_expr() const;

   private:
	struct Impl;
	mrpt::pimpl<Impl> m_impl;
	struct ExprVerbose;
	friend struct ExprVerbose;

	std::map<std::string, std::function<double()>> m_funcs_0;
	std::map<std::string, std::function<double(double)>> m_funcs_1;
	std::map<std::string, std::function<double(double, double)>> m_funcs_2;
	std::map<std::string, std::function<double(double, double, double)>>
		m_funcs_3;

};	// End of class def.

}  // namespace expr

}  // namespace mrpt
