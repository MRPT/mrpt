/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/pimpl.h>
#include <mrpt/base/link_pragmas.h>
#include <map>
#include <string>

PIMPL_FORWARD_DECLARATION(namespace exprtk { template <typename T> class expression; })

namespace mrpt
{
namespace math
{
	/** A wrapper of `exprtk` runtime expression compiler: it takes a string representing an
	 * expression (from a simple mathematical formula to a complete program), compiles it 
	 * and evaluates its result as many times as required. The result will change as the "variables"
	 * appearing in the expression (hold and managed by the user of this object) change.
	 *
	 * Refer to [exprtk documentation](https://github.com/ArashPartow/exprtk) for reference on supported formulas, control flow instructions, etc.
	 *
	 * This wrapper is provided to reduce the (very large) compilation time and memory required by the original 
	 * library, at the cost of only exposing the most commonly used part of its API:
	 *  - Only expressions returning `double` are supported.
	 *  - Variables must be provided via a `std::map` container **or** pointers to user-stored variables.
	 *
	 * See examples of usage in the [unit test file](https://github.com/MRPT/mrpt/blob/master/libs/base/src/math/CRuntimeCompiledExpression_unittest.cpp).
	 *
	 * \note (New in MRPT 1.5.0)
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CRuntimeCompiledExpression
	{
	public:
		CRuntimeCompiledExpression(); //!< Default ctor

		/** Initializes the object by compiling an expression. 
		  * \exception std::runtime_error On any syntax error or undefined symbol while compiling the expression. The `e.what()` message describes what is exactly the problem.
		  * \sa register_symbol_table()
		  */
		void compile(
			const std::string &expression,                  //!< [in] The expression to be compiled.
			const std::map<std::string, double> &variables = std::map<std::string, double>(),  //!< [in] Map of variables/constants by `name` ->  `value`. The references to the values in this map **must** be ensured to be valid thoughout all the life of the compiled expression.
			const std::string &expr_name_for_error_reporting = std::string()  //!< A descriptive name of this formula, to be used when generating error reports via an  exception, if needed
		);

		/** Can be used **before** calling compile() to register additional variables by means of **pointers** instead of a std::map  */
		void register_symbol_table(
			const std::map<std::string, double *> &variables  //!< [in] Map of variables/constants by `name` ->  `value`. The references to the values in this map **must** be ensured to be valid thoughout all the life of the compiled expression.
		);

		/** Evaluates the current value of the precompiled formula. 
		  * \exception std::runtime_error If the formula has not been compiled yet.
		*/
		double eval() const;

		bool is_compiled() const; //!< Returns true if compile() was called and ended without errors.
		const std::string & get_original_expression() const; //!< Returns the original formula passed to compile(), or an empty string if still not compiled.

		exprtk::expression<double> & get_raw_exprtk_expr(); //!< Access raw exprtk expression object.
		const exprtk::expression<double> & get_raw_exprtk_expr() const; //!< Access raw exprtk expression object.

	private:
		PIMPL_DECLARE_TYPE(exprtk::expression<double>, m_compiled_formula);
		std::string m_original_expr_str;

	}; // End of class def.

	} // End of namespace
} // End of namespace

