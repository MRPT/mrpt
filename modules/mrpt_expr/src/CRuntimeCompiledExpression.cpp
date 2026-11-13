/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#define _USE_MATH_DEFINES  // To have M_PI on MSVC (must precede any <cmath>)

#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>
#include <mrpt/system/string_utils.h>

#include <cmath>  // M_PI
#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

#define exprtk_disable_string_capabilities  // Workaround a bug in Ubuntu
// precise's GCC+libstdc++

// Disable some parts of exprtk to build faster and reduce lib size:
#define exprtk_disable_enhanced_features
#define exprtk_disable_superscalar_unroll
#define exprtk_disable_rtl_vecops
#define exprtk_disable_rtl_io_file

#if defined(MRPT_USE_SYSTEM_EXPRTK)
#include <exprtk.hpp>
#else
#include <3rdparty/exprtk.hpp>
#endif

using namespace mrpt;
using namespace mrpt::expr;

struct CRuntimeCompiledExpression::ExprVerbose
{
  static CRuntimeCompiledExpression::ExprVerbose& Instance()
  {
    static CRuntimeCompiledExpression::ExprVerbose obj;
    return obj;
  }
  void process(const CRuntimeCompiledExpression& rce, double ret);

 private:
  bool m_verbose_always_enabled = false;
  std::vector<std::string> m_verbose_matches;

  ExprVerbose()
  {
    const std::string s = mrpt::system::trim(mrpt::get_env<std::string>("MRPT_EXPR_VERBOSE"));

    if (s == std::string("1"))
    {
      m_verbose_always_enabled = true;
      return;
    }
    mrpt::system::tokenize(s, "|", m_verbose_matches);
  }
};

namespace
{
// exprtk adaptor for user functions held in a std::function (raw function
// pointers, lambdas with captures, Python callables...):
template <typename... Args>
struct FunctionAdaptor : public exprtk::ifunction<double>
{
  using exprtk::ifunction<double>::operator();

  explicit FunctionAdaptor(std::function<double(Args...)> f) :
      exprtk::ifunction<double>(sizeof...(Args)), func(std::move(f))
  {
  }

  double operator()(const Args&... args) override { return func(args...); }

  std::function<double(Args...)> func;
};
}  // namespace

// We only need this to be on this translation unit, hence the advantage of
// using our MRPT wrapper instead of the original exprtk sources.
struct CRuntimeCompiledExpression::Impl
{
  exprtk::expression<double> m_compiled_formula;
  std::string m_original_expr_str;
  bool m_compiled = false;

  /** Variables registered with register_symbol_table() */
  std::map<std::string, double*> m_user_variables;

  /** Adaptors of the user functions, referenced by the compiled formula */
  std::vector<std::shared_ptr<exprtk::ifunction<double>>> m_function_adaptors;

  template <typename... Args>
  void add_functions(
      const std::map<std::string, std::function<double(Args...)>>& funcs,
      exprtk::symbol_table<double>& symbol_table)
  {
    for (const auto& [name, func] : funcs)
    {
      auto adaptor = std::make_shared<FunctionAdaptor<Args...>>(func);
      if (!symbol_table.add_function(name, *adaptor))
      {
        THROW_EXCEPTION_FMT(
            "Cannot register user function `%s`: invalid, reserved or duplicated name",
            name.c_str());
      }
      m_function_adaptors.push_back(std::move(adaptor));
    }
  }
};

CRuntimeCompiledExpression::CRuntimeCompiledExpression() :
    m_impl(mrpt::make_impl<CRuntimeCompiledExpression::Impl>())
{
}
CRuntimeCompiledExpression::~CRuntimeCompiledExpression() = default;
CRuntimeCompiledExpression::CRuntimeCompiledExpression(const CRuntimeCompiledExpression&) = default;
CRuntimeCompiledExpression& CRuntimeCompiledExpression::operator=(
    const CRuntimeCompiledExpression&) = default;
CRuntimeCompiledExpression::CRuntimeCompiledExpression(CRuntimeCompiledExpression&&) noexcept =
    default;
CRuntimeCompiledExpression& CRuntimeCompiledExpression::operator=(
    CRuntimeCompiledExpression&&) noexcept = default;

void CRuntimeCompiledExpression::compile(
    const std::string& expression,
    const std::map<std::string, double>& variables,
    const std::string& expr_name_for_error_reporting)
{
  m_impl->m_original_expr_str = expression;
  m_impl->m_compiled = false;

  // Start from scratch: exprtk resolves each symbol in the first registered
  // table that defines it, so tables from former calls must not survive.
  m_impl->m_compiled_formula = exprtk::expression<double>();
  m_impl->m_function_adaptors.clear();

  exprtk::symbol_table<double> symbol_table;
  for (const auto& [name, ptr] : m_impl->m_user_variables)
  {
    symbol_table.add_variable(name, *ptr);
  }
  for (const auto& v : variables)
  {
    auto& var = const_cast<double&>(v.second);
    symbol_table.add_variable(v.first, var);
  }
  symbol_table.add_constant("M_PI", M_PI);
  symbol_table.add_constants();

  m_impl->add_functions(m_funcs_0, symbol_table);
  m_impl->add_functions(m_funcs_1, symbol_table);
  m_impl->add_functions(m_funcs_2, symbol_table);
  m_impl->add_functions(m_funcs_3, symbol_table);

  m_impl->m_compiled_formula.register_symbol_table(symbol_table);

  // Compile user-given expressions:
  exprtk::parser<double> parser;
  if (!parser.compile(expression, m_impl->m_compiled_formula))
  {
    THROW_EXCEPTION_FMT(
        "Error compiling expression (name=`%s`): `%s`. Error: `%s`",
        expr_name_for_error_reporting.c_str(), expression.c_str(), parser.error().c_str());
  }

  m_impl->m_compiled = true;
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
  for (const auto& [name, ptr] : variables)
  {
    m_impl->m_user_variables[name] = ptr;
  }
}

void CRuntimeCompiledExpression::register_function(
    const std::string& name, const std::function<double()>& func)
{
  m_funcs_0[name] = func;
}

void CRuntimeCompiledExpression::register_function(
    const std::string& name, const std::function<double(double)>& func)
{
  m_funcs_1[name] = func;
}

void CRuntimeCompiledExpression::register_function(
    const std::string& name, const std::function<double(double, double)>& func)
{
  m_funcs_2[name] = func;
}

void CRuntimeCompiledExpression::register_function(
    const std::string& name, const std::function<double(double, double, double)>& func)
{
  m_funcs_3[name] = func;
}

exprtk::expression<double>& CRuntimeCompiledExpression::get_raw_exprtk_expr()
{
  ASSERT_(m_impl);
  return m_impl->m_compiled_formula;
}
const exprtk::expression<double>& CRuntimeCompiledExpression::get_raw_exprtk_expr() const
{
  ASSERT_(m_impl);
  return m_impl->m_compiled_formula;
}

bool CRuntimeCompiledExpression::is_compiled() const
{
  ASSERT_(m_impl);
  return m_impl->m_compiled;
}
const std::string& CRuntimeCompiledExpression::get_original_expression() const
{
  return m_impl->m_original_expr_str;
}

void CRuntimeCompiledExpression::ExprVerbose::process(
    const CRuntimeCompiledExpression& rce, const double ret)
{
  if (!m_verbose_always_enabled && m_verbose_matches.empty())
  {
    return;
  }

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
    if (!matched)
    {
      return;
    }
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
  {
    std::cout << " * " << v.first << " = " << v.second << "\n";
  }
}
