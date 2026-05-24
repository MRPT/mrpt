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

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>

#include <cmath>
#include <cstdlib>

template class mrpt::CTraitsTest<mrpt::expr::CRuntimeCompiledExpression>;

// =========================================================================
//  VerboseTest suite — MUST appear first so SetUpTestSuite fires before any
//  eval() call constructs the ExprVerbose singleton.
// =========================================================================

class VerboseTest : public ::testing::Test
{
 public:
  static void SetUpTestSuite()
  {
#ifdef _WIN32
    _putenv_s("MRPT_EXPR_VERBOSE", "1");
#else
    ::setenv("MRPT_EXPR_VERBOSE", "1", 1);
#endif
  }
  static void TearDownTestSuite()
  {
#ifdef _WIN32
    _putenv_s("MRPT_EXPR_VERBOSE", "");
#else
    ::unsetenv("MRPT_EXPR_VERBOSE");
#endif
  }
};

TEST_F(VerboseTest, VerbosePath)
{
  // ExprVerbose singleton is constructed on first eval() call.
  // SetUpTestSuite has already set MRPT_EXPR_VERBOSE=1 above, so the
  // singleton will be built with verbose always-enabled.
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;
  vars["v"] = 7.0;
  expr.compile("v*2", vars);
  EXPECT_NEAR(expr.eval(), 14.0, 1e-9);
}

class VerboseMatchTest : public ::testing::Test
{
 public:
  static void SetUpTestSuite()
  {
#ifdef _WIN32
    _putenv_s("MRPT_EXPR_VERBOSE", "myvar");
#else
    ::setenv("MRPT_EXPR_VERBOSE", "myvar", 1);
#endif
  }
  static void TearDownTestSuite()
  {
#ifdef _WIN32
    _putenv_s("MRPT_EXPR_VERBOSE", "");
#else
    ::unsetenv("MRPT_EXPR_VERBOSE");
#endif
  }
};

TEST_F(VerboseMatchTest, VerboseMatchPath)
{
  // Matching expression — should print verbose info but not crash.
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;
  vars["myvar"] = 3.0;
  expr.compile("myvar+1", vars);
  EXPECT_NEAR(expr.eval(), 4.0, 1e-9);

  // Non-matching expression — ExprVerbose::process returns early.
  mrpt::expr::CRuntimeCompiledExpression expr2;
  std::map<std::string, double> vars2;
  vars2["z"] = 1.0;
  expr2.compile("z", vars2);
  EXPECT_NEAR(expr2.eval(), 1.0, 1e-9);
}

// =========================================================================
//  Regular tests (ExprVerbose singleton already constructed above)
// =========================================================================

TEST(RuntimeCompiledExpression, SimpleTest)
{
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;

  EXPECT_FALSE(expr.is_compiled());

  vars["x"] = 5.0;
  vars["y"] = 3.0;
  expr.compile("x^2+x*y+1", vars);

  EXPECT_TRUE(expr.is_compiled());

  EXPECT_NEAR(expr.eval(), vars["x"] * vars["x"] + vars["x"] * vars["y"] + 1.0, 1e-9);
}

namespace
{
double myNullary() { return 10.0; }

double myNeg(double x) { return -x; }

double myAdd(double x, double y) { return x + y; }

double myDiff(double x, double y, double z) { return (y - x) / z; }
}  // namespace

TEST(RuntimeCompiledExpression, CustomFunctions)
{
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;

  expr.register_function("myNullary", myNullary);
  expr.register_function("myNeg", myNeg);
  expr.register_function("myAdd", myAdd);
  expr.register_function("myDiff", myDiff);

  vars["x"] = 5.0;
  vars["y"] = 3.0;
  expr.compile("1+myAdd(x,y) + myNullary() + myNeg(x) + myDiff(x,y,x)", vars);

  EXPECT_NEAR(
      expr.eval(),
      1 + vars["x"] + vars["y"] + 10.0 - vars["x"] + (vars["y"] - vars["x"]) / vars["x"], 1e-9);
}

// --- New tests to increase coverage ---

TEST(RuntimeCompiledExpression, GetOriginalExpression)
{
  mrpt::expr::CRuntimeCompiledExpression expr;
  EXPECT_TRUE(expr.get_original_expression().empty());

  std::map<std::string, double> vars;
  vars["x"] = 1.0;
  expr.compile("x+1", vars);
  EXPECT_EQ(expr.get_original_expression(), "x+1");
}

TEST(RuntimeCompiledExpression, GetRawExprtk)
{
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;
  vars["x"] = 4.0;
  expr.compile("x*2", vars);

  // Verify the raw exprtk accessor compiles and returns a reference.
  // We exercise both const and non-const overloads; the result is verified
  // via the wrapper eval() which calls the same underlying formula.
  [[maybe_unused]] auto& raw = expr.get_raw_exprtk_expr();
  const auto& cexpr = expr;
  [[maybe_unused]] const auto& craw = cexpr.get_raw_exprtk_expr();

  EXPECT_NEAR(expr.eval(), 8.0, 1e-9);
}

TEST(RuntimeCompiledExpression, RegisterSymbolTablePointers)
{
  mrpt::expr::CRuntimeCompiledExpression expr;

  double a = 3.0, b = 4.0;
  std::map<std::string, double*> ptrs;
  ptrs["a"] = &a;
  ptrs["b"] = &b;
  expr.register_symbol_table(ptrs);

  // Compile with no extra vars so only the pointer-registered symbol table
  // is active; the expression holds references to a and b directly.
  expr.compile("a+b", {});
  EXPECT_NEAR(expr.eval(), 7.0, 1e-9);

  // Verify pointer semantics: mutating the originals is reflected in eval().
  a = 10.0;
  b = 20.0;
  EXPECT_NEAR(expr.eval(), 30.0, 1e-9);
}

TEST(RuntimeCompiledExpression, MathConstants)
{
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;
  expr.compile("M_PI", vars);
  EXPECT_NEAR(expr.eval(), M_PI, 1e-9);
}

TEST(RuntimeCompiledExpression, InvalidExpressionThrows)
{
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;
  EXPECT_THROW(expr.compile("x ??? 1", vars), std::exception);
  // After a failed compile, is_compiled() must remain false
  EXPECT_FALSE(expr.is_compiled());
}

TEST(RuntimeCompiledExpression, Recompile)
{
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;
  vars["x"] = 2.0;

  expr.compile("x+1", vars);
  EXPECT_NEAR(expr.eval(), 3.0, 1e-9);

  // Recompile with a different expression on the same object
  vars["x"] = 5.0;
  expr.compile("x*x", vars);
  EXPECT_NEAR(expr.eval(), 25.0, 1e-9);
  EXPECT_EQ(expr.get_original_expression(), "x*x");
}

TEST(RuntimeCompiledExpression, ExprNameInErrorMessage)
{
  mrpt::expr::CRuntimeCompiledExpression expr;
  std::map<std::string, double> vars;
  try
  {
    expr.compile("bad???", vars, "my_formula");
    FAIL() << "Expected exception not thrown";
  }
  catch (const std::exception& e)
  {
    const std::string msg(e.what());
    EXPECT_NE(msg.find("my_formula"), std::string::npos);
  }
}
