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

template class mrpt::CTraitsTest<mrpt::expr::CRuntimeCompiledExpression>;

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
