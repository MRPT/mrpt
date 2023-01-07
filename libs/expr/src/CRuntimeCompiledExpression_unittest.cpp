/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>

template class mrpt::CTraitsTest<mrpt::expr::CRuntimeCompiledExpression>;

TEST(RuntimeCompiledExpression, SimpleTest)
{
	mrpt::expr::CRuntimeCompiledExpression expr;
	std::map<std::string, double> vars;

	vars["x"] = 5.0;
	vars["y"] = 3.0;
	expr.compile("x^2+x*y+1", vars);

	EXPECT_NEAR(
		expr.eval(), vars["x"] * vars["x"] + vars["x"] * vars["y"] + 1.0, 1e-9);
}

static double myNullary() { return 10.0; }

static double myNeg(double x) { return -x; }

static double myAdd(double x, double y) { return x + y; }

static double myDiff(double x, double y, double z) { return (y - x) / z; }

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
		1 + vars["x"] + vars["y"] + 10.0 - vars["x"] +
			(vars["y"] - vars["x"]) / vars["x"],
		1e-9);
}
