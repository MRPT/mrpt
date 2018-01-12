/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/expr/CRuntimeCompiledExpression.h>
#include <CTraitsTest.h>
#include <gtest/gtest.h>

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
