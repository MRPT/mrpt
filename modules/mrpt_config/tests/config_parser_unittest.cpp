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

#include <gtest/gtest.h>
#include <mrpt/config/config_parser.h>

using mrpt::config::config_parser;

TEST(config_parser, backslashCRLFContinuation)
{
  // backslash + CR + LF must be swallowed as a single line-continuation:
  const std::string in = "a\\\r\nb";
  EXPECT_EQ(config_parser(in), "ab");
}

TEST(config_parser, backslashLFContinuation)
{
  const std::string in = "a\\\nb";
  EXPECT_EQ(config_parser(in), "ab");
}

TEST(config_parser, envVarSubstitution_missingClosingBrace_Throws)
{
  EXPECT_THROW(config_parser("$env{FOO"), std::runtime_error);
}

TEST(config_parser, evalExpr_missingClosingBrace_Throws)
{
  EXPECT_THROW(config_parser("$eval{1+2"), std::runtime_error);
}

TEST(config_parser, varRef_missingClosingBrace_Throws)
{
  EXPECT_THROW(config_parser("${incomplete"), std::runtime_error);
}

TEST(config_parser, varRef_unknownVariable_Throws)
{
  EXPECT_THROW(config_parser("${never_defined}"), std::runtime_error);
}

TEST(config_parser, varRef_knownVariable_Substitutes)
{
  const std::string in =
      "@define MAXSPEED 10\n"
      "value=${MAXSPEED}\n";
  const std::string out = config_parser(in);
  EXPECT_NE(out.find("10"), std::string::npos);
}

TEST(config_parser, evalExprWithinVarEval_missingClosingBrace_Throws)
{
  // "@define" values are themselves run through parse_process_var_eval();
  // an unterminated $eval{...} nested in the definition must throw too.
  EXPECT_THROW(config_parser("@define BAD $eval{1+2\n"), std::runtime_error);
}

TEST(config_parser, envVarWithinVarEval_missingClosingBrace_Throws)
{
  EXPECT_THROW(config_parser("@define BAD $env{HOME\n"), std::runtime_error);
}
