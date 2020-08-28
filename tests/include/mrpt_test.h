/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

#pragma once

#include <mrpt/core/exceptions.h>

#define MRPT_TEST(GROUP_, NAME_) \
	TEST(GROUP_, NAME_)          \
	{                            \
		try

#define MRPT_TEST_END()                         \
	catch (const std::exception& e)             \
	{                                           \
		std::cerr << mrpt::exception_to_str(e); \
		GTEST_FAIL();                           \
	}                                           \
	}
