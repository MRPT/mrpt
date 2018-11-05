/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/** \example core_exceptions_example/test.cpp */

//! [example-nested-exceptions]

#include <mrpt/core/exceptions.h>
#include <iostream>

void test_except_3rd_lvl()
{
	MRPT_START
	THROW_EXCEPTION("Aw!");
	MRPT_END
}

void test_except_2nd_lvl()
{
	MRPT_START
	test_except_3rd_lvl();
	MRPT_END
}

void test_except_toplevel()
{
	MRPT_START
	test_except_2nd_lvl();
	MRPT_END
}

int main()
{
	try
	{
		test_except_toplevel();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		return -1;
	}
}
//! [example-nested-exceptions]
