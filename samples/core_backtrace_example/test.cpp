/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */

#include <mrpt/core/backtrace.h>
#include <mrpt/core/exceptions.h>

#include <iostream>

class Foo
{
   public:
	static int func1(int a, int b)
	{
		mrpt::TCallStackBackTrace bt;
		mrpt::callStackBackTrace(bt);
		std::cout << bt.asString();
		return a + b;
	}
};

int main()
{
	try
	{
		Foo::func1(1, 2);
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
