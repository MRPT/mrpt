/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */

#include <mrpt/system/backtrace.h>
#include <mrpt/core/exceptions.h>
#include <iostream>

class Foo
{
   public:
	static int func1(int a, int b)
	{
		mrpt::system::TCallStackBackTrace bt;
		mrpt::system::getCallStackBackTrace(bt);
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
