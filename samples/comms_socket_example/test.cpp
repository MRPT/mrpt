/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#define SOCKET_TEST_VERBOSE
#include <iostream>
#include "SocketsTest_impl.cpp"

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		SocketsTest();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		std::cerr << "Untyped exception!!";
		return -1;
	}
}
