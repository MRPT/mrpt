/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/CMatrix.h>
#include <mrpt/system/os.h>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;


void FunctionFoo2(int, double)
{
	MRPT_START

	// Something really stupid:
	int  *ptr = NULL;
	ptr++;
	ptr++;

	(*ptr)++;

	MRPT_END
}


void FunctionFoo( CMatrix * )
{
	MRPT_START

	CMatrix	A;

	FunctionFoo2(0,0);

	CMatrix B = A.transpose();

	MRPT_END
}

// ------------------------------------------------------
//			TestExceptions
// ------------------------------------------------------
void TestExceptions()
{
	MRPT_START

	mrpt::system::registerFatalExceptionHandlers();

	FunctionFoo(NULL);

	MRPT_END
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestExceptions();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
