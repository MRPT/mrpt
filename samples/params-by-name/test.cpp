/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;


void MyCoolFunction( const TParametersDouble &params)
{
	cout << "'threshold' is " << params["threshold"] << endl;
	cout << "Is 'altitude' set? " << params.has("altitude") << endl;
	cout << "Is 'level' set? " << params.has("level") << endl;
	cout << "Level is : " << params.getWithDefaultVal("level", 666.0) << endl;
	cout << "Dump of all params:\n" << params.getAsString() << endl;
	cout << endl;
}

// ------------------------------------------------------
//				TestParameters
// ------------------------------------------------------
void TestParameters()
{
	{
		// Call #1
		cout << "CALL #1 ================================\n";
		TParametersDouble  p;
		p["threshold"] = 3.05;
		p["altitude"] = 100;

		MyCoolFunction(p);
	}

	{
		// Call #2
		cout << "CALL #2 ================================\n";
		TParametersDouble  p(
			"threshold",	3.05,
			"altitude",		100.0,  // *VERY IMPORTANT* If you put "100" here it will be an "int" and it will crash!!! Make sure all params are doubles!!
			"level",		-1.0,
			NULL );

		MyCoolFunction(p);
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestParameters();

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
