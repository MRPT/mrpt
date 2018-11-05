/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/system/TParameters.h>
#include <iostream>

using namespace std;
using namespace mrpt::system;

void MyCoolFunction(const TParametersDouble& params)
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
		TParametersDouble p;
		p["threshold"] = 3.05;
		p["altitude"] = 100;

		MyCoolFunction(p);
	}

	{
		// Call #2
		cout << "CALL #2 ================================\n";
		// clang-format off
		TParametersDouble p(
			{
				{"threshold", 3.05},
				{"altitude", 100.0},
				{"level", -1.0}
			}
		);
		// clang-format on

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
	}
	catch (const std::exception& e)
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
