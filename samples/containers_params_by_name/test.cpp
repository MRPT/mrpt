/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <iostream>

using std::cout;
using std::endl;

void MyCoolFunction(const mrpt::containers::yaml& params)
{
	cout << "'threshold' is " << params["threshold"] << endl;
	cout << "Is 'altitude' set? " << params.has("altitude") << endl;
	cout << "Is 'level' set? " << params.has("level") << endl;
	cout << "Level is : " << params.getOrDefault("level", 666.0) << endl;
	cout << "Dump of all params:\n";
	params.printAsYAML(cout);
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
		mrpt::containers::yaml p;
		p["threshold"] = 3.05;
		p["altitude"] = 100;

		MyCoolFunction(p);
	}

	{
		// Call #2
		cout << "CALL #2 ================================\n";
		// clang-format off
		mrpt::containers::yaml p = mrpt::containers::yaml::Map(
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

int main()
{
	try
	{
		TestParameters();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
