/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
