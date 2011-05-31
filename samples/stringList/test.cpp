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

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("stringList/") );


// ------------------------------------------------------
//				TestStringList
// ------------------------------------------------------
void TestStringList()
{
	CStringList		strings, strings2;

	std::string				wholeStr;

	strings.loadFromFile(myDataDir+string("test.cpp"));

	std::cout << "Number of lines: " << strings.size() << "\n";
	std::cout << "Line  [2]    : " << strings(2) << "\n";
	strings(2)=" // This was line 2!!";
	std::cout << "Mod. line [2]: " << strings(2) << "\n";

	strings.getText(wholeStr);
	std::cout << "Whole text: (Press any key to show)\n";
	os::getch();
	std::cout << wholeStr;

	strings2 = strings;
	strings2.saveToFile("_test_copy.txt");


	{
		CFileStream		f("_out_strings.bin",fomWrite);
		f << strings;
	}

	std::cout << "Number of lines: " << strings2.size() << "\n";
	strings2.clear();
	std::cout << "Number of lines: " << strings2.size() << "\n";

	{
		CFileStream		f("_out_strings.bin",fomRead);
		f >> strings2;
	}
	std::cout << "Number of lines: " << strings2.size() << "\n";
	strings2.getText(wholeStr);
	std::cout << "Whole text: (Press any key to show)\n";
	os::getch();
	std::cout << wholeStr;


}

void Test2()
{
	utils::CStringList		strings;

	strings.loadFromFile(myDataDir+string("test.ini"));

	cout << "OBSTACLES_GRID_MAX_RANGE=" << strings.get_float("OBSTACLES_GRID_MAX_RANGE") << endl;
	strings.set("x","");
	cout << "Now we read a non-existing entry, which should give raise to an exception:" << endl;
	mrpt::system::pause();
	cout << strings.get_string("GPT11_Type") << endl;

}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		 TestStringList();
		 Test2();

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
