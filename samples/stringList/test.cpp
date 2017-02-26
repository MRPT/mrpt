/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
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
