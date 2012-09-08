/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
