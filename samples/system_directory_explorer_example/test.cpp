/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>

#include <iostream>
#include <string>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

// ------------------------------------------------------
//				TestDirExplorer
// ------------------------------------------------------
void TestDirExplorer()
{
	CDirectoryExplorer::TFileInfoList lst;

	string path(mrpt::system::getcwd());
	printf("Exploring path: %s\n", path.c_str());

	CDirectoryExplorer::explore(
		path, FILE_ATTRIB_ARCHIVE | FILE_ATTRIB_DIRECTORY, lst);

	printf("Found %i files:\n", (unsigned int)lst.size());

	for (CDirectoryExplorer::TFileInfoList::iterator it = lst.begin();
		 it != lst.end(); ++it)
	{
		printf("name: %s\n", it->name.c_str());
		printf("wholePath: %s\n", it->wholePath.c_str());
		printf("isDir: %c\n", it->isDir ? 'Y' : 'N');
		printf("size: %lu bytes\n", (unsigned long)it->fileSize);
		printf("-----------------------\n");
	}
}

// ------------------------------------------------------
//				TestFileNames
// ------------------------------------------------------
void TestFileNames()
{
	// Test extractFileName
	string S;

	S = "foo.bar";
	cout << "file: " << S
		 << " -> extractFileName : " << mrpt::system::extractFileName(S)
		 << endl;

	S = "foo.b";
	cout << "file: " << S
		 << " -> extractFileName : " << mrpt::system::extractFileName(S)
		 << endl;

	S = "foo.bardotbar.too";
	cout << "file: " << S
		 << " -> extractFileName : " << mrpt::system::extractFileName(S)
		 << endl;

	S = "foo";
	cout << "file: " << S
		 << " -> extractFileName : " << mrpt::system::extractFileName(S)
		 << endl;

	S = "foo.";
	cout << "file: " << S
		 << " -> extractFileName : " << mrpt::system::extractFileName(S)
		 << endl;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestDirExplorer();
		TestFileNames();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
