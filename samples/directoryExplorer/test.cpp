/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/system.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

// ------------------------------------------------------
//				TestDirExplorer
// ------------------------------------------------------
void TestDirExplorer()
{
	CDirectoryExplorer::TFileInfoList	lst;

	string		path( mrpt::system::getcwd() );
	printf("Exploring path: %s\n",path.c_str());

	CDirectoryExplorer::explore(
		path,
		FILE_ATTRIB_ARCHIVE | FILE_ATTRIB_DIRECTORY,
		lst );

	printf("Found %i files:\n",(unsigned int)lst.size());

	for (CDirectoryExplorer::TFileInfoList::iterator it=lst.begin();it!=lst.end();++it)
	{
		printf("name: %s\n",it->name.c_str());
		printf("wholePath: %s\n",it->wholePath.c_str());
		printf("isDir: %c\n", it->isDir ? 'Y':'N' );
		printf("size: %lu bytes\n", (unsigned long)it->fileSize );
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
	cout << "file: " << S << " -> extractFileName : " << mrpt::system::extractFileName(S) << endl;

	S = "foo.b";
	cout << "file: " << S << " -> extractFileName : " << mrpt::system::extractFileName(S) << endl;

	S = "foo.bardotbar.too";
	cout << "file: " << S << " -> extractFileName : " << mrpt::system::extractFileName(S) << endl;

	S = "foo";
	cout << "file: " << S << " -> extractFileName : " << mrpt::system::extractFileName(S) << endl;

	S = "foo.";
	cout << "file: " << S << " -> extractFileName : " << mrpt::system::extractFileName(S) << endl;
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
