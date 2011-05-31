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
