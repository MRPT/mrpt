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
