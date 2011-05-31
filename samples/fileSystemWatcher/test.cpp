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
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

// ------------------------------------------------------
//				TestWatch
// ------------------------------------------------------
void TestWatch()
{
	CFileSystemWatcher::TFileSystemChangeList	lstChanges;
	CFileSystemWatcher::TFileSystemChangeList::iterator it;

	CFileSystemWatcher	watch(".");

	printf("Watching directory '.'...\n Press any key to exit.\n");


	while (!mrpt::system::os::kbhit())
	{
		watch.getChanges( lstChanges );

		for (it=lstChanges.begin();it!=lstChanges.end();it++)
		{
			cout << "changed: '" << it->path << "' ";
			if (it->isDir) 				cout << "isDir ";
			if (it->eventModified) 		cout << "modified ";
			if (it->eventCloseWrite) 	cout << "close_write ";
			if (it->eventDeleted) 		cout << "deleted ";
			if (it->eventMovedTo) 		cout << "moved_to ";
			if (it->eventMovedFrom) 	cout << "moved_from ";
			if (it->eventCreated) 		cout << "created ";
			if (it->eventAccessed) 		cout << "accessed";

			cout << endl;
		}

		mrpt::system::sleep(100);
	}

}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		 TestWatch();

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
