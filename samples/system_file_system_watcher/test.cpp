/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/system/CFileSystemWatcher.h>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

// ------------------------------------------------------
//				TestWatch
// ------------------------------------------------------
void TestWatch()
{
	CFileSystemWatcher::TFileSystemChangeList lstChanges;
	CFileSystemWatcher::TFileSystemChangeList::iterator it;

	CFileSystemWatcher watch(".");

	printf("Watching directory '.'...\n Press any key to exit.\n");

	while (!mrpt::system::os::kbhit())
	{
		watch.getChanges(lstChanges);

		for (it = lstChanges.begin(); it != lstChanges.end(); it++)
		{
			cout << "changed: '" << it->path << "' ";
			if (it->isDir) cout << "isDir ";
			if (it->eventModified) cout << "modified ";
			if (it->eventCloseWrite) cout << "close_write ";
			if (it->eventDeleted) cout << "deleted ";
			if (it->eventMovedTo) cout << "moved_to ";
			if (it->eventMovedFrom) cout << "moved_from ";
			if (it->eventCreated) cout << "created ";
			if (it->eventAccessed) cout << "accessed";

			cout << endl;
		}

		std::this_thread::sleep_for(100ms);
	}
}

int main()
{
	try
	{
		TestWatch();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
