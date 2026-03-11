/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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
      std::cout << "changed: '" << it->path << "' ";
      if (it->isDir) std::cout << "isDir ";
      if (it->eventModified) std::cout << "modified ";
      if (it->eventCloseWrite) std::cout << "close_write ";
      if (it->eventDeleted) std::cout << "deleted ";
      if (it->eventMovedTo) std::cout << "moved_to ";
      if (it->eventMovedFrom) std::cout << "moved_from ";
      if (it->eventCreated) std::cout << "created ";
      if (it->eventAccessed) std::cout << "accessed";

      std::cout << "\n";
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
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
}
