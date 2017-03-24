/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
// Threadable.cpp -- Threadable interface class
#ifndef WIN32
#include <pthread.h>
#endif
#include "ariaOSDef.h"
#include "ArASyncTask.h"
#include "ArLog.h"


AREXPORT ArASyncTask::ArASyncTask() :
  myFunc(this, &ArASyncTask::runThread, NULL)
{
}

AREXPORT ArASyncTask::~ArASyncTask()
{
}

AREXPORT int ArASyncTask::create(bool joinable, bool lowerPriority)
{
  return(ArThread::create(&myFunc, joinable, lowerPriority));
}

/**
   This will run the code of the ArASyncTask without creating a new
   thread to run it in. It performs the needed setup then calls runThread().
   This is good if you have a task which you wish to run multiple
   instances of and you want to use the main() thread  instead of having
   it block, waiting for exit of the program.
   @param arg the argument to pass to the runThread()
*/
AREXPORT void * ArASyncTask::runInThisThread(void *arg)
{
  myJoinable=true;
  myRunning=true;
#ifdef WIN32
  myThread=GetCurrentThread();
#else
  myThread=pthread_self();
#endif
  
  if (myName.size() == 0)
    ArLog::log(ourLogLevel, "Running anonymous thread with ID %d", 
	       myThread);
  else
    ArLog::log(ourLogLevel, "Running %s thread with ID %d", myName.c_str(),
	       myThread);
  
  ourThreadsMutex.lock();
  // MPL BUGFIX, this wasn't workign for some reason (was printing
  // 0)...  so I got rid of it and did it the easier way anyhow
  //printf("!!!! %d\n", ourThreads.insert(MapType::value_type(myThread, this)).second);
  ourThreads[myThread] = this;
  ourThreadsMutex.unlock();

  return(runThread(arg));
}
