/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/

#include "ArExport.h"
// ArThread.cc -- Thread classes


#include "ariaOSDef.h"
#include <list>
#include "ArThread.h"
#include "ArLog.h"
#include "ArSignalHandler.h"

#if defined(_MSC_VER)
	// For return((DWORD)ret);
	#pragma warning(disable:4311)
#endif

static DWORD WINAPI run(void *arg)
{
  ArThread *t=(ArThread*)arg;
  void *ret=NULL;

  if (t->getBlockAllSignals())
    ArSignalHandler::blockCommonThisThread();

  if (dynamic_cast<ArRetFunctor<void*>*>(t->getFunc()))
    ret=((ArRetFunctor<void*>*)t->getFunc())->invokeR();
  else
    t->getFunc()->invoke();

  return((DWORD)ret);
}

void ArThread::init()
{
  ArThread *main;
  ThreadType pt;
  MapType::iterator iter;

  pt=GetCurrentThread();

  ourThreadsMutex.lock();
  if (ourThreads.size())
  {
    ourThreadsMutex.unlock();
    return;
  }
  main=new ArThread;
  main->myJoinable=true;
  main->myRunning=true;
  main->myThread=pt;
  ourThreads.insert(MapType::value_type(pt, main));
  ourThreadsMutex.unlock();
}


AREXPORT ArThread * ArThread::self()
{
  ThreadType pt;
  MapType::iterator iter;

  ourThreadsMutex.lock();
  pt=GetCurrentThread();
  iter=ourThreads.find(pt);
  ourThreadsMutex.unlock();

  if (iter != ourThreads.end())
    return((*iter).second);
  else
    return(NULL);
}

AREXPORT void ArThread::cancelAll()
{
  DWORD ret=0;
  MapType::iterator iter;

  ourThreadsMutex.lock();
  for (iter=ourThreads.begin(); iter != ourThreads.end(); ++iter)
    TerminateThread((*iter).first, ret);
  ourThreads.clear();
  ourThreadsMutex.unlock();
}

AREXPORT int ArThread::create(ArFunctor *func, bool joinable,
			      bool lowerPriority)
{
  DWORD ret=0, err;

  myJoinable=joinable;
  myFunc=func;
  myRunning=true;

  myThread=CreateThread(0, 0, &run, this, 0, &ret);
  err=GetLastError();
  if (myThread == 0)
  {
    ArLog::log(ArLog::Terse, "ArThread::create: Failed to create thread.");
    return(STATUS_FAILED);
  }
  else
  {
    if (myName.size() == 0)
      ArLog::log(ourLogLevel, "Created anonymous thread with ID %d", 
		 myThread);
    else
      ArLog::log(ourLogLevel, "Created %s thread with ID %d", myName.c_str(),
		 myThread);
    ourThreadsMutex.lock();
    ourThreads.insert(MapType::value_type(myThread, this));
    ourThreadsMutex.unlock();
    if (lowerPriority)
      SetThreadPriority(myThread, THREAD_PRIORITY_IDLE);
    return(0);
  }
}

AREXPORT int ArThread::doJoin(void **iret)
{
  DWORD ret;

  ret=WaitForSingleObject(myThread, INFINITE);
  if (ret == WAIT_FAILED)
  {
    ArLog::log(ArLog::Terse, "ArThread::doJoin: Failed to join on thread.");
    return(STATUS_FAILED);
  }

  return(0);
}

AREXPORT int ArThread::detach()
{
  return(0);
}

AREXPORT void ArThread::cancel()
{
  DWORD ret=0;

  ourThreadsMutex.lock();
  ourThreads.erase(myThread);
  ourThreadsMutex.unlock();
  TerminateThread(myThread, ret);
}

AREXPORT void ArThread::yieldProcessor()
{
  Sleep(0);
}


AREXPORT void ArThread::threadStarted(void)
{
  if (myName.size() == 0)
    ArLog::log(ourLogLevel, "Anonymous thread (%d) is running",
	       myThread);
  else
    ArLog::log(ourLogLevel, "Thread %s (%d) is running", 
	       myName.c_str(), myThread);
}

AREXPORT void ArThread::logThreadInfo(void)
{
  if (myName.size() == 0)
    ArLog::log(ourLogLevel, "Anonymous thread (%d) is running",
	       myThread);
  else
    ArLog::log(ourLogLevel, "Thread %s (%d) is running", 
	       myName.c_str(), myThread);
}
