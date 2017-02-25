/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
// ArThread.cc -- Thread classes


#include <errno.h>
#include <list>
#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#include "ariaOSDef.h"
#include "ArThread.h"
#include "ArLog.h"
#include "ArSignalHandler.h"
#include <mrpt/utils/mrpt_macros.h>


static void * run(void *arg)
{
  ArThread *t=(ArThread*)arg;
  void *ret=NULL;

  if (t->getBlockAllSignals())
    ArSignalHandler::blockCommonThisThread();

  if (dynamic_cast<ArRetFunctor<void*>*>(t->getFunc()))
    ret=((ArRetFunctor<void*>*)t->getFunc())->invokeR();
  else
    t->getFunc()->invoke();

  return(ret);
}


/**
   Initializes the internal structures which keep track of what thread is
   what. This is called by Aria::init(), so the user will not normaly need
   to call this function themselves. This funtion *must* be called from the
   main thread of the application. In otherwords, it should be called by
   main().
*/
void ArThread::init()
{
  ArThread *main;
  ThreadType pt;

  pt=pthread_self();

  ourThreadsMutex.lock();
  if (!ourThreads.empty())
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

/**
   If a newly created thread calls self() on itself too soon, this will return
   NULL. This is due to the fact that the thread is first created and started.
   Then the operating system returns the thread ID and thread that called
   create() then updates the list of threads with the new thread ID. There
   is just not much that can be done about that. The use should be aware of
   this caveat.
*/
ArThread * ArThread::self()
{
  ThreadType pt;
  MapType::iterator iter;

  ourThreadsMutex.lock();
  pt=pthread_self();
  iter=ourThreads.find(pt);
  ourThreadsMutex.unlock();

  if (iter != ourThreads.end())
    return((*iter).second);
  else
    return(NULL);
}

void ArThread::cancelAll()
{
  MapType::iterator iter;

  ourThreadsMutex.lock();
  for (iter=ourThreads.begin(); iter != ourThreads.end(); ++iter)
  {
    pthread_cancel((*iter).first);
    (*iter).second->stopRunning();
  }
  ourThreads.clear();
  ourThreadsMutex.unlock();
}

int ArThread::create(ArFunctor *func, bool joinable, bool lowerPriority)
{
  MRPT_UNUSED_PARAM(lowerPriority);
  int ret;
  pthread_attr_t attr;

  pthread_attr_init(&attr);
  if (joinable)
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  else
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  myJoinable=joinable;
  myFunc=func;
  myRunning=true;
  if (myBlockAllSignals)
  {
    ArSignalHandler::blockCommonThisThread();
  }
  if ((ret=pthread_create(&myThread, &attr, &run, this)) != 0)
  {
    pthread_attr_destroy(&attr);
    if (ret == EAGAIN)
    {
      ArLog::log(ArLog::Terse, "ArThread::create: Error in create, not enough system resources in pthread_create()");
      return(STATUS_NORESOURCE);
    }
    else
    {
      ArLog::log(ArLog::Terse, "ArThread::create: Unknown error in create.");
      return(STATUS_FAILED);
    }
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
    pthread_attr_destroy(&attr);
    return(0);
  }
}

int ArThread::doJoin(void **iret)
{
  int ret;
  if ((ret=pthread_join(myThread, iret)) != 0)
  {
    if (ret == ESRCH)
    {
      ArLog::log(ArLog::Terse, "ArThread::join: Error in join: No such thread found");
      return(STATUS_NO_SUCH_THREAD);
    }
    else if (ret == EINVAL)
    {
      ArLog::log(ArLog::Terse, "ArThread::join: Error in join: Thread is detached or another thread is waiting");
      return(STATUS_INVALID);
    }
    else if (ret == EDEADLK)
    {
      ArLog::log(ArLog::Terse, "ArThread::join: Error in join: Trying to join on self");
      return(STATUS_JOIN_SELF);
    }
  }

  return(0);
}

int ArThread::detach()
{
  int ret;

  if ((ret=pthread_detach(myThread)) != 0)
  {
    if (ret == ESRCH)
    {
      ArLog::log(ArLog::Terse, "ArThread::detach: Error in detach: No such thread found");
      return(STATUS_NO_SUCH_THREAD);
    }
    else if (ret == EINVAL)
    {
      ArLog::log(ArLog::Terse, "ArThread::detach: Error in detach: ArThread is already detached");
      return(STATUS_ALREADY_DETATCHED);
    }
  }

  myJoinable=false;
  return(0);
}

void ArThread::cancel()
{
  ourThreadsMutex.lock();
  ourThreads.erase(myThread);
  ourThreadsMutex.unlock();
  pthread_cancel(myThread);
}

void ArThread::yieldProcessor()
{
  sched_yield();
}

AREXPORT void ArThread::threadStarted(void)
{
  myPID = getpid();
  if (myName.size() == 0)
    ArLog::log(ourLogLevel, "Anonymous thread (%d) is running with pid %d",
	       myThread, myPID);
  else
    ArLog::log(ourLogLevel, "Thread %s (%d) is running with pid %d",
	       myName.c_str(), myThread, myPID);
}


AREXPORT void ArThread::logThreadInfo(void)
{
  if (myName.size() == 0)
    ArLog::log(ourLogLevel, "Anonymous thread (%d) is running with pid %d",
	       myThread, myPID);
  else
    ArLog::log(ourLogLevel, "Thread %s (%d) is running with pid %d",
	       myName.c_str(), myThread, myPID);
}
