/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include <errno.h>
#include "ariaOSDef.h"
#include "ArMutex.h"
#include "ArLog.h"
#include "ArThread.h"

#include <sys/types.h>
#include <unistd.h>     // for getpid()

ArMutex::ArMutex() :
  myFailedInit(false),
  myMutex()
{
  myLog = false;
  if (pthread_mutex_init(&myMutex, 0) < 0)
  {
    myFailedInit=true;
    ArLog::logNoLock(ArLog::Terse, "ArMutex::ArMutex: Failed to initialize mutex");
  }
  else
    unlock();

  myStrMap[STATUS_FAILED_INIT]="Failed to initialize";
  myStrMap[STATUS_FAILED]="General failure";
  myStrMap[STATUS_ALREADY_LOCKED]="Mutex already locked";
}

ArMutex::~ArMutex()
{
  if (!myFailedInit && (pthread_mutex_destroy(&myMutex) < 0))
  {
    if (errno == EBUSY)
      ArLog::logNoLock(ArLog::Terse, "ArMutex::~ArMutex: Failed to destroy mutex. A thread is currently blocked waiting for this mutex.");
    else
      ArLog::logNoLock(ArLog::Terse, "ArMutex::~ArMutex: Failed to destroy mutex. Unknown error.");
  }      
}

/**
   Lock the mutex. This function will block until no other thread has this
   mutex locked. If it returns 0, then it obtained the lock and the thread
   is free to use the critical section that this mutex protects. Else it
   returns an error code. See getError().
*/
int ArMutex::lock() 
{
  if (myLog && ArThread::self() != NULL)
    ArLog::log(ArLog::Terse, "Locking %s from thread %s %d pid %d", 
	       myLogName.c_str(),
	       ArThread::self()->getThreadName(), 
	       *(ArThread::self()->getThread()), getpid());
  else if (myLog)
    ArLog::log(ArLog::Terse, 
	       "Locking %s probably from pid %d", myLogName.c_str(), getpid());
  if (myFailedInit)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Initialization of mutex failed, failed lock");
    return(STATUS_FAILED_INIT);
  }

  if (pthread_mutex_lock(&myMutex) < 0)
  {
    if (errno == EDEADLK)
    {
      ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Trying to lock a mutex which is already locked by this thread");
      return(STATUS_ALREADY_LOCKED);
    }
    else
    {
      ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Failed to lock due to an unknown error");
      return(STATUS_FAILED);
    }
  }

  return(0);
}

/**
   Try to lock the mutex. This function will not block if another thread has
   the mutex locked. It will return instantly if that is the case. It will
   return STATUS_ALREADY_LOCKED if another thread has the mutex locked. If it
   obtains the lock, it will return 0.
*/
int ArMutex::tryLock() 
{
  if (myFailedInit)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::tryLock: Initialization of mutex failed, failed trylock");
    return(STATUS_FAILED_INIT);
  }

  if (pthread_mutex_trylock(&myMutex) < 0)
  {
    if (errno == EBUSY)
    {
      ArLog::logNoLock(ArLog::Terse, "ArMutex::tryLock: Could not lock mutex because it is already locked");
      return(STATUS_ALREADY_LOCKED);
    }
    else
    {
      ArLog::logNoLock(ArLog::Terse, "ArMutex::trylock: Failed to trylock due to an unknown error");
      return(STATUS_FAILED);
    }
  }

  return(0);
}

int ArMutex::unlock() 
{
  if (myLog && ArThread::self() != NULL)
    ArLog::log(ArLog::Terse, "Unlocking %s from thread %s %d pid %d", 
	       myLogName.c_str(),
	       ArThread::self()->getThreadName(), 
	       *(ArThread::self()->getThread()), getpid());
  else if (myLog)
    ArLog::log(ArLog::Terse, 
	       "Unlocking %s probably from pid %d", myLogName.c_str(), getpid());
  if (myFailedInit)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::unlock: Initialization of mutex failed, failed unlock");
    return(STATUS_FAILED_INIT);
  }

  if (pthread_mutex_unlock(&myMutex) < 0)
  {
    if (errno == EPERM)
    {
      ArLog::logNoLock(ArLog::Terse, "ArMutex::unlock: Trying to unlock a mutex which this thread does not own");
      return(STATUS_ALREADY_LOCKED);
    }
    else
    {
      ArLog::logNoLock(ArLog::Terse, "ArMutex::unlock: Failed to unlock due to an unknown error");
      return(STATUS_FAILED);
    }
  }

  return(0);
}

AREXPORT const char *ArMutex::getError(int messageNumber) const
{
  ArStrMap::const_iterator it;
  if ((it = myStrMap.find(messageNumber)) != myStrMap.end())
    return (*it).second.c_str();
  else
    return NULL;
}
