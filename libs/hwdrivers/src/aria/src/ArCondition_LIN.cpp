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
#include <time.h>
#include <math.h>
#include "ariaOSDef.h"
#include "ArCondition.h"
#include "ArLog.h"
#include <sys/time.h>

ArStrMap ArCondition::ourStrMap;


AREXPORT ArCondition::ArCondition() :
  myFailedInit(false),
  myCond(),
  myMutex()
{
  pthread_condattr_t attr;

  pthread_condattr_init(&attr);
  if (pthread_cond_init(&myCond, &attr) != 0)
  {
    ArLog::log(ArLog::Terse, "ArCondition::ArCondition: Unknown error trying to create the condition.");
    myFailedInit=true;
  }

  pthread_condattr_destroy(&attr);

  ourStrMap[STATUS_FAILED]="General failure";
  ourStrMap[STATUS_FAILED_DESTROY]=
  "Another thread is waiting on this condition so it can not be destroyed";
  ourStrMap[STATUS_FAILED_INIT] =
  "Failed to initialize thread. Requested action is imposesible";
  ourStrMap[STATUS_MUTEX_FAILED_INIT]="The underlying mutex failed to init";
  ourStrMap[STATUS_MUTEX_FAILED]="The underlying mutex failed in some fashion";
}

AREXPORT ArCondition::~ArCondition()
{
  int ret;

  ret=pthread_cond_destroy(&myCond);
  if (ret == EBUSY)
    ArLog::log(ArLog::Terse, "ArCondition::~ArCondition: Trying to destroy a condition that another thread is waiting on.");
  else if (ret != 0)
    ArLog::log(ArLog::Terse, "ArCondition::~ArCondition: Unknown error while trying to destroy the condition.");
}

AREXPORT int ArCondition::signal()
{
  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::signal: Initialization of condition failed, failed to signal");
    return(STATUS_FAILED_INIT);
  }

  if (pthread_cond_signal(&myCond) != 0)
  {
    ArLog::log(ArLog::Terse, "ArCondition::signal: Unknown error while trying to signal the condition.");
    return(STATUS_FAILED);
  }

  return(0);
}

AREXPORT int ArCondition::broadcast()
{
  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::broadcast: Initialization of condition failed, failed to broadcast");
    return(STATUS_FAILED_INIT);
  }

  if (pthread_cond_broadcast(&myCond) != 0)
  {
    ArLog::log(ArLog::Terse, "ArCondition::broadcast: Unknown error while trying to broadcast the condition.");
    return(STATUS_FAILED);
  }

  return(0);
}

AREXPORT int ArCondition::wait()
{
  int ret;

  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::wait: Initialization of condition failed, failed to wait");
    return(STATUS_FAILED_INIT);
  }

  ret=myMutex.lock();
  if (ret != 0)
  {
    if (ret == ArMutex::STATUS_FAILED_INIT)
      return(STATUS_MUTEX_FAILED_INIT);
    else
      return(STATUS_MUTEX_FAILED);
  }

  ret=pthread_cond_wait(&myCond, &myMutex.getMutex());
  if (ret != 0)
  {
    if (ret == EINTR)
      return(STATUS_WAIT_INTR);
    else
    {
      ArLog::log(ArLog::Terse, "ArCondition::wait: Unknown error while trying to wait on the condition.");
      return(STATUS_FAILED);
    }
  }

  ret=myMutex.unlock();
  if (ret != 0)
  {
    if (ret == ArMutex::STATUS_FAILED_INIT)
      return(STATUS_MUTEX_FAILED_INIT);
    else
      return(STATUS_MUTEX_FAILED);
  }

  return(0);
}

AREXPORT int ArCondition::timedWait(unsigned int msecs)
{
  int ret;
  int retUnlock;
  struct timespec spec;
  struct timeval  tp;

  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::wait: Initialization of condition failed, failed to wait");
    return(STATUS_FAILED_INIT);
  }

  ret=myMutex.lock();
  if (ret != 0)
  {
    if (ret == ArMutex::STATUS_FAILED_INIT)
      return(STATUS_MUTEX_FAILED_INIT);
    else
      return(STATUS_MUTEX_FAILED);
  }

  gettimeofday(&tp, NULL);
  // convert time of day to pthread time structure
  spec.tv_sec = tp.tv_sec;
  spec.tv_nsec = tp.tv_usec * 1000;

  // add on time specified by msecs
  spec.tv_sec += (long int)rint(((float)msecs)/1000.0);
  // 1 millisecond = 1000 micro seconds = 1000000 nanoseconds
  spec.tv_nsec += (long int)( ( msecs % 1000 ) * 1000000);
//   printf("input millisecond=%d :: sec=%ld nsec=%ld curtime=%ld %ld\n", msecs, spec.tv_sec, spec.tv_nsec, tp.tv_sec, tp.tv_usec * 1000);

  ret=pthread_cond_timedwait(&myCond, &myMutex.getMutex(), &spec);

  // must unlock the mutex, even if we fail, since we reacquire lock
  // after timedwait times out
  retUnlock=myMutex.unlock();
  
  if (ret != 0)
  {
    if (ret == EINTR)
      return(STATUS_WAIT_INTR);
    else if (ret == ETIMEDOUT)
      return(STATUS_WAIT_TIMEDOUT);
    else
    {
      ArLog::log(ArLog::Terse, "ArCondition::timedWait: Unknown error while trying to wait on the condition.");
      return(STATUS_FAILED);
    }
  }

  if (retUnlock != 0)
  {
    if (retUnlock == ArMutex::STATUS_FAILED_INIT)
      return(STATUS_MUTEX_FAILED_INIT);
    else
      return(STATUS_MUTEX_FAILED);
  }

  return(0);
}

AREXPORT const char * ArCondition::getError(int messageNumber) const
{
  ArStrMap::const_iterator it;
  if ((it = ourStrMap.find(messageNumber)) != ourStrMap.end())
    return (*it).second.c_str();
  else
    return NULL;
}
