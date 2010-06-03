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
#include "ariaOSDef.h"
#include "ArMutex.h"
#include "ArLog.h"


ArMutex::ArMutex() :
  myFailedInit(false),
  myMutex()
{
  myLog = false;
  myLogName = "";
  myMutex=CreateMutex(0, true, 0);
  if (!myMutex)
  {
    myFailedInit=true;
    ArLog::logNoLock(ArLog::Terse, "ArMutex::ArMutex: Failed to initialize mutex %s", myLogName.c_str());
  }
  else
    unlock();

  myStrMap[STATUS_FAILED_INIT]="Failed to initialize";
  myStrMap[STATUS_FAILED]="General failure";
  myStrMap[STATUS_ALREADY_LOCKED]="Mutex already locked";
}

ArMutex::~ArMutex()
{
  if (!myFailedInit && !CloseHandle(myMutex))
    ArLog::logNoLock(ArLog::Terse, "ArMutex::~ArMutex: Failed to destroy mutex.");
}

int ArMutex::lock()
{
  DWORD ret;

  if (myLog)
    ArLog::log(ArLog::Terse, "Locking %s", myLogName.c_str());
  if (myFailedInit)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Initialization of mutex %s failed, failed lock", myLogName.c_str());
    return(STATUS_FAILED_INIT);
  }

  ret=WaitForSingleObject(myMutex, INFINITE);
  if (ret == WAIT_ABANDONED)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Tried to lock a mutex %s which was locked by a different thread and never unlocked before that thread exited. This is a recoverable error", myLogName.c_str());
    return(lock());
  }
  else if (ret == WAIT_OBJECT_0)
    return(0);
  else
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Failed to lock %s due to an unknown error", myLogName.c_str());
    return(STATUS_FAILED);
  }

  return(0);
}

int ArMutex::tryLock()
{
  DWORD ret;

  if (myFailedInit)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Initialization of mutex %s failed, failed lock", myLogName.c_str());
    return(STATUS_FAILED_INIT);
  }

  // Attempt to wait as little as posesible
  ret=WaitForSingleObject(myMutex, 1);
  if (ret == WAIT_ABANDONED)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Tried to lock mutex %s nwhich was locked by a different thread and never unlocked before that thread exited. This is a recoverable error", myLogName.c_str());
    return(lock());
  }
  else if (ret == WAIT_TIMEOUT)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::tryLock: Could not lock mutex %s because it is already locked", myLogName.c_str());
    return(STATUS_ALREADY_LOCKED);
  }
  else if (ret == WAIT_OBJECT_0)
    return(0);
  else
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::lock: Failed to lock %s due to an unknown error", myLogName.c_str());
    return(STATUS_FAILED);
  }

  return(0);
}

int ArMutex::unlock()
{
  if (myLog)
    ArLog::log(ArLog::Terse, "Unlocking %s", myLogName.c_str());
  if (myFailedInit)
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::unlock: Initialization of mutex %s failed, failed unlock", myLogName.c_str());
    return(STATUS_FAILED_INIT);
  }

  if (!ReleaseMutex(myMutex))
  {
    ArLog::logNoLock(ArLog::Terse, "ArMutex::unlock: Failed to unlock %s due to an unknown error", myLogName.c_str());
    return(STATUS_FAILED);
  }

  return(0);
}

AREXPORT const char * ArMutex::getError(int messageNumber) const
{
  ArStrMap::const_iterator it;
  if ((it = myStrMap.find(messageNumber)) != myStrMap.end())
    return (*it).second.c_str();
  else
    return NULL;

}
