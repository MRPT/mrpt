/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArCondition.h"
#include "ArLog.h"


ArStrMap ArCondition::ourStrMap;


AREXPORT ArCondition::ArCondition() :
  myFailedInit(false),
  myCond(),
  myCount(0)
{
  myCond=CreateEvent(0, FALSE, FALSE, 0);
  if (myCond == NULL)
  {
    ArLog::log(ArLog::Terse, "ArCondition::ArCondition: Unknown error trying to create the condition.");
    myFailedInit=true;
  }

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
  if (!myFailedInit && !CloseHandle(myCond))
    ArLog::log(ArLog::Terse, "ArCondition::~ArCondition: Unknown error while trying to destroy the condition.");
}

AREXPORT int ArCondition::signal()
{
  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::signal: Initialization of condition failed, failed to signal");
    return(STATUS_FAILED_INIT);
  }

  if (!PulseEvent(myCond))
  {
    ArLog::log(ArLog::Terse, "ArCondition::signal: Unknown error while trying to signal the condition.");
    return(STATUS_FAILED);
  }

  return(0);
}

AREXPORT int ArCondition::broadcast()
{
  int ret=0;

  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::broadcast: Initialization of condition failed, failed to broadcast");
    return(STATUS_FAILED_INIT);
  }

  for (; myCount != 0; --myCount)
  {
    if (PulseEvent(myCond) != 0)
    {
      ArLog::log(ArLog::Terse, "ArCondition::broadcast: Unknown error while trying to broadcast the condition.");
      ret=STATUS_FAILED;
    }
  }

  return(ret);
}

AREXPORT int ArCondition::wait()
{
  DWORD ret;

  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::wait: Initialization of condition failed, failed to wait");
    return(STATUS_FAILED_INIT);
  }

  ++myCount;
  ret=WaitForSingleObject(myCond, INFINITE);
  if (ret == WAIT_OBJECT_0)
    return(0);
  else
  {
    ArLog::logNoLock(ArLog::Terse, "ArCondition::wait: Failed to lock due to an unknown error");
    return(STATUS_FAILED);
  }
}

AREXPORT int ArCondition::timedWait(unsigned int msecs)
{
  int ret;

  if (myFailedInit)
  {
    ArLog::log(ArLog::Terse, "ArCondition::wait: Initialization of condition failed, failed to wait");
    return(STATUS_FAILED_INIT);
  }

  ++myCount;
  ret=WaitForSingleObject(myCond, msecs);
  if (ret == WAIT_OBJECT_0)
    return(0);
  else if (ret == WAIT_TIMEOUT)
    return(STATUS_WAIT_TIMEDOUT);
  else
  {
    ArLog::logNoLock(ArLog::Terse, "ArCondition::timedWait: Failed to lock due to an unknown error");
    return(STATUS_FAILED);
  }
}

AREXPORT const char *ArCondition::getError(int messageNumber) const
{
  ArStrMap::const_iterator it;
  if ((it = ourStrMap.find(messageNumber)) != ourStrMap.end())
    return (*it).second.c_str();
  else
    return NULL;
}
