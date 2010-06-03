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
#include "ArSyncLoop.h"
#include "ArLog.h"
#include "ariaUtil.h"
#include "ArRobot.h"


AREXPORT ArSyncLoop::ArSyncLoop() :
  ArASyncTask(),
  myStopRunIfNotConnected(false),
  myRobot(0)
{
  setThreadName("ArSyncLoop");
}

AREXPORT ArSyncLoop::~ArSyncLoop()
{
}

AREXPORT void ArSyncLoop::setRobot(ArRobot *robot)
{
  myRobot=robot;
}

AREXPORT void ArSyncLoop::stopRunIfNotConnected(bool stopRun)
{
  myStopRunIfNotConnected = stopRun;
}

AREXPORT void * ArSyncLoop::runThread(void *arg)
{
  threadStarted();

  long timeToSleep;
  ArTime loopEndTime;
  std::list<ArFunctor *> *runList;
  std::list<ArFunctor *>::iterator iter;
  ArTime lastLoop;
  bool firstLoop = true;
  bool warned = false;

  if (!myRobot)
  {
    ArLog::log(ArLog::Terse, "ArSyncLoop::runThread: Trying to run the synchronous loop without a robot.");
    return(0);
  }

  if (!myRobot->getSyncTaskRoot())
  {
    ArLog::log(ArLog::Terse, "ArSyncLoop::runThread: Can not run the synchronous loop without a task tree");
    return(0);
  }

  while (myRunning)
  {

    myRobot->lock();
    if (!firstLoop && !warned && !myRobot->getNoTimeWarningThisCycle() && 
	myRobot->getCycleWarningTime() != 0 && 
	lastLoop.mSecSince() > (signed int) myRobot->getCycleWarningTime())
    {
      ArLog::log(ArLog::Normal, 
 "Warning: ArRobot cycle took too long because the loop was waiting for lock.");
      ArLog::log(ArLog::Normal,
		 "\tThe cycle took %u ms, (%u ms normal %u ms warning)", 
		 lastLoop.mSecSince(), myRobot->getCycleTime(), 
		 myRobot->getCycleWarningTime());
    }
    myRobot->setNoTimeWarningThisCycle(false);
    firstLoop = false;
    warned = false;
    lastLoop.setToNow();

    loopEndTime.setToNow();
    loopEndTime.addMSec(myRobot->getCycleTime());
    myRobot->incCounter();
    myRobot->unlock();

    // note that all the stuff beyond here should maybe have a lock
    // but it should be okay because its just getting data
    myRobot->getSyncTaskRoot()->run();
    if (myStopRunIfNotConnected && !myRobot->isConnected())
    {
      if (myRunning)
	ArLog::log(ArLog::Normal, "Exiting robot run because of lost connection.");
      break;
    }
    timeToSleep = loopEndTime.mSecTo();
    // if the cycles chained and we're connected the packet handler will be 
    // doing the timing for us
    if (myRobot->isCycleChained() && myRobot->isConnected())
      timeToSleep = 0;

    if (!myRobot->getNoTimeWarningThisCycle() && 
	myRobot->getCycleWarningTime() != 0 && 
	lastLoop.mSecSince() > (signed int) myRobot->getCycleWarningTime())
    {
      ArLog::log(ArLog::Normal, 
	"Warning: ArRobot sync tasks too long at %u ms, (%u ms normal %u ms warning)", 
		 lastLoop.mSecSince(), myRobot->getCycleTime(), 
		 myRobot->getCycleWarningTime());
      warned = true;
    }
    

    if (timeToSleep > 0)
      ArUtil::sleep(timeToSleep);
  }   
  myRobot->lock();
  myRobot->wakeAllRunExitWaitingThreads();
  myRobot->unlock();

  myRobot->lock();
  runList=myRobot->getRunExitListCopy();
  myRobot->unlock();
  for (iter=runList->begin();
       iter != runList->end(); ++iter)
    (*iter)->invoke();
  delete runList;

  return(0);
}
