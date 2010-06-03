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

#include "ariaOSDef.h"
#include "ArCommands.h"
#include "ArExport.h"
#include "ArSonarAutoDisabler.h"
#include "ArRobot.h"

AREXPORT ArSonarAutoDisabler::ArSonarAutoDisabler(ArRobot *robot) :
  myUserTaskCB(this, &ArSonarAutoDisabler::userTask)
{
  myRobot = robot;
  myUserTaskCB.setName("SonarAutoDisabler");
  myRobot->addUserTask("SonarAutoDisabler", -50, &myUserTaskCB);
  myLastMoved.setToNow();
  mySonarEnabled = true;
}

AREXPORT ArSonarAutoDisabler::~ArSonarAutoDisabler()
{
  myRobot->remUserTask("SonarAutoDisabler");
}

AREXPORT void ArSonarAutoDisabler::userTask(void)
{
  // see if we moved
  if (myRobot->isTryingToMove() || fabs(myRobot->getVel()) > 3 || 
      fabs(myRobot->getRotVel()) > 3)
  {
    myLastMoved.setToNow();
    // if our sonar are disabled and we moved and our motors are
    // enabled then turn 'em on
    if (!mySonarEnabled && myRobot->areMotorsEnabled())
    {
      mySonarEnabled = true;
      myRobot->enableSonar();
    }
  }
  else
  {
    // if the sonar are on and we haven't moved in a while then turn
    // 'em off
    if (mySonarEnabled && myLastMoved.secSince() > 10)
    {
      mySonarEnabled = false;
      myRobot->disableSonar();
    }
  }
}
