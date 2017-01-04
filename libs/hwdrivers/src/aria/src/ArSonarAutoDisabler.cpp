/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
