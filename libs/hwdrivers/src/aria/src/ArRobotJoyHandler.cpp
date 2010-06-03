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
#include "ArRobotJoyHandler.h"
#include "ArRobot.h"
#include "ArCommands.h"

AREXPORT ArRobotJoyHandler::ArRobotJoyHandler(ArRobot *robot) : 
    myHandleJoystickPacketCB(this, &ArRobotJoyHandler::handleJoystickPacket),
    myConnectCB(this, &ArRobotJoyHandler::connectCallback)
{
  myRobot = robot;

  myHandleJoystickPacketCB.setName("ArRobotJoyHandler");
  myRobot->addConnectCB(&myConnectCB);
  myRobot->addPacketHandler(&myHandleJoystickPacketCB, ArListPos::FIRST);
  if (myRobot->isConnected())
    connectCallback();

  myStarted.setToNow();
  myButton1 = 0;
  myButton2 = 0;
  myJoyX = 0;
  myJoyY = 0;
  myThrottle = 1;
  myGotData = false;
}

AREXPORT ArRobotJoyHandler::~ArRobotJoyHandler()
{
  myRobot->remConnectCB(&myConnectCB);
  myRobot->remPacketHandler(&myHandleJoystickPacketCB);
}


AREXPORT void ArRobotJoyHandler::connectCallback(void)
{
  myRobot->comInt(ArCommands::JOYINFO, 2);
}

AREXPORT bool ArRobotJoyHandler::handleJoystickPacket(ArRobotPacket *packet)
{

  if (packet->getID() != 0xF8)
    return false;
  
  myDataReceived.setToNow();

  if (packet->bufToUByte() != 0)
    myButton1 = true;
  else
    myButton1 = false;

  if (packet->bufToUByte() != 0)
    myButton2 = true;
  else
    myButton2  = false;

  // these should vary between 1 and -1
  myJoyX = -((double)packet->bufToUByte2() - 512.0) / 512.0;
  myJoyY = ((double)packet->bufToUByte2() - 512.0) / 512.0;
  // these should vary between 1 and 0
  myThrottle = (double)packet->bufToUByte2() / 1024.0;

  /*
  ArLog::log(ArLog::Normal, 
	     "%10d.%03d ago %5.3f %5.3f %5.3f %d %d vel %4.0f %4.0f", 
	     myStarted.secSince(), myStarted.mSecSince() % 1000, 
	     myJoyX, myJoyY, myThrottle, myButton1, myButton2, 
	     myRobot->getVel(), myRobot->getRotVel());
  */
  //  printf("%d %d %g %g %g\n", myButton1, myButton2, myJoyX, myJoyY, myThrottle);
  if (!myGotData)
  {
    ArLog::log(ArLog::Verbose, "Received joystick information from the robot");
    myGotData = true;
  }
  return true;
}

AREXPORT void ArRobotJoyHandler::getDoubles(double *x, double *y, double *z)
{
  if (x != NULL)
    *x = myJoyX;
  if (y != NULL)
    *y = myJoyY;
  if (z != NULL)
    *z = myThrottle;
}
