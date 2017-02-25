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
