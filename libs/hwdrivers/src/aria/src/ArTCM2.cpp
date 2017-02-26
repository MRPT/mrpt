/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ariaOSDef.h"
#include "ArExport.h"
#include "ArCommands.h"
#include "ArRobot.h"
#include "ArTCM2.h"

AREXPORT ArTCM2::ArTCM2(ArRobot *robot) :
  myPacketHandlerCB(this, &ArTCM2::packetHandler)
{
  myRobot = robot;
  myPacketHandlerCB.setName("ArTCM2");
  if (myRobot != NULL)
    myRobot->addPacketHandler(&myPacketHandlerCB);
  
}

AREXPORT ArTCM2::~ArTCM2()
{
  if (myRobot != NULL)
    myRobot->remPacketHandler(&myPacketHandlerCB);
}

AREXPORT bool ArTCM2::packetHandler(ArRobotPacket *packet)
{
  if (packet->getID() != 0xC0)
    return false;
  
  myCompass = ArMath::fixAngle(packet->bufToByte2() / 10.0);
  myPitch = ArMath::fixAngle(packet->bufToByte2() / 10.0);
  myRoll = ArMath::fixAngle(packet->bufToByte2() / 10.0);
  myXMag = packet->bufToByte2() / 100.0;  
  myYMag = packet->bufToByte2() / 100.0;
  myZMag = packet->bufToByte2() / 100.0;
  myTemperature = packet->bufToByte2() / 10.0;
  myError = packet->bufToByte2();
  myCalibrationH = packet->bufToByte();
  myCalibrationV = packet->bufToByte();
  myCalibrationM = packet->bufToByte2() / 100.0;

  if (myTimeLastPacket != time(NULL)) 
  {
      myTimeLastPacket = time(NULL);
      myPacCount = myPacCurrentCount;
      myPacCurrentCount = 0;
  }
  myPacCurrentCount++;
  return true;
}

AREXPORT int ArTCM2::getPacCount()
{
  if (myTimeLastPacket == time(NULL))
    return myPacCount;
  if (myTimeLastPacket == time(NULL) - 1)
    return myPacCurrentCount;
  return 0;
}
