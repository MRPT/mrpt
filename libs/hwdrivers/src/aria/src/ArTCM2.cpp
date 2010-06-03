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
