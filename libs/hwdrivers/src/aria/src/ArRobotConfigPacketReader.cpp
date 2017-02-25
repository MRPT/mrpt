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
#include "ArRobotConfigPacketReader.h"
#include "ArRobot.h"
#include "ArRobotPacket.h"
#include "ArCommands.h"

/**
   @param robot is the robot to connect this to

   @param onlyOneRequest if this is true then only one request for a
   packet will ever be honored (so that you can save the settings from
   one point in time)
   
   @param packetArrivedCB a functor to call when the packet comes in,
   note the robot is locked during this callback
 **/
AREXPORT ArRobotConfigPacketReader::ArRobotConfigPacketReader(
	ArRobot *robot, bool onlyOneRequest, ArFunctor *packetArrivedCB) : 
  myPacketHandlerCB(this, &ArRobotConfigPacketReader::packetHandler),
  myConnectedCB(this, &ArRobotConfigPacketReader::connected)
{
  myRobot = robot;
  myPacketHandlerCB.setName("ArRobotConfigPacketReader");
  myRobot->addPacketHandler(&myPacketHandlerCB);
  myRobot->addConnectCB(&myConnectedCB);
  myOnlyOneRequest = onlyOneRequest;
  myPacketRequested = false;
  myPacketArrived = false;
  myPacketArrivedCB = packetArrivedCB;
}

AREXPORT ArRobotConfigPacketReader::~ArRobotConfigPacketReader(void)
{
  myRobot->remPacketHandler(&myPacketHandlerCB);
  myRobot->remConnectCB(&myConnectedCB);
}

AREXPORT bool ArRobotConfigPacketReader::requestPacket(void)
{
  // make sure we haven't already gotten one
  if (myOnlyOneRequest && myPacketArrived)
    return false;

  if (myPacketRequested && myLastPacketRequest.mSecSince() < 200)
    return true;

  myPacketArrived = false;
  myPacketRequested = true;
  myLastPacketRequest.setToNow();
  myRobot->comInt(ArCommands::CONFIG, 1);
  return true;
}

AREXPORT void ArRobotConfigPacketReader::connected(void)
{
  if (myPacketRequested)
    myRobot->comInt(ArCommands::CONFIG, 1);
}

AREXPORT bool ArRobotConfigPacketReader::packetHandler(ArRobotPacket *packet)
{
  char buf[256];

  // if this isn't the right packet ignore it
  if (packet->getID() != 0x20)
    return false;
  // if we've already gotten our one request ignore it
  if (myPacketArrived)
    return false;
  // if we haven't requested a packet ignore it
  if (!myPacketRequested)
    return false;

  myPacketRequested = false;
  myPacketArrived = true;
  // read in all the data
  packet->bufToStr(buf, sizeof(buf));
  myType = buf;
  packet->bufToStr(buf, sizeof(buf));
  mySubType = buf;
  packet->bufToStr(buf, sizeof(buf));
  mySerialNumber = buf;
  packet->bufToUByte();
  myRotVelTop = packet->bufToUByte2();
  myTransVelTop = packet->bufToUByte2();
  myRotAccelTop = packet->bufToUByte2();
  myTransAccelTop = packet->bufToUByte2();
  myPwmMax = packet->bufToUByte2();
  packet->bufToStr(buf, sizeof(buf));
  myName = buf;
  mySipCycleTime = packet->bufToUByte();
  myHostBaud = packet->bufToUByte();
  myAux1Baud = packet->bufToUByte();
  myHasGripper = (bool)packet->bufToUByte2();
  myFrontSonar = (bool) packet->bufToUByte2();
  myRearSonar = (bool) packet->bufToUByte();
  myLowBattery = packet->bufToUByte2();
  myRevCount = packet->bufToUByte2();
  myWatchdog = packet->bufToUByte2();
  myNormalMPacs = (bool) packet->bufToUByte();
  myStallVal = packet->bufToUByte2();
  myStallCount = packet->bufToUByte2();
  myJoyVel = packet->bufToUByte2();
  myJoyRotVel = packet->bufToUByte2();
  myRotVelMax = packet->bufToUByte2();
  myTransVelMax = packet->bufToUByte2();
  myRotAccel = packet->bufToUByte2();
  myRotDecel = packet->bufToUByte2();
  myRotKP = packet->bufToUByte2();
  myRotKV = packet->bufToUByte2();
  myRotKI = packet->bufToUByte2();
  myTransAccel = packet->bufToUByte2();
  myTransDecel = packet->bufToUByte2();
  myTransKP = packet->bufToUByte2();
  myTransKV = packet->bufToUByte2();
  myTransKI = packet->bufToUByte2();
  myFrontBumps = packet->bufToUByte();
  myRearBumps = packet->bufToUByte();
  myHasCharger = packet->bufToUByte();
  mySonarCycle = packet->bufToUByte();
  if (packet->bufToUByte() == 2)
    myResetBaud = true;
  else
    myResetBaud = false;
  myGyroType = packet->bufToUByte();
  if (myGyroType == 1)
    myHasGyro = true;
  else
    myHasGyro = false;
  myDriftFactor = packet->bufToUByte2();
  myAux2Baud = packet->bufToUByte();
  myAux3Baud = packet->bufToUByte();
  myTicksMM = packet->bufToUByte2();
  myShutdownVoltage = packet->bufToUByte2();

  if (myPacketArrivedCB != NULL)
  {
    myPacketArrivedCB->invoke();
  }
  return true;
}
  
AREXPORT void ArRobotConfigPacketReader::log(void) const
{
  std::string str;
  str = buildString();
  ArLog::log(ArLog::Terse, str.c_str());
}

/**
   Like most memory stuff this won't work across DLL's in windows,
   it should work fine in linux or with static library files in windows.
**/
AREXPORT std::string ArRobotConfigPacketReader::buildString(void) const
{
  std::string ret;

  char line[32000];
  sprintf(line, "General information:\n");
  ret += line;
  sprintf(line, "Robot is type '%s' subtype '%s'\n",
	     getType(), getSubType());
  ret += line;
  sprintf(line, "serial number '%s' name '%s'\n", 
	     getSerialNumber(), getName());
  ret += line;
  sprintf(line, "Intrinsic properties and unsettable maxes:\n");
  ret += line;
  sprintf(line, "RotVelTop %d RotAccelTop %d\n", 
	     getRotVelTop(), getRotAccelTop());
  ret += line;
  sprintf(line, "TransVelTop %d TransAccelTop %d\n", 
	     getTransVelTop(), getTransAccelTop());
  ret += line;
  sprintf(line, "PWMMax %d ResetBaud %s\n", getPwmMax(),
	     ArUtil::convertBool(getResetBaud()));
  ret += line;
  sprintf(line, "Current values:\n");
  ret += line;
  sprintf(line, "RotVelMax %d RotAccel %d RotDecel %d\n", 
	     getRotVelMax(), getRotAccel(), getRotDecel());
  ret += line;
  sprintf(line, "TransVelMax %d TransAccel %d TransDecel %d\n", 
	     getTransVelMax(), getTransAccel(), getTransDecel());
  ret += line;  
  sprintf(line, "Accessories:\n");
  ret += line;  
  sprintf(line, 
	     "Gripper %s FrontSonar %s RearSonar %s Charger %d Gyro %s\n", 
	     ArUtil::convertBool(getHasGripper()), 
	     ArUtil::convertBool(getFrontSonar()), 
	     ArUtil::convertBool(getRearSonar()), 
	     getHasCharger(),
	     ArUtil::convertBool(getHasGyro()));
  ret += line;  
  sprintf(line, "FrontBumps %d RearBumps %d\n", 
	     getFrontBumps(), getRearBumps());
  ret += line;  
  sprintf(line, "Settings:\n");
  ret += line;  
  sprintf(line, "SipCycle %d SonarCycle %d HostBaud %d Aux1Baud %d\n", getSipCycleTime(), getSonarCycle(), getHostBaud(), getAux1Baud());
  ret += line;  
  sprintf(line, "StallVal %d StallCount %d RevCount %d Watchdog %d\n",
	     getStallVal(), getStallCount(), getRevCount(), getWatchdog());
  ret += line;  
  sprintf(line, "JoyVel %d JoyRVel %d NormalMotorPackets %s\n", getJoyVel(), getJoyRotVel(), ArUtil::convertBool(getNormalMPacs()));
  ret += line;  
  sprintf(line, "PID Settings:\n");
  ret += line;  
  sprintf(line, "Rot kp %d kv %d ki %d\n", getRotKP(), getRotKV(),
	     getRotKI());
  ret += line;  
  sprintf(line, "Trans kp %d kv %d ki %d\n", getTransKP(), 
	     getTransKV(), getTransKI());
  ret += line;  
  sprintf(line, "DriftFactor %d\n", getDriftFactor());
  ret += line;  
  sprintf(line, "Aux2Baud setting %d, Aux3Baud setting %d\n", getAux2Baud(), getAux3Baud());
  ret += line;  
  sprintf(line, "TicksMM: %d\n", getTicksMM());
  ret += line;  
  sprintf(line, "Shutdown Voltage: %d\n", getShutdownVoltage());
  ret += line;  

  return ret;
}
