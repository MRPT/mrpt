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
#include "ArRobotParams.h"
#include "ariaInternal.h"

AREXPORT ArRobotParams::ArRobotParams() :
  ArConfig(NULL, true),
  myIRUnitGetFunctor(this, &ArRobotParams::getIRUnits),
  myIRUnitSetFunctor(this, &ArRobotParams::parseIRUnit),
  mySonarUnitGetFunctor(this, &ArRobotParams::getSonarUnits),
  mySonarUnitSetFunctor(this, &ArRobotParams::parseSonarUnit)
{
  sprintf(myClass, "Pioneer");
  mySubClass[0] = '\0';
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myRobotWidth = 400;
  myRobotLength = 500; 
  myRobotLengthFront = 0; 
  myRobotLengthRear = 0; 
  myHolonomic = true;
  myAbsoluteMaxVelocity = 0;
  myAbsoluteMaxRVelocity = 0;
  myHaveMoveCommand = true;
  myAngleConvFactor = 0.001534;
  myDistConvFactor = 0;
  myVelConvFactor = 1.0;
  myRangeConvFactor = 0;
  myVel2Divisor = 20;
  myGyroScaler = 1.626;
  myNumSonar = 0;
  myTableSensingIR = false;
  myNewTableSensingIR = false;
  myFrontBumpers = false;
  myNumFrontBumpers = 5;
  myRearBumpers = false;
  myNumRearBumpers = 5;
  myNumSonar = 0;
  myNumIR = 0;
  mySonarMap.clear();
  myIRMap.clear();
  myLaserPossessed = false;
  sprintf(myLaserPort, "COM3");
  myLaserFlipped = false;
  myLaserPowerControlled = true;
  myLaserX = 0;
  myLaserY = 0;
  myLaserTh = 0.0;
  myLaserIgnore[0] = '\0';
  
  myRequestIOPackets = false;
  myRequestEncoderPackets = false;
  mySwitchToBaudRate = 38400;

  mySettableVelMaxes = true;
  myTransVelMax = 0;
  myRotVelMax = 0;

  mySettableAccsDecs = true;
  myTransAccel = 0;
  myTransDecel = 0;
  myRotAccel = 0;
  myRotDecel = 0;

  addComment("Robot parameter file");
//  addComment("");
  //addComment("General settings");
  std::string section;
  section = "General settings";
  addParam(ArConfigArg("Class", myClass, "general type of robot", 
		 sizeof(myClass)), section.c_str(), ArPriority::TRIVIAL);
  addParam(ArConfigArg("Subclass", mySubClass, "specific type of robot", 
		       sizeof(mySubClass)), section.c_str(), 
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("RobotRadius", &myRobotRadius, "radius in mm"), 
	   section.c_str(), ArPriority::NORMAL);
  addParam(ArConfigArg("RobotDiagonal", &myRobotDiagonal, 
		 "half-height to diagonal of octagon"), "General settings",
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("RobotWidth", &myRobotWidth, "width in mm"), 
	   section.c_str(), ArPriority::NORMAL);
  addParam(ArConfigArg("RobotLength", &myRobotLength, "length in mm of the whole robot"),
	   section.c_str(), ArPriority::NORMAL);
  addParam(ArConfigArg("RobotLengthFront", &myRobotLengthFront, "length in mm to the front of the robot (if this is 0 (or non existant) this value will be set to half of RobotLength)"),
	   section.c_str(), ArPriority::NORMAL);
  addParam(ArConfigArg("RobotLengthRear", &myRobotLengthRear, "length in mm to the rear of the robot (if this is 0 (or non existant) this value will be set to half of RobotLength)"), 
	   section.c_str(), ArPriority::NORMAL);
  addParam(ArConfigArg("Holonomic", &myHolonomic, "turns in own radius"), 
	   section.c_str(), ArPriority::TRIVIAL);
  addParam(ArConfigArg("MaxRVelocity", &myAbsoluteMaxRVelocity, 
		       "absolute maximum degrees / sec"), section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("MaxVelocity", &myAbsoluteMaxVelocity, 
		 "absolute maximum mm / sec"), section.c_str(), 
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("HasMoveCommand", &myHaveMoveCommand, 
		 "has built in move command"), section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("RequestIOPackets", &myRequestIOPackets,
		 "automatically request IO packets"), section.c_str(),
	   ArPriority::NORMAL);
  addParam(ArConfigArg("RequestEncoderPackets", &myRequestEncoderPackets,
		       "automatically request encoder packets"), 
	   section.c_str(), ArPriority::NORMAL);
  addParam(ArConfigArg("SwitchToBaudRate", &mySwitchToBaudRate, 
		 "switch to this baud if non-0 and supported on robot"), 
	   section.c_str(), ArPriority::IMPORTANT);
  
  section = "Conversion factors";
  addParam(ArConfigArg("AngleConvFactor", &myAngleConvFactor,
		     "radians per angular unit (2PI/4096)"), section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("DistConvFactor", &myDistConvFactor,
		       "multiplier to mm from robot units"), section.c_str(),
	   ArPriority::IMPORTANT);
  addParam(ArConfigArg("VelConvFactor", &myVelConvFactor,
		     "multiplier to mm/sec from robot units"), 
	   section.c_str(),
	   ArPriority::NORMAL);
  addParam(ArConfigArg("RangeConvFactor", &myRangeConvFactor, 
		       "multiplier to mm from sonar units"), section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("DiffConvFactor", &myDiffConvFactor, 
		     "ratio of angular velocity to wheel velocity (unused in newer firmware that calculates and returns this)"), 
	   section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("Vel2Divisor", &myVel2Divisor, 
		       "divisor for VEL2 commands"), section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("GyroScaler", &myGyroScaler, 
		     "Scaling factor for gyro readings"), section.c_str(),
	   ArPriority::IMPORTANT);

  section = "Accessories the robot has";
  addParam(ArConfigArg("TableSensingIR", &myTableSensingIR,
		       "if robot has upwards facing table sensing IR"), 
	   section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("NewTableSensingIR", &myNewTableSensingIR,
		 "if table sensing IR are sent in IO packet"), 
	   section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("FrontBumpers", &myFrontBumpers, 
		 "if robot has a front bump ring"), section.c_str(),
	   ArPriority::IMPORTANT);
  addParam(ArConfigArg("NumFrontBumpers", &myNumFrontBumpers,
		     "number of front bumpers on the robot"), 
	   section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("RearBumpers", &myRearBumpers,
		       "if the robot has a rear bump ring"), section.c_str(),
	   ArPriority::IMPORTANT);
  addParam(ArConfigArg("NumRearBumpers", &myNumRearBumpers,
		       "number of rear bumpers on the robot"), section.c_str(),
	   ArPriority::TRIVIAL);

  section = "IR parameters";
  addParam(ArConfigArg("IRNum", &myNumIR, "number of IRs on the robot"), section.c_str(), ArPriority::NORMAL);
   addParam(ArConfigArg("IRUnit", &myIRUnitSetFunctor, &myIRUnitGetFunctor,
			"IRUnit <IR Number> <IR Type> <Persistance, cycles> <x position, mm> <y position, mm>"), 
	    section.c_str(), ArPriority::TRIVIAL);


  section = "Sonar parameters";
  addParam(ArConfigArg("SonarNum", &myNumSonar, 
		     "number of sonar on the robot"), section.c_str(),
	   ArPriority::NORMAL);
  addParam(ArConfigArg("SonarUnit", &mySonarUnitSetFunctor, 
		     &mySonarUnitGetFunctor,
		     "SonarUnit <sonarNumber> <x position, mm> <y position, mm> <heading of disc, degrees>"), section.c_str(), ArPriority::TRIVIAL);


  section = "Laser parameters";
  addParam(ArConfigArg("LaserPossessed", &myLaserPossessed, 
		     "if there is a laser on the robot"), section.c_str(),
	   ArPriority::IMPORTANT);
  addParam(ArConfigArg("LaserPort", myLaserPort, "port the laser is on", 
		     sizeof(myLaserPort)), section.c_str(),
	   ArPriority::NORMAL);
  addParam(ArConfigArg("LaserFlipped", &myLaserFlipped,
		     "if the laser is upside-down or not"), section.c_str(),
	   ArPriority::NORMAL);
  addParam(ArConfigArg("LaserPowerControlled", &myLaserPowerControlled,
		     "if the power to the laser is controlled by serial"), 
	   section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("LaserX", &myLaserX, "x location of laser, mm"), 
	   section.c_str(),
	   ArPriority::NORMAL);
  addParam(ArConfigArg("LaserY", &myLaserY, "y location of laser, mm"), 
	   section.c_str(),
	   ArPriority::NORMAL);
  addParam(ArConfigArg("LaserTh", &myLaserTh, "rotation of laser, deg"), 
	   section.c_str(),
	   ArPriority::NORMAL);
  addParam(ArConfigArg("LaserIgnore", myLaserIgnore, "Readings within a degree of the listed degrees (separated by a space) will be ignored", sizeof(myLaserIgnore)), 
	   section.c_str(),
	   ArPriority::NORMAL);

  section = "Movement control parameters";
  setSectionComment(section.c_str(), "if these are 0 the parameters from robot flash will be used, otherwise these values will be used");
  addParam(ArConfigArg("SettableVelMaxes", &mySettableVelMaxes, "if TransVelMax and RotVelMax can be set"), section.c_str(),
	   ArPriority::TRIVIAL);
  addParam(ArConfigArg("TransVelMax", &myTransVelMax, "maximum desired translational velocity for the robot"), section.c_str(), 
	   ArPriority::IMPORTANT);
  addParam(ArConfigArg("RotVelMax", &myRotVelMax, "maximum desired rotational velocity for the robot"), section.c_str(),
	   ArPriority::IMPORTANT);
  addParam(ArConfigArg("SettableAccsDecs", &mySettableAccsDecs, "if the accel and decel parameters can be set"), section.c_str(), ArPriority::TRIVIAL);
  addParam(ArConfigArg("TransAccel", &myTransAccel, "translational acceleration"), 
	   section.c_str(), ArPriority::IMPORTANT);
  addParam(ArConfigArg("TransDecel", &myTransDecel, "translational deceleration"), 

	   section.c_str(), ArPriority::IMPORTANT);
  addParam(ArConfigArg("RotAccel", &myRotAccel, "rotational acceleration"), 
	   section.c_str());
  addParam(ArConfigArg("RotDecel", &myRotDecel, "rotational deceleration"),
	   section.c_str(), ArPriority::IMPORTANT);

}

AREXPORT ArRobotParams::~ArRobotParams()
{

}


AREXPORT bool ArRobotParams::parseIRUnit(ArArgumentBuilder *builder)
{
  if (builder->getArgc() != 5 || !builder->isArgInt(0) || 
      !builder->isArgInt(1) || !builder->isArgInt(2) || 
      !builder->isArgInt(3) || !builder->isArgInt(4))
  {
    ArLog::log(ArLog::Terse, "ArRobotParams: IRUnit parameters invalid");
    return false;
  }
  myIRMap[builder->getArgInt(0)][IR_TYPE] = builder->getArgInt(1);
  myIRMap[builder->getArgInt(0)][IR_CYCLES] = builder->getArgInt(2);
  myIRMap[builder->getArgInt(0)][IR_X] = builder->getArgInt(3);
  myIRMap[builder->getArgInt(0)][IR_Y] = builder->getArgInt(4);
  return true;
}

AREXPORT const std::list<ArArgumentBuilder *> *ArRobotParams::getIRUnits(void)
{
  std::map<int, std::map<int, int> >::iterator it;
  int num, type, cycles,  x, y;
  ArArgumentBuilder *builder;

  for (it = myIRMap.begin(); it != myIRMap.end(); it++)
  {
    num = (*it).first;
    type = (*it).second[IR_TYPE];
    cycles = (*it).second[IR_CYCLES];
    x = (*it).second[IR_X];
    y = (*it).second[IR_Y];
    builder = new ArArgumentBuilder;
    builder->add("%d %d %d %d %d", num, type, cycles, x, y);
    myGetIRUnitList.push_back(builder);
  }
  return &myGetIRUnitList;
}

AREXPORT void ArRobotParams::internalSetIR(int num, int type, int cycles, int x, int y)
{
  myIRMap[num][IR_TYPE] = type;
  myIRMap[num][IR_CYCLES] = cycles;
  myIRMap[num][IR_X] = x;
  myIRMap[num][IR_Y] = y;
}

AREXPORT bool ArRobotParams::parseSonarUnit(ArArgumentBuilder *builder)
{
  if (builder->getArgc() != 4 || !builder->isArgInt(0) || 
      !builder->isArgInt(1) || !builder->isArgInt(2) ||
      !builder->isArgInt(3))
  {
    ArLog::log(ArLog::Terse, "ArRobotParams: SonarUnit parameters invalid");
    return false;
  }
  mySonarMap[builder->getArgInt(0)][SONAR_X] = builder->getArgInt(1);
  mySonarMap[builder->getArgInt(0)][SONAR_Y] = builder->getArgInt(2);
  mySonarMap[builder->getArgInt(0)][SONAR_TH] = builder->getArgInt(3);
  return true;
}


AREXPORT const std::list<ArArgumentBuilder *> *ArRobotParams::getSonarUnits(void)
{
  std::map<int, std::map<int, int> >::iterator it;
  int num, x, y, th;
  ArArgumentBuilder *builder;

  for (it = mySonarMap.begin(); it != mySonarMap.end(); it++)
  {
    num = (*it).first;
    x = (*it).second[SONAR_X];
    y = (*it).second[SONAR_Y];
    th = (*it).second[SONAR_TH];
    builder = new ArArgumentBuilder;
    builder->add("%d %d %d %d", num, x, y, th);
    myGetSonarUnitList.push_back(builder);
  }
  return &myGetSonarUnitList;
}

AREXPORT void ArRobotParams::internalSetSonar(int num, int x, 
					      int y, int th)
{
  mySonarMap[num][SONAR_X] = x;
  mySonarMap[num][SONAR_Y] = y;
  mySonarMap[num][SONAR_TH] = th;
}

AREXPORT bool ArRobotParams::save(void)
{
  char buf[10000];
  sprintf(buf, "%sparams/", Aria::getDirectory());
  setBaseDirectory(buf);
  sprintf(buf, "%s.p", getSubClassName());
  return writeFile(buf, false, NULL, false);
}
