/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARROBOTPARAMS_H
#define ARROBOTPARAMS_H

#include "ariaTypedefs.h"
#include "ArConfig.h"

/// Contains the robot parameters, according to the parameter file
class ArRobotParams : public ArConfig
{
public:
  /// Constructor
  AREXPORT ArRobotParams();
  /// Destructor
  AREXPORT virtual ~ArRobotParams();
  /// Returns the class from the parameter file
  const char *getClassName(void) const { return myClass; }
  /// Returns the subclass from the parameter file
  const char *getSubClassName(void) const { return mySubClass; }
  /// Returns the robot's radius
  double getRobotRadius(void) const { return myRobotRadius; }
  /// Returns the robot diagonal (half-height to diagonal of octagon)
  double getRobotDiagonal(void) const { return myRobotDiagonal; }
  /// Returns the robot's width
  double getRobotWidth(void) const { return myRobotWidth; }
  /// Returns the robot's length
  double getRobotLength(void) const { return myRobotLength; }
  /// Returns the robot's length to the front of the robot
  double getRobotLengthFront(void) const { return myRobotLengthFront; }
  /// Returns the robot's length to the rear of the robot
  double getRobotLengthRear(void) const { return myRobotLengthRear; }
  /// Returns whether the robot is holonomic or not
  bool isHolonomic(void) const { return myHolonomic; }
  /// Returns if the robot has a built in move command
  bool hasMoveCommand(void) const { return myHaveMoveCommand; }
  /// Returns the max velocity of the robot
  int getAbsoluteMaxVelocity(void) const { return myAbsoluteMaxVelocity; }
  /// Returns the max rotational velocity of the robot
  int getAbsoluteMaxRotVelocity(void) const { return myAbsoluteMaxRVelocity; }
  /// Returns true if IO packets are automatically requested upon connection to the robot.
  bool getRequestIOPackets(void) const { return myRequestIOPackets; }
  /// Returns true if encoder packets are automatically requested upon connection to the robot.
  bool getRequestEncoderPackets(void) const { return myRequestEncoderPackets; }
  /// Returns the baud rate set in the param to talk to the robot at
  int getSwitchToBaudRate(void) const { return mySwitchToBaudRate; }
  /// Returns the angle conversion factor 
  double getAngleConvFactor(void) const { return myAngleConvFactor; }
  /// Returns the distance conversion factor
  double getDistConvFactor(void) const { return myDistConvFactor; }
  /// Returns the velocity conversion factor
  double getVelConvFactor(void) const { return myVelConvFactor; }
  /// Returns the sonar range conversion factor
  double getRangeConvFactor(void) const { return myRangeConvFactor; }
  /// Returns the wheel velocity difference to angular velocity conv factor
  double getDiffConvFactor(void) const { return myDiffConvFactor; }
  /// Returns the multiplier for VEL2 commands
  double getVel2Divisor(void) const { return myVel2Divisor; }
  /// Returns the multiplier for the Analog Gyro
  double getGyroScaler(void) const { return myGyroScaler; }
  /// Returns true if the robot has table sensing IR
  bool haveTableSensingIR(void) const { return myTableSensingIR; }
  /// Returns true if the robot's table sensing IR bits are sent in the 4th-byte of the IO packet
  bool haveNewTableSensingIR(void) const { return myNewTableSensingIR; }
  /// Returns true if the robot has front bumpers
  bool haveFrontBumpers(void) const { return myFrontBumpers; }
  /// Returns the number of front bumpers
  int numFrontBumpers(void) const { return myNumFrontBumpers; }
  /// Returns true if the robot has rear bumpers
  bool haveRearBumpers(void) const { return myRearBumpers; }
  /// Returns the number of rear bumpers
  int numRearBumpers(void) const { return myNumRearBumpers; }
  /// Returns the number of IRs
  int getNumIR(void) const { return myNumIR; }
  /// Returns if the IR of the given number is valid
  bool haveIR(int number) const
    {
      if (myIRMap.find(number) != myIRMap.end())
	return true;
      else
	return false;
    }
  /// Returns the X location of the given numbered IR
  int getIRX(int number)
    {
      std::map<int, std::map<int, int> >::iterator it;
      std::map<int, int>::iterator it2;
      if ((it = myIRMap.find(number)) == myIRMap.end())
	return 0;
      if ((it2 = (*it).second.find(IR_X)) == (*it).second.end())

	return 0;
      return (*it2).second;
    }
  /// Returns the Y location of the given numbered IR
  int getIRY(int number)
    {
      std::map<int, std::map<int, int> >::iterator it;
      std::map<int, int>::iterator it2;
      if ((it = myIRMap.find(number)) == myIRMap.end())
	return 0;
      if ((it2 = (*it).second.find(IR_Y)) == (*it).second.end())
	return 0;
      return (*it2).second;
    }
  int getIRType(int number)
    {
      std::map<int, std::map<int, int> >::iterator it;
      std::map<int, int>::iterator it2;
      if ((it = myIRMap.find(number)) == myIRMap.end())
	return 0;
      if ((it2 = (*it).second.find(IR_TYPE)) == (*it).second.end())
	return 0;
      return (*it2).second;
    }
  int getIRCycles(int number)
    {
      std::map<int, std::map<int, int> >::iterator it;
      std::map<int, int>::iterator it2;
      if ((it = myIRMap.find(number)) == myIRMap.end())
	return 0;
      if ((it2 = (*it).second.find(IR_CYCLES)) == (*it).second.end())
	return 0;
      return (*it2).second;
    }
  /// Returns the number of sonar
  int getNumSonar(void) const { return myNumSonar; }
  /// Returns if the sonar of the given number is valid
  bool haveSonar(int number)
    {
      if (mySonarMap.find(number) != mySonarMap.end())
	return true;
      else
	return false;
    }
  /// Returns the X location of the given numbered sonar disc
  int getSonarX(int number)
    {
      std::map<int, std::map<int, int> >::iterator it;
      std::map<int, int>::iterator it2;
      if ((it = mySonarMap.find(number)) == mySonarMap.end())
	return 0;
      if ((it2 = (*it).second.find(SONAR_X)) == (*it).second.end())
	return 0;
      return (*it2).second;
    }
  /// Returns the Y location of the given numbered sonar disc
  int getSonarY(int number)
    {
      std::map<int, std::map<int, int> >::iterator it;
      std::map<int, int>::iterator it2;
      if ((it = mySonarMap.find(number)) == mySonarMap.end())
	return 0;
      if ((it2 = (*it).second.find(SONAR_Y)) == (*it).second.end())
	return 0;
      return (*it2).second;
    }
  /// Returns the heading of the given numbered sonar disc
  int getSonarTh(int number)
    {
      std::map<int, std::map<int, int> >::iterator it;
      std::map<int, int>::iterator it2;
      if ((it = mySonarMap.find(number)) == mySonarMap.end())
	return 0;
      if ((it2 = (*it).second.find(SONAR_TH)) == (*it).second.end())
	return 0;
      return (*it2).second;
    }
  /// Returns if the robot has a laser (according to param file)
  bool getLaserPossessed(void) const { return myLaserPossessed; }
  /// What port the laser is on
  const char *getLaserPort(void) const { return myLaserPort; }
  /// If the laser power is controlled by the serial port lines
  bool getLaserPowerControlled(void) const { return myLaserPowerControlled; }
  /// If the laser is flipped on the robot
  bool getLaserFlipped(void) const { return myLaserFlipped; }
  /// The X location of the laser
  int getLaserX(void) const { return myLaserX; }
  /// The Y location of the laser 
  int getLaserY(void) const { return myLaserY; }
  /// The rotation of the laser on the robot
  double getLaserTh(void) const { return myLaserTh; }
  /// Gets the string that is the readings the sick should ignore
  const char *getLaserIgnore(void) const { return myLaserIgnore; }
  /// Gets whether the VelMax values are settable or not
  bool hasSettableVelMaxes(void) const { return mySettableVelMaxes; }
  /// Gets the max trans vel from param file (0 uses microcontroller param)
  int getTransVelMax(void) const { return myTransVelMax; }
  /// Gets the max rot vel from param file (0 uses microcontroller param)
  int getRotVelMax(void) const { return myRotVelMax; }
  /// Whether the accelerations and decelerations are settable or not
  bool hasSettableAccsDecs(void) const { return mySettableAccsDecs; }
  /// Gets the trans accel from param file (0 uses microcontroller param)
  int getTransAccel(void) const { return myTransAccel; }
  /// Gets the trans decel from param file (0 uses microcontroller param)
  int getTransDecel(void) const { return myTransDecel; }
  /// Gets the rot accel from param file (0 uses microcontroller param)
  int getRotAccel(void) const { return myRotAccel; }
  /// Gets the rot decel from param file (0 uses microcontroller param)
  int getRotDecel(void) const { return myRotDecel; }
  /// Saves it to the subtype.p in Aria::getDirectory/params
  AREXPORT bool save(void);
protected:
  char myClass[1024];
  char mySubClass[1024];
  double myRobotRadius;
  double myRobotDiagonal;
  double myRobotWidth;
  double myRobotLength;
  double myRobotLengthFront;
  double myRobotLengthRear;
  bool myHolonomic;
  int myAbsoluteMaxRVelocity;
  int myAbsoluteMaxVelocity;
  bool myHaveMoveCommand;
  bool myRequestIOPackets;
  bool myRequestEncoderPackets;
  int mySwitchToBaudRate;
  double myAngleConvFactor;
  double myDistConvFactor;
  double myVelConvFactor;
  double myRangeConvFactor;
  double myDiffConvFactor;
  double myVel2Divisor;
  double myGyroScaler;
  bool myTableSensingIR;
  bool myNewTableSensingIR;
  bool myFrontBumpers;
  int myNumFrontBumpers;
  bool myRearBumpers;
  int myNumRearBumpers;
  bool myLaserPossessed;
  char myLaserPort[1024];
  bool myLaserFlipped;
  bool myLaserPowerControlled;
  int myLaserX;
  int myLaserY;
  double myLaserTh;
  char myLaserIgnore[1024];
  
  bool mySettableVelMaxes;
  int myTransVelMax;
  int myRotVelMax;
  bool mySettableAccsDecs;
  int myTransAccel;
  int myTransDecel;
  int myRotAccel;
  int myRotDecel;

  // IRs
  int myNumIR;
  std::map<int, std::map<int, int> > myIRMap;
  enum IRInfo 
  { 
    IR_X, 
    IR_Y,
    IR_TYPE,
    IR_CYCLES
  };
  AREXPORT void internalSetIR(int num, int type, int cycles, int x, int y);
  AREXPORT bool parseIRUnit(ArArgumentBuilder *builder);
  AREXPORT const std::list<ArArgumentBuilder *> *getIRUnits(void);
  std::list<ArArgumentBuilder *> myGetIRUnitList;
  ArRetFunctorC<const std::list<ArArgumentBuilder *> *, ArRobotParams> myIRUnitGetFunctor;
  ArRetFunctor1C<bool, ArRobotParams, ArArgumentBuilder *> myIRUnitSetFunctor;

  // Sonar
  int myNumSonar;
  std::map<int, std::map<int, int> > mySonarMap;
  enum SonarInfo 
  { 
    SONAR_X, 
    SONAR_Y, 
    SONAR_TH
  };
  AREXPORT void internalSetSonar(int num, int x, int y, int th);
  AREXPORT bool parseSonarUnit(ArArgumentBuilder *builder);
  AREXPORT const std::list<ArArgumentBuilder *> *getSonarUnits(void);
  std::list<ArArgumentBuilder *> myGetSonarUnitList;
  ArRetFunctorC<const std::list<ArArgumentBuilder *> *, ArRobotParams> mySonarUnitGetFunctor;
  ArRetFunctor1C<bool, ArRobotParams, ArArgumentBuilder *> mySonarUnitSetFunctor;
};

#endif // ARROBOTPARAMS_H
