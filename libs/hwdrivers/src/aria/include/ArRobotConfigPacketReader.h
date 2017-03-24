/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARROBOTCONFIGPACKETREADER_H
#define ARROBOTCONFIGPACKETREADER_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"

class ArRobot;
class ArRobotPacket;

/// This class will read a config packet from the robot
class ArRobotConfigPacketReader
{
public:
  /// Constructor
  AREXPORT ArRobotConfigPacketReader(ArRobot *robot, 
				     bool onlyOneRequest = false,
				     ArFunctor *packetedArrivedCB = NULL);
  /// Destructor
  AREXPORT ~ArRobotConfigPacketReader();
  /// Request a packet.. true if we could, false if onlyOneRequest already done
  AREXPORT bool requestPacket(void);
  /// See if we've requested a packet yet
  bool hasPacketBeenRequested(void) const { return myPacketRequested; }
  /// See if we've gotten the data
  bool hasPacketArrived(void) const { return myPacketArrived; }
  /// Log the config
  AREXPORT void log(void) const;
  /// Builds a string of the info
  AREXPORT std::string buildString(void) const;
  /// Gets the type of robot
  const char *getType(void) const { return myType.c_str(); }
  /// Gets the subtype of robot
  const char *getSubType(void) const { return mySubType.c_str(); }
  /// Gets the serial number of the robot
  const char *getSerialNumber(void) const { return mySerialNumber.c_str(); }
  /// Gets the absolute maximum rotational velocity in deg/sec (cannot be set above this in firmware or through software)
  int getRotVelTop(void) const { return myRotVelTop; }
  /// Gets the absolute maximum translational velocity in mm/sec (cannot be set above this in firmware or through software)
  int getTransVelTop(void) const { return myTransVelTop; }
  /// Gets the absolute maximum rotational acceleration in deg/sec/sec (cannot be set above this in firmware or through software)
  int getRotAccelTop(void) const { return myRotAccelTop; }
  /// Gets the absolute maximum translational acceleration in mm/sec/sec (cannot be set above this in firmware or through software)
  int getTransAccelTop(void) const { return myTransAccelTop; }
  /// Gets the maximum PWM the robot will have (stallval cannot be above this)
  int getPwmMax(void) const { return myPwmMax; }
  /// Gets the name of the robot
  const char *getName(void) const { return myName.c_str(); }
  /// Gets the cycle time in ms of the motor packets
  int getSipCycleTime(void) const { return mySipCycleTime; }
  /// Gets the host baud number, look at the manual for what these mean
  int getHostBaud(void) const { return myHostBaud; }
  /// Gets the host baud number, look at the manual for what these mean
  int getAux1Baud(void) const { return myAux1Baud; }
  /// Gets the gripper value (whether or not the robot has a gripper)
  bool getHasGripper(void) const { return myHasGripper; }
  /// Gets whether or not the robot has front sonar
  bool getFrontSonar(void) const { return myFrontSonar; }
  /// Gets whether or not the robot has rear sonar
  bool getRearSonar(void) const { return myRearSonar; }
  /// Gets the low battery beeping indicating voltage times 10
  int getLowBattery(void) const { return myLowBattery; }
  /// Gets the revcount
  int getRevCount(void) const { return myRevCount; }
  /// Gets the watchdog (how many ms after command robot stops)
  int getWatchdog(void) const { return myWatchdog; }
  /// Returns if the robot is using normal packets or new style packets
  bool getNormalMPacs(void) const { return myNormalMPacs; }
  /// Returns the stallval (pwms at which robot stalls)
  int getStallVal(void) const { return myStallVal; }
  /// Returns the stallcount (how many 10ms increments robot stops after stall)
  int getStallCount(void) const { return myStallCount; }
  /// Returns the joystick translational velocity
  int getJoyVel(void) const { return myJoyVel; }
  /// Returns the joystick rotational velocity
  int getJoyRotVel(void) const { return myJoyRotVel; }
  /// Returns the current maximum rotational velocity (deg/sec) (can be set)
  int getRotVelMax(void) const { return myRotVelMax; } 
  /// Returns the current maximum translational velocity (mm/sec) (can be set)
  int getTransVelMax(void) const { return myTransVelMax; } 
  /// Returns the rotational acceleration
  int getRotAccel(void) const { return myRotAccel; }
  /// Returns the rotational deceleration
  int getRotDecel(void) const { return myRotDecel; }
  /// Returns the rotational KP value (look at the manual)
  int getRotKP(void) const { return myRotKP; }
  /// Returns the rotational KV value (look at the manual)
  int getRotKV(void) const { return myRotKV; }
  /// Returns the rotational KI value (look at the manual)
  int getRotKI(void) const { return myRotKI; }
  /// Returns the translational acceleration
  int getTransAccel(void) const { return myTransAccel; }
  /// Returns the rotational deceleration
  int getTransDecel(void) const { return myTransDecel; }
  /// Returns the translational KP value (look at the manual)
  int getTransKP(void) const { return myTransKP; }
  /// Returns the translational KV value (look at the manual)
  int getTransKV(void) const { return myTransKV; }
  /// Returns the translational KI value (look at the manual)
  int getTransKI(void) const { return myTransKI; }
  /// Returns the number of front bumpers
  int getFrontBumps(void) const { return myFrontBumps; }
  /// Returns the number of rear bumpers
  int getRearBumps(void) const { return myRearBumps; }
  /// Returns whether the robot has a charger
  int getHasCharger(void) const { return myHasCharger; }
  /// Returns the number of ms the sonar cycle is (default is 40)
  int getSonarCycle(void) const { return mySonarCycle; }
  /// Returns the baud rate
  bool getResetBaud(void) const { return myResetBaud; }
  /// Returns if the robot has a gyro or not
  bool getHasGyro(void) const { return myHasGyro; }
  /// Returns if the robot has a gyro or not
  int getGyroType(void) const { return myGyroType; }
  /// Returns the DriftFactor value (see the manual)
  int getDriftFactor(void) const { return myDriftFactor; }
  /// Returns the Aux2 baud number, look at the manual for what these mean
  int getAux2Baud(void) const { return myAux2Baud; }
  /// Returns the Aux3 baud number, look at the manual for what these mean
  int getAux3Baud(void) const { return myAux3Baud; }
  /// Returns the Ticks/MM for the robot
  int getTicksMM(void) const { return myTicksMM; }
  /// Returns the voltage (x10) that the robot will shut down the computer at
  int getShutdownVoltage(void) const { return myShutdownVoltage; }
  /// internal, packet handler
  AREXPORT bool packetHandler(ArRobotPacket *packet);
  /// internal, connection callback
  AREXPORT void connected(void);
protected:
  // the different parameters from the robot
  std::string myType;
  std::string mySubType;
  std::string mySerialNumber;
  int myRotVelTop;
  int myTransVelTop;
  int myRotAccelTop;
  int myTransAccelTop;
  int myPwmMax;
  std::string myName;
  int mySipCycleTime;
  int myHostBaud;
  int myAux1Baud;
  bool myHasGripper;
  bool myFrontSonar;
  bool myRearSonar;
  int myLowBattery;
  int myRevCount;
  int myWatchdog;
  bool myNormalMPacs;
  int myStallVal;
  int myStallCount;
  int myJoyVel;
  int myJoyRotVel;
  int myRotVelMax;
  int myTransVelMax;
  int myRotAccel;
  int myRotDecel;
  int myRotKP;
  int myRotKV;
  int myRotKI;
  int myTransAccel;
  int myTransDecel;
  int myTransKP;
  int myTransKV;
  int myTransKI;
  int myFrontBumps;
  int myRearBumps;
  int myHasCharger;
  int mySonarCycle;
  bool myResetBaud;
  bool myHasGyro;
  int myGyroType;
  int myDriftFactor;
  int myAux2Baud;
  int myAux3Baud;
  int myTicksMM;
  int myShutdownVoltage;

  // the robot
  ArRobot *myRobot;
  // whether only one request can be done or not
  bool myOnlyOneRequest;
  // whether a request has been made or not
  bool myPacketRequested;
  // whether our data has been received or not
  bool myPacketArrived;
  // last time we requested a packet (we'll only ask every 200 ms)
  ArTime myLastPacketRequest;
  // the callback
  ArRetFunctor1C<bool, ArRobotConfigPacketReader, ArRobotPacket *> myPacketHandlerCB;
  // callback for connectiosn in case we need to request a config packet
  ArFunctorC<ArRobotConfigPacketReader> myConnectedCB;
  // callback for when we get a packet
  ArFunctor *myPacketArrivedCB;
  
};

#endif // ARROBOTCONFIGPACKETREADER_H
