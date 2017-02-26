/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARROBOT_H
#define ARROBOT_H

#include "ariaTypedefs.h"
#include "ArRobotPacketSender.h"
#include "ArRobotPacketReceiver.h"
#include "ArFunctor.h"
#include "ArFunctor.h"
#include "ArSyncTask.h"
#include "ArSensorReading.h"
#include "ArMutex.h"
#include "ArCondition.h"
#include "ArSyncLoop.h"
#include "ArRobotParams.h"
#include "ArActionDesired.h"
#include "ArResolver.h"
#include "ArTransform.h"
#include "ArInterpolation.h"
#include "ArKeyHandler.h"
#include <list>

class ArAction;
class ArRobotConfigPacketReader;
class ArDeviceConnection;
class ArRangeDevice;
class ArRobotPacket;
class ArPTZ;

/// Central class for communicating with and operating the robot
/**
    This is the most important class. It is used to communicate with
    the robot by sending commands and retrieving data (including
    wheel odometry, digital and analog inputs, sonar data, and more).
    It is also used
    to provide access to objects for controlling attached accessories,
    ArRangeDevice objects, ArAction objects, and others.  For details
    on usage, and how the task cycle and obot state synchronization works,
    see the @ref robot "ArRobot section" and the
    @ref ClientCommands "Commands and Actions section" of the ARIA overview.

    @note In Windows you cannot make an ArRobot object a global variable,
    it will crash because the compiler initializes the constructors in
    the wrong order. You can, however, make a pointer to an ArRobot and then
    allocate it with 'new' at program start.

    @see ArSimpleConnector
**/
class ArRobot
{
public:

  typedef enum {
    WAIT_CONNECTED, ///< The robot has connected
    WAIT_FAILED_CONN, ///< The robot failed to connect
    WAIT_RUN_EXIT, ///< The run loop has exited
    WAIT_TIMEDOUT, ///< The wait reached the timeout specified
    WAIT_INTR, ///< The wait was interupted by a signal
    WAIT_FAIL ///< The wait failed due to an error
  } WaitState;

  enum ChargeState {
    CHARGING_UNKNOWN = -1,
    CHARGING_NOT = 0,
    CHARGING_BULK = 1,
    CHARGING_OVERCHARGE = 2,
    CHARGING_FLOAT = 3
  };
  /// Constructor
  AREXPORT ArRobot(const char * name = NULL, bool ignored = true,
		   bool doSigHandle=true,
		   bool normalInit = true, bool addAriaExitCallback = true);

  /// Destructor
  AREXPORT ~ArRobot();

  /// Starts the instance to do processing in this thread
  AREXPORT void run(bool stopRunIfNotConnected);
  /// Starts the instance to do processing in its own new thread
  AREXPORT void runAsync(bool stopRunIfNotConnected);

  /// Returns whether the robot is currently running or not
  AREXPORT bool isRunning(void) const;

  /// Stops the robot from doing any more processing
  AREXPORT void stopRunning(bool doDisconnect=true);

  /// Sets the connection this instance uses
  AREXPORT void setDeviceConnection(ArDeviceConnection *connection);
  /// Gets the connection this instance uses
  AREXPORT ArDeviceConnection *getDeviceConnection(void) const;

  /// Questions whether the robot is connected or not
  /**
      @return true if connected to a robot, false if not
  */
  /*AREXPORT*/ bool isConnected(void) const { return myIsConnected; }
  /// Connects to a robot, not returning until connection made or failed
  AREXPORT bool blockingConnect(void);
  /// Connects to a robot, from the robots own thread
  AREXPORT bool asyncConnect(void);
  /// Disconnects from a robot
  AREXPORT bool disconnect(void);

  /// Clears what direct motion commands have been given, so actions work
  AREXPORT void clearDirectMotion(void);
  /// Returns true if direct motion commands are blocking actions
  AREXPORT bool isDirectMotion(void) const;

  /// Sets the state reflection to be inactive (until motion or clearDirectMotion)
  /// @see clearDirectMotion
  AREXPORT void stopStateReflection(void);


  /// Enables the motors on the robot
  AREXPORT void enableMotors();
  /// Disables the motors on the robot
  AREXPORT void disableMotors();

  /// Enables the sonar on the robot
  AREXPORT void enableSonar();
  /// Disables the sonar on the robot
  AREXPORT void disableSonar();

  /// Stops the robot
  /// @see clearDirectMotion
  AREXPORT void stop(void);
  /// Sets the velocity
  /// @see clearDirectMotion
  AREXPORT void setVel(double velocity);
  /// Sets the velocity of the wheels independently
  AREXPORT void setVel2(double leftVelocity, double rightVelocity);
  /// Move the given distance forward/backwards
  AREXPORT void move(double distance);
  /// Sees if the robot is done moving the previously given move
  AREXPORT bool isMoveDone(double delta = 0.0);
  /// Sets the difference required for being done with a move
  /*AREXPORT*/ void setMoveDoneDist(double dist) { myMoveDoneDist = dist; }
  /// Gets the difference required for being done with a move
  /*AREXPORT*/ double getMoveDoneDist(void) { return myMoveDoneDist; }
  /// Sets the heading
  AREXPORT void setHeading(double heading);
  /// Sets the rotational velocity
  AREXPORT void setRotVel(double velocity);
  /// Sets the delta heading
  AREXPORT void setDeltaHeading(double deltaHeading);
  /// Sees if the robot is done changing to the previously given setHeading
  AREXPORT bool isHeadingDone(double delta = 0.0) const;
  /// sets the difference required for being done with a heading change
  /*AREXPORT*/void setHeadingDoneDiff(double degrees)
    { myHeadingDoneDiff = degrees; }
  /// Gets the difference required for being done with a heading change
  /*AREXPORT*/ double getHeadingDoneDiff(void) const { return myHeadingDoneDiff; }


  /// Sets the length of time a direct motion command will take precedence
  /// over actions, in milliseconds
  AREXPORT void setDirectMotionPrecedenceTime(int mSec);

  /// Gets the length of time a direct motion command will take precedence
  /// over actions, in milliseconds
  AREXPORT unsigned int getDirectMotionPrecedenceTime(void) const;

  /// Sends a command to the robot with no arguments
  AREXPORT bool com(unsigned char command);
  /// Sends a command to the robot with an int for argument
  AREXPORT bool comInt(unsigned char command, short int argument);
  /// Sends a command to the robot with two bytes for argument
  AREXPORT bool com2Bytes(unsigned char command, char high, char low);
  /// Sends a command to the robot with a length-prefixed string for argument
  AREXPORT bool comStr(unsigned char command, const char *argument);
  /// Sends a command to the robot with a length-prefixed string for argument
  AREXPORT bool comStrN(unsigned char command, const char *str, int size);
  /// Sends a command containing exactly the data in the given buffer as argument
  AREXPORT bool comDataN(unsigned char command, const char *data, int size);

  /// Returns the robot's name that is set in its onboard firmware configuration
  /*AREXPORT*/ const char * getRobotName(void) const { return myRobotName.c_str();}
  /// Returns the type of the robot we are currently connected to
  /*AREXPORT*/ const char * getRobotType(void) const { return myRobotType.c_str();}
  /// Returns the subtype of the robot we are currently connected to
  /*AREXPORT*/const char * getRobotSubType(void) const
    { return myRobotSubType.c_str(); }

  /// Gets the robot's absolute maximum translational velocity
  /*AREXPORT*/double getAbsoluteMaxTransVel(void) const
    { return myAbsoluteMaxTransVel; }
  /// Sets the robot's absolute maximum translational velocity
  AREXPORT bool setAbsoluteMaxTransVel(double maxVel);

  /// Gets the robot's absolute maximum translational acceleration
  /*AREXPORT*/double getAbsoluteMaxTransAccel(void) const
    { return myAbsoluteMaxTransAccel; }
  /// Sets the robot's absolute maximum translational acceleration
  AREXPORT bool setAbsoluteMaxTransAccel(double maxAccel);

  /// Gets the robot's absolute maximum translational deceleration
  /*AREXPORT*/double getAbsoluteMaxTransDecel(void) const
    { return myAbsoluteMaxTransDecel; }
  /// Sets the robot's absolute maximum translational deceleration
  AREXPORT bool setAbsoluteMaxTransDecel(double maxDecel);

  /// Gets the robot's absolute maximum rotational velocity
  /*AREXPORT*/double getAbsoluteMaxRotVel(void) const
    { return myAbsoluteMaxRotVel; }
  /// Sets the robot's absolute maximum rotational velocity
  AREXPORT bool setAbsoluteMaxRotVel(double maxVel);

  /// Gets the robot's absolute maximum rotational acceleration
  /*AREXPORT*/double getAbsoluteMaxRotAccel(void) const
    { return myAbsoluteMaxRotAccel; }
  /// Sets the robot's absolute maximum rotational acceleration
  AREXPORT bool setAbsoluteMaxRotAccel(double maxAccel);

  /// Gets the robot's absolute maximum rotational deceleration
  /*AREXPORT*/double getAbsoluteMaxRotDecel(void) const
    { return myAbsoluteMaxRotDecel; }
  /// Sets the robot's absolute maximum rotational deceleration
  AREXPORT bool setAbsoluteMaxRotDecel(double maxDecel);


  // Accessors

  /** @brief Get the current stored global position of the robot.
   *  This pose is updated by data reported by the robot as it
   *  moves, and may also be changed by other program components,
   *  such as a localization process (see moveTo()).
   *  @sa getEncoderPose()
   *  @sa moveTo()
   */
  ArPose getPose(void) const { return myGlobalPose; }
  /// Gets the global X location of the robot
  double getX(void) const { return  myGlobalPose.getX(); }
  /// Gets the global Y location of the robot
  double getY(void) const { return myGlobalPose.getY(); }
  /// Gets the global Th location of the robot
  double getTh(void) const { return myGlobalPose.getTh(); }
  /// Gets the distance to a point from the robot
  double findDistanceTo(const ArPose pose)
    { return myGlobalPose.findDistanceTo(pose); }
  /// Gets the distance to a point from the robot
  double findAngleTo(const ArPose pose)
    { return myGlobalPose.findAngleTo(pose); }
  /// Gets the distance to a point from the robot
  double findDeltaHeadingTo(const ArPose pose)
    { return ArMath::subAngle(myGlobalPose.findAngleTo(pose),
			      myGlobalPose.getTh()); }


  /// Gets the translational velocity of the robot
  double getVel(void) const { return myVel; }
  /// Gets the rotational velocity of the robot
  /**
     Note that with new firmware versions (as of April 2006 or so)
     this is the velocity reported by the robot.  With older firmware
     this number is come up with by the difference between the wheel
     velocities multiplied by diffConvFactor from the .p files.
  **/
  double getRotVel(void) const { return myRotVel; }
  /// Gets the robot radius (in mm)
  double getRobotRadius(void) const { return myParams->getRobotRadius(); }
  /// Gets the robot width (in mm)
  double getRobotWidth(void) const { return myParams->getRobotWidth(); }
  /// Gets the robot length (in mm)
  double getRobotLength(void) const { return myParams->getRobotLength(); }
  /// Gets the robot length to the front (in mm)
  double getRobotLengthFront(void) const { return myRobotLengthFront; }
  /// Gets the robot length to the front (in mm)
  double getRobotLengthRear(void) const { return myRobotLengthRear; }
  /// Gets the robot diagonal (half-height to diagonal of octagon) (in mm)
  double getRobotDiagonal(void) const { return myParams->getRobotDiagonal(); }
  /// Gets the battery voltage of the robot (normalized to 12 volt system)
  /**
     This value is averaged over a number of readings, (You can get
     this number by calling getBatteryVoltageAverageOfNum and set this with
     setBatteryVoltageAverageOfNum... you can call
     getBatteryVoltageNow to get the reading from the last packet.

     This is a value normalized to 12 volts, if you want what the
     actual voltage of the robot is use getRealBatteryVoltage.
   **/
  double getBatteryVoltage(void) const {return myBatteryAverager.getAverage();}
  /// Gets the instaneous battery voltage
  /**
     This is a value normalized to 12 volts, if you want what the
     actual voltage of the robot is use getRealBatteryVoltage.
   **/
  double getBatteryVoltageNow(void) const { return myBatteryVoltage; }
  /// Gets the real battery voltage of the robot
  /**
     This value is averaged over a number of readings, you can get
     this by calling getRealBatteryVoltageAverageOfNum and set this with
     setRealBatteryVoltageAverageOfNum... you can call
     getRealBatteryVoltageNow to get the reading from the last packet.

     This is whatever the actual voltage of the robot is, if you want
     a value normalized to 12 volts use getBatteryVoltage.

     If the robot connected to doesn't send the real battery voltage
     it will just use the normal battery voltage (normalized to 12
     volt scale).
   **/
  double getRealBatteryVoltage(void) const
    { return myRealBatteryAverager.getAverage(); }
  /// Gets the instaneous battery voltage
  /**
     This is whatever the actual voltage of the robot is, if you want
     a value normalized to 12 volts use getBatteryVoltage.  If the
     robot doesn't support this number the voltage will be less than 0
     and you should use getBatteryVoltageNow.
  **/
  double getRealBatteryVoltageNow(void) const { return myRealBatteryVoltage; }
  /// Gets the velocity of the left wheel
  double getLeftVel(void) const { return myLeftVel; }
  /// Gets the velocity of the right wheel
  double getRightVel(void) const { return myRightVel; }
  /// Gets the 2 bytes of stall and bumper flags from the robot (see operations manual for details)
  int getStallValue(void) const { return myStallValue; }
  /// Returns true if the left motor is stalled
  bool isLeftMotorStalled(void) const
    { return (myStallValue & 0xff) & ArUtil::BIT0; }
  /// Returns true if the left motor is stalled
  bool isRightMotorStalled(void) const
    { return ((myStallValue & 0xff00) >> 8) & ArUtil::BIT0; }
  /// Gets the control heading
  /**
    Gets the control heading as an offset from the current heading.
    @see getTh
  */
  double getControl(void) const { return myControl; }
  /// Gets the flags values
  int getFlags(void) const { return myFlags; }
  /// Gets the fault flags values
  int getFaultFlags(void) const { return myFaultFlags; }
  /// Gets whether or not we're getting the fault flags values
  bool hasFaultFlags(void) const { return myHasFaultFlags; }
  /// returns true if the motors are enabled
  bool areMotorsEnabled(void) const { return (myFlags & ArUtil::BIT0); }
  /// returns true if the sonars are enabled (note that if the robot has no sonars at all, this will return false)
  bool areSonarsEnabled(void) const {
    return (myFlags &
	    (ArUtil::BIT1 | ArUtil::BIT2 | ArUtil::BIT3 | ArUtil::BIT4)); }
  /// returns true if the estop is pressed (or unrelieved)
  bool isEStopPressed(void) const { return (myFlags & ArUtil::BIT5); }
  /// Gets the compass heading from the robot
  double getCompass(void) const { return myCompass; }
  /// Gets which analog port is selected
  int getAnalogPortSelected(void) const { return myAnalogPortSelected; }
  /// Gets the analog value
  unsigned char getAnalog(void) const { return myAnalog; }
  /// Gets the byte representing digital input status
  unsigned char getDigIn(void) const { return myDigIn; }
  /// Gets the byte representing digital output status
  unsigned char getDigOut(void) const { return myDigOut; }
  /// Gets the charge state of the robot (see long docs)
  AREXPORT ChargeState getChargeState(void) const;

  /// Gets the number of bytes in the analog IO buffer
  int getIOAnalogSize(void) const { return myIOAnalogSize; }
  /// Gets the number of bytes in the digital input IO buffer
  int getIODigInSize(void) const { return myIODigInSize; }
  /// Gets the number of bytes in the digital output IO buffer
  int getIODigOutSize(void) const { return myIODigOutSize; }

  /// Gets the n'th byte from the analog input data from the IO packet
  AREXPORT int getIOAnalog(int num) const;
  /// Gets the n'th byte from the analog input data from the IO packet
  AREXPORT double getIOAnalogVoltage(int num) const;
  /// Gets the n'th byte from the digital input data from the IO packet
  AREXPORT unsigned char getIODigIn(int num) const;
  /// Gets the n'th byte from the digital output data from the IO packet
  AREXPORT unsigned char getIODigOut(int num) const;

  /// Gets whether the robot has table sensing IR or not (see params in docs)
  bool hasTableSensingIR(void) const { return myParams->haveTableSensingIR(); }
  /// Returns true if the left table sensing IR is triggered
  AREXPORT bool isLeftTableSensingIRTriggered(void) const;
  /// Returns true if the right table sensing IR is triggered
  AREXPORT bool isRightTableSensingIRTriggered(void) const;
  /// Returns true if the left break beam IR is triggered
  AREXPORT bool isLeftBreakBeamTriggered(void) const;
  /// Returns true if the right break beam IR is triggered
  AREXPORT bool isRightBreakBeamTriggered(void) const;
  /// Returns the time received of the last IO packet
  ArTime getIOPacketTime(void) const { return myLastIOPacketReceivedTime; }

  /// Returns true if the E-Stop button is pressed
  /*AREXPORT*/ bool getEstop(void) { return (myFlags & ArUtil::BIT5); }

  /// Gets whether the robot has front bumpers (see ARCOS parameters in the robot manual)
  bool hasFrontBumpers(void) const { return myParams->haveFrontBumpers(); }
  /// Get the number of the front bumper switches
  unsigned int getNumFrontBumpers(void) const
    { return myParams->numFrontBumpers(); }
  /// Gets whether the robot has rear bumpers (see ARCOS parameters in the robot manual)
  bool hasRearBumpers(void) const { return myParams->haveRearBumpers(); }
  /// Gets the number of  rear bumper switches
  unsigned int getNumRearBumpers(void) const
    { return myParams->numRearBumpers(); }

  /** @brief Get the position of the robot according to the last robot SIP,
   * possibly with gyro correction if installed and enabled.
   * @sa getPose()
   * @sa getRawEncoderPose()
   */
  ArPose getEncoderPose(void) const { return myEncoderPose; }

  /// Gets if the robot is trying to move or not
  /**
     This is so that if the robot is trying to move, but is prevented
     (mainly in actions) there'll still be some indication that the
     robot is trying to move so that things like the sonar aren't
     turned off because the robot's stopped, note that this flag
     doesn't have anything to do with if the robot is really moving or
     not and that you'll still want to check the vels for that
     @sa forceTryingToMove
   **/
  bool isTryingToMove(void) { return myTryingToMove; }
  /// Sets the flag that says the robot is trying to move
  /**
     This is so that things that might move can make the robot look
     like it is trying to move.  For example, so teleop mode can say
     that it might make the robot move, so the sonar aren't turned off
     @sa isTryingToMove
   **/
  void forceTryingToMove(void) { myTryingToMove = true; }


  /// Gets the number of motor packets received in the last second
  AREXPORT int getMotorPacCount(void) const;
  /// Gets the number of sonar returns received in the last second
  AREXPORT int getSonarPacCount(void) const;

  /// Gets the range of the last sonar reading for the given sonar
  AREXPORT int getSonarRange(int num) const;
  /// Find out if the given sonar reading was newly refreshed by the last incoming SIP received.
  AREXPORT bool isSonarNew(int num) const;
  /// Find the number of sonar sensors (that the robot has yet returned values for)
  /*AREXPORT*/ int getNumSonar(void) const { return myNumSonar; }
  /// Returns the sonar reading for the given sonar
  AREXPORT ArSensorReading *getSonarReading(int num) const;
  /// Returns the closest of the current sonar reading in the given range
  AREXPORT int getClosestSonarRange(double startAngle, double endAngle) const;
  /// Returns the number of the sonar that has the closest current reading in the given range
  AREXPORT int getClosestSonarNumber(double startAngle, double endAngle) const;

  /// Gets the robots name in ARIAs list
  AREXPORT const char *getName(void) const;
  /// Sets the robots name in ARIAs list
  AREXPORT void setName(const char *name);

  /// Change stored pose (i.e. the value returned by getPose())
  AREXPORT void moveTo(ArPose pose, bool doCumulative = true);
  /// Change stored pose (i.e. the value returned by getPose())
  AREXPORT void moveTo(ArPose to, ArPose from, bool doCumulative = true);

  /// Gets the number of readings the battery voltage is the average of
  size_t getBatteryVoltageAverageOfNum(void)
    { return myBatteryAverager.getNumToAverage(); }

  /// Sets the number of readings the battery voltage is the average of (default 20)
  void setBatteryVoltageAverageOfNum(size_t numToAverage)
    { myBatteryAverager.setNumToAverage(numToAverage); }

  /// Gets the number of readings the battery voltage is the average of
  size_t getRealBatteryVoltageAverageOfNum(void)
    { return myRealBatteryAverager.getNumToAverage(); }

  /// Sets the number of readings the real battery voltage is the average of (default 20)
  void setRealBatteryVoltageAverageOfNum(size_t numToAverage)
    { myRealBatteryAverager.setNumToAverage(numToAverage); }

  /// Starts a continuous stream of encoder packets
  AREXPORT void requestEncoderPackets(void);

  /// Starts a continuous stream of IO packets
  AREXPORT void requestIOPackets(void);

  /// Stops a continuous stream of encoder packets
  AREXPORT void stopEncoderPackets(void);

  /// Stops a continuous stream of IO packets
  AREXPORT void stopIOPackets(void);

  /// Gets packet data from the left encoder
  AREXPORT long int getLeftEncoder(void);

  /// Gets packet data from the right encoder
  AREXPORT long int getRightEncoder(void);

  /// Changes the transform
  AREXPORT void setEncoderTransform(ArPose deadReconPos,
				    ArPose globalPos);

  /// Changes the transform directly
  AREXPORT void setEncoderTransform(ArPose transformPos);

  /// Gets the encoder transform
  AREXPORT ArTransform getEncoderTransform(void) const;

  /// This gets the transform from local coords to global coords
  AREXPORT ArTransform getToGlobalTransform(void) const;

  /// This gets the transform for going from global coords to local coords
  AREXPORT ArTransform getToLocalTransform(void) const;

  /// This applies a transform to all the robot range devices and to the sonar
  AREXPORT void applyTransform(ArTransform trans, bool doCumulative = true);

  /// Sets the dead recon position of the robot
  AREXPORT void setDeadReconPose(ArPose pose);

  /// This gets the distance the robot has travelled (mm)
  /*AREXPORT*/ double getOdometerDistance(void) { return myOdometerDistance; }

  /// This gets the number of degrees the robot has turned (deg)
  /*AREXPORT*/ double getOdometerDegrees(void) { return myOdometerDegrees; }

  /// This gets the time since the odometer was reset (sec)
  /*AREXPORT*/ double getOdometerTime(void) { return myOdometerStart.secSince(); }

  /// Resets the odometer
  AREXPORT void resetOdometer(void);

  /// Adds a rangeDevice to the robot's list of them, and set the device's robot pointer
  AREXPORT void addRangeDevice(ArRangeDevice *device);
  /// Remove a range device from the robot's list, by name
  AREXPORT void remRangeDevice(const char *name);
  /// Remove a range device from the robot's list, by instance
  AREXPORT void remRangeDevice(ArRangeDevice *device);

  /// Finds a rangeDevice in the robot's list
  AREXPORT const ArRangeDevice *findRangeDevice(const char *name) const;

  /// Finds a rangeDevice in the robot's list
  AREXPORT ArRangeDevice *findRangeDevice(const char *name);

  /// Gets the range device list
  AREXPORT std::list<ArRangeDevice *> *getRangeDeviceList(void);

  /// Finds whether a particular range device is attached to this robot or not
  AREXPORT bool hasRangeDevice(ArRangeDevice *device) const;

  /// Goes through all the range devices and checks them
  AREXPORT double checkRangeDevicesCurrentPolar(
	  double startAngle, double endAngle, double *angle = NULL,
	  const ArRangeDevice **rangeDevice = NULL,
	  bool useLocationDependentDevices = true) const;

  /// Goes through all the range devices and checks them
  AREXPORT double checkRangeDevicesCumulativePolar(
	  double startAngle, double endAngle, double *angle = NULL,
	  const ArRangeDevice **rangeDevice = NULL,
	  bool useLocationDependentDevices = true) const;


  // Goes through all the range devices and checks them
  AREXPORT double checkRangeDevicesCurrentBox(
	  double x1, double y1, double x2, double y2,
	  ArPose *readingPos = NULL,
	  const ArRangeDevice **rangeDevice = NULL,
	  bool useLocationDependentDevices = true) const;

  // Goes through all the range devices and checks them
  AREXPORT double checkRangeDevicesCumulativeBox(
	  double x1, double y1, double x2, double y2,
	  ArPose *readingPos = NULL,
	  const ArRangeDevice **rangeDevice = NULL,
	  bool useLocationDependentDevices = true) const;

  /// Sets the camera this robot is using
  void setPTZ(ArPTZ *ptz) { myPtz = ptz; }
  /// Sets the camera this robot is using
  ArPTZ *getPTZ(void) { return myPtz; }

  /// Sets the number of milliseconds between state reflection refreshes
  /// if the state has not changed
  AREXPORT void setStateReflectionRefreshTime(int msec);

  /// Sets the number of milliseconds between state reflection refreshes
  /// if the state has not changed
  AREXPORT int getStateReflectionRefreshTime(void) const;

  /// Adds a packet handler to the list of packet handlers
  AREXPORT void addPacketHandler(
	  ArRetFunctor1<bool, ArRobotPacket *> *functor,
	  ArListPos::Pos position = ArListPos::LAST);

  /// Removes a packet handler from the list of packet handlers
  AREXPORT void remPacketHandler(
	  ArRetFunctor1<bool, ArRobotPacket *> *functor);

  /// Adds a connect callback
  AREXPORT void addConnectCB(ArFunctor *functor,
			     ArListPos::Pos position = ArListPos::LAST);
  /// Removes a connect callback
  AREXPORT void remConnectCB(ArFunctor *functor);

  /// Adds a callback for when a connection to the robot is failed
  AREXPORT void addFailedConnectCB(ArFunctor *functor,
				   ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback for when a connection to the robot is failed
  AREXPORT void remFailedConnectCB(ArFunctor *functor);

  /// Adds a callback for when disconnect is called while connected
  AREXPORT void addDisconnectNormallyCB(ArFunctor *functor,
				ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback for when disconnect is called while connected
  AREXPORT void remDisconnectNormallyCB(ArFunctor *functor);

  /// Adds a callback for when disconnection happens because of an error
  AREXPORT void addDisconnectOnErrorCB(ArFunctor *functor,
				   ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback for when disconnection happens because of an error
  AREXPORT void remDisconnectOnErrorCB(ArFunctor *functor);

  /// Adds a callback for when the run loop exits for what ever reason
  AREXPORT void addRunExitCB(ArFunctor *functor,
			     ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback for when the run loop exits for what ever reason
  AREXPORT void remRunExitCB(ArFunctor *functor);

  /// Suspend calling thread until the ArRobot is connected
  AREXPORT WaitState waitForConnect(unsigned int msecs=0);
  /// Suspend calling thread until the ArRobot is connected or fails to connect
  AREXPORT WaitState waitForConnectOrConnFail(unsigned int msecs=0);
  /// Suspend calling thread until the ArRobot run loop has exited
  AREXPORT WaitState waitForRunExit(unsigned int msecs=0);

  /// Wake up all threads waiting on this robot
  AREXPORT void wakeAllWaitingThreads();
  /// Wake up all threads waiting for connection
  AREXPORT void wakeAllConnWaitingThreads();
  /// Wake up all threads waiting for connection or connection failure
  AREXPORT void wakeAllConnOrFailWaitingThreads();
  /// Wake up all threads waiting for the run loop to exit
  AREXPORT void wakeAllRunExitWaitingThreads();

  /// Adds a user task to the list of synchronous taskes
  AREXPORT bool addUserTask(const char *name, int position,
			       ArFunctor *functor,
			       ArTaskState::State *state = NULL);
  /// Removes a user task from the list of synchronous taskes by name
  AREXPORT void remUserTask(const char *name);
  /// Removes a user task from the list of synchronous taskes by functor
  AREXPORT void remUserTask(ArFunctor *functor);

  /// Finds a user task by name
  AREXPORT ArSyncTask *findUserTask(const char *name);
  /// Finds a user task by functor
  AREXPORT ArSyncTask *findUserTask(ArFunctor *functor);

  /// Logs the list of user tasks, strictly for your viewing pleasure
  AREXPORT void logUserTasks(void) const;
  /// Logs the list of all tasks, strictly for your viewing pleasure
  AREXPORT void logAllTasks(void) const;

  /// Adds a task under the sensor interp part of the syncronous tasks
  AREXPORT bool addSensorInterpTask(const char *name, int position,
				       ArFunctor *functor,
	       			       ArTaskState::State *state = NULL);
  /// Removes a sensor interp tasks by name
  AREXPORT void remSensorInterpTask(const char *name);
  /// Removes a sensor interp tasks by functor
  AREXPORT void remSensorInterpTask(ArFunctor *functor);

  /// Finds a task by name
  AREXPORT ArSyncTask *findTask(const char *name);
  /// Finds a task by functor
  AREXPORT ArSyncTask *findTask(ArFunctor *functor);

  /// Adds an action to the list with the given priority
  AREXPORT bool addAction(ArAction *action, int priority);
  /// Removes an action from the list, by pointer
  AREXPORT bool remAction(ArAction *action);
  /// Removes an action from the list, by name
  AREXPORT bool remAction(const char *actionName);
  /// Returns the first (highest priority) action with the given name (or NULL)
  AREXPORT ArAction *findAction(const char *actionName);
  /// Returns the map of actions... don't do this unless you really
  /// know what you're doing
  AREXPORT ArResolver::ActionMap *getActionMap(void);
  /// Deactivates all the actions
  AREXPORT void deactivateActions(void);

  /// Logs out the actions and their priorities
  AREXPORT void logActions(bool logDeactivated = false) const;

  /// Gets the resolver the robot is using
  AREXPORT ArResolver *getResolver(void);

  /// Sets the resolver the robot is using
  AREXPORT void setResolver(ArResolver *resolver);

  /// Sets the encoderCorrectionCallback
  AREXPORT void setEncoderCorrectionCallback(
	  ArRetFunctor1<double, ArPoseWithTime> *functor);
  /// Gets the encoderCorrectionCallback
  AREXPORT ArRetFunctor1<double, ArPoseWithTime> *
          getEncoderCorrectionCallback(void) const;

  // set up some of the internals of how the ArRobot class works
  /// Sets the number of ms between cycles
  AREXPORT void setCycleTime(unsigned int ms);
  /// Gets the number of ms between cycles
  AREXPORT unsigned int getCycleTime(void) const;
  /// Sets the number of ms between cycles to warn over
  AREXPORT void setCycleWarningTime(unsigned int ms);
  /// Gets the number of ms between cycles to warn over
  AREXPORT unsigned int getCycleWarningTime(void) const;
  /// Gets the number of ms between cycles to warn over
  AREXPORT unsigned int getCycleWarningTime(void);
  /// Sets the multiplier for how many cycles ArRobot waits when connecting
  AREXPORT void setConnectionCycleMultiplier(unsigned int multiplier);
  /// Gets the multiplier for how many cycles ArRobot waits when connecting
  AREXPORT unsigned int getConnectionCycleMultiplier(void) const;

  /// Sets whether to chain the robot cycle to when we get in SIP packets
  void setCycleChained(bool cycleChained) { myCycleChained = cycleChained; }
  /// Gets whether we chain the robot cycle to when we get in SIP packets
  bool isCycleChained(void) const { return myCycleChained; }
  /// Sets the time without a response until connection assumed lost
  AREXPORT void setConnectionTimeoutTime(int mSecs);
  /// Gets the time without a response until connection assumed lost
  AREXPORT int getConnectionTimeoutTime(void) const;
  /// Gets the time the last packet was received
  AREXPORT ArTime getLastPacketTime(void) const;

  /// Sets the number of packets back in time the ArInterpolation goes
  /*AREXPORT*/ void setPoseInterpNumReadings(size_t numReadings)
    { myInterpolation.setNumberOfReadings(numReadings); }

  /// Sets the number of packets back in time the position interpol goes
  /*AREXPORT*/size_t getPoseInterpNumReadings(void) const
    { return myInterpolation.getNumberOfReadings(); }

  /// Gets the position the robot was at at the given timestamp
  /** @see ArInterpolation::getPose
   */
  /*AREXPORT*/int getPoseInterpPosition(ArTime timeStamp, ArPose *position)
    { return myInterpolation.getPose(timeStamp, position); }

  /// Sets the number of packets back in time the ArInterpolation goes for encoder readings
  /*AREXPORT*/void setEncoderPoseInterpNumReadings(size_t numReadings)
    { myEncoderInterpolation.setNumberOfReadings(numReadings); }

  /// Sets the number of packets back in time the encoder position interpolation goes
  /*AREXPORT*/size_t getEncoderPoseInterpNumReadings(void) const
    { return myEncoderInterpolation.getNumberOfReadings(); }

  /// Gets the encoder position the robot was at at the given timestamp
  /** @see ArInterpolation::getPose
   */
  /*AREXPORT*/int getEncoderPoseInterpPosition(ArTime timeStamp, ArPose *position)
    { return myEncoderInterpolation.getPose(timeStamp, position); }

  /// Gets the Counter for the time through the loop
  /*AREXPORT*/ unsigned int getCounter(void) const { return myCounter; }

  /// Gets the parameters the robot is using
  AREXPORT const ArRobotParams *getRobotParams(void) const;

  /// Gets the original robot config packet information
  AREXPORT const ArRobotConfigPacketReader *getOrigRobotConfig(void) const;

  /// Sets the maximum translational velocity
  AREXPORT void setTransVelMax(double vel);
  /// Sets the translational acceleration
  AREXPORT void setTransAccel(double acc);
  /// Sets the translational acceleration
  AREXPORT void setTransDecel(double decel);
  /// Sets the maximum rotational velocity
  AREXPORT void setRotVelMax(double vel);
  /// Sets the rotational acceleration
  AREXPORT void setRotAccel(double acc);
  /// Sets the rotational acceleration
  AREXPORT void setRotDecel(double decel);

  /// If the robot has settable maximum velocities
  bool hasSettableVelMaxes(void) const
    { return myParams->hasSettableVelMaxes(); }
  /// Gets the maximum translational velocity
  AREXPORT double getTransVelMax(void) const;
  /// Gets the maximum rotational velocity
  AREXPORT double getRotVelMax(void) const;
  /// If the robot has settable accels and decels
  bool hasSettableAccsDecs(void)
      const { return myParams->hasSettableAccsDecs(); }
  /// Gets the translational acceleration
  AREXPORT double getTransAccel(void) const;
  /// Gets the translational acceleration
  AREXPORT double getTransDecel(void) const;
  /// Gets the rotational acceleration
  AREXPORT double getRotAccel(void) const;
  /// Gets the rotational acceleration
  AREXPORT double getRotDecel(void) const;

  /// Loads a parameter file (replacing all other params)
  AREXPORT bool loadParamFile(const char *file);

  /// Attachs a key handler
  AREXPORT void attachKeyHandler(ArKeyHandler *keyHandler,
				 bool exitOnEscape = true,
				 bool useExitNotShutdown = true);
  /// Gets the key handler attached to this robot
  AREXPORT ArKeyHandler *getKeyHandler(void) const;

  /// Lock the robot instance
  /*AREXPORT*/ int lock() {return(myMutex.lock());}
  /// Try to lock the robot instance without blocking
  /*AREXPORT*/ int tryLock() {return(myMutex.tryLock());}
  /// Unlock the robot instance
  /*AREXPORT*/ int unlock() {return(myMutex.unlock());}

  /// This tells us if we're in the preconnection state
  /*AREXPORT*/ bool isStabilizing(void) { return myIsStabilizing; }

  /// How long we should stabilize for in ms (0 disables stabilizing)
  AREXPORT void setStabilizingTime(int mSecs);

  /// How long we stabilize for in ms (0 means no stabilizng)
  AREXPORT int getStabilizingTime(void) const;


  /// Adds a callback called when the robot starts stabilizing before declaring connection
  AREXPORT void addStabilizingCB(ArFunctor *functor,
			     ArListPos::Pos position = ArListPos::LAST);
  /// Removes stabilizing callback
  AREXPORT void remStabilizingCB(ArFunctor *functor);

  /// This gets the root of the syncronous task tree, only serious
  /// developers should use it
  AREXPORT ArSyncTask *getSyncTaskRoot(void);

  /// This function loops once...  only serious developers should use it
  AREXPORT void loopOnce(void);

  /// Sets the delay in the odometry readings
  /**
     Note that this doesn't cause a delay, its informational so that
     the delay can be adjusted for and causes nothing to happen in
     this class.
  **/
  /*AREXPORT*/ void setOdometryDelay(int msec) { myOdometryDelay = msec; }

  /// Gets the delay in odometry readings
  /**
     This gets the odometry delay, not that this is just information
     about what the delay is it doesn't cause anything to happen in
     this class.
  **/
  /*AREXPORT*/ int getOdometryDelay(void) { return myOdometryDelay; }

  /// Gets if we're logging all the movement commands sent down
  /*AREXPORT*/ bool getLogMovementSent(void) { return myLogMovementSent; }
  /// Sets if we're logging all the movement commands sent down
  /*AREXPORT*/ void setLogMovementSent(bool logMovementSent)
    { myLogMovementSent = logMovementSent; }

  /// Gets if we're logging all the positions received from the robot
  /*AREXPORT*/ bool getLogMovementReceived(void) { return myLogMovementReceived; }
  /// Sets if we're logging all the positions received from the robot
  /*AREXPORT*/void setLogMovementReceived(bool logMovementReceived)
    { myLogMovementReceived = logMovementReceived; }

  /// Gets if we're logging all the velocities (and heading) received
  /*AREXPORT*/bool getLogVelocitiesReceived(void)
    { return myLogVelocitiesReceived; }
  /// Sets if we're logging all the velocities (and heading) received
  /*AREXPORT*/void setLogVelocitiesReceived(bool logVelocitiesReceived)
    { myLogVelocitiesReceived = logVelocitiesReceived; }

  /// Gets if we're logging all the packets received (just times and types)
  /*AREXPORT*/bool getPacketsReceivedTracking(void)
    { return myPacketsReceivedTracking; }
  /// Sets if we're logging all the packets received (just times and types)
  AREXPORT void setPacketsReceivedTracking(bool packetsReceivedTracking);

  /// Gets if we're logging all the packets sent and their payload
  /*AREXPORT*/bool getPacketsSentTracking(void)
    { return myPacketsSentTracking; }
  /// Sets if we're logging all the packets received (just times and types)
  /*AREXPORT*/void setPacketsSentTracking(bool packetsSentTracking)
    { myPacketsSentTracking = packetsSentTracking; }

  /// Gets if we're logging all the actions as they happen
  /*AREXPORT*/ bool getLogActions(void) { return myLogActions; }
  /// Sets if we're logging all the movement commands sent down
  /*AREXPORT*/void setLogActions(bool logActions)
  { myLogActions = logActions; }


  /// This is only for use by syncLoop
  /*AREXPORT*/ void incCounter(void) { myCounter++; }

  /// Packet Handler, internal
  AREXPORT void packetHandler(void);
  /// Action Handler, internal
  AREXPORT void actionHandler(void);
  /// State Reflector, internal
  AREXPORT void stateReflector(void);
  /// Robot locker, internal
  AREXPORT void robotLocker(void);
  /// Robot unlocker, internal
  AREXPORT void robotUnlocker(void);

  /// For the key handler, escape calls this to exit, internal
  AREXPORT void keyHandlerExit(void);

  /// Processes a motor packet, internal
  AREXPORT bool processMotorPacket(ArRobotPacket *packet);
  /// Processes a new sonar reading, internal
  AREXPORT void processNewSonar(char number, int range, ArTime timeReceived);
  /// Processes a new encoder packet, internal
  AREXPORT bool processEncoderPacket(ArRobotPacket *packet);
  /// Processes a new IO packet, internal
  AREXPORT bool processIOPacket(ArRobotPacket *packet);

  /// Internal function, shouldn't be used
  AREXPORT void init(void);

  /// Internal function, shouldn't be used, sets up the default sync list
  AREXPORT void setUpSyncList(void);
  /// Internal function, shouldn't be used, sets up the default packet handlers
  AREXPORT void setUpPacketHandlers(void);

  ArRetFunctor1C<bool, ArRobot, ArRobotPacket *> myMotorPacketCB;
  ArRetFunctor1C<bool, ArRobot, ArRobotPacket *> myEncoderPacketCB;
  ArRetFunctor1C<bool, ArRobot, ArRobotPacket *> myIOPacketCB;
  ArFunctorC<ArRobot> myPacketHandlerCB;
  ArFunctorC<ArRobot> myActionHandlerCB;
  ArFunctorC<ArRobot> myStateReflectorCB;
  ArFunctorC<ArRobot> myRobotLockerCB;
  ArFunctorC<ArRobot> myRobotUnlockerCB;
  ArFunctorC<ArRobot> myKeyHandlerExitCB;
  ArFunctorC<ArKeyHandler> *myKeyHandlerCB;

  // These four are internal... only people monkeying deeply should mess
  // with them, so they aren't documented... these process the cblists
  // and such
  /// Internal function, shouldn't be used, does a single run of connecting
  AREXPORT int asyncConnectHandler(bool tryHarderToConnect);

  /// Internal function, shouldn't be used, drops the conn because of error
  AREXPORT void dropConnection(void);
  /// Internal function, shouldn't be used, denotes the conn failed
  AREXPORT void failedConnect(void);
  /// Internal function, shouldn't be used, does the initial conn stuff
  AREXPORT bool madeConnection(void);
  /// Internal function, shouldn't be used, calls the preconnected stuff
  AREXPORT void startStabilization(void);
  /// Internal function, shouldn't be used, does the after conn stuff
  AREXPORT void finishedConnection(void);
  /// Internal function, shouldn't be used, cancels the connection quietly
  AREXPORT void cancelConnection(void);

  /// Internal function, takes a packet and passes it to the packet handlers,
  /// returns true if handled, false otherwise
  AREXPORT bool handlePacket(ArRobotPacket *packet);
  /// Internal function, shouldn't be used, does what its name says
  AREXPORT std::list<ArFunctor *> * getRunExitListCopy();
  /// Internal function, processes a parameter file
  AREXPORT void processParamFile(void);

  /** @brief Get the position of the robot according to the last robot SIP only,
   *  with no correction by the gyro, other devices or software proceses.
   *
   * @note For the most accurate pose, use getPose() or getEncoderPose();
   * only use this method if you must have raw encoder pose with no correction.
   * @sa getPose()
   * @sa getEncoderPose()
   */
  /*AREXPORT*/ ArPose getRawEncoderPose(void) const { return myRawEncoderPose; }

  /// Internal function for sync loop and sync task to see if we should warn this cycle or not
  /*AREXPORT*/ bool getNoTimeWarningThisCycle(void)
    { return myNoTimeWarningThisCycle; }
  /// Internal function for sync loop and sync task to say if we should warn this cycle or not
  /*AREXPORT*/void setNoTimeWarningThisCycle(bool noTimeWarningThisCycle)
    { myNoTimeWarningThisCycle = noTimeWarningThisCycle; }
  // callbacks for warning time and if we should warn now to pass to sync tasks
  ArRetFunctorC<unsigned int, ArRobot> myGetCycleWarningTimeCB;
  ArRetFunctorC<bool, ArRobot> myGetNoTimeWarningThisCycleCB;
  /// internal function called when Aria::exit is called
  AREXPORT void ariaExitCallback(void);
  /// internal call that will let the robot connect even if it can't find params
  void setConnectWithNoParams(bool connectWithNoParams)
    { myConnectWithNoParams = connectWithNoParams; }
protected:
  enum RotDesired {
    ROT_NONE,
    ROT_IGNORE,
    ROT_HEADING,
    ROT_VEL
  };
  enum TransDesired {
    TRANS_NONE,
    TRANS_IGNORE,
    TRANS_VEL,
    TRANS_VEL2,
    TRANS_DIST,
    TRANS_DIST_NEW
  };
  void reset(void);


  // the config the robot had at connection
  ArRobotConfigPacketReader *myOrigRobotConfig;
  // the values we'll maintain for the different motion parameters
  double myRotVelMax;
  double myRotAccel;
  double myRotDecel;
  double myTransVelMax;
  double myTransAccel;
  double myTransDecel;

  ArPTZ *myPtz;
  bool myNoTimeWarningThisCycle;

  long int myLeftEncoder;
  long int myRightEncoder;
  bool myFirstEncoderPose;
  ArPoseWithTime myRawEncoderPose;

  ArTransform myEncoderTransform;

  bool myLogMovementSent;
  bool myLogMovementReceived;
  bool myPacketsReceivedTracking;
  bool myLogActions;
  bool myLogVelocitiesReceived;
  double myLastVel;
  double myLastRotVel;
  double myLastHeading;
  double myLastCalculatedRotVel;

  bool myTryingToMove;

  long myPacketsReceivedTrackingCount;
  ArTime myPacketsReceivedTrackingStarted;
  bool myPacketsSentTracking;
  ArMutex myMutex;
  ArSyncTask *mySyncTaskRoot;
  std::list<ArRetFunctor1<bool, ArRobotPacket *> *> myPacketHandlerList;

  ArSyncLoop mySyncLoop;

  std::list<ArFunctor *> myStabilizingCBList;
  std::list<ArFunctor *> myConnectCBList;
  std::list<ArFunctor *> myFailedConnectCBList;
  std::list<ArFunctor *> myDisconnectNormallyCBList;
  std::list<ArFunctor *> myDisconnectOnErrorCBList;
  std::list<ArFunctor *> myRunExitCBList;

  ArRetFunctor1<double, ArPoseWithTime> *myEncoderCorrectionCB;
  std::list<ArRangeDevice *> myRangeDeviceList;

  ArCondition myConnectCond;
  ArCondition myConnOrFailCond;
  ArCondition myRunExitCond;

  ArResolver::ActionMap myActions;
  bool myOwnTheResolver;
  ArResolver *myResolver;

  std::map<int, ArSensorReading *> mySonars;
  int myNumSonar;

  unsigned int myCounter;
  bool myIsConnected;
  bool myIsStabilizing;

  bool myBlockingConnectRun;
  bool myAsyncConnectFlag;
  int myAsyncConnectState;
  int myAsyncConnectNoPacketCount;
  int myAsyncConnectTimesTried;
  ArTime myAsyncStartedConnection;
  int myAsyncConnectStartBaud;
  ArTime myAsyncConnectStartedChangeBaud;
  bool myAsyncConnectSentChangeBaud;
  ArTime myStartedStabilizing;

  int myStabilizingTime;

  bool mySentPulse;

  double myTransVal;
  double myTransVal2;
  int myLastTransVal;
  int myLastTransVal2;
  TransDesired myTransType;
  TransDesired myLastTransType;
  ArTime myTransSetTime;
  ArTime myLastTransSent;
  int myLastActionTransVal;
  bool myActionTransSet;
  ArPose myTransDistStart;
  double myMoveDoneDist;

  double myRotVal;
  int myLastRotVal;
  RotDesired myRotType;
  RotDesired myLastRotType;
  ArTime myRotSetTime;
  ArTime myLastRotSent;
  int myLastActionRotVal;
  bool myLastActionRotHeading;
  bool myLastActionRotStopped;
  bool myActionRotSet;
  double myHeadingDoneDiff;

  double myLastSentTransVelMax;
  double myLastSentTransAccel;
  double myLastSentTransDecel;
  double myLastSentRotVelMax;
  double myLastSentRotAccel;
  double myLastSentRotDecel;

  ArTime myLastPulseSent;

  int myDirectPrecedenceTime;

  int myStateReflectionRefreshTime;

  ArActionDesired myActionDesired;

  std::string myName;
  std::string myRobotName;
  std::string myRobotType;
  std::string myRobotSubType;

  double myAbsoluteMaxTransVel;
  double myAbsoluteMaxTransAccel;
  double myAbsoluteMaxTransDecel;
  double myAbsoluteMaxRotVel;
  double myAbsoluteMaxRotAccel;
  double myAbsoluteMaxRotDecel;

  ArDeviceConnection *myConn;

  ArRobotPacketSender mySender;
  ArRobotPacketReceiver myReceiver;

  ArRobotParams *myParams;
  double myRobotLengthFront;
  double myRobotLengthRear;

  ArInterpolation myInterpolation;
  ArInterpolation myEncoderInterpolation;

  ArKeyHandler *myKeyHandler;
  bool myKeyHandlerUseExitNotShutdown;

  bool myConnectWithNoParams;
  bool myWarnedAboutExtraSonar;

  // variables for tracking the data stream
  time_t myTimeLastMotorPacket;
  int myMotorPacCurrentCount;
  int myMotorPacCount;
  time_t myTimeLastSonarPacket;
  int mySonarPacCurrentCount;
  int mySonarPacCount;
  unsigned int myCycleTime;
  unsigned int myCycleWarningTime;
  unsigned int myConnectionCycleMultiplier;
  bool myCycleChained;
  ArTime myLastPacketReceivedTime;
  int myTimeoutTime;

  bool myRequestedIOPackets;
  bool myRequestedEncoderPackets;

  // all the state reflecing variables
  ArPoseWithTime myEncoderPose;
  ArTime myEncoderPoseTaken;
  ArPose myGlobalPose;
  // um, this myEncoderGlobalTrans doesn't do anything
  ArTransform myEncoderGlobalTrans;
  double myLeftVel;
  double myRightVel;
  double myBatteryVoltage;
  ArRunningAverage myBatteryAverager;
  double myRealBatteryVoltage;
  ArRunningAverage myRealBatteryAverager;
  int myStallValue;
  double myControl;
  int myFlags;
  int myFaultFlags;
  bool myHasFaultFlags;
  double myCompass;
  int myAnalogPortSelected;
  unsigned char myAnalog;
  unsigned char myDigIn;
  unsigned char myDigOut;
  int myIOAnalog[128];
  unsigned char myIODigIn[255];
  unsigned char myIODigOut[255];
  int myIOAnalogSize;
  int myIODigInSize;
  int myIODigOutSize;
  ArTime myLastIOPacketReceivedTime;
  double myVel;
  double myRotVel;
  int myLastX;
  int myLastY;
  int myLastTh;
  ChargeState myChargeState;

  double myOdometerDistance;
  double myOdometerDegrees;
  ArTime myOdometerStart;

  int myOdometryDelay;

  bool myAddedAriaExitCB;
  ArFunctorC<ArRobot> myAriaExitCB;
};


#endif // ARROBOT_H
