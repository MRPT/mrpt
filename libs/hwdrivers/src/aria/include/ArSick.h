/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSICK_H
#define ARSICK_H

#include "ariaTypedefs.h"
#include "ArSickPacket.h"
#include "ArSickPacketReceiver.h"
#include "ArRobotPacket.h"
#include "ArRangeDeviceThreaded.h"
#include "ArFunctor.h"
#include "ArCondition.h"

/// Interface to a SICK laser range device
/**
 * This class processes incoming data from a SICK
 * laser rangefinding device in a background thread, and provides
 * it through the standard ArRangeDevice API, to be used via ArRobot
 * (see ArRobot::addRangeDevice()), used by an ArAction, or used directly.
 *
 * An ArSick instance must be connected to the laser through a serial port
 * (or simulator): the typical procedure is to allow your ArSimpleConnector
 * to configure the laser based on the robot connection type and command
 * line parameters; then initiate the ArSick background thread; and finally
 * connect ArSick to the laser device.
 * For example:
 * @code
 *  ArRobot robot;
 *  ArSick laser;
 *  ArSimpleConnector connector(...);
 *  ...
 *   Setup the simple connector and connect to the robot --
 *   see the example programs.
 *  ...
 *  connector.setupLaser(&laser);
 *  laser.runAsync();
 *  if(!laser.blockingConnect())
 *  {
 *    // Error...
 *    ...
 *  }
 *  ...
 * @endcode
 *
 * The most important methods in this class are the constructor, runAsync(),
 * blockingConnect(), getSensorPosition(), isConnected(), addConnectCB(),
 * asyncConnect(), configure(), in addition to the ArRangeDevice interface.
 *
 * @note The "extra int" on the raw readings returned by
 * ArRangeDevice::getRawReadings() is like other laser
 * devices and is the reflectance value, if enabled, ranging between 0 and 255.
**/
class ArSick : public ArRangeDeviceThreaded
{
public:
  enum BaudRate {
    BAUD9600, ///< 9600 Baud
    BAUD19200, ///< 19200 Baud
    BAUD38400 ///< 38400 Baud
  };
  enum Degrees {
    DEGREES180, ///< 180 Degrees
    DEGREES100 ///< 100 Degrees
  };
  enum Increment {
    INCREMENT_ONE, ///< One degree increments
    INCREMENT_HALF ///< Half a degree increments
  };
  enum Units {
    UNITS_1MM, ///< Uses 1 mm resolution (8/16/32 meter max range)
    UNITS_1CM, ///< Uses 1 cm resolution (80/160/320 meter max range)
    UNITS_10CM ///< Uses 10 cm resolution (150 meter max range)
  };
  enum Bits {
    BITS_1REFLECTOR, ///< Uses 1 reflector bits (means 32/320/150 meter range)
    BITS_2REFLECTOR, ///< Uses 2 reflector bits (means 16/160/150 meter range)
    BITS_3REFLECTOR ///< Uses 3 reflector bits (means 8/80/150 meter range)
  };
  /// Constructor
  AREXPORT ArSick(size_t currentBufferSize = 361,
		  size_t cumulativeBufferSize = 0,
		  const char *name = "laser",
		  bool addAriaExitCB = true);
  /// Destructor
  AREXPORT virtual ~ArSick();
  /// Use this to manually configure the laser before connecting to it
  AREXPORT void configure(bool useSim = false, bool powerControl = true,
			  bool laserFlipped = false,
			  BaudRate baud = BAUD38400,
			  Degrees deg = DEGREES180,
			  Increment incr = INCREMENT_ONE);
  /// Shorter configure for the laser (settings are overridden by the .p file)
  AREXPORT void configureShort(bool useSim = false,
			       BaudRate baud = BAUD38400,
			       Degrees deg = DEGREES180,
			       Increment incr = INCREMENT_ONE);
  /// Sets the ranging/reflector information
  AREXPORT void setRangeInformation(Bits bits = BITS_1REFLECTOR,
				    Units units = UNITS_1MM);
  /// Sets the position of the laser on the robot
  AREXPORT void setSensorPosition(double x, double y, double th);
  /// Sets the position of the laser on the robot
  AREXPORT void setSensorPosition(ArPose pose);
  /// Gets the position of the laser on the robot
  AREXPORT ArPose getSensorPosition();
  /// Gets the X position of the laser on the robot
  AREXPORT double getSensorPositionX();
  /// Gets the Y position of the laser on the robot
  AREXPORT double getSensorPositionY();
  /// Gets the heading of the laser on the robot
  AREXPORT double getSensorPositionTh();
  /// Connect to the laser while blocking
  AREXPORT bool blockingConnect(void);
  /// Connect to the laser asyncronously
  AREXPORT bool asyncConnect(void);
  /// Disconnect from the laser
  AREXPORT bool disconnect(bool doNotLockRobotForSim = false);
  /// Sets the device connection
  AREXPORT void setDeviceConnection(ArDeviceConnection *conn);
  /// Gets the device connection
  AREXPORT ArDeviceConnection *getDeviceConnection(void);
  /// Sees if this is connected to the laser
  bool isConnected(void)
    { if (myState == STATE_CONNECTED) return true; else return false; }
  /// Sees if this is trying to connect to the laser at the moment
  bool tryingToConnect(void)
    { if (myState != STATE_CONNECTED && myState != STATE_NONE)
      return true; else return false; }

  /// Gets the minimum range for this device (defaults to 100 mm)
  /*AREXPORT*/ unsigned int getMinRange(void) { return myMinRange; }
  /// Sets the maximum range for this device (defaults to 100 mm)
  /*AREXPORT*/ void setMinRange(unsigned int minRange) { myMinRange = minRange; }
  /// Adds a degree at which to ignore readings (within 1 degree)
  /*AREXPORT*/void addIgnoreReading(double ignoreReading)
               { myIgnoreReadings.push_back(ignoreReading); }
  /// Clears the degrees we ignore readings at
  /*AREXPORT*/ void clearIgnoreReadings(void) { myIgnoreReadings.clear(); }
  /// Gets the list of readings that we ignore
  /*AREXPORT*/const std::list<double> *getIgnoreReadings(void) const
             { return &myIgnoreReadings; }
  /// Current readings closer than this are discarded as too close
  AREXPORT void setFilterNearDist(double dist);
  /// Current readings closer than this are discarded as too close
  AREXPORT double getFilterNearDist(void);

  /// You should see setMaxDistToKeepCumulative (it replaces this)
  /*AREXPORT*/void setFilterCumulativeMaxDist(double dist)
    { setMaxDistToKeepCumulative(dist); }
  /// You should see getMaxDistToKeepCumulative (it replaces this)
  /*AREXPORT*/double getFilterCumulativeMaxDist(void)
    { return getMaxDistToKeepCumulative(); }
  /// Cumulative readings must be taken within this distance to the robot to be added
  AREXPORT void setFilterCumulativeInsertMaxDist(double dist);
  /// Cumulative readings must be taken within this distance to the robot to be added
  AREXPORT double getFilterCumulativeInsertMaxDist(void);
  /// Cumulative readings closer than this are discarded as too close
  AREXPORT void setFilterCumulativeNearDist(double dist);
  /// Cumulative readings closer than this are discarded as too close
  AREXPORT double getFilterCumulativeNearDist(void);
  /// Cumulative readings that are this close to current beams are discarded
  AREXPORT void setFilterCumulativeCleanDist(double dist);
  /// Cumulative readings that are this close to current beams are discarded
  AREXPORT double getFilterCumulativeCleanDist(void);
  /// Cumulative readings are cleaned every this number of milliseconds
  AREXPORT void setFilterCleanCumulativeInterval(int milliSeconds);
  /// Cumulative readings are cleaned every this number of milliseconds
  AREXPORT int getFilterCleanCumulativeInterval(void);
  /** you should use setMaxSecondsToKeepCumulative instead, (replaces this)
   *  @deprecated
   */
  void setFilterCumulativeMaxAge(int seconds)
    { setMaxSecondsToKeepCumulative(seconds); }
  /** you should use getMaxSecondsToKeepCumulative instead, (replaces this)
   *  @deprecated
   */
  int getFilterCumulativeMaxAge(void)
    { return getMaxSecondsToKeepCumulative(); }

  /// Runs the laser off of the robot
  AREXPORT bool runOnRobot(void);


  /// Gets the number of laser packets received in the last second
  AREXPORT int getSickPacCount();

  /// Adds a connect callback
  AREXPORT void addConnectCB(ArFunctor *functor,
			     ArListPos::Pos position = ArListPos::LAST);
  /// Adds a disconnect callback
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

  /// Adds a callback that is called whenever a laser packet is processed
  AREXPORT void addDataCB(ArFunctor *functor,
			       ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback that is called whenever a laser packet is processed
  AREXPORT void remDataCB(ArFunctor *functor);

  /// Sets the time without a response until connection assumed lost
  AREXPORT void setConnectionTimeoutTime(int mSecs);
  /// Gets the time without a response until connection assumed lost
  AREXPORT int getConnectionTimeoutTime(void);

  /// Gets the time data was last receieved
  AREXPORT ArTime getLastReadingTime(void);


  /// Gets whether the laser is simulated or not
  AREXPORT bool isUsingSim(void);
  /// Gets whether the computer is controling laser power or not
  AREXPORT bool isControllingPower(void);
  /// Gets whether the laser is flipped over or not
  AREXPORT bool isLaserFlipped(void);
  /// Gets the degrees the laser is scanning
  AREXPORT Degrees getDegrees(void);
  /// Gets the amount each scan increments
  AREXPORT Increment getIncrement(void);
  /// Gets the Bits the laser is using
  AREXPORT Bits getBits(void);
  /// Gets the Units the laser is using
  AREXPORT Units getUnits(void);

  /// Sets whether the laser is simulated or not
  AREXPORT void setIsUsingSim(bool usingSim);
  /// Sets whether the computer is controling laser power or not
  AREXPORT void setIsControllingPower(bool controlPower);
  /// Sets whether the laser is flipped over or not
  AREXPORT void setIsLaserFlipped(bool laserFlipped);

  /** The packet handler for when connected to the simulator
   *  @internal
   */
  AREXPORT bool simPacketHandler(ArRobotPacket * packet);
  /** The function called if the laser isn't running in its own thread and
   *  isn't simulated
   *  @internal
   */
  AREXPORT void sensorInterpCallback(void);
  /** An internal function
   *  @internal
   */
  AREXPORT bool internalConnectSim(void);
  /// An internal function, single loop event to connect to laser
  AREXPORT int internalConnectHandler(void);
  /** The internal function used by the ArRangeDeviceThreaded
   *  @internal
   */
  AREXPORT virtual void * runThread(void *arg);
  /** The internal function which processes the sickPackets
   *  @internal
   */
  AREXPORT void processPacket(ArSickPacket *packet, ArPose pose,
			      ArPose encoderPose, unsigned int counter,
			      bool deinterlace, ArPose deinterlaceDelta);
  /** The internal function that gets does the work
   *  @internal
   */
  AREXPORT void runOnce(bool lockRobot);
  AREXPORT virtual void setRobot(ArRobot *robot);
  /** Internal function, shouldn't be used, drops the conn because of error
   *  @internal
   */
  AREXPORT void dropConnection(void);
  /** Internal function, shouldn't be used, denotes the conn failed
   *  @internal
   */
  AREXPORT void failedConnect(void);
  /** Internal function, shouldn't be used, does the after conn stuff
   *  @internal
   */
  AREXPORT void madeConnection(void);
  /** Internal function, shouldn't be used, gets params from the robot
   *  @internal
   */
  AREXPORT void robotConnectCallback(void);
  /// Applies a transform to the buffers
  AREXPORT virtual void applyTransform(ArTransform trans,
                                        bool doCumulative = true);
protected:
  enum State {
    STATE_NONE, ///< Nothing, haven't tried to connect or anything
    STATE_INIT, ///< Initializing the laser
    STATE_WAIT_FOR_POWER_ON, ///< Waiting for power on
    STATE_CHANGE_BAUD, ///< Change the baud, no confirm here
    STATE_CONFIGURE, ///< Send the width and increment to the laser
    STATE_WAIT_FOR_CONFIGURE_ACK, ///< Wait for the configuration Ack
    STATE_INSTALL_MODE, ///< Switch to install mode
    STATE_WAIT_FOR_INSTALL_MODE_ACK, ///< Wait until its switched to install mode
    STATE_SET_MODE, ///< Set the mode (mm/cm) and extra field bits
    STATE_WAIT_FOR_SET_MODE_ACK, ///< Waiting for set-mode ack
    STATE_START_READINGS, ///< Switch to monitoring mode
    STATE_WAIT_FOR_START_ACK, ///< Waiting for the switch-mode ack
    STATE_CONNECTED ///< We're connected and getting readings
  };
  /// Internal function for filtering the raw readings and updating buffers
  void filterReadings();
  /// Internal function for managing the cumulative
  void filterAddAndCleanCumulative(double x, double y, bool clean);
  /// Internal function for managing the cumulative
  void filterFarAndOldCumulative(void);
  /// Internal function for switching states
  AREXPORT void switchState(State state);
  /// Readings we ignore
  std::list<double> myIgnoreReadings;
  State myState;
  unsigned int myMinRange;
  ArTime myStateStart;
  ArFunctorC<ArSick> myRobotConnectCB;
  ArRetFunctor1C<bool, ArSick, ArRobotPacket *> mySimPacketHandler;
  ArFunctorC<ArSick> mySensorInterpCB;
  std::list<ArSensorReading *>::iterator myIter;
  bool myStartConnect;
  bool myRunningOnRobot;
  int myTimeoutTime;

  // range buffers to hold current range set and assembling range set
  std::list<ArSensorReading *> *myAssembleReadings;
  std::list<ArSensorReading *> *myCurrentReadings;

  bool myProcessImmediately;
  bool myInterpolation;
  // list of packets, so we can process them from the sensor callback
  std::list<ArSickPacket *> myPackets;

  // a lock for the device (so we are mostly unlocked for people to get data,
  // but so we don't have people trying to read multiple times from the same
  // device
  ArMutex myConnLock;

  // whether we did a real configure or a short configure
  bool myRealConfigured;

  // these two are just for the sim packets
  unsigned int myWhichReading;
  unsigned int myTotalNumReadings;

  // some variables so we don't have to do a tedios if every time
  double myOffsetAmount;
  double myIncrementAmount;

  // packet stuff
  ArSickPacket myPacket;
  bool myRunInOwnThread;
  bool myUseSim;
  bool myLaserFlipped;
  ArPose mySensorPose;
  bool myPowerControl;
  BaudRate myBaud;
  Degrees myDegrees;
  Increment myIncrement;
  Units myUnits;
  Bits myBits;
  ArTime myLastReading;

  // stuff for the sim packet
  ArPose mySimPacketStart;
  ArTransform mySimPacketTrans;
  ArTransform mySimPacketEncoderTrans;
  unsigned int mySimPacketCounter;

  // packet count
  time_t myTimeLastSickPacket;
  int mySickPacCurrentCount;
  int mySickPacCount;

  // connection
  ArSickPacketReceiver mySickPacketReceiver;
  ArDeviceConnection *myConn;
  std::list<ArFunctor *> myConnectCBList;
  std::list<ArFunctor *> myFailedConnectCBList;
  std::list<ArFunctor *> myDisconnectNormallyCBList;
  std::list<ArFunctor *> myDisconnectOnErrorCBList;
  std::list<ArFunctor *> myDataCBList;

  ArTime myLastCleanedCumulative;
  // filtering parameters, see the docs in the acessor
  double myFilterNearDist;
  double myFilterCumulativeInsertMaxDist;
  double myFilterSquaredCumulativeInsertMaxDist;
  double myFilterCumulativeNearDist;
  double myFilterSquaredCumulativeNearDist;
  double myFilterCumulativeCleanDist;
  double myFilterSquaredCumulativeCleanDist;
  int myFilterCleanCumulativeInterval;

  ArMutex myStateMutex;
  ArRetFunctor1C<bool, ArSick, bool> myAriaExitCB;
};

#endif //ARSICK_H
