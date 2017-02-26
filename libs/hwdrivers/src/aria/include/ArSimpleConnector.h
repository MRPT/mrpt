/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSIMPLECONNECTOR_H
#define ARSIMPLECONNECTOR_H

#include "ariaTypedefs.h"
#include "ArSerialConnection.h"
#include "ArTcpConnection.h"
#include "ArArgumentBuilder.h"
#include "ArArgumentParser.h"
#include "ariaUtil.h"
#include "ArSick.h"

class ArRobot;



/// Connect to robot and laser based on run-time availablitily and command-line arguments
/**
   ArSimpleConnector makes a robot connection either through a TCP
   port (for the simulator or for robots with Ethernet-serial bridge
   devices instead of onboard computers), or through a direct serial 
   port connection.  Normally, it first attempts a TCP connection on 
   @a localhost port 8101, to use a simulator if running. If the simulator
   is not running, then it normally then connects using the serial port
   (the first serial port, COM1, by default).  Various other connection
   parameters are configurable through command-line arguments.
  
   When you create your ArSimpleConnector, pass it command line parameters via
   either the argc and argv variables from main(), or pass it an
   ArArgumentBuilder or ArArgumentParser object. (ArArgumentBuilder
   is able to obtain command line parameters from a Windows program
   that uses WinMain() instead of main()).
   ArSimpleConnector registers a callback with the global Aria class. Use
   Aria::parseArgs() to parse all command line parameters to the program, and
   Aria::logOptions() to print out information about all registered command-line parameters.

   See the documentation
   for parseArgs() for a list of recognized command line parameters.

   You can prepare an ArRobot object for connection (with various connection
   options configured via the command line parameters) and initiate the connection
   attempt by that object by calling connectRobot().
    
   After it's connected, you must then begin the robot processing cycle by calling
   ArRobot::runAsync() or ArRobot::run().

   You can then configure ArSimpleConnector for the SICK laser based on the robot connection, and 
   command line parameters with setupLaser(). After calling setupLaser(),
   you must then run the laser processing thread (with ArSick::runAsync() or
   ArSick()::run()) and then use ArSimpleConnector::connectLaser() to connect
   with the laser if specifically requested on the command line using the -connectLaser option
   (or simply call ArSick::blockingConnect() (or similar) to attempt a laser connection regardless
   of whether or not the -connectLaser option was given; use this latter technique if your program 
   always prefers or requires use of the laser).


 **/
class ArSimpleConnector
{
public:
  /// Constructor that takes args from the main
  AREXPORT ArSimpleConnector(int *argc, char **argv);
  /// Constructor that takes argument builder
  AREXPORT ArSimpleConnector(ArArgumentBuilder *arguments);
  /// Constructor that takes argument parser
  AREXPORT ArSimpleConnector(ArArgumentParser *parser);
  /// Destructor
  AREXPORT ~ArSimpleConnector(void);
  /// Sets up the robot to be connected
  AREXPORT bool setupRobot(ArRobot *robot);
  /// Sets up the robot then connects it
  AREXPORT bool connectRobot(ArRobot *robot);
  /// Sets up the laser to be connected
  AREXPORT bool setupLaser(ArSick *sick);
  /// Sets up a second laser to be connected
  AREXPORT bool setupSecondLaser(ArSick *sick);
  /// Sets up a laser t obe connected (make sure you setMaxNumLasers)
  AREXPORT bool setupLaserArbitrary(ArSick *sick, int laserNumber);
  /// Connects the laser synchronously (will take up to a minute)
  AREXPORT bool connectLaser(ArSick *sick);
  /// Connects the laser synchronously (will take up to a minute)
  AREXPORT bool connectSecondLaser(ArSick *sick);
  /// Connects the laser synchronously  (make sure you setMaxNumLasers)
  AREXPORT bool connectLaserArbitrary(ArSick *sick, int laserNumber);
  /// Function to parse the arguments given in the constructor
  AREXPORT bool parseArgs(void);
  /// Function to parse the arguments given in an arbitrary parser
  AREXPORT bool parseArgs(ArArgumentParser *parser);
  /// Log the options the simple connector has
  AREXPORT void logOptions(void) const;
  /// Sets the number of possible lasers 
  AREXPORT void setMaxNumLasers(int maxNumLasers = 1);
protected:
/// Class that holds information about the laser data
  class LaserData
  {
  public:
    LaserData(int number) 
      { myNumber = number; myConnect = false; myPort = NULL; 
      myRemoteTcpPort = 0; myFlipped = false; myFlippedReallySet = false; 
      myPowerControlled = true; myPowerControlledReallySet = false; 
      myDegrees = NULL; mySickDegrees = ArSick::DEGREES180; myIncrement = NULL;
      mySickIncrement = ArSick::INCREMENT_ONE; myUnits = NULL; 
      mySickUnits = ArSick::UNITS_1MM; myBits = NULL; 
      mySickBits = ArSick::BITS_1REFLECTOR; }
    virtual ~LaserData() {}
    int myNumber;
    // if we want to connect the laser
    bool myConnect;
    // the port we want to connect the laser on
    const char *myPort;
    // laser tcp port if we're doing a remote host
    int myRemoteTcpPort;  
    // if we have the laser flipped
    bool myFlipped;
    // if our flipped was really set
    bool myFlippedReallySet;
    // if we are controlling the laser power
    bool myPowerControlled;
    // if our flipped was really set
    bool myPowerControlledReallySet;
    // the degrees we want to use (for the sick)
    const char *myDegrees;
    // the enum value for those degrees
    ArSick::Degrees mySickDegrees;
    // the increment we want to use
    const char *myIncrement;
    // the angular res we want to use (the sick value)
    ArSick::Increment mySickIncrement;
    /// the units we want to use for the sick
    const char *myUnits;
    // the units we want to use (the sick value)
    ArSick::Units mySickUnits;
    /// the units we want to use for the sick
    const char *myBits;
    // the units we want to use (the sick value)
    ArSick::Bits mySickBits;
    // our tcp connection
    ArTcpConnection myTcpConn;
    // our serial connection
    ArSerialConnection mySerConn;
  };
  int myMaxNumLasers;
  std::list<LaserData *> myLasers;
  
  /// Parses the laser arguments
  AREXPORT bool parseLaserArgs(ArArgumentParser *parser, int laserNumber);
  /// Logs the laser parameters
  AREXPORT void logLaserOptions(unsigned int laserNumber) const;
  void reset(void);
  // the robot we've set up (so we can find its params)
  ArRobot *myRobot; 
  // if we're using the sim or not
  bool myUsingSim;
  // if we're connecting via tcp (not to the sim), what remote host
  const char *myRemoteHost;
  // robot port, if there isn't one this'll be NULL, which will just
  // be the default of ArUtil::COM1
  const char *myRobotPort;
  // baud for the serial
  int myRobotBaud;
  // robot tcp port if we're doing a remote host (defaults to 8101)
  int myRemoteRobotTcpPort;
  
  // whether we're connecting to a remote sim or not (so we don't try
  // to open a port for the laser)
  bool myRemoteIsSim;
  // our parser
  ArArgumentParser *myParser;
  bool myOwnParser;


  // a few device connections to use to connect to the robot
  ArTcpConnection myRobotTcpConn;
  ArSerialConnection myRobotSerConn;
  // a few device connections to use to connect to the laser
  ArTcpConnection myLaserTcpConn;
  ArSerialConnection myLaserSerConn;
  // device connections to use to connect to the second laser
  ArTcpConnection myLaserTcpConn2;
  ArSerialConnection myLaserSerConn2;

  ArRetFunctorC<bool, ArSimpleConnector> myParseArgsCB;
  ArConstFunctorC<ArSimpleConnector> myLogOptionsCB;
};

#endif // ARSIMPLECONNECTOR_H
