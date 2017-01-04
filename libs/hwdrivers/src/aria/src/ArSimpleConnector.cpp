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
#include "ArSimpleConnector.h"
#include "ArRobot.h"
#include "ArSick.h"
#include "ariaInternal.h"

AREXPORT ArSimpleConnector::ArSimpleConnector(int *argc, char **argv) :
  myParseArgsCB(this, &ArSimpleConnector::parseArgs),
  myLogOptionsCB(this, &ArSimpleConnector::logOptions)
{
  reset();
  myParser = new ArArgumentParser(argc, argv);
  myOwnParser = true;
  myParseArgsCB.setName("ArSimpleConnector");
  Aria::addParseArgsCB(&myParseArgsCB, 75);
  myLogOptionsCB.setName("ArSimpleConnector");
  Aria::addLogOptionsCB(&myLogOptionsCB, 90);
}

AREXPORT ArSimpleConnector::ArSimpleConnector(ArArgumentBuilder *builder) :
  myParseArgsCB(this, &ArSimpleConnector::parseArgs),
  myLogOptionsCB(this, &ArSimpleConnector::logOptions)
{
  reset();
  myParser = new ArArgumentParser(builder);
  myOwnParser = true;
  myParseArgsCB.setName("ArSimpleConnector");
  Aria::addParseArgsCB(&myParseArgsCB, 75);
  myLogOptionsCB.setName("ArSimpleConnector");
  Aria::addLogOptionsCB(&myLogOptionsCB, 90);
}

AREXPORT ArSimpleConnector::ArSimpleConnector(ArArgumentParser *parser) :
  myParseArgsCB(this, &ArSimpleConnector::parseArgs),
  myLogOptionsCB(this, &ArSimpleConnector::logOptions)
{
  reset();
  myParser = parser;
  myOwnParser = false;
  myParseArgsCB.setName("ArSimpleConnector");
  Aria::addParseArgsCB(&myParseArgsCB, 75);
  myLogOptionsCB.setName("ArSimpleConnector");
  Aria::addLogOptionsCB(&myLogOptionsCB, 90);
}

AREXPORT ArSimpleConnector::~ArSimpleConnector(void)
{

}

void ArSimpleConnector::reset(void)
{
  myRobot = NULL;
  myRemoteHost = NULL;
  myRobotPort = NULL;
  myRemoteRobotTcpPort = 8101;
  myRobotBaud = 9600;
  myRemoteIsSim = false;
  setMaxNumLasers();
  ArUtil::deleteSet(myLasers.begin(), myLasers.end());
  myLasers.clear();
}

/**
   This gets rid of all of the data that was set for the lasers, so
   call this before you parse.
 **/
AREXPORT void ArSimpleConnector::setMaxNumLasers(int maxNumLasers) 
{
  if (maxNumLasers > 0)
    myMaxNumLasers = maxNumLasers;
  else
    myMaxNumLasers = 0;
  ArUtil::deleteSet(myLasers.begin(), myLasers.end());
  myLasers.clear();
}



/**
 * Parse command line arguments using the ArArgumentParser given in the ArSimpleConnector constructor.
 *
 * See parseArgs(ArArgumentParser*) for details about argument parsing.
 * 
  @return true if the arguments were parsed successfully false if not
 **/

AREXPORT bool ArSimpleConnector::parseArgs(void)
{
  return parseArgs(myParser);
}

/**
 * Parse command line arguments held by the given ArArgumentParser.
 *
  @return true if the arguments were parsed successfully false if not

   The following arguments are used for the robot connection:

   <dl>
    <dt><code>-robotPort</code> <i>port</i></dt>
    <dt><code>-rp</code> <i>port</i></dt>
    <dd>Use the given serial port device name for a serial port connection (e.g. <code>COM1</code>, or <code>/dev/ttyS0</code> if on Linux.)
    The default is the first serial port, or COM1, which is the typical Pioneer setup.
    </dd>

    <dt><code>-remoteHost</code> <i>hostname</i></dt>
    <dt><code>-rh</code> <i>hostname</i></dt>
    <dd>Use a TCP connection to a remote computer with the given network host name instead of a serial port connection</dd>

    <dt><code>-remoteRobotTcpPort</code> <i>port</i></dt>
    <dt><code>-rrtp</code> <i>port</i></dt>
    <dd>Use the given TCP port number if connecting to a remote robot using TCP due to <code>-remoteHost</code> having been given.</dd>

    <dt><code>-remoteIsSim</code></dt>
    <dt><code>-ris</code></dt>
    <dd>The remote TCP robot given by <code>-remoteHost</code> or <code>-rh</code> is actually a simulator. Use any alternative
     behavior intended for the simulator (e.g. tell the laser device object to request laser data from the simulator rather
     than trying to connect to a real laser device on the local computer)</dd>

    <dt><code>-robotBaud</code> <i>baudrate</i></dt>
    <dd><code>-rb</code> <i>baudrate</i></dt>
    <dd>Use the given baud rate when connecting over a serial port, instead of trying to use the normal rate.</dd>
  </dl>

  The following arguments are accepted for laser connections.  A program may request support for more than one laser
  using setMaxNumLasers(); if multi-laser support is enabled in this way, then these arguments must have the laser index
  number appended. For example, "-laserPort" for laser 1 would instead by "-laserPort1", and for laser 2 it would be
  "-laserPort2".

  <dl>
    <dt>-laserPort <i>port</i></dt>
    <dt>-lp <i>port</i></dt>
    <dd>Use the given port device name when connecting to a laser. For example, <code>COM2</code> or on Linux, <code>/dev/ttyS1</code>.
    The default laser port is COM2, which is the typical Pioneer laser port setup.
    </dd>

    <dt>-laserFlipped <i>true|false</i></dt>
    <dt>-lf <i>true|false</i></dt>
    <dd>If <code>true</code>, then the laser is mounted upside-down on the robot and the ordering of readings
    should be reversed.</dd>

    <dt>-connectLaser</dt>
    <dt>-cl</dt>
    <dd>Explicitly request that the client program connect to a laser, if it does not always do so</dd>

    <dt>-laserPowerControlled <i>true|false</i></dt>
    <dt>-lpc <i>true|false</i></dt>
    <dd>If <code>true</code>, then the laser is powered on when the serial port is initially opened, so enable
    certain features when connecting such as a waiting period as the laser initializes.</dd>

    <dt>-laserDegrees <i>degrees</i></dt>
    <dt>-ld <i>degrees</i></dt>
    <dd>Indicate the size of the laser field of view, either <code>180</code> (default) or <code>100</code>.</dd>

    <dt>-laserIncrement <i>increment</i></dt>
    <dt>-li <i>increment</i></dt>
    <dd>Configures the laser's angular resolution. If <code>one</code>, then configure the laser to take a reading every degree.
     If <code>half</code>, then configure it for a reading every 1/2 degrees.</dd>

    <dt>-laserUnits <i>units</i></dt>
    <dt>-lu <i>units</i></dt>
    <dd>Configures the laser's range resolution.  May be 1mm for one milimiter, 1cm for ten milimeters, or 10cm for one hundred milimeters.</dd>

    <dt>-laserReflectorBits <i>bits</i></dt>
    <dt>-lrb <i>bits</i></dt>
    <dd>Enables special reflectance detection, and configures the granularity of reflector detection information. Using more bits allows the laser to provide values for several different
    reflectance levels, but also may force a reduction in range.  (Note, the SICK LMS-200 only detects high reflectance on special reflector material
    manufactured by SICK.)
    </dd>
  </dl>

 **/

AREXPORT bool ArSimpleConnector::parseArgs(ArArgumentParser *parser)
{
  int i;
  if (myMaxNumLasers > 0)
  {
    for (i = 1; i <= myMaxNumLasers; i++)
      myLasers.push_front(new LaserData(i));
  }

  if (parser->checkArgument("-remoteIsSim") ||
      parser->checkArgument("-ris"))
    myRemoteIsSim = true;

  if (!parser->checkParameterArgumentString("-remoteHost", 
					     &myRemoteHost) ||
      !parser->checkParameterArgumentString("-rh", 
					     &myRemoteHost) ||
      !parser->checkParameterArgumentString("-robotPort",
					     &myRobotPort) ||
      !parser->checkParameterArgumentString("-rp",
					     &myRobotPort) ||
      !parser->checkParameterArgumentInteger("-remoteRobotTcpPort",
					      &myRemoteRobotTcpPort) ||
      !parser->checkParameterArgumentInteger("-rrtp",
					      &myRemoteRobotTcpPort) ||
      !parser->checkParameterArgumentInteger("-robotBaud",
					      &myRobotBaud) ||
      !parser->checkParameterArgumentInteger("-rb",
					     &myRobotBaud))
    return false;
  else
  {
    for (i = 1; i <= myMaxNumLasers; i++)
      if (!parseLaserArgs(parser, i))
	return false;
    return true;
  }

}

AREXPORT bool ArSimpleConnector::parseLaserArgs(ArArgumentParser *parser, 
						int laserNumber)
{
  char buf[512];
  std::list<LaserData *>::iterator it;
  LaserData *laserData = NULL;
    
  for (it = myLasers.begin(); it != myLasers.end(); it++)
  {
    if ((*it)->myNumber == laserNumber)
    {
      laserData = (*it);
      break;
    }
  }
  if (laserData == NULL)
  {
    ArLog::log(ArLog::Terse, "Do not have laser %d", laserNumber);
    return false;
  }

  if (laserData->myNumber == 1)
    buf[0] = '\0';
  else
    sprintf(buf, "%d", laserData->myNumber);

  if (parser->checkArgumentVar("-connectLaser%s", buf) || 
      parser->checkArgumentVar("-cl%s", buf))
  {
    laserData->myConnect = true;
  }

  if (!parser->checkParameterArgumentStringVar(NULL, &laserData->myPort, 
					       "-laserPort%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myPort,
					    "-lp%s", buf) ||
      !parser->checkParameterArgumentIntegerVar(NULL,
	      &laserData->myRemoteTcpPort, 
	      "-remoteLaserTcpPort%s", buf) ||
      !parser->checkParameterArgumentIntegerVar(NULL,
	      &laserData->myRemoteTcpPort,
	      "-rltp%s", buf) ||
      !parser->checkParameterArgumentBoolVar(&laserData->myFlippedReallySet,
					     &laserData->myFlipped,
					     "-laserFlipped%s", buf) ||
      !parser->checkParameterArgumentBoolVar(&laserData->myFlippedReallySet,
					     &laserData->myFlipped,
					     "-lf%s", buf) ||
      !parser->checkParameterArgumentBoolVar(
	      &laserData->myPowerControlledReallySet,
	      &laserData->myPowerControlled,
	      "-laserPowerControlled%s", buf) ||
      !parser->checkParameterArgumentBoolVar(
	      &laserData->myPowerControlledReallySet,
	      &laserData->myPowerControlled,
					     "-lpc%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myDegrees,
					       "-laserDegrees%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myDegrees,
					       "-ld%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myIncrement,
					       "-laserIncrement%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myIncrement,
					       "-li%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myUnits,
					       "-laserUnits%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myUnits,
					       "-lu%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myBits,
					       "-laserReflectorBits%s", buf) ||
      !parser->checkParameterArgumentStringVar(NULL, &laserData->myBits,
					       "-lrb%s", buf))
  {
    return false;
  }
  if (laserData->myDegrees == NULL || laserData->myDegrees[0] == '\0')
    laserData->mySickDegrees = ArSick::DEGREES180;
  else if (strcasecmp(laserData->myDegrees, "180") == 0)
    laserData->mySickDegrees = ArSick::DEGREES180;
  else if (strcasecmp(laserData->myDegrees, "100") == 0)
    laserData->mySickDegrees = ArSick::DEGREES100;
  else
  {
    ArLog::log(ArLog::Normal, 
	       "Could not set laserDegrees%s, it should be set to 180 or 100",
	       buf);
    return false;
  }
  
  if (laserData->myIncrement == NULL || laserData->myIncrement[0] == '\0')
    laserData->mySickIncrement = ArSick::INCREMENT_ONE;
  else if (strcasecmp(laserData->myIncrement, "one") == 0)
    laserData->mySickIncrement = ArSick::INCREMENT_ONE;
  else if (strcasecmp(laserData->myIncrement, "half") == 0)
    laserData->mySickIncrement = ArSick::INCREMENT_HALF;
  else
  {
    ArLog::log(ArLog::Normal, 
	"Could not set laserIncrement%s, it should be set to 'one' or 'half'",
	       buf);
    return false;
  }

  if (laserData->myUnits == NULL || laserData->myUnits[0] == '\0')
    laserData->mySickUnits = ArSick::UNITS_1MM;
  else if (strcasecmp(laserData->myUnits, "1mm") == 0)
    laserData->mySickUnits = ArSick::UNITS_1MM;
  else if (strcasecmp(laserData->myUnits, "1cm") == 0)
    laserData->mySickUnits = ArSick::UNITS_1CM;
  else if (strcasecmp(laserData->myUnits, "10cm") == 0)
    laserData->mySickUnits = ArSick::UNITS_10CM;
  else
  {
    ArLog::log(ArLog::Normal, 
	       "Could not set laserUnits%s, it should be set to '1mm', '1cm', or '10cm'",
	       buf);
    return false;
  }

  if (laserData->myBits == NULL || laserData->myBits[0] == '\0')
    laserData->mySickBits = ArSick::BITS_1REFLECTOR;
  else if (strcasecmp(laserData->myBits, "1ref") == 0)
    laserData->mySickBits = ArSick::BITS_1REFLECTOR;
  else if (strcasecmp(laserData->myBits, "2ref") == 0)
    laserData->mySickBits = ArSick::BITS_2REFLECTOR;
  else if (strcasecmp(laserData->myBits, "3ref") == 0)
    laserData->mySickBits = ArSick::BITS_3REFLECTOR;
  else
  {
    ArLog::log(ArLog::Normal, 
	       "Could not set laserReflectorBits%s, it should be set to '1ref', '2ref', or '3ref'",
	       buf);
    return false;
  }

  return true;
}

AREXPORT void ArSimpleConnector::logOptions(void) const
{
  ArLog::log(ArLog::Terse, "Options for ArSimpleConnector (see docs for more details):");
  ArLog::log(ArLog::Terse, "");
  ArLog::log(ArLog::Terse, "Robot options:");
  ArLog::log(ArLog::Terse, "-remoteHost <remoteHostNameOrIP>");
  ArLog::log(ArLog::Terse, "-rh <remoteHostNameOrIP>");
  ArLog::log(ArLog::Terse, "-robotPort <robotSerialPort>");
  ArLog::log(ArLog::Terse, "-rp <robotSerialPort>");
  ArLog::log(ArLog::Terse, "-robotBaud <baud>");
  ArLog::log(ArLog::Terse, "-rb <baud>");
  ArLog::log(ArLog::Terse, "-remoteRobotTcpPort <remoteRobotTcpPort>");
  ArLog::log(ArLog::Terse, "-rrtp <remoteRobotTcpPort>");
  ArLog::log(ArLog::Terse, "-remoteIsSim");
  ArLog::log(ArLog::Terse, "-ris");
  
  for (int i = 1; i <= myMaxNumLasers; i++)
    logLaserOptions(i);
}

AREXPORT void ArSimpleConnector::logLaserOptions(
	unsigned int laserNumber) const
{
  char buf[512];
  
  if (laserNumber == 1)
    buf[0] = '\0';
  else
    sprintf(buf, "%d", laserNumber);

  ArLog::log(ArLog::Terse, "");
  ArLog::log(ArLog::Terse, "Laser%s options:", buf);
  ArLog::log(ArLog::Terse, "-connectLaser%s", buf);
  ArLog::log(ArLog::Terse, "-cl%s", buf);
  ArLog::log(ArLog::Terse, "-laserPort%s <laserSerialPort>", buf);
  ArLog::log(ArLog::Terse, "-lp%s <laserSerialPort>", buf);
  ArLog::log(ArLog::Terse, "-remoteLaserTcpPort%s <remoteLaserTcpPort>", buf);
  ArLog::log(ArLog::Terse, "-rltp%s <remoteLaserTcpPort>", buf);  
  ArLog::log(ArLog::Terse, "-laserFlipped%s <true|false>", buf);
  ArLog::log(ArLog::Terse, "-lf%s <true|false>", buf);
  ArLog::log(ArLog::Terse, "-laserPowerControlled%s <true|false>", buf);
  ArLog::log(ArLog::Terse, "-lpc%s <true|false>", buf);
  ArLog::log(ArLog::Terse, "-laserDegrees%s <180|100>", buf);
  ArLog::log(ArLog::Terse, "-ld%s <180|100>", buf);
  ArLog::log(ArLog::Terse, "-laserIncrement%s <one|half>", buf);
  ArLog::log(ArLog::Terse, "-li%s <one|half>", buf);
  ArLog::log(ArLog::Terse, "-laserUnits%s <1mm|1cm|10cm>", buf);
  ArLog::log(ArLog::Terse, "-lu%s <1mm|1cm|10cm>", buf);
  ArLog::log(ArLog::Terse, "-laserReflectorBits%s <1ref|2ref|3ref>", buf);
  ArLog::log(ArLog::Terse, "-lrb%s <1ref|2ref|3ref>", buf);

}




/**
 * This method is normally used internally by connectRobot(), but you may 
 * use it if you wish.
 *
 * If -remoteHost was given, then open that TCP port. If it was not given,
 * then try to open a TCP port to the simulator on localhost.
 * If that fails, then use a local serial port connection.
 * Sets the given ArRobot's device connection pointer to this object.
 * Sets up internal settings determined by command line arguments such
 * as serial port and baud rate, etc.
 *
 * After calling this function  (and it returns true), then you may connect
 * ArRobot to the robot using ArRobot::blockingConnect() (or similar).
 *
 * @return false if -remoteHost was given and there was an error connecting to
 * the remote host, true otherwise.
 **/
AREXPORT bool ArSimpleConnector::setupRobot(ArRobot *robot)
{
  myRobot = robot;
  // First we see if we can open the tcp connection, if we can we'll
  // assume we're connecting to the sim, and just go on...  if we
  // can't open the tcp it means the sim isn't there, so just try the
  // robot

  // see if we're doing remote host or not
  if (myRemoteHost != NULL)
    myRobotTcpConn.setPort(myRemoteHost, myRemoteRobotTcpPort);
  else
    myRobotTcpConn.setPort("localhost", myRemoteRobotTcpPort);

  // see if we can get to the simulator  (true is success)
  if (myRobotTcpConn.openSimple())
  {
    robot->setDeviceConnection(&myRobotTcpConn);
    // we could get to the sim, so set the robots device connection to the sim
    if (myRemoteHost != NULL)
    {
      ArLog::log(ArLog::Normal, "Connected to remote host %s through tcp.\n", 
		 myRemoteHost);
      if (myRemoteIsSim)
	myUsingSim = true;
      else
	myUsingSim = false;
    }
    else
    {
      ArLog::log(ArLog::Normal, "Connecting to simulator through tcp.\n");
      myUsingSim = true;
    }
  }
  else
  {
    // if we were trying for a remote host and it failed, just exit
    if (myRemoteHost != NULL)
    {
      ArLog::log(ArLog::Terse, "Could not connect robot to remote host %s, port %d.\n", myRemoteHost, myRemoteRobotTcpPort);
      return false;
    }
    // we couldn't get to the sim, so set the port on the serial
    // connection and then set the serial connection as the robots
    // device

    myRobotSerConn.setPort(myRobotPort);
    myRobotSerConn.setBaud(myRobotBaud);
    ArLog::log(ArLog::Normal,
	       "Could not connect to simulator, connecting to robot through serial port %s.", 
	       myRobotSerConn.getPort());
    robot->setDeviceConnection(&myRobotSerConn);
    myUsingSim = false;
  }
  return true;
}

/** Prepares the given ArRobot object for connection, then begins
 * a blocking connection attempt.
 * If you wish to simply prepare the ArRobot object, but not begin
 * the connection, then use setupRobot().
 */
AREXPORT bool ArSimpleConnector::connectRobot(ArRobot *robot)
{
  if (!setupRobot(robot))
    return false;
  else
    return robot->blockingConnect();
}

/**
   Description of the logic for connection to the laser:  If
   --remoteHost then the laser will a tcp connection will be opened to
   that remoteHost at port 8102 or --remoteLaserTcpPort if that
   argument is given, if this connection fails then the setup fails.
   If --remoteHost wasn't provided and the robot connected to a
   simulator as described elsewhere then the laser is just configured
   to be simulated, if the robot isn't connected to a simulator it
   tries to open a serial connection to ArUtil::COM3 or --laserPort if
   that argument is given.
**/

AREXPORT bool ArSimpleConnector::setupLaser(ArSick *sick)
{
  return setupLaserArbitrary(sick, 1);
}

/**
   Description of the logic for connecting to a second laser:  Given
   the fact that there are no parameters for the location of a second
   laser, the laser's port must be passed in to ArSimpleConnector from
   the main or from ArArgumentBuilder.  Similarly, a tcp connection must
   be explicitly defined with the --remoteLaserTcpPort2 argument.
**/
AREXPORT bool ArSimpleConnector::setupSecondLaser(ArSick *sick)
{
  return setupLaserArbitrary(sick, 2);
}

AREXPORT bool ArSimpleConnector::setupLaserArbitrary(ArSick *sick, 
						    int laserNumber)
{
  std::list<LaserData *>::iterator it;
  LaserData *laserData = NULL;
  const ArRobotParams *params;
    
  for (it = myLasers.begin(); it != myLasers.end(); it++)
  {
    if ((*it)->myNumber == laserNumber)
    {
      laserData = (*it);
      break;
    }
  }
  if (laserData == NULL)
  {
    ArLog::log(ArLog::Terse, "ArSimpleConnector::setupLaser: Do not have laser #%d (parseArgs not called or beyond maximum number of lasers allowed?)", laserNumber);
    return false;
  }
  
  if (laserData->myNumber == 1 && myRobot != NULL && 
      myRobot->isConnected())
  {
    params = myRobot->getRobotParams();
    if (!laserData->myFlippedReallySet)
      laserData->myFlipped = params->getLaserFlipped();
    if (!laserData->myPowerControlledReallySet)
      laserData->myPowerControlled = params->getLaserPowerControlled();
    if (laserData->myPort == NULL || laserData->myPort[0] == '\0')
    {
      if (strcmp(params->getLaserPort(), "COM1") == 0)
	laserData->myPort = ArUtil::COM1;
      else if (strcmp(params->getLaserPort(), "COM2") == 0)
	laserData->myPort = ArUtil::COM2;
      else if (strcmp(params->getLaserPort(), "COM3") == 0)
	laserData->myPort = ArUtil::COM3;
      else if (strcmp(params->getLaserPort(), "COM4") == 0)
	laserData->myPort = ArUtil::COM4;
      else
      {
	if (params->getLaserPort() != NULL)
	  laserData->myPort = params->getLaserPort();
	else
	  laserData->myPort = ArUtil::COM3;
	ArLog::log(ArLog::Normal,
		   "Could not find LaserPort from robot parameters, using %s", 
		   laserData->myPort);

      }
    }
    
  }

  if (laserData->myPort == NULL || strlen(laserData->myPort) == 0)
  {
    ArLog::log(ArLog::Normal, "There is no port defined for laser %d", 
	       laserData->myNumber);
    return false;
  }
  sick->setRangeInformation(laserData->mySickBits, laserData->mySickUnits);
  if (!myUsingSim && myRemoteHost != NULL && strlen(myRemoteHost) > 0)
    sick->configure(myUsingSim, laserData->myPowerControlled,
		    laserData->myFlipped,
		    ArSick::BAUD9600, laserData->mySickDegrees, 
		    laserData->mySickIncrement);
  else if (myUsingSim && laserData->myNumber != 1)
  {
    ArLog::log(ArLog::Normal, "Cannot use the simulator with multiple lasers");
    return false;
  }
  else
    sick->configure(myUsingSim, laserData->myPowerControlled,
		    laserData->myFlipped,
		    ArSick::BAUD38400, laserData->mySickDegrees, 
		    laserData->mySickIncrement);

  // set the port, if we need to
  if (!myUsingSim)
  { 
    if (myRemoteHost != NULL && strlen(myRemoteHost) > 0)
    { 
      laserData->myTcpConn.setPort(myRemoteHost, 
				   laserData->myRemoteTcpPort);
      sick->setDeviceConnection(&laserData->myTcpConn);
      if (!laserData->myTcpConn.openSimple())
      { 
        ArLog::log(ArLog::Terse, "Could not connect laser to remote host %s.\n", myRemoteHost);
	return false; 
      } 
    } 
    else 
    { 
      laserData->mySerConn.setPort(laserData->myPort);
      sick->setDeviceConnection(&laserData->mySerConn); 
    } 
  }
  return true;

}

/**
   This will setup and connect the laser if the command line switch
   was given to do so or simply return true if no connection was
   wanted.
**/

AREXPORT bool ArSimpleConnector::connectLaser(ArSick *sick)
{
  return connectLaserArbitrary(sick, 1);
}


/**
   This will setup and connect the laser if the command line switch
   was given to do so or simply return true if no connection was
   requested.
**/
AREXPORT bool ArSimpleConnector::connectSecondLaser(ArSick *sick)
{
  return connectLaserArbitrary(sick, 2);
}

AREXPORT bool ArSimpleConnector::connectLaserArbitrary(ArSick *sick,
						      int laserNumber)
{
  std::list<LaserData *>::iterator it;
  LaserData *laserData = NULL;
  
  for (it = myLasers.begin(); it != myLasers.end(); it++)
  {
    if ((*it)->myNumber == laserNumber)
    {
      laserData = (*it);
      break;
    }
  }
  if (laserData == NULL)
  {
    ArLog::log(ArLog::Terse, "Do not have laser %d", laserNumber);
    return false;
  }

  sick->lockDevice();
  // set up the laser regardless
  setupLaserArbitrary(sick, laserNumber);
  sick->unlockDevice();
  // see if we want to connect
  if (!laserData->myConnect)
    return true;
  else
    return sick->blockingConnect();
}
