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
#include <time.h>
#include <ctype.h>

#include "ArRobot.h"
#include "ArLog.h"
#include "ArDeviceConnection.h"
#include "ArTcpConnection.h"
#include "ArSerialConnection.h"
#include "ArLogFileConnection.h"
#include "ariaUtil.h"
#include "ArSocket.h"
#include "ArCommands.h"
#include "ArRobotTypes.h"
#include "ArSignalHandler.h"
#include "ArPriorityResolver.h"
#include "ArAction.h"
#include "ArRangeDevice.h"
#include "ArRobotConfigPacketReader.h"
#include "ariaInternal.h"

#include <mrpt/utils/mrpt_macros.h>

/**
 * The parameters only rarely need to be specified.
 *
 * @param name A name for this robot, useful if a program has more than one
 *  ArRobot object


   @param obsolete This parameter is ignored.
   (It used to turn off state reflection if false, but that
   is no longer possible.)

   @param normalInit whether the robot should initializes its
   structures or the calling program will take care of it.  No one
   will probalby ever use this value, since if they are doing that
   then overriding will probably be more useful, but there it is.

   @param doSigHandle do normal signal handling and have this robot
   instance stopRunning() when the program is signaled

**/

AREXPORT ArRobot::ArRobot(const char *name, bool obsolete,
			  bool doSigHandle, bool normalInit,
			  bool addAriaExitCallback) :
  myMotorPacketCB(this, &ArRobot::processMotorPacket),
  myEncoderPacketCB(this, &ArRobot::processEncoderPacket),
  myIOPacketCB(this, &ArRobot::processIOPacket),
  myPacketHandlerCB(this, &ArRobot::packetHandler),
  myActionHandlerCB(this, &ArRobot::actionHandler),
  myStateReflectorCB(this, &ArRobot::stateReflector),
  myRobotLockerCB(this, &ArRobot::robotLocker),
  myRobotUnlockerCB(this, &ArRobot::robotUnlocker),
  myKeyHandlerExitCB(this, &ArRobot::keyHandlerExit),
  myGetCycleWarningTimeCB(this, &ArRobot::getCycleWarningTime),
  myGetNoTimeWarningThisCycleCB(this, &ArRobot::getNoTimeWarningThisCycle),
  myBatteryAverager(20),
  myRealBatteryAverager(20),
  myAriaExitCB(this, &ArRobot::ariaExitCallback)
{
  MRPT_UNUSED_PARAM(obsolete);
  setName(name);
  myAriaExitCB.setName("ArRobotExit");
  myNoTimeWarningThisCycle = false;
  myGlobalPose.setPose(0, 0, 0);

  myParams = new ArRobotGeneric("");

  myMotorPacketCB.setName("ArRobot::motorPacket");
  myEncoderPacketCB.setName("ArRobot::encoderPacket");
  myIOPacketCB.setName("ArRobot::IOPacket");

  myPtz = NULL;
  myKeyHandler = NULL;
  myKeyHandlerCB = NULL;

  myConn = NULL;

  myOwnTheResolver = false;

  myBlockingConnectRun = false;
  myAsyncConnectFlag = false;

  myLogMovementSent = false;
  myLogMovementReceived = false;
  myLogVelocitiesReceived = false;
  myLogActions = false;
  myLastVel = 0;
  myLastRotVel = 0;
  myLastHeading = 0;
  myLastCalculatedRotVel = 0;

  myPacketsSentTracking = false;
  myPacketsReceivedTracking = false;
  myPacketsReceivedTrackingCount = false;
  myPacketsReceivedTrackingStarted.setToNow();

  myCycleTime = 100;
  myCycleWarningTime = 250;
  myConnectionCycleMultiplier = 2;
  myTimeoutTime = 8000;
  myStabilizingTime = 0;
  myCounter = 1;
  myResolver = NULL;
  myNumSonar = 0;

  myRequestedIOPackets = false;
  myRequestedEncoderPackets = false;
  myEncoderCorrectionCB = NULL;

  myCycleChained = true;

  myMoveDoneDist = 40;
  myHeadingDoneDiff = 3;

  myOrigRobotConfig = NULL;
  reset();
  if (normalInit)
    init();

  mySyncLoop.setRobot(this);

  if (doSigHandle)
    Aria::addRobot(this);
  if (addAriaExitCallback)
  {
    Aria::addExitCallback(&myAriaExitCB, 0);
    myAddedAriaExitCB = true;
  }
  else
  {
    myAddedAriaExitCB = false;
  }

  myConnectWithNoParams = false;
}



AREXPORT ArRobot::~ArRobot()
{
  ArResolver::ActionMap::iterator it;

  stopRunning();
  delete mySyncTaskRoot;
  ArUtil::deleteSetPairs(mySonars.begin(), mySonars.end());
  Aria::delRobot(this);

  if (myKeyHandlerCB != NULL)
    delete myKeyHandlerCB;

  for (it = myActions.begin(); it != myActions.end(); ++it)
  {
    (*it).second->setRobot(NULL);
  }
}


/**
   Sets up the packet handlers, sets up the sync list and makes the default
   priority resolver.
**/
AREXPORT void ArRobot::init(void)
{
  setUpPacketHandlers();
  setUpSyncList();
  myOwnTheResolver = true;
  myResolver = new ArPriorityResolver;
}

/**
   This starts the list of tasks to be run through until stopped.
   This function doesn't return until something calls stop on this
   instance.

   @param stopRunIfNotConnected if true, the run will return if there
   is no connection to the robot at any given point, this is good for
   one-shot programs... if it is false the run won't return unless
   stop is called on the instance
**/
AREXPORT void ArRobot::run(bool stopRunIfNotConnected)
{
  if (mySyncLoop.getRunning())
  {
    ArLog::log(ArLog::Terse,
	       "The robot is already running, cannot run it again.");
    return;
  }
  mySyncLoop.setRunning(true);
  mySyncLoop.stopRunIfNotConnected(stopRunIfNotConnected);
  mySyncLoop.runInThisThread();
}

/**
   @return true if the robot is currently running in a run or runAsync,
   otherwise false
**/

AREXPORT bool ArRobot::isRunning(void) const
{
  return mySyncLoop.getRunning();
}

/**
   This starts a new thread then has runs through the tasks until stopped.
   This function doesn't return until something calls stop on this instance.
   This function returns immediately
   @param stopRunIfNotConnected if true, the run will stop if there is no
   connection to the robot at any given point, this is good for one-shot
   programs... if it is false the run won't stop unless stop is called on the
   instance
**/
AREXPORT void ArRobot::runAsync(bool stopRunIfNotConnected)
{
  if (mySyncLoop.getRunning())
  {
    ArLog::log(ArLog::Terse,
	       "The robot is already running, cannot run it again.");
    return;
  }
  mySyncLoop.stopRunIfNotConnected(stopRunIfNotConnected);
  // Joinable, but do NOT lower priority. The robot thread is the most
  // important one around.
  mySyncLoop.create(true, false);
}

/**
   This stops this robot's processing cycle.  If it is running
   in a background thread (from runAsync()), it will cause that thread
   to exit.  If it is running synchronously (from run()), then run() will
   return.
   @param doDisconnect If true, also disconnect from the robot connection (default is true).
   @sa isRunning
**/
AREXPORT void ArRobot::stopRunning(bool doDisconnect)
{
  if (myKeyHandler != NULL)
    myKeyHandler->restore();
  mySyncLoop.stopRunning();
  myBlockingConnectRun = false;
  if (doDisconnect && (isConnected() || myIsStabilizing))
  {
    waitForRunExit();
    disconnect();
  }
  wakeAllWaitingThreads();
}

AREXPORT void ArRobot::setUpSyncList(void)
{
  mySyncTaskRoot = new ArSyncTask("SyncTasks");
  mySyncTaskRoot->setWarningTimeCB(&myGetCycleWarningTimeCB);
  mySyncTaskRoot->setNoTimeWarningCB(&myGetNoTimeWarningThisCycleCB);
  mySyncTaskRoot->addNewLeaf("Packet Handler", 85, &myPacketHandlerCB);
  mySyncTaskRoot->addNewLeaf("Robot Locker", 70, &myRobotLockerCB);
  mySyncTaskRoot->addNewBranch("Sensor Interp", 65);
  mySyncTaskRoot->addNewLeaf("Action Handler", 55, &myActionHandlerCB);
  mySyncTaskRoot->addNewLeaf("State Reflector", 45, &myStateReflectorCB);
  mySyncTaskRoot->addNewBranch("User Tasks", 25);
  mySyncTaskRoot->addNewLeaf("Robot Unlocker", 20, &myRobotUnlockerCB);
}


AREXPORT void ArRobot::setUpPacketHandlers(void)
{
  addPacketHandler(&myMotorPacketCB, ArListPos::FIRST);
  addPacketHandler(&myEncoderPacketCB, ArListPos::LAST);
  addPacketHandler(&myIOPacketCB, ArListPos::LAST);
}

void ArRobot::reset(void)
{
  resetOdometer();
  myInterpolation.reset();
  myEncoderInterpolation.reset();
  if (myOrigRobotConfig != NULL)
    delete myOrigRobotConfig;
  myOrigRobotConfig = new ArRobotConfigPacketReader(this, true);
  myFirstEncoderPose = true;
  //myEncoderPose.setPose(0, 0, 0);
  //myEncoderPoseTaken.setToNow();
  //myRawEncoderPose.setPose(0, 0, 0);
  //myGlobalPose.setPose(0, 0, 0);
  //myEncoderTransform.setTransform(myEncoderPose, myGlobalPose);
  myTransVelMax = 0;
  myTransAccel = 0;
  myTransDecel = 0;
  myRotVelMax = 0;
  myRotAccel = 0;
  myRotDecel = 0;

  myRobotLengthFront = 0;
  myRobotLengthRear = 0;

  myLeftVel = 0;
  myRightVel = 0;
  myBatteryVoltage = 13;
  myBatteryAverager.clear();
  myBatteryAverager.add(myBatteryVoltage);
  myStallValue = 0;
  myControl = 0;
  myFlags = 0;
  myFaultFlags = 0;
  myHasFaultFlags = 0;
  myCompass = 0;
  myAnalogPortSelected = 0;
  myAnalog = 0;
  myDigIn = 0;
  myDigOut = 0;
  myIOAnalogSize = 0;
  myIODigInSize = 0;
  myIODigOutSize = 0;
  myLastIOPacketReceivedTime.setSec(0);
  myLastIOPacketReceivedTime.setMSec(0);
  myVel = 0;
  myRotVel = 0;
  myChargeState = CHARGING_UNKNOWN;
  myLastX = 0;
  myLastY = 0;
  myLastTh = 0;
  mySentPulse = false;
  myIsConnected = false;
  myIsStabilizing = false;

  myTransVal = 0;
  myTransVal2 = 0;
  myTransType = TRANS_NONE;
  myLastTransVal = 0;
  myLastTransVal2 = 0;
  myLastTransType = TRANS_NONE;
  myLastTransSent.setToNow();
  myActionTransSet = false;
  myLastActionRotStopped = false;
  myLastActionRotHeading = false;

  myRotVal = 0;
  myLastRotVal = 0;
  myRotType = ROT_NONE;
  myLastRotType = ROT_NONE;
  myLastRotSent.setToNow();
  myActionRotSet = false;

  myLastPulseSent.setToNow();

  myDirectPrecedenceTime = 0;
  myStateReflectionRefreshTime = 500;

  myActionDesired.reset();

  myMotorPacCurrentCount = 0;
  myMotorPacCount = 0;
  myTimeLastMotorPacket = 0;

  mySonarPacCurrentCount = 0;
  mySonarPacCount = 0;
  myTimeLastSonarPacket = 0;

  myLeftEncoder = 0;
  myRightEncoder = 0;

  myWarnedAboutExtraSonar = false;
  myOdometryDelay = 0;
}

AREXPORT void ArRobot::setTransVelMax(double vel)
{
  if (vel > myAbsoluteMaxTransVel)
  {
    ArLog::log(ArLog::Terse, "ArRobot: setTransVelMax of %g is over the absolute max of %g, capping it", vel, myAbsoluteMaxTransVel);
    vel = myAbsoluteMaxTransVel;
  }
  myTransVelMax = vel;
}

AREXPORT void ArRobot::setTransAccel(double acc)
{
  if (acc > myAbsoluteMaxTransAccel)
  {
    ArLog::log(ArLog::Terse, "ArRobot: setTransAccel of %g is over the absolute max of %g, capping it", acc, myAbsoluteMaxTransAccel);
    acc = myAbsoluteMaxTransAccel;
  }
  myTransAccel = acc;
}

AREXPORT void ArRobot::setTransDecel(double decel)
{
  if (fabs(decel) > myAbsoluteMaxTransDecel)
  {
    ArLog::log(ArLog::Terse, "ArRobot: setTransDecel of %g is over the absolute max of %g, capping it", decel, myAbsoluteMaxTransDecel);
    decel = myAbsoluteMaxTransDecel;
  }
  myTransDecel = ArMath::fabs(decel);
}

AREXPORT void ArRobot::setRotVelMax(double vel)
{
  if (vel > myAbsoluteMaxRotVel)
  {
    ArLog::log(ArLog::Terse, "ArRobot: rotVelMax of %g is over the absolute max of %g, capping it", vel, myAbsoluteMaxRotVel);
    vel = myAbsoluteMaxRotVel;
  }
  myRotVelMax = vel;
}

AREXPORT void ArRobot::setRotAccel(double acc)
{
  if (acc > myAbsoluteMaxRotAccel)
  {
    ArLog::log(ArLog::Terse, "ArRobot: setRotAccel of %g is over the absolute max of %g, capping it", acc, myAbsoluteMaxRotAccel);
    acc = myAbsoluteMaxRotAccel;
  }
  myRotAccel = acc;
}

AREXPORT void ArRobot::setRotDecel(double decel)
{
  if (fabs(decel) > myAbsoluteMaxRotDecel)
  {
    ArLog::log(ArLog::Terse, "ArRobot: setRotDecel of %g is over the absolute max of %g, capping it", decel, myAbsoluteMaxRotDecel);
    decel = myAbsoluteMaxRotDecel;
  }
  myRotDecel = ArMath::fabs(decel);
}

AREXPORT double ArRobot::getTransVelMax(void) const
{
  return myTransVelMax;
}

AREXPORT double ArRobot::getTransAccel(void) const
{
  return myTransAccel;
}

AREXPORT double ArRobot::getTransDecel(void) const
{
  return myTransDecel;
}

AREXPORT double ArRobot::getRotVelMax(void) const
{
  return myRotVelMax;
}

AREXPORT double ArRobot::getRotAccel(void) const
{
  return myRotAccel;
}

AREXPORT double ArRobot::getRotDecel(void) const
{
  return myRotDecel;
}

/**
   Sets the connection this instance uses to the actual robot.  This is where
   commands will be sent and packets will be received from

   @param connection The deviceConnection to use for this robot

   @see ArDeviceConnection, ArSerialConnection, ArTcpConnection
**/
AREXPORT void ArRobot::setDeviceConnection(ArDeviceConnection *connection)
{
  myConn = connection;
  mySender.setDeviceConnection(myConn);
  myReceiver.setDeviceConnection(myConn);
}

/**
   Gets the connection this instance uses to the actual robot.  This is where
   commands will be sent and packets will be received from
   @return the deviceConnection used for this robot
   @see ArDeviceConnection
   @see ArSerialConnection
   @see ArTcpConnection
**/
AREXPORT ArDeviceConnection *ArRobot::getDeviceConnection(void) const
{
  return myConn;
}

/**
   Sets the number of milliseconds to go without a response from the robot
   until it is assumed that the connection with the robot has been
   broken and the disconnect on error events will happen.  Note that
   this will only happen with the default packet handler.

   @param mSecs if seconds is 0 then the connection timeout feature
   will be disabled, otherwise disconnect on error will be triggered
   after this number of seconds...
**/
AREXPORT void ArRobot::setConnectionTimeoutTime(int mSecs)
{
  if (mSecs > 0)
    myTimeoutTime = mSecs;
  else
    myTimeoutTime = 0;
}

/**
   Gets the number of seconds to go without response from the robot
   until it is assumed tha tthe connection with the robot has been
   broken and the disconnect on error events will happen.

**/
AREXPORT int ArRobot::getConnectionTimeoutTime(void) const
{
  return myTimeoutTime;
}

/**
   This gets the ArTime that the last packet was received at

   @return the time the last packet was received
**/
AREXPORT ArTime ArRobot::getLastPacketTime(void) const
{
  return myLastPacketReceivedTime;
}
/**
    Sets up the robot to connect, then returns, but the robot must be
    running (ie from runAsync) before you do this.  Also this will
    fail if the robot is already connected.  If you want to know what
    happened because of the connect then look at the callbacks.  NOTE,
    this will not lock robot before setting values, so you MUST lock
    the robot before you call this function and unlock the robot after
    you call this function.  If you fail to lock the robot, you'll may
    wind up with wierd behavior.  Other than the aspect of blocking or
    not the only difference between async and blocking connects (other
    than the blocking) is that async is run every robot cycle, whereas
    blocking runs as fast as it can... also blocking will try to
    reconnect a radio modem if it looks like it didn't get connected
    in the first place, so blocking can wind up taking 10 or 12
    seconds to decide it can't connect, whereas async doesn't try hard
    at all to reconnect the radio modem (beyond its first try) (under
    the assumption the async connect is user driven, so they'll just
    try again, and so that it won't mess up the sync loop by blocking
    for so long).

    @return true if the robot is running and the robot will try to
    connect, false if the robot isn't running so won't try to connect
    or if the robot is already connected

    @see addConnectCB

    @see addFailedConnectCB

    @see runAsync
**/

AREXPORT bool ArRobot::asyncConnect(void)
{
  if (!mySyncLoop.getRunning() || isConnected())
    return false;

  myAsyncConnectFlag = true;
  myBlockingConnectRun = false;
  myAsyncConnectState = -1;
  return true;
}

/**
    Connects to the robot, returning only when a connection has been
    made or it has been established a connection can't be made.  This
    connection usually is fast, but can take up to 30 seconds if the
    robot is in a wierd state (this is not often).  If the robot is
    connected via ArSerialConnection then the connect will also
    connect the radio modems.  Upon a successful connection all of the
    Connection Callback Functors that have been registered will be
    called.  NOTE, this will lock the robot before setting values, so
    you MUST not have the robot locked from where you call this
    function.  If you do, you'll wind up in a deadlock.  This behavior
    is there because otherwise you'd have to lock the robot before
    calling this function, and normally blockingConnect will be called
    from a separate thread, and that thread won't be doing anything
    else with the robot at that time.  Other than the aspect of
    blocking or not the only difference between async and blocking
    connects (other than the blocking) is that async is run every
    robot cycle, whereas blocking runs as fast as it can... also
    blocking will try to reconnect a radio modem if it looks like it
    didn't get connected in the first place, so blocking can wind up
    taking 10 or 12 seconds to decide it can't connect, whereas async
    doesn't try hard at all to reconnect the radio modem (under the
    assumption the async connect is user driven, so they'll just try
    again, and so that it won't mess up the sync loop by blocking for
    so long).

    @return true if a connection could be made, false otherwise

**/

AREXPORT bool ArRobot::blockingConnect(void)
{
  int ret = 0;

  lock();
  if (myIsConnected)
    disconnect();
  myBlockingConnectRun = true;
  myAsyncConnectState = -1;
  while ((ret = asyncConnectHandler(true)) == 0 && myBlockingConnectRun)
    ArUtil::sleep(ArMath::roundInt(getCycleTime() * .80));
  unlock();
  if (ret == 1)
    return true;
  else
    return false;
}

/**
   This is an internal function that is used both for async connects and
   blocking connects use to connect.  It does about the same thing for both,
   and it should only be used by asyncConnect and blockingConnect really.
   But here it is.  The only difference between when its being used
   by blocking/async connect is that in blocking mode if it thinks there
   may be problems with the radio modem it pauses for two seconds trying
   to deal with this... whereas in async mode it tries to deal with this in a
   simpler way.
   @param tryHarderToConnect if this is true, then if the radio modems look
   like they aren't working, it'll take about 2 seconds to try and connect
   them, whereas if its false, it'll do a little try, but won't try very hard
   @return 0 if its still trying to connect, 1 if it connected, 2 if it failed
**/

AREXPORT int ArRobot::asyncConnectHandler(bool tryHarderToConnect)
{
  int ret = 0;
  ArRobotPacket *packet;
  ArRobotPacket *tempPacket = NULL;
  char robotSubType[255];
  int i;
  int len;
  ArTime endTime;
  int timeToWait;
  ArSerialConnection *serConn;

  endTime.setToNow();
  endTime.addMSec(getCycleTime()*myConnectionCycleMultiplier);

  myNoTimeWarningThisCycle = true;

  // if this is -1, then we're doing initialization stuff, then returning
  if (myAsyncConnectState == -1)
  {
    myAsyncConnectSentChangeBaud = false;
    myAsyncConnectNoPacketCount = 0;
    myAsyncConnectTimesTried = 0;
    reset();
    if (myConn == NULL)
    {
      ArLog::log(ArLog::Terse, "Cannot connect, no connection has been set.");
      failedConnect();
      return 2;
    }

    if (myConn->getStatus() != ArDeviceConnection::STATUS_OPEN)
    {
      if (!myConn->openSimple())
      {
	/*str = myConn->getStatusMessage(myConn->getStatus());
	  ArLog::log(ArLog::Terse, "Trying to connect to robot but connection not opened, it is %s", str.c_str());*/
        ArLog::log(ArLog::Terse,
                   "Could not connect, because open on the device connection failed.");
        failedConnect();
        return 2;
      }
    }

    // check for log file connection: just return success
    if (dynamic_cast<ArLogFileConnection *>(myConn))
    {
      ArLogFileConnection *con = dynamic_cast<ArLogFileConnection *>(myConn);
      myRobotName = con->myName;
      myRobotType = con->myType;
      myRobotSubType = con->mySubtype;
      if (con->havePose)        // we know where to move
        moveTo(con->myPose);
      setCycleChained(false);   // don't process packets all at once
      madeConnection();
      finishedConnection();
      return 1;
    }


    if (dynamic_cast<ArSerialConnection *>(myConn))
    {
      myConn->write("WMS2\15", strlen("WMS2\15"));
    }
    // here we're flushing out all previously received packets
    while ( endTime.mSecTo() > 0 && myReceiver.receivePacket(0) != NULL);

    myAsyncConnectState = 0;
    return 0;
  }

  if (myAsyncConnectState >= 3)
  {
    bool handled;
    std::list<ArRetFunctor1<bool, ArRobotPacket *> *>::iterator it;
    while ((packet = myReceiver.receivePacket(0)) != NULL)
    {
      //printf("0x%x\n", packet->getID());
      for (handled = false, it = myPacketHandlerList.begin();
	   it != myPacketHandlerList.end() && handled == false;
	   it++)
      {
	if ((*it) != NULL && (*it)->invokeR(packet))
	  handled = true;
	else
	  packet->resetRead();
      }
    }
  }

  if (myAsyncConnectState == 3)
  {
    //if (!myOrigRobotConfig->hasPacketBeenRequested())
    if (!myOrigRobotConfig->hasPacketArrived())
    {
      myOrigRobotConfig->requestPacket();
    }
    // if we've gotten our config packet or if we've timed out then
    // set our vel and acc/decel params and skip to the next part
    if (myOrigRobotConfig->hasPacketArrived() ||
	myAsyncStartedConnection.mSecSince() > 1000)
    {
      bool gotConfig;
      // if we have data from the robot use that
      if (myOrigRobotConfig->hasPacketArrived())
      {
	gotConfig = true;
	setAbsoluteMaxTransVel(myOrigRobotConfig->getTransVelTop());
	setAbsoluteMaxTransAccel(myOrigRobotConfig->getTransAccelTop());
	setAbsoluteMaxTransDecel(myOrigRobotConfig->getTransAccelTop());
	setAbsoluteMaxRotVel(myOrigRobotConfig->getRotVelTop());
	setAbsoluteMaxRotAccel(myOrigRobotConfig->getRotAccelTop());
	setAbsoluteMaxRotDecel(myOrigRobotConfig->getRotAccelTop());
	setTransVelMax(myOrigRobotConfig->getTransVelMax());
	setTransAccel(myOrigRobotConfig->getTransAccel());
	setTransDecel(myOrigRobotConfig->getTransDecel());
	setRotVelMax(myOrigRobotConfig->getRotVelMax());
	setRotAccel(myOrigRobotConfig->getRotAccel());
	setRotDecel(myOrigRobotConfig->getRotDecel());
      }
      // if our absolute maximums weren't set then set them
      else
      {
	gotConfig = false;
	setAbsoluteMaxTransVel(myParams->getAbsoluteMaxVelocity());
	setAbsoluteMaxTransAccel(1000);
	setAbsoluteMaxTransDecel(1000);
	setAbsoluteMaxRotVel(myParams->getAbsoluteMaxRotVelocity());
	setAbsoluteMaxRotAccel(100);
	setAbsoluteMaxRotDecel(100);
      }
      // okay we got in that data, now put over it any of the things
      // that we got from the robot parameter file... if we don't have
      // max vels from above or the parameters then use the absolutes
      // which we do have for everything
      if (ArMath::fabs(myParams->getTransVelMax()) > 1)
	setTransVelMax(myParams->getTransVelMax());
      else if (!gotConfig)
	setTransVelMax(myParams->getAbsoluteMaxVelocity());
      if (ArMath::fabs(myParams->getRotVelMax()) > 1)
	setRotVelMax(myParams->getRotVelMax());
      else if (!gotConfig)
	setRotVelMax(myParams->getAbsoluteMaxRotVelocity());
      if (ArMath::fabs(myParams->getTransAccel()) > 1)
	setTransAccel(myParams->getTransAccel());
      if (ArMath::fabs(myParams->getTransDecel()) > 1)
	setTransDecel(myParams->getTransDecel());
      if (ArMath::fabs(myParams->getRotAccel()) > 1)
	setRotAccel(myParams->getRotAccel());
      if (ArMath::fabs(myParams->getRotDecel()) > 1)
	setRotDecel(myParams->getRotDecel());
      myAsyncConnectState = 4;
    }
    else
      return 0;
  }
  // we want to see if we should switch baud
  if (myAsyncConnectState == 4)
  {
    serConn = dynamic_cast<ArSerialConnection *>(myConn);
    // if we didn't get a config packet or we can't change the baud or
    // we aren't using a serial port then don't switch the baud
    if (!myOrigRobotConfig->hasPacketArrived() ||
	!myOrigRobotConfig->getResetBaud() ||
	serConn == NULL ||
	myParams->getSwitchToBaudRate() == 0)
    {
      // if we're using a serial connection store our baud rate
      if (serConn != NULL)
	myAsyncConnectStartBaud = serConn->getBaud();
      myAsyncConnectState = 5;
    }
    // if we did get a packet and can change baud send the command
    else if (!myAsyncConnectSentChangeBaud)
    {
      // if we're using a serial connection store our baud rate
      if (serConn != NULL)
	myAsyncConnectStartBaud = serConn->getBaud();

      int baudNum = -1;

      // first suck up all the packets we have now
      while ((packet = myReceiver.receivePacket(0)) != NULL);
      if (myParams->getSwitchToBaudRate() == 9600)
	baudNum = 0;
      else if (myParams->getSwitchToBaudRate() == 19200)
	baudNum = 1;
      else if (myParams->getSwitchToBaudRate() == 38400)
	baudNum = 2;
      else if (myParams->getSwitchToBaudRate() == 57600)
	baudNum = 3;
      else if (myParams->getSwitchToBaudRate() == 115200)
	baudNum = 4;
      else
      {
	ArLog::log(ArLog::Normal, "Warning: SwitchToBaud is set to %d baud, ignoring.",
		   myParams->getSwitchToBaudRate());
	ArLog::log(ArLog::Normal, "\tGood bauds are 9600 19200 38400 56800 116200.");
	myAsyncConnectState = 5;
	baudNum = -1;
	return 0;
      }
      if (baudNum != -1)
      {
	// now switch it over
	comInt(ArCommands::HOSTBAUD, baudNum);
	ArUtil::sleep(10);
	myAsyncConnectSentChangeBaud = true;
	myAsyncConnectStartedChangeBaud.setToNow();
	serConn->setBaud(myParams->getSwitchToBaudRate());
	//serConn->setBaud(19200);
	ArUtil::sleep(10);
	com(0);
	return 0;
      }
    }
    // if we did send the command then wait and see if we get any packets back
    else
    {
      packet = myReceiver.receivePacket(100);
      com(0);
      // if we got any packet we're good
      if (packet != NULL)
      {
	myAsyncConnectState = 5;
      }
      // if we didn't get it and its been 500 ms then fail
      else if (myAsyncConnectStartedChangeBaud.mSecSince() > 900)
      {
	ArLog::log(ArLog::Verbose, "Controller did not switch to baud, reset to %d baud.", myAsyncConnectStartBaud);
	serConn->setBaud(myAsyncConnectStartBaud);
	myAsyncConnectState = 5;
      }
      else
	return 0;
    }
  }

  if (myAsyncConnectState == 5)
  {
    if (!myIsStabilizing)
      startStabilization();
    if (myStabilizingTime == 0 ||
	myStartedStabilizing.mSecSince() > myStabilizingTime)
    {
      finishedConnection();
      return 1;
    }
    else
      return 0;
  }

  // here we've gone beyond the initialization, so read set a time limit,
  // read in one packet, then if its a bad packet type, read in all the
  // packets there are to read... if its a good packet, continue with sequence
  myAsyncConnectTimesTried++;
  ArLog::log(ArLog::Normal, "Syncing %d", myAsyncConnectState);
  mySender.com(myAsyncConnectState);
  //  packet = myReceiver.receivePacket(endTime.mSecTo());
  packet = myReceiver.receivePacket(1000);

  if (packet != NULL)
  {
    ret = packet->getID();
    //printf("Got a packet %d\n", ret);

    if (ret == 50)
    {
      ArLog::log(ArLog::Normal, "Attempting to close previous connection.");
      comInt(ArCommands::CLOSE,1);
      while (endTime.mSecTo() > 0)
      {
	timeToWait = endTime.mSecTo();
	if (timeToWait < 0)
	  timeToWait = 0;
	tempPacket = myReceiver.receivePacket(timeToWait);
	if (tempPacket != NULL)
	  ArLog::log(ArLog::Verbose, "Got in another packet!");
	if (tempPacket != NULL && tempPacket->getID() == 50)
	  comInt(ArCommands::CLOSE,1);
      }
      myAsyncConnectState = 0;
    }
    else if (ret == 255)
    {
      ArLog::log(ArLog::Normal, "Attempting to correct syncCount");
      /*
      while (endTime.mSecTo() > 0)
      {
	timeToWait = endTime.mSecTo();
	if (timeToWait < 0)
	  timeToWait = 0;
	tempPacket = myReceiver.receivePacket(timeToWait);
	if (tempPacket != NULL)
	  ArLog::log(ArLog::Verbose, "Got in another packet!");
      }
      */
      while ((tempPacket = myReceiver.receivePacket(0)) != NULL);

      if (tempPacket != NULL && tempPacket->getID() == 0)
	myAsyncConnectState = 1;
      else
	myAsyncConnectState = 0;
      return 0;
    }
    else if  (ret != myAsyncConnectState++)
    {
      myAsyncConnectState = 0;
    }
  }
  else
  {
    ArLog::log(ArLog::Normal, "No packet.");
    myAsyncConnectNoPacketCount++;
    if ((myAsyncConnectNoPacketCount > 5
	 && myAsyncConnectNoPacketCount >= myAsyncConnectTimesTried)
	|| (myAsyncConnectNoPacketCount > 10))
    {
      ArLog::log(ArLog::Terse, "Could not connect, no robot responding.");
      failedConnect();
      return 2;
    }

      /* This code to connect the radio modems should never really be needed
	 so is being removed for now
	 if (myAsyncConnectNoPacketCount >= 4)
	 {

	 if (tryHarderToConnect && dynamic_cast<ArSerialConnection *>(myConn))
	 {
	 ArLog::log(ArLog::Normal,
	 "Radio modem may not connected, trying to connect it.");
	 ArUtil::sleep(1000);
	 myConn->write("|||\15", strlen("!!!\15"));
	 ArUtil::sleep(60);
	 myConn->write("WMN\15", strlen("WMN\15"));
	 ArUtil::sleep(60);
	 myConn->write("WMS2\15", strlen("WMS2\15"));
	 ArUtil::sleep(1000);
	 }
	 else if (dynamic_cast<ArSerialConnection *>(myConn))
	 {
	 ArLog::log(ArLog::Normal,
	 "Radio modem may not connected, trying to connect it.");
	 myConn->write("WMS2\15", strlen("WMS2\15"));
	 ArUtil::sleep(60);
	 }
	 }
      */

    // If we get no response first we dump close commands at the thing
    // in different bauds (if its a serial connection)
    if (myAsyncConnectNoPacketCount == 2 &&
	myAsyncConnectNoPacketCount >= myAsyncConnectTimesTried &&
	(serConn = dynamic_cast<ArSerialConnection *>(myConn)) != NULL)
    {
      int origBaud;
      ArLog::log(ArLog::Normal, "Trying to close possible old connection");
      origBaud = serConn->getBaud();
      serConn->setBaud(9600);
      comInt(ArCommands::CLOSE,1);
      ArUtil::sleep(3);
      serConn->setBaud(38400);
      comInt(ArCommands::CLOSE,1);
      ArUtil::sleep(3);
      serConn->setBaud(115200);
      comInt(ArCommands::CLOSE,1);
      ArUtil::sleep(3);
      serConn->setBaud(19200);
      comInt(ArCommands::CLOSE,1);
      ArUtil::sleep(3);
      serConn->setBaud(57600);
      comInt(ArCommands::CLOSE,1);
      ArUtil::sleep(3);
      serConn->setBaud(origBaud);
    }

    if (myAsyncConnectNoPacketCount > 3)
    {
      ArLog::log(ArLog::Normal,
		 " Robot may be connected but not open, trying to dislodge.");
      mySender.comInt(ArCommands::OPEN,1);
    }

    myAsyncConnectState = 0;
  }
  // if we've connected and have a packet get the connection
  // information from it
  if (myAsyncConnectState == 3 && packet != NULL)
  {
    char nameBuf[512];
    ArLog::log(ArLog::Terse, "Connected to robot.");
    packet->bufToStr(nameBuf, 512);
    myRobotName = nameBuf;
    ArLog::log(ArLog::Normal, "Name: %s", myRobotName.c_str());
    packet->bufToStr(nameBuf, 512);
    myRobotType = nameBuf;
    ArLog::log(ArLog::Normal, "Type: %s", myRobotType.c_str());
    packet->bufToStr(nameBuf, 512);
    myRobotSubType = nameBuf;
    strcpy(robotSubType, myRobotSubType.c_str());
    len = strlen(robotSubType);
    for (i = 0; i < len; i++)
      robotSubType[i] = tolower(robotSubType[i]);
    myRobotSubType = robotSubType;
    ArLog::log(ArLog::Normal, "Subtype: %s", myRobotSubType.c_str());
    ArUtil::sleep(getCycleTime());
    mySender.comInt(ArCommands::OPEN,1);
    if (!madeConnection())
    {
      mySender.comInt(ArCommands::CLOSE,1);
      failedConnect();
      return 2;
    }

    // now we return off so we can handle the rest of connecting, if
    // you're just using this for your own connections you can skip
    // this part because its really connected now
    myAsyncStartedConnection.setToNow();
    return asyncConnectHandler(tryHarderToConnect);
  }
  if (myAsyncConnectTimesTried > 50)
  {
    failedConnect();
    return 2;
  }
  return 0;
}

/**
   @return true if the file could be loaded, false otherwise
 **/
AREXPORT bool ArRobot::loadParamFile(const char *file)
{
  if (myParams != NULL)
    delete myParams;

  myParams = new ArRobotGeneric("");
  if (!myParams->parseFile(file, false, true))
  {
    ArLog::log(ArLog::Normal, "ArRobot::loadParamFile: Could not find file '%s' to load.", file);
    return false;
  }
  processParamFile();
  ArLog::log(ArLog::Normal, "Loaded robot parameters from %s.", file);
  return true;
}

AREXPORT void ArRobot::processParamFile(void)
{
  for (int i = 0; i < myParams->getNumSonar(); ++i)
  {
    //printf("sonar %d %d %d %d\n", i, myParams->getSonarX(i),
    //myParams->getSonarY(i), myParams->getSonarTh(i));
    if (mySonars.find(i) == mySonars.end())
    {
      mySonars[i] = new ArSensorReading(myParams->getSonarX(i),
					myParams->getSonarY(i),
					myParams->getSonarTh(i));
      mySonars[i]->setIgnoreThisReading(true);
    }
    else
    {
      mySonars[i]->resetSensorPosition(myParams->getSonarX(i),
				      myParams->getSonarY(i),
				      myParams->getSonarTh(i));
      mySonars[i]->setIgnoreThisReading(true);
    }
    if ((i + 1) > myNumSonar)
      myNumSonar = i + 1;
  }
  //myRobotType = myParams->getClassName();
  //myRobotSubType = myParams->getSubClassName();
  myAbsoluteMaxTransVel = myParams->getAbsoluteMaxVelocity();
  myAbsoluteMaxRotVel = myParams->getAbsoluteMaxRotVelocity();

  if (myParams->getRobotLengthFront() == 0)
    myRobotLengthFront = myParams->getRobotLength() / 2.0;
  else
    myRobotLengthFront = myParams->getRobotLengthFront();

  if (myParams->getRobotLengthRear() == 0)
    myRobotLengthRear = myParams->getRobotLength() / 2.0;
  else
    myRobotLengthRear = myParams->getRobotLengthRear();
}


AREXPORT bool ArRobot::madeConnection(void)
{
  std::string subTypeParamName;
  std::string paramFileName;
  bool loadedSubTypeParam;
  bool loadedNameParam;
  bool hadDefault = true;

  if (myParams != NULL)
    delete myParams;

  // Find the robot parameters to load and get them into the structure we have
  if (ArUtil::strcmp(myRobotSubType, "p2dx") == 0)
    myParams = new ArRobotP2DX;
  else if (ArUtil::strcmp(myRobotSubType, "p2ce") == 0)
    myParams = new ArRobotP2CE;
  else if (ArUtil::strcmp(myRobotSubType, "p2de") == 0)
    myParams = new ArRobotP2DXe;
  else if (ArUtil::strcmp(myRobotSubType, "p2df") == 0)
    myParams = new ArRobotP2DF;
  else if (ArUtil::strcmp(myRobotSubType, "p2d8") == 0)
    myParams = new ArRobotP2D8;
  else if (ArUtil::strcmp(myRobotSubType, "amigo") == 0)
    myParams = new ArRobotAmigo;
  else if (ArUtil::strcmp(myRobotSubType, "amigo-sh") == 0)
    myParams = new ArRobotAmigoSh;
  else if (ArUtil::strcmp(myRobotSubType, "p2at") == 0)
    myParams = new ArRobotP2AT;
  else if (ArUtil::strcmp(myRobotSubType, "p2at8") == 0)
    myParams = new ArRobotP2AT8;
  else if (ArUtil::strcmp(myRobotSubType, "p2it") == 0)
    myParams = new ArRobotP2IT;
  else if (ArUtil::strcmp(myRobotSubType, "p2pb") == 0)
    myParams = new ArRobotP2PB;
  else if (ArUtil::strcmp(myRobotSubType, "p2pp") == 0)
    myParams = new ArRobotP2PP;
  else if (ArUtil::strcmp(myRobotSubType, "p3at") == 0)
    myParams = new ArRobotP3AT;
  else if (ArUtil::strcmp(myRobotSubType, "p3dx") == 0)
    myParams = new ArRobotP3DX;
  else if (ArUtil::strcmp(myRobotSubType, "perfpb") == 0)
    myParams = new ArRobotPerfPB;
  else if (ArUtil::strcmp(myRobotSubType, "pion1m") == 0)
    myParams = new ArRobotPion1M;
  else if (ArUtil::strcmp(myRobotSubType, "pion1x") == 0)
    myParams = new ArRobotPion1X;
  else if (ArUtil::strcmp(myRobotSubType, "psos1m") == 0)
    myParams = new ArRobotPsos1M;
  else if (ArUtil::strcmp(myRobotSubType, "psos43m") == 0)
    myParams = new ArRobotPsos43M;
  else if (ArUtil::strcmp(myRobotSubType, "psos1x") == 0)
    myParams = new ArRobotPsos1X;
  else if (ArUtil::strcmp(myRobotSubType, "pionat") == 0)
    myParams = new ArRobotPionAT;
  else if (ArUtil::strcmp(myRobotSubType, "mappr") == 0)
    myParams = new ArRobotMapper;
  else if (ArUtil::strcmp(myRobotSubType, "powerbot") == 0)
    myParams = new ArRobotPowerBot;
  else if (ArUtil::strcmp(myRobotSubType, "p2d8+") == 0)
    myParams = new ArRobotP2D8Plus;
  else if (ArUtil::strcmp(myRobotSubType, "p2at8+") == 0)
    myParams = new ArRobotP2AT8Plus;
  else if (ArUtil::strcmp(myRobotSubType, "perfpb+") == 0)
    myParams = new ArRobotPerfPBPlus;
  else if (ArUtil::strcmp(myRobotSubType, "p3dx-sh") == 0)
    myParams = new ArRobotP3DXSH;
  else if (ArUtil::strcmp(myRobotSubType, "p3at-sh") == 0)
    myParams = new ArRobotP3ATSH;
  else if (ArUtil::strcmp(myRobotSubType, "p3atiw-sh") == 0)
    myParams = new ArRobotP3ATIWSH;
  else if (ArUtil::strcmp(myRobotSubType, "patrolbot-sh") == 0)
    myParams = new ArRobotPatrolBotSH;
  else if (ArUtil::strcmp(myRobotSubType, "peoplebot-sh") == 0)
    myParams = new ArRobotPeopleBotSH;
  else if (ArUtil::strcmp(myRobotSubType, "powerbot-sh") == 0)
    myParams = new ArRobotPowerBotSH;
  else if (ArUtil::strcmp(myRobotSubType, "wheelchair-sh") == 0)
    myParams = new ArRobotWheelchairSH;
  else
  {
    hadDefault = false;
    myParams = new ArRobotGeneric(myRobotName.c_str());
  }

  // load up the param file for the subtype
  paramFileName = Aria::getDirectory();
  paramFileName += "params/";
  paramFileName += myRobotSubType;
  paramFileName += ".p";
  if ((loadedSubTypeParam = myParams->parseFile(paramFileName.c_str(), true, true)))
      ArLog::log(ArLog::Normal,
		 "Loaded robot parameters from %s.p",
		 myRobotSubType.c_str());
  /* If the above line was replaced with this one line
     paramFile->load();
     then the sonartest (and lots of other stuff probably) would break
  */
  // then the one for the particular name, if we can
  paramFileName = Aria::getDirectory();
  paramFileName += "params/";
  paramFileName += myRobotName;
  paramFileName += ".p";
  if ((loadedNameParam = myParams->parseFile(paramFileName.c_str(),
					     true, true)))
  {
    if (loadedSubTypeParam)
      ArLog::log(ArLog::Normal,
 "Loaded robot parameters from %s.p on top of %s.p robot parameters",
		 myRobotName.c_str(), myRobotSubType.c_str());
    else
      ArLog::log(ArLog::Normal, "Loaded robot parameters from %s.p",
		 myRobotName.c_str());
  }

  if (!loadedSubTypeParam && !loadedNameParam)
  {
    if (hadDefault)
      ArLog::log(ArLog::Normal, "Using default parameters for a %s robot",
		 myRobotSubType.c_str());
    else
    {
      ArLog::log(ArLog::Terse, "Error: Have no parameters for this robot, bad configuration or out of date Aria");
      // in the default state (not connecting if we don't have params)
      // we will return false... if we can connect without params then
      // we'll keep going (this really shouldn't be used except by
      // downloaders and such)
      if (!myConnectWithNoParams)
	return false;
    }
  }

  processParamFile();

  if (myParams->getRequestIOPackets() || myRequestedIOPackets)
    comInt(ArCommands::IOREQUEST, 2);

  if(myParams->getRequestEncoderPackets() || myRequestedEncoderPackets)
    comInt(ArCommands::ENCODER, 2);

  return true;
}

/** Encoder packet data may then be from
 * getLeftEncoder(), getRightEncoder().
 *
 *  Encoder packets may be stopped with stopEncoderPackets().
 */
AREXPORT void ArRobot::requestEncoderPackets(void)
{
  comInt(ArCommands::ENCODER, 2);
  myRequestedEncoderPackets = true;
}

AREXPORT void ArRobot::requestIOPackets(void)
{
  comInt(ArCommands::IOREQUEST, 2);
  myRequestedIOPackets = true;
}

AREXPORT void ArRobot::stopEncoderPackets(void)
{
  comInt(ArCommands::ENCODER, 0);
  myRequestedEncoderPackets = false;
}

AREXPORT void ArRobot::stopIOPackets(void)
{
  comInt(ArCommands::IOREQUEST, 0);
  myRequestedIOPackets = false;
}

AREXPORT void ArRobot::startStabilization(void)
{
  std::list<ArFunctor *>::iterator it;
  myIsStabilizing = true;
  myStartedStabilizing.setToNow();

  for (it = myStabilizingCBList.begin();
       it != myStabilizingCBList.end();
       it++)
    (*it)->invoke();

}

AREXPORT void ArRobot::finishedConnection(void)
{
  std::list<ArFunctor *>::iterator it;

  myIsStabilizing = false;
  myIsConnected = true;
  myAsyncConnectFlag = false;
  myBlockingConnectRun = false;
  resetOdometer();

  for (it = myConnectCBList.begin(); it != myConnectCBList.end(); it++)
    (*it)->invoke();
  myLastPacketReceivedTime.setToNow();

  wakeAllConnWaitingThreads();
}

AREXPORT void ArRobot::failedConnect(void)
{
  std::list<ArFunctor *>::iterator it;

  myAsyncConnectFlag = false;
  myBlockingConnectRun = false;
  ArLog::log(ArLog::Terse, "Failed to connect to robot.");
  myIsConnected = false;
  for (it = myFailedConnectCBList.begin();
       it != myFailedConnectCBList.end();
       it++)
    (*it)->invoke();

  if (myConn != NULL)
    myConn->close();
  wakeAllConnOrFailWaitingThreads();
}

/**
    Disconnects from a robot.  This also calls of the DisconnectNormally
    Callback Functors if the robot was actually connected to a robot when
    this member was called.
    @return true if not connected to a robot (so no disconnect can happen, but
    it didn't failed either), also true if the command could be sent to the
    robot (ie connection hasn't failed)
**/

AREXPORT bool ArRobot::disconnect(void)
{
  std::list<ArFunctor *>::iterator it;
  bool ret;
  ArSerialConnection *serConn;

  if (!myIsConnected && !myIsStabilizing)
    return true;

  ArLog::log(ArLog::Terse, "Disconnecting from robot.");
  myNoTimeWarningThisCycle = true;
  myIsConnected = false;
  myIsStabilizing = false;
  if (myIsConnected)
  {
    for (it = myDisconnectNormallyCBList.begin();
	 it != myDisconnectNormallyCBList.end();
	 it++)
      (*it)->invoke();
  }
  ret = mySender.comInt(ArCommands::CLOSE, 1);
  ArUtil::sleep(1000);
  if (ret == true)
  {
    if (myConn != NULL)
    {
      ret = myConn->close();
      if ((serConn = dynamic_cast<ArSerialConnection *>(myConn)) != NULL)
      	serConn->setBaud(myAsyncConnectStartBaud);
    }
    else
      ret = false;
  }
  else if (myConn != NULL)
    myConn->close();

  return ret;
}

AREXPORT void ArRobot::dropConnection(void)
{
  std::list<ArFunctor *>::iterator it;

  if (!myIsConnected)
    return;

  ArLog::log(ArLog::Terse, "Lost connection to the robot because of error.");
  myIsConnected = false;
  for (it = myDisconnectOnErrorCBList.begin();
       it != myDisconnectOnErrorCBList.end();
       it++)
    (*it)->invoke();

  if (myConn != NULL)
    myConn->close();
  return;
}

AREXPORT void ArRobot::cancelConnection(void)
{
  //std::list<ArFunctor *>::iterator it;

  ArLog::log(ArLog::Verbose, "Cancelled connection to the robot because of command.");
  myIsConnected = false;
  myNoTimeWarningThisCycle = true;
  myIsStabilizing = false;
  return;
}

/**
   Stops the robot, by telling it to have a translational velocity and
   rotational velocity of 0.  Also note that if you are using actions, this
   will cause the actions to be ignored until the direct motion precedence
   timeout has been exceeded or clearDirectMotion is called.
   @see setDirectMotionPrecedenceTime
   @see getDirectMotionPrecedenceTime
   @see clearDirectMotion
**/
AREXPORT void ArRobot::stop(void)
{
  comInt(ArCommands::VEL, 0);
  comInt(ArCommands::RVEL, 0);
  setVel(0);
  setRotVel(0);
  myLastActionTransVal = 0;
  myLastActionRotStopped = true;
  myLastActionRotHeading = false;
}

/**
   Sets the desired translational velocity of the robot.
   ArRobot caches this value, and sends it with a VEL command during the next cycle.

   @param velocity the desired translational velocity of the robot (mm/sec)
**/
AREXPORT void ArRobot::setVel(double velocity)
{
  myTransType = TRANS_VEL;
  myTransVal = velocity;
  myTransVal2 = 0;
  myTransSetTime.setToNow();
}

/**
    Sets the velocity of each of the wheels on the robot
    independently.  ArRobot caches these values, and sends them with
    a VEL2 command during the next
    cycle.  Note that this cancels both translational velocity AND
    rotational velocity, and is canceled by any of the other direct
    motion commands.
    @sa setVel
    @sa setRotVel
    @sa clearDirectMotion

    @param leftVelocity the desired velocity of the left wheel
    @param rightVelocity the desired velocity of the right wheel
**/
AREXPORT void ArRobot::setVel2(double leftVelocity, double rightVelocity)
{
  myTransType = TRANS_VEL2;
  myTransVal = leftVelocity;
  myTransVal2 = rightVelocity;
  myRotType = ROT_IGNORE;
  myRotVal = 0;
  myTransSetTime.setToNow();
}

/**
   Tells the robot to begin moving the specified distance forward/backwards.
   ArRobot caches this value, and sends it during the next cycle (with a MOVE
   or VEL command, depending on the robot).

   @param distance the distance for the robot to move (millimeters). Note,
    due to the implementation of the MOVE command on some robots, it is best to
    restrict the distance to the range (5000mm, 5000mm] if possible. Not all
    robots have this restriction, however.
**/
AREXPORT void ArRobot::move(double distance)
{
  myTransType = TRANS_DIST_NEW;
  myTransDistStart = getPose();
  myTransVal = distance;
  myTransVal2 = 0;
  myTransSetTime.setToNow();
}

/**
   Determines if a move command is finished, to within a small
   distance threshold, "delta".  If delta = 0 (default), the delta distance
   set with setMoveDoneDist() will be used.

   @param delta how close to the goal distance the robot must be, or 0 for previously set default

   @return true if the robot has finished the distance given in a move
   command or if the robot is no longer in a move mode (because its
   now running off of actions, or setVel() or setVel2() was called).

   @sa setMoveDoneDist
   @sa getMoveDoneDist
**/
AREXPORT bool ArRobot::isMoveDone(double delta)
{
  if (fabs(delta) < 0.001)
    delta = myMoveDoneDist;
  if (myTransType != TRANS_DIST && myTransType != TRANS_DIST_NEW)
    return true;		// no distance command operative
  if (myTransDistStart.findDistanceTo(getPose()) <
      fabs(myTransVal) - delta)
    return false;
  return true;
}


/**
   Determines if a setHeading() command is finished, to within a small
   distance.  If delta = 0 (default), the delta distance is what was
   set with setHeadingDoneDiff(), you can get the distnace with
   getHeadingDoneDiff()

   @param delta how close to the goal distance the robot must be

   @return true if the robot has achieved the heading given in a move
   command or if the robot is no longer in heading mode mode (because
   its now running off of actions, setDHeading(), or setRotVel() was called).
**/
AREXPORT bool ArRobot::isHeadingDone(double delta) const
{
  if (fabs(delta) < 0.001)
    delta = myHeadingDoneDiff;
  if (myRotType != ROT_HEADING)
    return true;		// no heading command operative
  if(fabs(ArMath::subAngle(getTh(), myRotVal)) > delta)
    return false;
  return true;
}



/**
   Sets the heading of the robot, it caches this value, and sends it
   during the next cycle.

   @param heading the desired heading of the robot (degrees)
**/
AREXPORT void ArRobot::setHeading(double heading)
{
  myRotVal = heading;
  myRotType = ROT_HEADING;
  myRotSetTime.setToNow();
  if (myTransType == TRANS_VEL2)
  {
    myTransType = TRANS_IGNORE;
    myTransVal = 0;
    myTransVal2 = 0;
  }
}

/**
   Sets the rotational velocity of the robot, it caches this value,
   and sends it during the next cycle.

   @param velocity the desired rotational velocity of the robot (deg/sec)
**/
AREXPORT void ArRobot::setRotVel(double velocity)
{
  myRotVal = velocity;
  myRotType = ROT_VEL;
  myRotSetTime.setToNow();
  if (myTransType == TRANS_VEL2)
  {
    myTransType = TRANS_IGNORE;
    myTransVal = 0;
    myTransVal2 = 0;
  }
}

/**
   Sets a delta heading to the robot, it caches this value, and sends
   it during the next cycle.

   @param deltaHeading the desired amount to change the heading of the robot by
**/
AREXPORT void ArRobot::setDeltaHeading(double deltaHeading)
{
  myRotVal = ArMath::addAngle(getTh(), deltaHeading);
  myRotType = ROT_HEADING;
  myRotSetTime.setToNow();
  if (myTransType == TRANS_VEL2)
  {
    myTransType = TRANS_IGNORE;
    myTransVal = 0;
    myTransVal2 = 0;
  }
}

/**
   This sets the absolute maximum velocity the robot will go... the
   maximum velocity can also be set by the actions and by
   setTransVelMax, but it will not be allowed to go higher than this
   value.  You should not set this very often, if you want to
   manipulate this value you should use the actions or setTransVelMax.

   @param maxVel the maximum velocity to be set, it must be a non-zero
   number

   @return true if the value is good, false othrewise
**/

AREXPORT bool ArRobot::setAbsoluteMaxTransVel(double maxVel)
{
  if (maxVel < 0)
    return false;
  myAbsoluteMaxTransVel = maxVel;
  if (getTransVelMax() > myAbsoluteMaxTransVel)
    setTransVelMax(myAbsoluteMaxTransVel);
  return true;
}

/**
   This sets the absolute maximum translational acceleration the robot
   will do... the acceleration can also be set by the actions and by
   setTransAccel, but it will not be allowed to go higher than this
   value.  You should not set this very often, if you want to
   manipulate this value you should use the actions or setTransAccel.

   @param maxAccel the maximum acceleration to be set, it must be a non-zero
   number

   @return true if the value is good, false othrewise
**/

AREXPORT bool ArRobot::setAbsoluteMaxTransAccel(double maxAccel)
{
  if (maxAccel < 0)
    return false;
  myAbsoluteMaxTransAccel = maxAccel;
  if (getTransAccel() > myAbsoluteMaxTransAccel)
    setTransAccel(myAbsoluteMaxTransAccel);
  return true;
}

/**
   This sets the absolute maximum translational deceleration the robot
   will do... the deceleration can also be set by the actions and by
   setTransDecel, but it will not be allowed to go higher than this
   value.  You should not set this very often, if you want to
   manipulate this value you should use the actions or setTransDecel.

   @param maxDecel the maximum deceleration to be set, it must be a non-zero
   number

   @return true if the value is good, false othrewise
**/

AREXPORT bool ArRobot::setAbsoluteMaxTransDecel(double maxDecel)
{
  if (maxDecel < 0)
    return false;
  myAbsoluteMaxTransDecel = maxDecel;
  if (getTransDecel() > myAbsoluteMaxTransDecel)
    setTransDecel(myAbsoluteMaxTransDecel);
  return true;
}

/**
   This sets the absolute maximum velocity the robot will go... the
   maximum velocity can also be set by the actions and by
   setRotVelMax, but it will not be allowed to go higher than this
   value.  You should not set this very often, if you want to
   manipulate this value you should use the actions or setRotVelMax.

   @param maxVel the maximum velocity to be set, it must be a non-zero number
   @return true if the value is good, false othrewise
**/

AREXPORT bool ArRobot::setAbsoluteMaxRotVel(double maxVel)
{
  if (maxVel < 0)
    return false;
  myAbsoluteMaxRotVel = maxVel;
  if (getRotVelMax() > myAbsoluteMaxRotVel)
    setRotVelMax(myAbsoluteMaxRotVel);
  return true;
}

/**
   This sets the absolute maximum rotational acceleration the robot
   will do... the acceleration can also be set by the actions and by
   setRotAccel, but it will not be allowed to go higher than this
   value.  You should not set this very often, if you want to
   manipulate this value you should use the actions or setRotAccel.

   @param maxAccel the maximum acceleration to be set, it must be a non-zero
   number

   @return true if the value is good, false othrewise
**/

AREXPORT bool ArRobot::setAbsoluteMaxRotAccel(double maxAccel)
{
  if (maxAccel < 0)
    return false;
  myAbsoluteMaxRotAccel = maxAccel;
  if (getRotAccel() > myAbsoluteMaxRotAccel)
    setRotAccel(myAbsoluteMaxRotAccel);
  return true;
}

/**
   This sets the absolute maximum rotational deceleration the robot
   will do... the deceleration can also be set by the actions and by
   setRotDecel, but it will not be allowed to go higher than this
   value.  You should not set this very often, if you want to
   manipulate this value you should use the actions or setRotDecel.

   @param maxDecel the maximum deceleration to be set, it must be a non-zero
   number

   @return true if the value is good, false othrewise
**/

AREXPORT bool ArRobot::setAbsoluteMaxRotDecel(double maxDecel)
{
  if (maxDecel < 0)
    return false;
  myAbsoluteMaxRotDecel = maxDecel;
  if (getRotDecel() > myAbsoluteMaxRotDecel)
    setRotDecel(myAbsoluteMaxRotDecel);
  return true;
}

/**
   This gets the raw IO Analog value, which is a number between 0 and
   1024 (2^10).
 **/
AREXPORT int ArRobot::getIOAnalog(int num) const
{
  if (num <= getIOAnalogSize())
    return myIOAnalog[num];
  else
    return 0;
}

/**
   This gets the IO Analog value converted to a voltage between 0 and 5 voltes.
 **/
AREXPORT double ArRobot::getIOAnalogVoltage(int num) const
{
  if (num <= getIOAnalogSize())
  {
    return (myIOAnalog[num] & 0xfff) * .0048828;
  }
  else
    return 0;
}

AREXPORT unsigned char ArRobot::getIODigIn(int num) const
{
  if (num <= getIODigInSize())
    return myIODigIn[num];
  else
    return (unsigned char) 0;
}

AREXPORT unsigned char  ArRobot::getIODigOut(int num) const
{
  if (num <= getIODigOutSize())
    return myIODigOut[num];
  else
    return (unsigned char) 0;
}

/**
    @return the ArRobotParams instance the robot is using for its parameters
**/
AREXPORT const ArRobotParams *ArRobot::getRobotParams(void) const
{
  return myParams;
}

/**
    @return the ArRobotConfigPacketReader taken when this instance got
    connected to the robot
**/
AREXPORT const ArRobotConfigPacketReader *ArRobot::getOrigRobotConfig(void) const
{
  return myOrigRobotConfig;
}

/**
   Adds a packet handler.  A packet handler is an ArRetFunctor1<bool, ArRobotPacket*>,
   (e.g.  created as an instance of ArRetFunctor1C.  The return is a boolean, while the functor
   takes an ArRobotPacket pointer as the argument.  This functor is placed in
   a list of functors to call when a packet arrives. This list is processed
   in order until one of the handlers returns true. Your packet handler
   function may be invoked for any packet, so it should test the packet type
   ID (see ArRobotPacket::getID()). If you handler gets data from the packet
   (it "handles" it) it should return true, to prevent ArRobot from invoking
   other handlers with the packet (with data removed). If you hander
   cannot interpret the packet, it should leave it unmodified and return
   false to allow other handlers a chance to receive it.
   @param functor the functor to call when the packet comes in
   @param position whether to place the functor first or last
   @see remPacketHandler
**/
AREXPORT void ArRobot::addPacketHandler(
	ArRetFunctor1<bool, ArRobotPacket *> *functor,
	ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myPacketHandlerList.push_front(functor);
  else if (position == ArListPos::LAST)
    myPacketHandlerList.push_back(functor);
  else
    ArLog::log(ArLog::Terse, "ArRobot::addPacketHandler: Invalid position.");

}

/**
   @param functor the functor to remove from the list of packet handlers
   @see addPacketHandler
**/
AREXPORT void ArRobot::remPacketHandler(
	ArRetFunctor1<bool, ArRobotPacket *> *functor)
{
  myPacketHandlerList.remove(functor);
}

/**
   Adds a connect callback, which is an ArFunctor, (created as an ArFunctorC).
   The entire list of connect callbacks is called when a connection is made
   with the robot.  If you have some sort of module that adds a callback,
   that module must remove the callback when the module is removed.
   @param functor A functor (created from ArFunctorC) which refers to the
   function to call.
   @param position whether to place the functor first or last
   @see remConnectCB
**/
AREXPORT void ArRobot::addConnectCB(ArFunctor *functor,
				    ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myConnectCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myConnectCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArRobot::addConnectCallback: Invalid position.");
}

/**
    @param functor the functor to remove from the list of connect callbacks
    @see addConnectCB
**/
AREXPORT void ArRobot::remConnectCB(ArFunctor *functor)
{
  myConnectCBList.remove(functor);
}


/** Adds a failed connect callback,which is an ArFunctor, created as an
    ArFunctorC.  This whole list of failed connect callbacks is called when
    an attempt is made to connect to the robot, but fails.   The usual reason
    for this failure is either that there is no robot/sim where the connection
    was tried to be made, the robot wasn't given a connection, or the radio
    modems that communicate with the robot aren't on.  If you have some sort
    of module that adds a callback, that module must remove the callback
    when the module removed.
    @param functor functor created from ArFunctorC which refers to the
    function to call.
    @param position whether to place the functor first or last
    @see remFailedConnectCB
**/
AREXPORT void ArRobot::addFailedConnectCB(ArFunctor *functor,
					  ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myFailedConnectCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myFailedConnectCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArRobot::addFailedCallback: Invalid position.");
}

/**
    @param functor the functor to remove from the list of connect callbacks
    @see addFailedConnectCB
**/
AREXPORT void ArRobot::remFailedConnectCB(ArFunctor *functor)
{
  myFailedConnectCBList.remove(functor);
}

/** Adds a disconnect normally callback,which is an ArFunctor, created as an
    ArFunctorC.  This whole list of disconnect normally callbacks is called
    when something calls disconnect if the instance isConnected.  If there is
    no connection and disconnect is called nothing is done.  If you have some
    sort of module that adds a callback, that module must remove the callback
    when the module is removed.
    @param functor functor created from ArFunctorC which refers to the
    function to call.
    @param position whether to place the functor first or last
    @see remFailedConnectCB
**/
AREXPORT void ArRobot::addDisconnectNormallyCB(ArFunctor *functor,
					       ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myDisconnectNormallyCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myDisconnectNormallyCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArRobot::addDisconnectNormallyCB: Invalid position.");
}

/**
    @param functor the functor to remove from the list of connect callbacks
    @see addDisconnectNormallyCB
**/
AREXPORT void ArRobot::remDisconnectNormallyCB(ArFunctor *functor)
{
  myDisconnectNormallyCBList.remove(functor);
}

/** Adds a disconnect on error callback, which is an ArFunctor, created as an
    ArFunctorC.  This whole list of disconnect on error callbacks is called
    when ARIA loses connection to a robot because of an error.  This can occur
    if the physical connection (ie serial cable) between the robot and the
    computer is severed/disconnected, if one of a pair of radio modems that
    connect the robot and computer are disconnected, if someone presses the
    reset button on the robot, or if the simulator is closed while ARIA
    is connected to it.  Note that if the link between the two is lost the
    ARIA assumes it is temporary until it reaches a timeout value set with
    setConnectionTimeoutTime.  If you have some sort of module that adds a
    callback, that module must remove the callback when the module removed.
    @param functor functor created from ArFunctorC which refers to the
    function to call.
    @param position whether to place the functor first or last
    @see remDisconnectOnErrorCB
**/
AREXPORT void ArRobot::addDisconnectOnErrorCB(ArFunctor *functor,
					      ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myDisconnectOnErrorCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myDisconnectOnErrorCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArRobot::addDisconnectOnErrorCB: Invalid position");
}

/**
    @param functor the functor to remove from the list of connect callbacks
    @see addDisconnectOnErrorCB
**/
AREXPORT void ArRobot::remDisconnectOnErrorCB(ArFunctor *functor)
{
  myDisconnectOnErrorCBList.remove(functor);
}

/**
   Adds a callback that is called when the run loop exits. The functor is
   which is an ArFunctor, created as an ArFunctorC. The whole list of
   functors is called when the run loop exits. This is most usefull for
   threaded programs that run the robot using ArRobot::runAsync. This will
   allow user threads to know when the robot loop has exited.
   @param functor functor created from ArFunctorC which refers to the
   function to call.
   @param position whether to place the functor first or last
   @see remRunExitCB
**/
AREXPORT void ArRobot::addRunExitCB(ArFunctor *functor,
				    ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myRunExitCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myRunExitCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse, "ArRobot::addRunExitCB: Invalid position");
}

/**
    @param functor  The functor to remove from the list of run exit callbacks
    @see addRunExitCB
**/
AREXPORT void ArRobot::remRunExitCB(ArFunctor *functor)
{
  myRunExitCBList.remove(functor);
}

/**
   Adds a stablizing callback, which is an ArFunctor, created as an
   ArFunctorC.  The entire list of connect callbacks is called just
   before the connection is called done to the robot.  This time can
   be used to calibtrate readings (on things like gyros).

   @param functor The functor to call (e.g. ArFunctorC)

   @param position whether to place the functor first or last

   @see remConnectCB
**/
AREXPORT void ArRobot::addStabilizingCB(ArFunctor *functor,
				       ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myStabilizingCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myStabilizingCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArRobot::addConnectCallback: Invalid position.");
}

/**
    @param functor the functor to remove from the list of stabilizing callbacks
    @see addConnectCB
**/
AREXPORT void ArRobot::remStabilizingCB(ArFunctor *functor)
{
  myStabilizingCBList.remove(functor);
}


AREXPORT std::list<ArFunctor *> * ArRobot::getRunExitListCopy()
{
  return(new std::list<ArFunctor *>(myRunExitCBList));
}

/**
   This will suspend the calling thread until the ArRobot's run loop has
   managed to connect with the robot. There is an optional paramater of
   milliseconds to wait for the ArRobot to connect. If msecs is set to 0,
   it will wait until the ArRobot connects. This function will never
   return if the robot can not be connected with. If you want to be able
   to handle that case within the calling thread, you must call
   waitForConnectOrConnFail().
   @param msecs milliseconds in which to wait for the ArRobot to connect
   @return WAIT_CONNECTED for success
   @see waitForConnectOrConnFail
   @see wakeAllWaitingThreads
   @see wakeAllConnWaitingThreads
   @see wakeAllRunExitWaitingThreads
**/
AREXPORT ArRobot::WaitState ArRobot::waitForConnect(unsigned int msecs)
{
  int ret;

  if (isConnected())
    return(WAIT_CONNECTED);

  if (msecs == 0)
    ret=myConnectCond.wait();
  else
    ret=myConnectCond.timedWait(msecs);

  if (ret == ArCondition::STATUS_WAIT_INTR)
    return(WAIT_INTR);
  else if (ret == ArCondition::STATUS_WAIT_TIMEDOUT)
    return(WAIT_TIMEDOUT);
  else if (ret == 0)
    return(WAIT_CONNECTED);
  else
    return(WAIT_FAIL);
}

/**
   This will suspend the calling thread until the ArRobot's run loop has
   managed to connect with the robot or fails to connect with the robot.
   There is an optional paramater of milliseconds to wait for the ArRobot
   to connect. If msecs is set to 0, it will wait until the ArRobot connects.
   @param msecs milliseconds in which to wait for the ArRobot to connect
   @return WAIT_CONNECTED for success
   @see waitForConnect
**/
AREXPORT ArRobot::WaitState
ArRobot::waitForConnectOrConnFail(unsigned int msecs)
{
  int ret;

  if (isConnected())
    return(WAIT_CONNECTED);

  if (msecs == 0)
    ret=myConnOrFailCond.wait();
  else
    ret=myConnOrFailCond.timedWait(msecs);

  if (ret == ArCondition::STATUS_WAIT_INTR)
    return(WAIT_INTR);
  else if (ret == ArCondition::STATUS_WAIT_TIMEDOUT)
    return(WAIT_TIMEDOUT);
  else if (ret == 0)
  {
    if (isConnected())
      return(WAIT_CONNECTED);
    else
      return(WAIT_FAILED_CONN);
  }
  else
    return(WAIT_FAIL);
}

/**
   This will suspend the calling thread until the ArRobot's run loop has
   exited. There is an optional paramater of milliseconds to wait for the
   ArRobot run loop to exit . If msecs is set to 0, it will wait until
   the ArRrobot run loop exits.
   @param msecs milliseconds in which to wait for the robot to connect
   @return WAIT_RUN_EXIT for success
**/
AREXPORT ArRobot::WaitState ArRobot::waitForRunExit(unsigned int msecs)
{
  int ret;

  if (!isRunning())
    return(WAIT_RUN_EXIT);

  if (msecs == 0)
    ret=myRunExitCond.wait();
  else
    ret=myRunExitCond.timedWait(msecs);

  if (ret == ArCondition::STATUS_WAIT_INTR)
    return(WAIT_INTR);
  else if (ret == ArCondition::STATUS_WAIT_TIMEDOUT)
    return(WAIT_TIMEDOUT);
  else if (ret == 0)
    return(WAIT_RUN_EXIT);
  else
    return(WAIT_FAIL);
}

/**
   This will wake all the threads waiting for various major state changes
   in this particular ArRobot. This includes all threads waiting for the
   robot to be connected and all threads waiting for the run loop to exit.
   @see wakeAllConnWaitingThreads
   @see wakeAllRunExitWaitingThreads
**/
AREXPORT void ArRobot::wakeAllWaitingThreads()
{
  wakeAllConnWaitingThreads();
  wakeAllRunExitWaitingThreads();
}

/**
   This will wake all the threads waiting for the robot to be connected.
   @see wakeAllWaitingThreads
   @see wakeAllRunExitWaitingThreads
**/
AREXPORT void ArRobot::wakeAllConnWaitingThreads()
{
  myConnectCond.broadcast();
  myConnOrFailCond.broadcast();
}

/**
   This will wake all the threads waiting for the robot to be connected or
   waiting for the robot to fail to connect.
   @see wakeAllWaitingThreads
   @see wakeAllRunExitWaitingThreads
**/
AREXPORT void ArRobot::wakeAllConnOrFailWaitingThreads()
{
  myConnOrFailCond.broadcast();
}

/**
   This will wake all the threads waiting for the run loop to exit.
   @see wakeAllWaitingThreads
   @see wakeAllConnWaitingThreads
**/
AREXPORT void ArRobot::wakeAllRunExitWaitingThreads()
{
  myRunExitCond.broadcast();
}

/**
   This gets the root of the synchronous task tree, so that someone can add
   their own new types of tasks, or find out more information about
   each task... only serious developers should use this.
   @return the root of the sycnhronous task tree
   @see ArSyncTask
**/
AREXPORT ArSyncTask *ArRobot::getSyncTaskRoot(void)
{
  return mySyncTaskRoot;
}

/**
   The synchronous tasks get called every robot cycle (every 100 ms by
   default).

   @param name the name to give to the task, should be unique

   @param position the place in the list of user tasks to place this
   task, this can be any integer, though by convention 0 to 100 is
   used.  The tasks are called in order of highest number to lowest
   position number.

   @param functor functor created from ArFunctorC which refers to the
   function to call.

   @see remUserTask
**/
AREXPORT bool ArRobot::addUserTask(const char *name, int position,
				      ArFunctor *functor,
				      ArTaskState::State *state)
{
  ArSyncTask *proc;
  if (mySyncTaskRoot == NULL)
    return false;

  proc = mySyncTaskRoot->findNonRecursive("User Tasks");
  if (proc == NULL)
    return false;

  proc->addNewLeaf(name, position, functor, state);
  return true;
}

/**
   @see addUserTask
   @see remUserTask(ArFunctor *functor)
**/
AREXPORT void ArRobot::remUserTask(const char *name)
{
  ArSyncTask *proc;
  ArSyncTask *userProc;

  if (mySyncTaskRoot == NULL)
    return;

  proc = mySyncTaskRoot->findNonRecursive("User Tasks");
  if (proc == NULL)
    return;

  userProc = proc->findNonRecursive(name);
  if (userProc == NULL)
    return;


  delete userProc;

}

/**
   @see addUserTask
   @see remUserTask(std::string name)
**/
AREXPORT void ArRobot::remUserTask(ArFunctor *functor)
{
  ArSyncTask *proc;
  ArSyncTask *userProc;

  if (mySyncTaskRoot == NULL)
    return;

  proc = mySyncTaskRoot->findNonRecursive("User Tasks");
  if (proc == NULL)
    return;


  userProc = proc->findNonRecursive(functor);
  if (userProc == NULL)
    return;


  delete userProc;

}

/**
   The synchronous tasks get called every robot cycle (every 100 ms by
   default).
   @param name the name to give to the task, should be unique
   @param position the place in the list of user tasks to place this
   task, this can be any integer, though by convention 0 to 100 is used.
   The tasks are called in order of highest number to lowest number.
   @param functor functor created from ArFunctorC which refers to the
   function to call.
   @see remSensorInterpTask
**/
AREXPORT bool ArRobot::addSensorInterpTask(const char *name, int position,
					      ArFunctor *functor,
					      ArTaskState::State *state)
{
  ArSyncTask *proc;
  if (mySyncTaskRoot == NULL)
    return false;

  proc = mySyncTaskRoot->findNonRecursive("Sensor Interp");
  if (proc == NULL)
    return false;

  proc->addNewLeaf(name, position, functor, state);
  return true;
}

/**
   @see addSensorInterpTask
   @see remSensorInterpTask(ArFunctor *functor)
**/
AREXPORT void ArRobot::remSensorInterpTask(const char *name)
{
  ArSyncTask *proc;
  ArSyncTask *sensorInterpProc;

  if (mySyncTaskRoot == NULL)
    return;

  proc = mySyncTaskRoot->findNonRecursive("Sensor Interp");
  if (proc == NULL)
    return;

  sensorInterpProc = proc->findNonRecursive(name);
  if (sensorInterpProc == NULL)
    return;


  delete sensorInterpProc;

}

/**
   @see addSensorInterpTask
   @see remSensorInterpTask(std::string name)
**/
AREXPORT void ArRobot::remSensorInterpTask(ArFunctor *functor)
{
  ArSyncTask *proc;
  ArSyncTask *sensorInterpProc;

  if (mySyncTaskRoot == NULL)
    return;

  proc = mySyncTaskRoot->findNonRecursive("Sensor Interp");
  if (proc == NULL)
    return;


  sensorInterpProc = proc->findNonRecursive(functor);
  if (sensorInterpProc == NULL)
    return;


  delete sensorInterpProc;

}

/**
    @see ArLog
**/
AREXPORT void ArRobot::logUserTasks(void) const
{
  ArSyncTask *proc;
  if (mySyncTaskRoot == NULL)
    return;

  proc = mySyncTaskRoot->findNonRecursive("User Tasks");
  if (proc == NULL)
    return;

  proc->log();
}

/**
   @see ArLog
**/
AREXPORT void ArRobot::logAllTasks(void) const
{
  if (mySyncTaskRoot != NULL)
    mySyncTaskRoot->log();
}

/**
   Finds a user task by its name, searching the entire space of tasks
   @return NULL if no user task of that name found, otherwise a pointer to
   the ArSyncTask for the first task found with that name
**/
AREXPORT ArSyncTask *ArRobot::findUserTask(const char *name)
{
  ArSyncTask *proc;
  if (mySyncTaskRoot == NULL)
    return NULL;

  proc = mySyncTaskRoot->findNonRecursive("User Tasks");
  if (proc == NULL)
    return NULL;

  return proc->find(name);
}

/**
   Finds a user task by its functor, searching the entire space of tasks
   @return NULL if no user task with that functor found, otherwise a pointer
   to the ArSyncTask for the first task found with that functor
**/
AREXPORT ArSyncTask *ArRobot::findUserTask(ArFunctor *functor)
{
  ArSyncTask *proc;
  if (mySyncTaskRoot == NULL)
    return NULL;

  proc = mySyncTaskRoot->findNonRecursive("User Tasks");
  if (proc == NULL)
    return NULL;

  return proc->find(functor);
}

/**
   Finds a task by its name, searching the entire space of tasks
   @return NULL if no task of that name found, otherwise a pointer to the
   ArSyncTask for the first task found with that name
**/
AREXPORT ArSyncTask *ArRobot::findTask(const char *name)
{
  if (mySyncTaskRoot != NULL)
    return mySyncTaskRoot->find(name);
  else
    return NULL;

}

/**
   Finds a task by its functor, searching the entire space of tasks
   @return NULL if no task with that functor found, otherwise a pointer
   to the ArSyncTask for the first task found with that functor
**/
AREXPORT ArSyncTask *ArRobot::findTask(ArFunctor *functor)
{
  if (mySyncTaskRoot != NULL)
    return mySyncTaskRoot->find(functor);
  else
    return NULL;

}

/**
    Adds an action to the list of actions with the given priority. In
    the case of two (or more) actions with the same priority, the
    default resolver (ArPriorityResolver) averages the the multiple
    readings. The priority can be any integer, but as a convention 0
    to 100 is used, with 100 being the highest priority. The default
    resolver (ArPriorityResolver) resolves the actions in order of descending
    priority. For example, an action with priority 100 is evaluated before
    one with priority 99, followed by 50, etc.  This means that an action with
    a higher priority may be able to supercede a lower-priority action's
    desired value for a certain output to a lesser or greater degree, depending
    on how high a "strength" value it sets.  See the overview of ARIA in this
    reference manual for more discussion on Actions.

    @param action the action to add
    @param priority what importance to give the action; how to order the actions.  High priority actions are evaluated by the action resolvel before lower priority actions.
    @return true if the action was successfully added, false on error (e.g. the action was NULL)
    @sa remAction(ArAction*)
    @sa remAction(const char*)
    @sa findAction(const char*)
*/
AREXPORT bool ArRobot::addAction(ArAction *action, int priority)
{
  if (action == NULL)
  {
    ArLog::log(ArLog::Terse,
      "ArRobot::addAction: an attempt was made to add a NULL action pointer");
    return false;
  }

  action->setRobot(this);
  myActions.insert(std::pair<int, ArAction *>(priority, action));
  return true;
}

/**
   Finds the action with the given name and removes it from the actions...
   if more than one action has that name it find the one with the lowest
   priority
   @param actionName the name of the action we want to find
   @return whether remAction found anything with that action to remove or not
   @sa addAction()
   @sa remAction(ArAction*)
   @sa findAction(const char*)
**/
AREXPORT bool ArRobot::remAction(const char *actionName)
{
  ArResolver::ActionMap::iterator it;
  ArAction *act;

  for (it = myActions.begin(); it != myActions.end(); ++it)
  {
    act = (*it).second;
    if (strcmp(actionName, act->getName()) == 0)
      break;
  }
  if (it != myActions.end())
  {
    myActions.erase(it);
    return true;
  }
  return false;

}

/**
   Finds the action with the given pointer and removes it from the actions...
   if more than one action has that pointer it find the one with the lowest
   priority
   @param action the action we want to remove
   @return whether remAction found anything with that action to remove or not
   *  @sa addAction
   *  @sa remAction(const char*)
   *  @sa findAction(const char*)
**/
AREXPORT bool ArRobot::remAction(ArAction *action)
{
  ArResolver::ActionMap::iterator it;
  ArAction *act;

  for (it = myActions.begin(); it != myActions.end(); ++it)
  {
    act = (*it).second;
    if (act == action)
      break;
  }
  if (it != myActions.end())
  {
    myActions.erase(it);
    return true;
  }
  return false;

}


/**
   Finds the action with the given name... if more than one action
   has that name it find the one with the highest priority
   @param actionName the name of the action we want to find
   @return the action, if found.  If not found, NULL
   @sa addAction
   @sa remAction(ArAction*)
   @sa remAction(const char*)
**/
AREXPORT ArAction *ArRobot::findAction(const char *actionName)
{
  ArResolver::ActionMap::reverse_iterator it;
  ArAction *act;

  for (it = myActions.rbegin(); it != myActions.rend(); ++it)
  {
    act = (*it).second;
    if (strcmp(actionName, act->getName()) == 0)
      return act;
  }
  return NULL;
}

/**
 * @internal
   This returns the actionMap the robot has... do not mess with this
   list except by using ArRobot::addAction() and ArRobot::remAction()...
   This is jsut for the things like ArActionGroup that want to
   deactivate or activate all the actions (well, only deactivating
   everything makes sense).
   @return the actions the robot is using
**/

AREXPORT ArResolver::ActionMap *ArRobot::getActionMap(void)
{
  return &myActions;
}

AREXPORT void ArRobot::deactivateActions(void)
{
  ArResolver::ActionMap *am;
  ArResolver::ActionMap::iterator amit;

  am = getActionMap();
  if (am == NULL)
  {
    ArLog::log(ArLog::Terse,
            "ArRobot::deactivateActions: NULL action map... failed.");
    return;
  }
  for (amit = am->begin(); amit != am->end(); amit++)
    (*amit).second->deactivate();


}


AREXPORT void ArRobot::logActions(bool logDeactivated) const
{
  ArResolver::ActionMap::const_reverse_iterator it;
  int lastPriority=0;
  bool first = true;
  const ArAction *action;

  if (logDeactivated)
    ArLog::log(ArLog::Terse, "The action list (%d total):",
	       myActions.size());
  else
    ArLog::log(ArLog::Terse, "The active action list:");

  for (it = myActions.rbegin(); it != myActions.rend(); ++it)
  {
    action = (*it).second;
    if ((logDeactivated || action->isActive()) &&
	(first || lastPriority != (*it).first))
    {
      ArLog::log(ArLog::Terse, "Priority %d:", (*it).first);
      first = false;
      lastPriority = (*it).first;
    }
    action->log(false);
  }
}

AREXPORT ArResolver *ArRobot::getResolver(void)
{
  return myResolver;
}

AREXPORT void ArRobot::setResolver(ArResolver *resolver)
{
  if (myOwnTheResolver)
  {
    delete myResolver;
    myResolver = NULL;
  }

  myResolver = resolver;
}

/**
 * @internal
 *
   If state reflecting (really direct motion command reflecting) was
   enabled in the constructor (ArRobot::ArRobot) then this will see if
   there are any direct motion commands to send, and if not then send
   the command given by the actions.  If state reflection is disabled
   this will send a pulse to the robot every state reflection refresh
   time (setStateReflectionRefreshTime), if you don't wish this to happen
   simply set this to a very large value.
**/
AREXPORT void ArRobot::stateReflector(void)
{
  short transVal;
  short transVal2;
  short maxVel;
  short maxNegVel;
  double maxTransVel;
  double maxNegTransVel;
  double maxRotVel;
  double transAccel;
  double transDecel;
  short rotVal;
  double rotAccel;
  double rotDecel;
  bool rotStopped = false;
  bool rotHeading = false;
  double encTh=0;
  double rawTh;


  if (!myIsConnected)
    return;

  myTryingToMove = false;

  // if this is true actions can't go
  if ((myTransType != TRANS_NONE && myDirectPrecedenceTime == 0) ||
      (myTransType != TRANS_NONE && myDirectPrecedenceTime != 0 &&
       myTransSetTime.mSecSince() < myDirectPrecedenceTime))
  {
    myActionTransSet = false;
    transVal = ArMath::roundShort(myTransVal);
    transVal2 = 0;

    if (hasSettableVelMaxes() &&
	ArMath::fabs(myLastSentTransVelMax - myTransVelMax) >= 1)
    {
      comInt(ArCommands::SETV,
	     ArMath::roundShort(myTransVelMax));
      myLastSentTransVelMax = myTransVelMax;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "Non-action trans max vel of %d",
		   ArMath::roundShort(myTransVelMax));
    }

    if (hasSettableAccsDecs() && ArMath::fabs(myTransAccel) > 1 &&
	ArMath::fabs(myLastSentTransAccel - myTransAccel) >= 1)
    {
      comInt(ArCommands::SETA,
	     ArMath::roundShort(myTransAccel));
      myLastSentTransAccel = myTransAccel;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "Non-action trans accel of %d",
		   ArMath::roundShort(myTransAccel));
    }

    if (hasSettableAccsDecs() && ArMath::fabs(myTransDecel) > 1 &&
	ArMath::fabs(myLastSentTransDecel - myTransDecel) >= 1)
    {
      comInt(ArCommands::SETA,
	     -ArMath::roundShort(myTransDecel));
      myLastSentTransDecel = myTransDecel;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "Non-action trans decel of %d",
		   -ArMath::roundShort(myTransDecel));
    }

    if (myTransType == TRANS_VEL)
    {
      maxVel = ArMath::roundShort(myTransVelMax);
      if (transVal > maxVel)
	transVal = maxVel;
      if (transVal < -maxVel)
	transVal = -maxVel;
      if (myLastTransVal != transVal || myLastTransType != myTransType ||
	  (myLastTransSent.mSecSince() >= myStateReflectionRefreshTime))
      {
	comInt(ArCommands::VEL, ArMath::roundShort(transVal));
	myLastTransSent.setToNow();
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "Non-action trans vel of %d",
		     ArMath::roundShort(transVal));
	//printf("Sent command vel!\n");
      }
      if (fabs((double)transVal) > (double).5)
	myTryingToMove = true;
    }
    else if (myTransType == TRANS_VEL2)
    {
      if (ArMath::roundShort(myTransVal/myParams->getVel2Divisor()) > 128)
	transVal = 128;
      else if (ArMath::roundShort(myTransVal/myParams->getVel2Divisor()) < -128)
	transVal = -128;
      else
	transVal = ArMath::roundShort(myTransVal/myParams->getVel2Divisor());
      if (ArMath::roundShort(myTransVal2/myParams->getVel2Divisor()) > 128)
	transVal2 = 128;
      else if (ArMath::roundShort(myTransVal2/myParams->getVel2Divisor()) < -128)
	transVal2 = -128;
      else
	transVal2 = ArMath::roundShort(myTransVal2/myParams->getVel2Divisor());
      if (myLastTransVal != transVal || myLastTransVal2 != transVal2 ||
	  myLastTransType != myTransType ||
	  (myLastTransSent.mSecSince() >= myStateReflectionRefreshTime))
      {
	com2Bytes(ArCommands::VEL2, transVal, transVal2);
	myLastTransSent.setToNow();
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "Non-action vel2 of %d %d",
		     transVal, transVal2);
	//printf("Sent command vel2!\n");
      }
      if (fabs((double)transVal) > (double).5 || fabs((double)transVal2) > (double).5)
	myTryingToMove = true;
    }
    else if (myTransType == TRANS_DIST_NEW || myTransType == TRANS_DIST)
    {
      // if the robot doesn't have its own distance command
      if (!myParams->hasMoveCommand())
      {
	double distGone;
	double distToGo;
	double vel;

	myTransType = TRANS_DIST;
	distGone = myTransDistStart.findDistanceTo(getPose());
	distToGo = fabs(fabs(myTransVal) - distGone);
	if (distGone > fabs(myTransVal) ||
	    (distToGo < 10 && fabs(getVel()) < 30))
	{
	  comInt(ArCommands::VEL, 0);
	  myTransType = TRANS_VEL;
	  myTransVal = 0;
	}
	else
	  myTryingToMove = true;
	vel = sqrt(distToGo * 200 * 2);
	if (vel > getTransVelMax())
	  vel = getTransVelMax();
	if (myTransVal < 0)
	  vel *= -1;
	comInt(ArCommands::VEL, ArMath::roundShort(vel));
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "Non-action move-helper of %d",
		     ArMath::roundShort(vel));
      }
      else if (myParams->hasMoveCommand() && myTransType == TRANS_DIST_NEW)
      {
	comInt(ArCommands::MOVE, transVal);
	myLastTransSent.setToNow();
	myTransType = TRANS_DIST;
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "Non-action move of %d",
		     transVal);
	myTryingToMove = true;
      }
      else if (myTransType == TRANS_DIST &&
	  (myLastTransSent.mSecSince() >= myStateReflectionRefreshTime))
      {
	com(0);
	myLastPulseSent.setToNow();
	myLastTransSent.setToNow();
	//printf("Sent pulse for dist!\n");
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "Non-action pulse for dist");
      }
      //printf("Sent command move!\n");
    }
    else if (myTransType == TRANS_IGNORE)
    {
      //printf("No trans command sent\n");
    }
    else
      ArLog::log(ArLog::Terse,
		 "ArRobot::stateReflector: Invalid translational type %d.",
		 myTransType);
    myLastTransVal = transVal;
    myLastTransVal2 = transVal2;
    myLastTransType = myTransType;
  }
  else // if actions can go
  {
    if (hasSettableVelMaxes() &&
	ArMath::fabs(myLastSentTransVelMax - myTransVelMax) >= 1)
    {
      comInt(ArCommands::SETV,
	     ArMath::roundShort(myTransVelMax));
      myLastSentTransVelMax = myTransVelMax;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "Action-but-robot trans max vel of %d",
		   ArMath::roundShort(myTransVelMax));
    }

    // first we'll handle all of the accel decel things
    if (myActionDesired.getTransAccelStrength() >=
	ArActionDesired::MIN_STRENGTH)
    {
      transAccel = ArMath::roundShort(myActionDesired.getTransAccel());
      if (hasSettableAccsDecs() && ArMath::fabs(transAccel) > 1 &&
	  ArMath::fabs(myLastSentTransAccel - transAccel) >= 1)
      {
	comInt(ArCommands::SETA,
	       ArMath::roundShort(transAccel));
	myLastSentTransAccel = transAccel;
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "Action trans accel of %d",
		     ArMath::roundShort(transAccel));
      }
    }
    else if (hasSettableAccsDecs() && ArMath::fabs(myTransAccel) > 1 &&
	ArMath::fabs(myLastSentTransAccel - myTransAccel) >= 1)
    {
      comInt(ArCommands::SETA,
	     ArMath::roundShort(myTransAccel));
      myLastSentTransAccel = myTransAccel;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "Action-but-robot trans accel of %d",
		   ArMath::roundShort(myTransAccel));
    }

    if (myActionDesired.getTransDecelStrength() >=
	ArActionDesired::MIN_STRENGTH)
    {
      transDecel = ArMath::roundShort(myActionDesired.getTransDecel());
      if (hasSettableAccsDecs() && ArMath::fabs(transDecel) > 1 &&
	  ArMath::fabs(myLastSentTransDecel - transDecel) >= 1)
      {
	comInt(ArCommands::SETA,
	       -ArMath::roundShort(transDecel));
	myLastSentTransDecel = transDecel;
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "Action trans decel of %d",
		     -ArMath::roundShort(transDecel));
      }
    }
    else if (hasSettableAccsDecs() && ArMath::fabs(myTransDecel) > 1 &&
	ArMath::fabs(myLastSentTransDecel - myTransDecel) >= 1)
    {
      comInt(ArCommands::SETA,
	     -ArMath::roundShort(myTransDecel));
      myLastSentTransDecel = myTransDecel;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "Action-but-robot trans decel of %d",
		   -ArMath::roundShort(myTransDecel));
    }

    if (myActionDesired.getMaxVelStrength() >= ArActionDesired::MIN_STRENGTH)
    {
      maxTransVel = myActionDesired.getMaxVel();
      if (maxTransVel > myTransVelMax)
	maxTransVel = myTransVelMax;
    }
    else
      maxTransVel = myTransVelMax;

    if (myActionDesired.getMaxNegVelStrength() >=
	ArActionDesired::MIN_STRENGTH)
    {
      maxNegTransVel = -ArMath::fabs(myActionDesired.getMaxNegVel());
      if (maxNegTransVel < -myTransVelMax)
	maxNegTransVel = -myTransVelMax;
    }
    else
      maxNegTransVel = -myTransVelMax;

    if (myActionDesired.getVelStrength() >= ArActionDesired::MIN_STRENGTH)
    {
      transVal = ArMath::roundShort(myActionDesired.getVel());
      myActionTransSet = true;
    }
    else
    {
      transVal = myLastActionTransVal;
    }

    if (fabs((double)transVal) > (double).5)
      myTryingToMove = true;

    maxVel = ArMath::roundShort(maxTransVel);
    maxNegVel = ArMath::roundShort(maxNegTransVel);
    if (transVal > maxVel)
      transVal = maxVel;
    if (transVal < maxNegVel)
      transVal = maxNegVel;

    if (myActionTransSet &&
	(myLastTransSent.mSecSince() >= myStateReflectionRefreshTime ||
	 transVal != myLastActionTransVal))
    {
      comInt(ArCommands::VEL, ArMath::roundShort(transVal));
      myLastTransSent.setToNow();
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "Action trans vel of %d",
		   ArMath::roundShort(transVal));
    }
    myLastActionTransVal = transVal;
  }

  // if this is true actions can't go
  if ((myRotType != ROT_NONE && myDirectPrecedenceTime == 0) ||
      (myRotType != ROT_NONE && myDirectPrecedenceTime != 0 &&
       myRotSetTime.mSecSince() < myDirectPrecedenceTime))
  {
    if (hasSettableVelMaxes() &&
	ArMath::fabs(myLastSentRotVelMax - myRotVelMax) >= 1)
    {
      comInt(ArCommands::SETRV,
	     ArMath::roundShort(myRotVelMax));
      myLastSentRotVelMax = myRotVelMax;
    }
    if (hasSettableAccsDecs() && ArMath::fabs(myRotAccel) > 1 &&
	ArMath::fabs(myLastSentRotAccel - myRotAccel) >= 1)
    {
      comInt(ArCommands::SETRA,
	     ArMath::roundShort(myRotAccel));
      myLastSentRotAccel = myRotAccel;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "%25sNon-action rot accel of %d", "",
		   ArMath::roundShort(myRotAccel));
    }
    if (hasSettableAccsDecs() && ArMath::fabs(myRotDecel) > 1 &&
	ArMath::fabs(myLastSentRotDecel - myRotDecel) >= 1)
    {
      comInt(ArCommands::SETRA,
	     -ArMath::roundShort(myRotDecel));
      myLastSentRotDecel = myRotDecel;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "%25sNon-action rot decel of %d", "",
		   -ArMath::roundShort(myRotDecel));
    }

    myActionRotSet = false;
    rotVal = ArMath::roundShort(myRotVal);
    if (myRotType == ROT_HEADING)
    {
      encTh = ArMath::subAngle(myRotVal, myEncoderTransform.getTh());
      rawTh = ArMath::addAngle(encTh,
			       ArMath::subAngle(myRawEncoderPose.getTh(),
						myEncoderPose.getTh()));
      rotVal = ArMath::roundShort(rawTh);

      // if we were using a different heading type, a different heading
      // our heading doesn't match what we want it to be, or its been a while
      // since we sent the heading, send it again
      if (myLastRotVal != rotVal || myLastRotType != myRotType ||
	  fabs(ArMath::subAngle(rotVal, getTh())) > 1 ||
	  (myLastRotSent.mSecSince() >= myStateReflectionRefreshTime))
      {
	comInt(ArCommands::HEAD, rotVal);
	myLastRotSent.setToNow();
	//printf("sent command, heading\n");
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal,
	     "%25sNon-action rot heading of %d (encoder %d, raw %d)",
		     "",
		     ArMath::roundShort(myRotVal),
		     ArMath::roundShort(encTh),
		     ArMath::roundShort(rotVal));
      }
      if (fabs(ArMath::subAngle(rotVal, getTh())) > 1)
	myTryingToMove = true;
    }
    else if (myRotType == ROT_VEL)
    {
      if (myLastRotVal != rotVal || myLastRotType != myRotType ||
	  (myLastRotSent.mSecSince() >= myStateReflectionRefreshTime))
      {
	comInt(ArCommands::RVEL, rotVal);
	myLastRotSent.setToNow();
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "%25sNon-action rot vel of %d", "",
		     rotVal);
	//printf("sent command, rot vel\n");
	if (fabs((double)rotVal) > (double).5)
	  myTryingToMove = true;
      }
    }
    else if (myRotType == ROT_IGNORE)
    {
      //printf("Not sending any command, rot is set to ignore");
    }
    else
      ArLog::log(ArLog::Terse,
		 "ArRobot::stateReflector: Invalid rotation type %d.",
		 myRotType);
    myLastRotVal = rotVal;
    myLastRotType = myRotType;
  }
  else // if the action can fire
  {
    // first we'll handle all of the accel decel things
    if (myActionDesired.getMaxRotVelStrength() >=
	ArActionDesired::MIN_STRENGTH)
    {
      maxRotVel = myActionDesired.getMaxRotVel();
      if (maxRotVel > myAbsoluteMaxRotVel)
	maxRotVel = myAbsoluteMaxRotVel;
      maxRotVel = ArMath::roundShort(maxRotVel);
      if (ArMath::fabs(myLastSentRotVelMax - maxRotVel) >= 1)
      {
	myLastSentRotVelMax = maxRotVel;
	comInt(ArCommands::SETRV,
	       ArMath::roundShort(maxRotVel));
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "%25sAction rot vel max of %d", "",
		     ArMath::roundShort(maxRotVel));
      }
    }
    else if (hasSettableVelMaxes() &&
	     ArMath::fabs(myLastSentRotVelMax - myRotVelMax) >= 1)
    {
      comInt(ArCommands::SETRV,
	     ArMath::roundShort(myRotVelMax));
      myLastSentRotVelMax = myRotVelMax;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal,
		   "%25sAction-but-robot rot vel max of %d",
		   "",  ArMath::roundShort(myRotVelMax));
    }

    if (myActionDesired.getRotAccelStrength() >= ArActionDesired::MIN_STRENGTH)
    {
      rotAccel = ArMath::roundShort(myActionDesired.getRotAccel());
      if (ArMath::fabs(myLastSentRotAccel - rotAccel) >= 1)
      {
	comInt(ArCommands::SETRA,
	       ArMath::roundShort(rotAccel));
	myLastSentRotAccel = rotAccel;
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "%25sAction rot accel of %d", "",
		     ArMath::roundShort(rotAccel));
      }
    }
    else if (hasSettableAccsDecs() && ArMath::fabs(myRotAccel) > 1 &&
	ArMath::fabs(myLastSentRotAccel - myRotAccel) >= 1)
    {
      comInt(ArCommands::SETRA,
	     ArMath::roundShort(myRotAccel));
      myLastSentRotAccel = myRotAccel;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "%25sAction-but-robot rot accel of %d",
		   "", ArMath::roundShort(myRotAccel));
    }

    if (myActionDesired.getRotDecelStrength() >= ArActionDesired::MIN_STRENGTH)
    {
      rotDecel = ArMath::roundShort(myActionDesired.getRotDecel());
      if (ArMath::fabs(myLastSentRotDecel - rotDecel) >= 1)
      {
	comInt(ArCommands::SETRA,
	       -ArMath::roundShort(rotDecel));
	myLastSentRotDecel = rotDecel;
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "%25sAction rot decel of %d", "",
		     -ArMath::roundShort(rotDecel));
      }
    }
    else if (hasSettableAccsDecs() && ArMath::fabs(myRotDecel) > 1 &&
	ArMath::fabs(myLastSentRotDecel - myRotDecel) >= 1)
    {
      comInt(ArCommands::SETRA,
	     -ArMath::roundShort(myRotDecel));
      myLastSentRotDecel = myRotDecel;
      if (myLogMovementSent)
	ArLog::log(ArLog::Normal, "%25sAction-but-robot rot decel of %d",
		   "", -ArMath::roundShort(myRotDecel));
    }



    if (myActionDesired.getDeltaHeadingStrength() >=
	                                     ArActionDesired::MIN_STRENGTH)
    {
      if (ArMath::roundShort(myActionDesired.getDeltaHeading()) == 0)
      {
	rotStopped = true;
	rotVal = 0;
	rotHeading = false;
      }
      else
      {
	//printf("delta %.0f\n", myActionDesired.getDeltaHeading());
	//encTh = ArMath::subAngle(myRotVal, myEncoderTransform.getTh());
	encTh = ArMath::subAngle(
		ArMath::addAngle(myActionDesired.getDeltaHeading(),
				 getTh()),
		myEncoderTransform.getTh());
	//printf("final th %.0f\n", th);
	rawTh = ArMath::addAngle(encTh,
				 ArMath::subAngle(myRawEncoderPose.getTh(),
						  myEncoderPose.getTh()));
	rotVal = ArMath::roundShort(rawTh);
	rotStopped = false;
	rotHeading = true;
	myTryingToMove = true;
      }
      myActionRotSet = true;
    }
    else if (myActionDesired.getRotVelStrength() >=
	                                     ArActionDesired::MIN_STRENGTH)
    {
      if (ArMath::roundShort(myActionDesired.getRotVel()) == 0)
      {
	rotStopped = true;
	rotVal = 0;
	rotHeading = false;
      }
      else
      {
	rotStopped = false;
	rotVal = ArMath::roundShort(myActionDesired.getRotVel());
	rotHeading = false;
	myTryingToMove = true;
      }
      myActionRotSet = true;
    }
    else
    {
      rotStopped = myLastActionRotStopped;
      rotVal = myLastActionRotVal;
      rotHeading = myLastActionRotHeading;
    }

    if (myActionRotSet &&
	(myLastRotSent.mSecSince() > myStateReflectionRefreshTime ||
	 rotStopped != myLastActionRotStopped ||
	 rotVal != myLastActionRotVal ||
	 rotHeading != myLastActionRotHeading))
    {
      if (rotStopped)
      {
	comInt(ArCommands::RVEL, 0);
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal,
		     "%25sAction rot vel of 0 (rotStopped)",
		     "");
      }
      else if (rotHeading)
      {
	comInt(ArCommands::HEAD, rotVal);
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal,
		     "%25sAction rot heading of %d (encoder %d, raw %d)",
		     "",
		     ArMath::roundShort(ArMath::addAngle(
			     myActionDesired.getDeltaHeading(),
			     getTh())),
		     ArMath::roundShort(encTh),
		     ArMath::roundShort(rotVal));
      }
      else
      {
	comInt(ArCommands::RVEL, rotVal);
	if (myLogMovementSent)
	  ArLog::log(ArLog::Normal, "%25sAction rot vel of %d", "",
		     rotVal);
      }
      myLastRotSent.setToNow();
    }

    myLastActionRotVal = rotVal;
    myLastActionRotStopped = rotStopped;
    myLastActionRotHeading = rotHeading;
  }

  if (myLastRotSent.mSecSince() > myStateReflectionRefreshTime &&
      myLastTransSent.mSecSince() > myStateReflectionRefreshTime &&
      myLastPulseSent.mSecSince() > myStateReflectionRefreshTime)
  {
    com(ArCommands::PULSE);
    myLastPulseSent.setToNow();
    if (myLogMovementSent)
      ArLog::log(ArLog::Normal, "Pulse");

  }
}

AREXPORT bool ArRobot::handlePacket(ArRobotPacket *packet)
{
  std::list<ArRetFunctor1<bool, ArRobotPacket *> *>::iterator it;
  bool handled;

  lock();
  //printf("ms since last packet %ld this type 0x%x\n", myLastPacketReceivedTime.mSecSince(packet->getTimeReceived()), packet->getID());
  myLastPacketReceivedTime = packet->getTimeReceived();
  if (packet->getID() == 0xff)
  {
    ArLog::log(ArLog::Terse, "Losing connection because robot was reset.");
    dropConnection();
    unlock();
    return false;
  }

  for (handled = false, it = myPacketHandlerList.begin();
       it != myPacketHandlerList.end() && handled == false;
       it++)
  {
    if ((*it) != NULL && (*it)->invokeR(packet))
      handled = true;
    else
      packet->resetRead();
  }
  if (!handled)
    ArLog::log(ArLog::Normal,
	       "No packet handler wanted packet with ID: 0x%x",
	       packet->getID());
  unlock();
  return handled;
}


/** @note You must first start the encoder packet stream by calling
 * requestEncoderPackets() before this function will return encoder values.
 */
AREXPORT long int ArRobot::getLeftEncoder()
{
  return myLeftEncoder;
}

/** @note You must first start the encoder packet stream by calling
 * requestEncoderPackets() before this function will return encoder values.
 */
AREXPORT long int ArRobot::getRightEncoder()
{
  return myRightEncoder;
}


/**
 * @internal
   This just locks the robot, so that its locked for all the user tasks
**/
AREXPORT void ArRobot::robotLocker(void)
{
  lock();
}

/**
 * @internal
   This just unlocks the robot
**/
AREXPORT void ArRobot::robotUnlocker(void)
{
  unlock();
}

/**
   Reads in all of the packets that are available to read in, then runs through
   the list of packet handlers and tries to get each packet handled.
   @see addPacketHandler
   @see remPacketHandler
**/
AREXPORT void ArRobot::packetHandler(void)
{
  ArRobotPacket *packet;
  int timeToWait;
  ArTime start;
  bool sipHandled = false;

  if (myAsyncConnectFlag)
  {
    lock();
    asyncConnectHandler(false);
    unlock();
    return;
  }

  if (!isConnected())
    return;

  start.setToNow();
  /*
    The basic idea is that if we're chained to the sip we run through
    and see if we have any packets available now (like if we got
    backed up), we only check this for half the cycle time
    though... if we know the cycle time of the robot (from config)
    then we go for half that, if we don't know the cycle time of the
    robot (from config) then we go for half of whatever our cycle time
    is set to

    if we don't have any packets waiting then we chill and wait for
    it, if we got one, just get on with it
  **/
  packet = NULL;
  // read all the packets that are available
  while ((packet = myReceiver.receivePacket(0)) != NULL)
  {
    if (myPacketsReceivedTracking)
    {
      ArLog::log(ArLog::Normal,
		 "Rcvd: prePacket (%ld) 0x%x at %ld (%ld)",
		 myPacketsReceivedTrackingCount,
		 packet->getID(), start.mSecSince(),
		 myPacketsReceivedTrackingStarted.mSecSince());
      myPacketsReceivedTrackingCount++;
    }

    handlePacket(packet);
    if ((packet->getID() & 0xf0) == 0x30)
      sipHandled = true;
    packet = NULL;

    // if we've taken too long here then break
    if ((getOrigRobotConfig()->hasPacketArrived() &&
	 start.mSecSince() > getOrigRobotConfig()->getSipCycleTime() / 2) ||
	(!getOrigRobotConfig()->hasPacketArrived() &&
	 (unsigned int) start.mSecSince() > myCycleTime / 2))
    {
      break;
    }
  }

  if (isCycleChained())
    timeToWait = getCycleTime() * 2 - start.mSecSince();

  // if we didn't get a sip and we're chained to the sip, wait for the sip
  while (isCycleChained() && !sipHandled && isRunning() &&
	 (packet = myReceiver.receivePacket(timeToWait)) != NULL)
  {
    if (myPacketsReceivedTracking)
    {
      ArLog::log(ArLog::Normal, "Rcvd: Packet (%ld) 0x%x at %ld (%ld)",
		 myPacketsReceivedTrackingCount,
		 packet->getID(), start.mSecSince(),
		 myPacketsReceivedTrackingStarted.mSecSince());
      myPacketsReceivedTrackingCount++;
    }

    handlePacket(packet);
    if ((packet->getID() & 0xf0) == 0x30)
      break;
    timeToWait = getCycleTime() * 2 - start.mSecSince();
    if (timeToWait < 0)
      timeToWait = 0;
  }

  if (myTimeoutTime > 0 &&
      ((-myLastPacketReceivedTime.mSecTo()) > myTimeoutTime))
  {
    ArLog::log(ArLog::Terse,
	       "Losing connection because nothing received from robot in %d milliseconds.",
	       myTimeoutTime);
    dropConnection();
  }

  if (myPacketsReceivedTracking)
    ArLog::log(ArLog::Normal, "Rcvd: time taken %ld", start.mSecSince());

}

/**
   Runs the resolver on the actions, it just saves these values for
   use by the stateReflector, otherwise it sends these values straight
   down to the robot.
   @see addAction
   @see remAction
**/
AREXPORT void ArRobot::actionHandler(void)
{
  ArActionDesired *actDesired;

  if (myResolver == NULL || myActions.size() == 0 || !isConnected())
    return;

  actDesired = myResolver->resolve(&myActions, this, myLogActions);

  myActionDesired.reset();

  if (actDesired == NULL)
    return;

  myActionDesired.merge(actDesired);

  if (myLogActions)
  {
    ArLog::log(ArLog::Normal, "Final resolved desired:");
    myActionDesired.log();
  }
}

/**
   Sets a time such that if the number of milliseconds between cycles
   goes over this then there will be an ArLog::log(ArLog::Normal)
   warning.

   @param ms the number of milliseconds between cycles to warn over, 0
   turns warning off
 **/
AREXPORT void ArRobot::setCycleWarningTime(unsigned int ms)
{
  myCycleWarningTime = ms;
  // we don't have to send it down because the functor gets it each cycle
}

/**
   Sets a time such that if the number of milliseconds between cycles
   goes over this then there will be an ArLog::log(ArLog::Normal)
   warning.

   @return the number of milliseconds between cycles to warn over, 0
   means warning is off
**/
AREXPORT unsigned int ArRobot::getCycleWarningTime(void) const
{
  return myCycleWarningTime;
}

/**
   Sets a time such that if the number of milliseconds between cycles
   goes over this then there will be an ArLog::log(ArLog::Normal)
   warning.

   @return the number of milliseconds between cycles to warn over, 0
   means warning is off
**/
AREXPORT unsigned int ArRobot::getCycleWarningTime(void)
{
  return myCycleWarningTime;
}

/**
   Sets the number of milliseconds between cycles, at each cycle is
   when all packets are processed, all sensors are interpretted, all
   actions are called, and all user tasks are serviced.  Be warned,
   if you set this too small you could overflow your serial connection.
   @param ms the number of milliseconds between cycles
 **/
AREXPORT void ArRobot::setCycleTime(unsigned int ms)
{
  myCycleTime = ms;
}

/**
   This is the amount of time the robot will stabilize for after it
   has connected to the robot (it won't report it is connected until
   after this time is over).  By convention you should never set this
   lower than what you find the value at (though it will let you) this
   is so that everything can get itself stabilized before we let
   things drive.

   @param mSecs the amount of time to stabilize for (0 disables)

   @see addStabilizingCB
**/
AREXPORT void ArRobot::setStabilizingTime(int mSecs)
{
  if (mSecs > 0)
    myStabilizingTime = mSecs;
  else
    myStabilizingTime = 0;
}

/**
   This is the amount of time the robot will stabilize for after it
   has connected to the robot (it won't report it is connected until
   after this time is over).
**/
AREXPORT int ArRobot::getStabilizingTime(void) const
{
  return myStabilizingTime;
}

/**
   Finds the number of milliseconds between cycles, at each cycle is
   when all packets are processed, all sensors are interpretted, all
   actions are called, and all user tasks are serviced.  Be warned,
   if you set this too small you could overflow your serial connection.
   @return the number of milliseconds between cycles
**/
AREXPORT unsigned int ArRobot::getCycleTime(void) const
{
  return myCycleTime;
}



/**
   @param multiplier when the ArRobot is waiting for a connection
   packet back from a robot, it waits for this multiplier times the
   cycle time for the packet to come back before it gives up on it...
   This should be small for normal connections but if doing something
   over a slow network then you may want to make it larger

 **/
AREXPORT void ArRobot::setConnectionCycleMultiplier(unsigned int multiplier)
{
  myConnectionCycleMultiplier = multiplier;
}

/**
   @return when the ArRobot is waiting for a connection packet back
   from a robot, it waits for this multiplier times the cycle time for
   the packet to come back before it gives up on it...  This should be
   small for normal connections but if doing something over a slow
   network then you may want to make it larger
**/
AREXPORT unsigned int ArRobot::getConnectionCycleMultiplier(void) const
{
  return myConnectionCycleMultiplier;
}


/**
    This function is only for serious developers, it basically runs the
    loop once.  You would use this function if you were wanting to use robot
    control in some other monolithic program, so you could work within its
    framework, rather than trying to get it to work in ARIA.
**/
AREXPORT void ArRobot::loopOnce(void)
{
  if (mySyncTaskRoot != NULL)
    mySyncTaskRoot->run();

  incCounter();
}


// DigIn IR logic is reverse.  0 means broken, 1 means not broken

AREXPORT bool ArRobot::isLeftTableSensingIRTriggered(void) const
{
  if (myParams->haveTableSensingIR())
  {
    if (myParams->haveNewTableSensingIR() && myIODigInSize > 3)
      return !(getIODigIn(3) & ArUtil::BIT1);
    else
      return !(getDigIn() & ArUtil::BIT0);
  }
  return 0;
}

AREXPORT bool ArRobot::isRightTableSensingIRTriggered(void) const
{
  if (myParams->haveTableSensingIR())
  {
    if (myParams->haveNewTableSensingIR() && myIODigInSize > 3)
      return !(getIODigIn(3) & ArUtil::BIT0);
    else
      return !(getDigIn() & ArUtil::BIT1);
  }
  return 0;
}

AREXPORT bool ArRobot::isLeftBreakBeamTriggered(void) const
{
  if (myParams->haveTableSensingIR())
  {
    if (myParams->haveNewTableSensingIR() && myIODigInSize > 3)
      return !(getIODigIn(3) & ArUtil::BIT2);
    else
      return !(getDigIn() & ArUtil::BIT3);
  }
  return 0;
}

AREXPORT bool ArRobot::isRightBreakBeamTriggered(void) const
{
  if (myParams->haveTableSensingIR())
  {
    if (myParams->haveNewTableSensingIR() && myIODigInSize > 3)
      return !(getIODigIn(3) & ArUtil::BIT3);
    else
      return !(getDigIn() & ArUtil::BIT2);
  }
  return 0;
}

AREXPORT int ArRobot::getMotorPacCount(void) const
{
  if (myTimeLastMotorPacket == time(NULL))
    return myMotorPacCount;
  if (myTimeLastMotorPacket == time(NULL) - 1)
    return myMotorPacCurrentCount;
  return 0;
}

AREXPORT int ArRobot::getSonarPacCount(void) const
{
  if (myTimeLastSonarPacket == time(NULL))
    return mySonarPacCount;
  if (myTimeLastSonarPacket == time(NULL) - 1)
    return mySonarPacCurrentCount;
  return 0;
}


AREXPORT bool ArRobot::processMotorPacket(ArRobotPacket *packet)
{
  int x, y, th, qx, qy, qth;
  double deltaX, deltaY, deltaTh;

  int numReadings;
  int sonarNum;
  int sonarRange;

  if (packet->getID() != 0x32 && packet->getID() != 0x33)
    return false;

  // upkeep the counting variable
  if (myTimeLastMotorPacket != time(NULL))
  {
    myTimeLastMotorPacket = time(NULL);
    myMotorPacCount = myMotorPacCurrentCount;
    myMotorPacCurrentCount = 0;
  }
  myMotorPacCurrentCount++;

  x = (packet->bufToUByte2() & 0x7fff);
  y = (packet->bufToUByte2() & 0x7fff);
  th = packet->bufToByte2();

  if (myFirstEncoderPose)
  {
    qx = 0;
    qy = 0;
    qth = 0;
    myFirstEncoderPose = false;
    myRawEncoderPose.setPose(
	    myParams->getDistConvFactor() * x,
	    myParams->getDistConvFactor() * y,
	    ArMath::radToDeg(myParams->getAngleConvFactor() * (double)th));
    myEncoderPose = myRawEncoderPose;
    myEncoderTransform.setTransform(myEncoderPose, myGlobalPose);
  }
  else
  {
    qx = x - myLastX;
    qy = y - myLastY;
    qth = th - myLastTh;
  }


  //ArLog::log(ArLog::Terse, "Damnit qx %d qy %d,  x %d y %d,  lastx %d lasty %d", qx, qy, x, y, myLastX, myLastY);
  myLastX = x;
  myLastY = y;
  myLastTh = th;

  if (qx > 0x1000)
    qx -= 0x8000;
  if (qx < -0x1000)
    qx += 0x8000;

  if (qy > 0x1000)
    qy -= 0x8000;
  if (qy < -0x1000)
    qy += 0x8000;

  deltaX = myParams->getDistConvFactor() * (double)qx;
  deltaY = myParams->getDistConvFactor() * (double)qy;
  deltaTh = ArMath::radToDeg(myParams->getAngleConvFactor() * (double)qth);
  //encoderTh = ArMath::radToDeg(myParams->getAngleConvFactor() * (double)(th));


  // encoder stuff was here



  myLeftVel = myParams->getVelConvFactor() * packet->bufToByte2();
  myRightVel = myParams->getVelConvFactor() * packet->bufToByte2();
  myVel = (myLeftVel + myRightVel)/2.0;

  myBatteryVoltage = packet->bufToUByte() * .1;
  myBatteryAverager.add(myBatteryVoltage);
  myStallValue = packet->bufToByte2();

  //ArLog::log("x %.1f y %.1f th %.1f vel %.1f voltage %.1f", myX, myY, myTh,
  //myVel, myBatteryVoltage);

  myControl = ArMath::fixAngle(ArMath::radToDeg(myParams->getAngleConvFactor() *
				  (packet->bufToByte2() - th)));
  myFlags = packet->bufToUByte2();
  myCompass = 2*packet->bufToUByte();

  for (numReadings = packet->bufToByte(); numReadings > 0; numReadings--)
  {
    sonarNum = packet->bufToByte();
    sonarRange = ArMath::roundInt(
	    (double)packet->bufToUByte2() * myParams->getRangeConvFactor());
    processNewSonar(sonarNum, sonarRange, packet->getTimeReceived());
  }

  if (packet->getDataLength() - packet->getDataReadLength() > 0)
  {
    myAnalogPortSelected = packet->bufToUByte2();
    myAnalog = packet->bufToByte();
    myDigIn = packet->bufToByte();
    myDigOut = packet->bufToByte();
  }

  if (packet->getDataLength() - packet->getDataReadLength() > 0)
    myRealBatteryVoltage = packet->bufToUByte2() * .1;
  else
    myRealBatteryVoltage = myBatteryVoltage;

  myRealBatteryAverager.add(myRealBatteryVoltage);

  if (packet->getDataLength() - packet->getDataReadLength() > 0)
    myChargeState = (ChargeState) packet->bufToUByte();
  else
    myChargeState = CHARGING_UNKNOWN;

  if (packet->getDataLength() - packet->getDataReadLength() > 0)
    myRotVel = (double)packet->bufToByte2() / 10.0;
  else
    myRotVel = ArMath::radToDeg((myRightVel - myLeftVel) / 2.0 *
				myParams->getDiffConvFactor());

  if (packet->getDataLength() - packet->getDataReadLength() > 0)
  {
    myHasFaultFlags = true;
    myFaultFlags = packet->bufToUByte2();
  }
  else
  {
    myHasFaultFlags = false;
    myFaultFlags = packet->bufToUByte2();
  }
  /*
    Okay how this works is like so.

    We keep around the raw encoder position, because we must use this
    to find differences between last position and this position.

    We find the difference in x and y positions (deltaX and deltaY)
    and keep these around for later use, but we also add these to our
    raw encoder readings for X and Y.  We also find the change in
    angle (deltaTh), which is used for inertial corrections, and added
    to the raw encoder heading to find which the current raw encoder
    heading.


    From here there are two paths:

    Path 1) Have a callback.  If we have a callback it means that we
    have an inertial nav device of some kind.  If this is the case,
    then we pass the callback the delta between last position and
    current position, along with the time of the current position,
    then the callback gives us back a new delta theta (deltaTh).  We
    then need to rotate the deltaX and deltaY into our corrected
    encoder space.  We do this by making a transform that takes the
    raw encoder heading and transforms it to what our new heading is
    (adding deltaTh to our current encoder th), and then applying that
    transform, taking the results as our new deltaX and deltaY.

    Path 2) We have no callback, we just use the heading that came
    back from the robot as our delta theta (deltaTh);

    From here the two paths unify again.  deltaX and deltaY are added
    to the encoder pose (this is the corrected encoder pose), and the
    encoder heading is set to the newTh.

    Note that this leaves a difference between rawEncoder heading and
    our heading, which is fine, BUT if you are sending heading
    commands to the robot you need to compenstate for the difference
    between these.

    Note above that we return deltaTh instead of just heading so that
    we can turn inertial on and off without losing track of where
    we're at... since we're just adding in deltas from the heading it
    doesn't matter how we switch around the callback.

  **/

  myRawEncoderPose.setX(myRawEncoderPose.getX() + deltaX);
  myRawEncoderPose.setY(myRawEncoderPose.getY() + deltaY);
  myRawEncoderPose.setTh(myRawEncoderPose.getTh() + deltaTh);

  // check if there is a correction callback, if there is get the new
  // heading out of it instead of using the raw encoder heading
  if (myEncoderCorrectionCB != NULL)
  {
    ArPoseWithTime deltaPose(deltaX, deltaY, deltaTh,
			     packet->getTimeReceived());
    deltaTh = myEncoderCorrectionCB->invokeR(deltaPose);
    ArTransform trans(ArPose(0, 0, myRawEncoderPose.getTh()),
		      ArPose(0, 0,
			     ArMath::addAngle(myEncoderPose.getTh(),
					      deltaTh)));

    ArPose rotatedDelta = trans.doTransform(ArPose(deltaX, deltaY, 0));

    deltaX = rotatedDelta.getX();
    deltaY = rotatedDelta.getY();
  }

  myEncoderPose.setTime(packet->getTimeReceived());
  myEncoderPose.setX(myEncoderPose.getX() + deltaX);
  myEncoderPose.setY(myEncoderPose.getY() + deltaY);
  myEncoderPose.setTh(ArMath::addAngle(myEncoderPose.getTh(), deltaTh));

  myGlobalPose = myEncoderTransform.doTransform(myEncoderPose);

  myOdometerDegrees += fabs(deltaTh);
  myOdometerDistance += sqrt(fabs(deltaX * deltaX + deltaY * deltaY));

  if (myLogMovementReceived)
    ArLog::log(ArLog::Normal,
	       "Global (%5.0f %5.0f %5.0f) Encoder (%5.0f %5.0f %5.0f) Raw (%5.0f %5.0f %5.0f) Rawest (%5d %5d %5d) Delta (%5.0f %5.0f %5.0f) Conv %5.2f",
	       myGlobalPose.getX(), myGlobalPose.getY(),
	       myGlobalPose.getTh(),
	       myEncoderPose.getX(), myEncoderPose.getY(),
	       myEncoderPose.getTh(),
	       myRawEncoderPose.getX(), myRawEncoderPose.getY(),
	       myRawEncoderPose.getTh(), x, y, th, deltaX, deltaY, deltaTh,
	       myParams->getDistConvFactor());

  if (myLogVelocitiesReceived)
    ArLog::log(ArLog::Normal,
	       "     TransVel: %4.0f RotVel: %4.0f(%4.0f) Heading %4.0f TransAcc %4.0f RotAcc %4.0f(%4.0f)",
	       myVel, myRotVel, myLastHeading - getTh(), getTh(),
	       myVel - myLastVel, myRotVel - myLastRotVel,
	       myLastCalculatedRotVel - (myLastHeading - getTh()));
  myLastVel = myVel;
  myLastRotVel = myRotVel;
  myLastHeading = getTh();
  myLastCalculatedRotVel = myLastHeading - getTh();

  //ArLog::log(ArLog::Terse, "(%.0f %.0f) (%.0f %.0f)", deltaX, deltaY, myGlobalPose.getX(),	     myGlobalPose.getY());

  myInterpolation.addReading(packet->getTimeReceived(), myGlobalPose);
  myEncoderInterpolation.addReading(packet->getTimeReceived(), myEncoderPose);

  return true;
}

AREXPORT void ArRobot::processNewSonar(char number, int range,
				       ArTime timeReceived)
{
  /**
     This function used to just create more sonar readings if it
     didn't have the sonar number that was given (only the case if
     that sonar didn't have an entry in the param file), this caused
     some silent bugs in other peoples code and so was
     removed... especially since we don't know where the sonar are at
     if they weren't in the parameter file anyways.
   **/
  std::map<int, ArSensorReading *>::iterator it;
  ArSensorReading *sonar;
  ArTransform encoderTrans;
  ArPose encoderPose;

  if ((it = mySonars.find(number)) != mySonars.end())
  {
    sonar = (*it).second;
    sonar->newData(range, getPose(), getEncoderPose(), getToGlobalTransform(),
		   getCounter(), timeReceived);
    if (myTimeLastSonarPacket != time(NULL))
    {
      myTimeLastSonarPacket = time(NULL);
      mySonarPacCount = mySonarPacCurrentCount;
      mySonarPacCurrentCount = 0;
    }
    mySonarPacCurrentCount++;
  }
  else if (!myWarnedAboutExtraSonar)
  {
    ArLog::log(ArLog::Normal, "Robot gave back extra sonar reading!  Either the parameter file for the robot or the firmware needs updating.");
    myWarnedAboutExtraSonar = true;
  }
}



AREXPORT bool ArRobot::processEncoderPacket(ArRobotPacket *packet)
{
  if (packet->getID() != 0x90)
    return false;
  myLeftEncoder = packet->bufToByte4();
  myRightEncoder = packet->bufToByte4();
  return true;
}

AREXPORT bool ArRobot::processIOPacket(ArRobotPacket *packet)
{
  int i, num;

  if (packet->getID() != 0xf0)
    return false;

  myLastIOPacketReceivedTime = packet->getTimeReceived();

  // number of DigIn bytes
  num = packet->bufToUByte();
  for (i = 0; i < num; ++i)
    myIODigIn[i] = packet->bufToUByte();
  myIODigInSize = num;

  // number of DigOut bytes
  num = packet->bufToUByte();
  for (i = 0; i < num; ++i)
    myIODigOut[i] = packet->bufToUByte();
  myIODigOutSize = num;

  // number of A/D bytes
  num = packet->bufToUByte();
  for (i = 0; i < num; ++i)
    myIOAnalog[i] = packet->bufToUByte2();
  myIOAnalogSize = num;

  return true;
}

/**
   @param num the sonar number to check, should be between 0 and the number of
   sonar, the function won't fail if a bad number is given, will just return
   -1
   @return -1 if the sonar has never returned a reading, otherwise the sonar
   range, which is the distance from the physical sonar disc to where the sonar
   bounced back
   @see getNumSonar
**/
AREXPORT int ArRobot::getSonarRange(int num) const
{
  std::map<int, ArSensorReading *>::const_iterator it;

  if ((it = mySonars.find(num)) != mySonars.end())
    return (*it).second->getRange();
  else
    return -1;
}

/**
   @param num the sonar number to check, should be between 0 and the number of
   sonar, the function won't fail if a bad number is given, will just return
   false
   @return false if the sonar reading is old, or if there was no reading from
   that sonar, in the current SIP cycle.
   For best results, use this function in sync with the SIP cycle, for example,
   from a Sensor Interpretation Task Callback (see addSensorInterpTask).
**/

AREXPORT bool ArRobot::isSonarNew(int num) const
{
  std::map<int, ArSensorReading *>::const_iterator it;

  if ((it = mySonars.find(num)) != mySonars.end())
    return (*it).second->isNew(getCounter());
  else
    return false;
}

/**
   @param num the sonar number to check, should be between 0 and the number of
   sonar, the function won't fail if a bad number is given, will just return
   false
   @return NULL if there is no sonar defined for the given number, otherwise
   it returns a pointer to an instance of the ArSensorReading, note that this
   class retains ownership, so the instance pointed to should not be deleted
   and no pointers to it should be stored.  Note that often there will be sonar
   defined but no readings for it (since the readings may be created by the
   parameter reader), if there has never been a reading from the sonar then
   the range of that sonar will be -1 and its counterTaken value will be 0
**/
AREXPORT ArSensorReading *ArRobot::getSonarReading(int num) const
{
  std::map<int, ArSensorReading *>::const_iterator it;

  if ((it = mySonars.find(num)) != mySonars.end())
    return (*it).second;
  else
    return NULL;
}


/**
   @param command the command number to send
   @return whether the command could be sent or not
   @sa ArCommands
**/
AREXPORT bool ArRobot::com(unsigned char command)
{
  if (myPacketsSentTracking)
    ArLog::log(ArLog::Normal, "Sent: com(%d)", command);
  return mySender.com(command);
}

/**
   @param command the command number to send
   @param argument the integer argument to send with the command
   @return whether the command could be sent or not
   @sa ArCommands
**/
AREXPORT bool ArRobot::comInt(unsigned char command, short int argument)
{
  if (myPacketsSentTracking)
    ArLog::log(ArLog::Normal, "Sent: comInt(%d, %d)", command, argument);
  return mySender.comInt(command, argument);
}

/**
   @param command the command number to send
   @param high the high byte to send with the command
   @param low the low byte to send with the command
   @return whether the command could be sent or not
   @sa ArCommands
**/
AREXPORT bool ArRobot::com2Bytes(unsigned char command, char high, char low)
{
  if (myPacketsSentTracking)
    ArLog::log(ArLog::Normal, "Sent: com2Bytes(%d, %d, %d)", command,
	       high, low);
  return mySender.com2Bytes(command, high, low);
}

/**
   @param command the command number to send
   @param argument NULL-terminated string to get data from to send with the command; length to send with packet is determined by strlen
   @return whether the command could be sent or not
   @sa ArCommands
**/
AREXPORT bool ArRobot::comStr(unsigned char command, const char *argument)
{
  if (myPacketsSentTracking)
    ArLog::log(ArLog::Normal, "Sent: comStr(%d, '%s')", command,
	       argument);
  return mySender.comStr(command, argument);
}

/**
 * Sends a length-prefixed string command to the robot, copying 'size'
 * bytes of data from 'str' into the packet.
   @param command the command number to send
   @param str copy data to send from this character array
   @param size length of the string to send; copy this many bytes from 'str'; use this value as the length prefix byte before the sent string. This length must be less than the maximum packet size of 200.
   @return whether the command could be sent or not
   @sa ArCommands
**/
AREXPORT bool ArRobot::comStrN(unsigned char command, const char *str,
			       int size)
{
  if (myPacketsSentTracking)
  {
    char strBuf[512];
    strncpy(strBuf, str, size);
    strBuf[size] = '\0';
    ArLog::log(ArLog::Normal, "Sent: comStrN(%d, '%s') (size %d)",
	       command, strBuf, size);
  }
  return mySender.comStrN(command, str, size);
}

AREXPORT bool ArRobot::comDataN(unsigned char command, const char* data, int size)
{
  if(myPacketsSentTracking)
  {
    ArLog::log(ArLog::Normal, "Sent: comDataN(%d, <data...>) (size %d)", command, size);
  }
  return mySender.comDataN(command, data, size);
}


AREXPORT int ArRobot::getClosestSonarRange(double startAngle, double endAngle) const
{
  int num;
  num = getClosestSonarNumber(startAngle, endAngle);
  if (num == -1)
    return -1;
  else
    return getSonarRange(num);
}

AREXPORT int ArRobot::getClosestSonarNumber(double startAngle, double endAngle) const
{
  int i;
  ArSensorReading *sonar;
  int closestReading=0;
  int closestSonar=0;
  bool noReadings = true;

  for (i = 0; i < getNumSonar(); i++)
  {
    sonar = getSonarReading(i);
    if (sonar == NULL)
    {
      ArLog::log(ArLog::Terse, "Have an empty sonar at number %d, there should be %d sonar.", i, getNumSonar());
      continue;
    }
    if (ArMath::angleBetween(sonar->getSensorTh(), startAngle, endAngle))
    {
      if (noReadings)
      {
	closestReading = sonar->getRange();
	closestSonar = i;
	noReadings = false;
      }
      else if (sonar->getRange() < closestReading)
      {
	closestReading = sonar->getRange();
	closestSonar = i;
      }
    }
  }

  if (noReadings)
    return -1;
  else
    return closestSonar;
}

AREXPORT void ArRobot::addRangeDevice(ArRangeDevice *device)
{
  device->setRobot(this);
  myRangeDeviceList.push_front(device);
}

/**
   @param name remove the first device with this name
**/
AREXPORT void ArRobot::remRangeDevice(const char *name)
{
  std::list<ArRangeDevice *>::iterator it;
  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    if (strcmp(name, (*it)->getName()) == 0)
    {
      myRangeDeviceList.erase(it);
      return;
    }
  }
}

/**
   @param device remove the first device with this pointer value
**/
AREXPORT void ArRobot::remRangeDevice(ArRangeDevice *device)
{
  std::list<ArRangeDevice *>::iterator it;
  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    if ((*it) == device)
    {
      myRangeDeviceList.erase(it);
      return;
    }
  }
}

/**
   @param name return the first device with this name
   @return if found, a range device with the given name, if not found NULL
**/
AREXPORT ArRangeDevice *ArRobot::findRangeDevice(const char *name)
{
  std::list<ArRangeDevice *>::iterator it;
  ArRangeDevice *device;

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    device = (*it);
    if(strcmp(name, device->getName()) == 0)
    {
      return device;
    }
  }
  return NULL;
}

/**
   @param name return the first device with this name
   @return if found, a range device with the given name, if not found NULL
**/
AREXPORT const ArRangeDevice *ArRobot::findRangeDevice(const char *name) const
{
  std::list<ArRangeDevice *>::const_iterator it;
  ArRangeDevice *device;

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    device = (*it);
    if(strcmp(name, device->getName()) == 0)
    {
      return device;
    }
  }
  return NULL;
}

/**
    This gets the list of range devices attached to this robot, do NOT
    manipulate this list directly.  If you want to manipulate use the
    appropriate addRangeDevice, or remRangeDevice
    @return the list of range dvices attached to this robot
**/
AREXPORT std::list<ArRangeDevice *> *ArRobot::getRangeDeviceList(void)
{
  return &myRangeDeviceList;
}

/**
   @param device the device to check for
**/
AREXPORT bool ArRobot::hasRangeDevice(ArRangeDevice *device) const
{
  std::list<ArRangeDevice *>::const_iterator it;
  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    if ((*it) == device)
      return true;
  }
  return false;
}

/**
 *  Find the closest reading from any range device's set of current readings
 *  within a polar region or "slice" defined by the given angle range.
 *  This function iterates through each registered range device (see
 *  addRangeDevice()), calls ArRangeDevice::lockDevice(), uses
 *  ArRangeDevice::currentReadingPolar() to find a reading, then calls
 *  ArRangeDevice::unlockDevice().
 *
 *  @copydoc ArRangeDevice::currentReadingPolar()
 *  @param rangeDevice If not null, then a pointer to the ArRangeDevice
 *    that provided the returned reading is placed in this variable.
 *
**/

AREXPORT double ArRobot::checkRangeDevicesCurrentPolar(
	double startAngle, double endAngle, double *angle,
	const ArRangeDevice **rangeDevice,
	bool useLocationDependentDevices) const
{
  double closest = 32000;
  double closeAngle, tempDist, tempAngle;
  std::list<ArRangeDevice *>::const_iterator it;
  ArRangeDevice *device;
  bool foundOne = false;
  const ArRangeDevice *closestRangeDevice = NULL;

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    device = (*it);
    device->lockDevice();
    if (!useLocationDependentDevices && device->isLocationDependent())
    {
      device->unlockDevice();
      continue;
    }
    if (!foundOne ||
	(tempDist = device->currentReadingPolar(startAngle, endAngle,
						&tempAngle)) < closest)
    {
      if (!foundOne)
      {
	closest = device->currentReadingPolar(startAngle, endAngle,
					      &closeAngle);
	closestRangeDevice = device;
      }
      else
      {
	closest = tempDist;
	closeAngle = tempAngle;
	closestRangeDevice = device;
      }
      foundOne = true;
    }
    device->unlockDevice();
  }
  if (!foundOne)
    return -1;
  if (angle != NULL)
    *angle = closeAngle;
  if (rangeDevice != NULL)
    *rangeDevice = closestRangeDevice;
  return closest;

}


/**
 *  Find the closest reading from any range device's set of cumulative readings
 *  within a polar region or "slice" defined by the given angle range.
 *  This function iterates through each registered range device (see
 *  addRangeDevice()), calls ArRangeDevice::lockDevice(), uses
 *  ArRangeDevice::cumulativeReadingPolar() to find a reading, then calls
 *  ArRangeDevice::unlockDevice().
 *
 *  @copydoc ArRangeDevice::cumulativeReadingPolar()
 *  @param rangeDevice If not null, then a pointer to the ArRangeDevice
 *    that provided the returned reading is placed in this variable.
**/
AREXPORT double ArRobot::checkRangeDevicesCumulativePolar(
	double startAngle, double endAngle, double *angle,
	const ArRangeDevice **rangeDevice,
	bool useLocationDependentDevices) const
{
  double closest = 32000;
  double closeAngle, tempDist, tempAngle;
  std::list<ArRangeDevice *>::const_iterator it;
  ArRangeDevice *device;
  bool foundOne = false;
  const ArRangeDevice *closestRangeDevice = NULL;

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    device = (*it);
    device->lockDevice();
    if (!useLocationDependentDevices && device->isLocationDependent())
    {
      device->unlockDevice();
      continue;
    }
    if (!foundOne ||
	(tempDist = device->cumulativeReadingPolar(startAngle, endAngle,
						 &tempAngle)) < closest)
    {
      if (!foundOne)
      {
	closest = device->cumulativeReadingPolar(startAngle, endAngle,
					      &closeAngle);
	closestRangeDevice = device;
      }
      else
      {
	closest = tempDist;
	closeAngle = tempAngle;
	closestRangeDevice = device;
      }
      foundOne = true;
    }
    device->unlockDevice();
  }
  if (!foundOne)
    return -1;
  if (angle != NULL)
    *angle = closeAngle;
  if (rangeDevice != NULL)
    *rangeDevice = closestRangeDevice;
  return closest;

}

/**
   This goes through all of the registered range devices and locks each,
   calls currentReadingBox on it, and then unlocks it.

   Gets the closest reading in a region defined by the two points of a
   rectangle.
   @param x1 the x coordinate of one of the rectangle points
   @param y1 the y coordinate of one of the rectangle points
   @param x2 the x coordinate of the other rectangle point
   @param y2 the y coordinate of the other rectangle point
   @param readingPos a pointer to a position in which to store the location of
   the closest position
   @return if the return is >= 0 then this is the distance to the closest
   reading, if it is < 0 then there were no readings in the given region
**/

AREXPORT double ArRobot::checkRangeDevicesCurrentBox(
	double x1, double y1, double x2, double y2, ArPose *readingPos,
	const ArRangeDevice **rangeDevice,
	bool useLocationDependentDevices) const
{

  double closest = 32000;
  double tempDist;
  ArPose closestPos, tempPos;
  std::list<ArRangeDevice *>::const_iterator it;
  ArRangeDevice *device;
  bool foundOne = false;
  const ArRangeDevice *closestRangeDevice = NULL;

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    device = (*it);
    device->lockDevice();
    if (!useLocationDependentDevices && device->isLocationDependent())
    {
      device->unlockDevice();
      continue;
    }
    if (!foundOne ||
	(tempDist = device->currentReadingBox(x1, y1, x2, y2, &tempPos)) < closest)
    {
      if (!foundOne)
      {
	closest = device->currentReadingBox(x1, y1, x2, y2, &closestPos);
	closestRangeDevice = device;
      }
      else
      {
	closest = tempDist;
	closestPos = tempPos;
	closestRangeDevice = device;
      }
      foundOne = true;
    }
    device->unlockDevice();
  }
  if (!foundOne)
    return -1;
  if (readingPos != NULL)
    *readingPos = closestPos;
  if (rangeDevice != NULL)
    *rangeDevice = closestRangeDevice;
  return closest;
}

/**
   This goes through all of the registered range devices and locks each,
   calls cumulativeReadingBox on it, and then unlocks it.

   Gets the closest reading in a region defined by the two points of a
   rectangle.
   @param x1 the x coordinate of one of the rectangle points
   @param y1 the y coordinate of one of the rectangle points
   @param x2 the x coordinate of the other rectangle point
   @param y2 the y coordinate of the other rectangle point
   @param readingPos a pointer to a position in which to store the location of
   the closest position
   @return if the return is >= 0 then this is the distance to the closest
   reading, if it is < 0 then there were no readings in the given region
**/

AREXPORT double ArRobot::checkRangeDevicesCumulativeBox(
	double x1, double y1, double x2, double y2, ArPose *readingPos,
	const ArRangeDevice **rangeDevice,
	bool useLocationDependentDevices) const
{

  double closest = 32000;
  double tempDist;
  ArPose closestPos, tempPos;
  std::list<ArRangeDevice *>::const_iterator it;
  ArRangeDevice *device;
  bool foundOne = false;
  const ArRangeDevice *closestRangeDevice = NULL;

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); ++it)
  {
    device = (*it);
    device->lockDevice();
    if (!useLocationDependentDevices && device->isLocationDependent())
    {
      device->unlockDevice();
      continue;
    }
    if (!foundOne ||
	(tempDist = device->cumulativeReadingBox(x1, y1, x2, y2, &tempPos)) < closest)
    {
      if (!foundOne)
      {
	closest = device->cumulativeReadingBox(x1, y1, x2, y2, &closestPos);
	closestRangeDevice = device;
      }
      else
      {
	closest = tempDist;
	closestPos = tempPos;
	closestRangeDevice = device;
      }
      foundOne = true;
    }
    device->unlockDevice();
  }
  if (!foundOne)
    return -1;
  if (readingPos != NULL)
    *readingPos = closestPos;
  if (rangeDevice != NULL)
    *rangeDevice = closestRangeDevice;
  return closest;
}

/**
 *
 * The robot-relative positions of the readings of attached range
 * devices, plus sonar readings stored in this object, will also be updated.
 *
 * @note This simply changes our stored pose value, it does not cause the robot
 * to drive. Use setVel(), setRotVel(), move(), setHeading(), setDeltaHeading(),
 * or the actions system.
 *
    @param pose New pose to set (in absolute world coordinates)
    @param doCumulative whether to update the cumulative buffers of range devices
**/
AREXPORT void ArRobot::moveTo(ArPose pose, bool doCumulative)
{
  std::list<ArRangeDevice *>::iterator it;
  ArSensorReading *son;
  int i;

  // we need to get this one now because changing the encoder
  // transform and global pose will change the local transform
  ArTransform localTransform;
  localTransform = getToLocalTransform();

  myEncoderTransform.setTransform(myEncoderPose, pose);
  myGlobalPose = myEncoderTransform.doTransform(myEncoderPose);

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); it++)
  {
    (*it)->lockDevice();
    (*it)->applyTransform(localTransform, doCumulative);
    (*it)->applyTransform(getToGlobalTransform(), doCumulative);
    (*it)->unlockDevice();
  }

  for (i = 0; i < getNumSonar(); i++)
  {
    son = getSonarReading(i);
    if (son != NULL)
    {
      son->applyTransform(localTransform);
      son->applyTransform(getToGlobalTransform());
    }
  }

  //ArLog::log(ArLog::Normal, "Robot moved to %.0f %.0f %.1f", getX(), getY(), getTh());
}

/**
 * The robot-relative positions of the readings of attached range
 * devices, plus sonar readings stored in this object, will also be updated.
 * This variant allows you to manually specify a pose to use as the robot's
 * old pose when updating range device readings (rather than ArRobot's
 * currently stored pose).
 *
 * @note This simply changes our stored pose value, it does not cause the robot
 * to drive. Use setVel(), setRotVel(), move(), setHeading(), setDeltaHeading(),
 * or the actions system.
 *
    @param poseTo the new absolute real world position
    @param poseFrom the original absolute real world position
    @param doCumulative whether to update the cumulative buffers of range devices
**/
AREXPORT void ArRobot::moveTo(ArPose poseTo, ArPose poseFrom,
			      bool doCumulative)
{
  std::list<ArRangeDevice *>::iterator it;
  ArSensorReading *son;
  int i;

  ArPose result = myEncoderTransform.doInvTransform(poseFrom);

  // we need to get this one now because changing the encoder
  // transform and global pose will change the local transform
  ArTransform localTransform;
  localTransform = getToLocalTransform();

  myEncoderTransform.setTransform(result, poseTo);
  myGlobalPose = myEncoderTransform.doTransform(myEncoderPose);

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); it++)
  {
    (*it)->lockDevice();
    (*it)->applyTransform(localTransform, doCumulative);
    (*it)->applyTransform(getToGlobalTransform(), doCumulative);
    (*it)->unlockDevice();
  }

  for (i = 0; i < getNumSonar(); i++)
  {
    son = getSonarReading(i);
    if (son != NULL)
    {
      son->applyTransform(localTransform);
      son->applyTransform(getToGlobalTransform());
    }
  }

  //ArLog::log(ArLog::Normal, "Robot moved to %.0f %.0f %.1f", getX(), getY(), getTh());
}

/**
   @param deadReconPos the dead recon position to transform from
   @param globalPos the real world global position to transform to
**/
AREXPORT void ArRobot::setEncoderTransform(ArPose deadReconPos,
				    ArPose globalPos)
{
  myEncoderTransform.setTransform(deadReconPos, globalPos);
  myGlobalPose = myEncoderTransform.doTransform(myEncoderPose);
}

/**
   @param transformPos the position to transform to
**/
AREXPORT void ArRobot::setEncoderTransform(ArPose transformPos)
{
  myEncoderTransform.setTransform(transformPos);
  myGlobalPose = myEncoderTransform.doTransform(myEncoderPose);
}

/**
   @return the transform from encoder to global coords
**/
AREXPORT ArTransform ArRobot::getEncoderTransform(void) const
{
  return myEncoderTransform;
}

/**
   @param pose the position to set the dead recon position to
**/
AREXPORT void ArRobot::setDeadReconPose(ArPose pose)
{
  myEncoderPose.setPose(pose);
  myEncoderTransform.setTransform(myEncoderPose, myGlobalPose);
  myGlobalPose = myEncoderTransform.doTransform(myEncoderPose);
}


/**
    @return an ArTransform which can be used for transforming a position
    in local coordinates to one in global coordinates
**/
AREXPORT ArTransform ArRobot::getToGlobalTransform(void) const
{
  ArTransform trans;
  ArPose origin(0, 0, 0);
  ArPose pose = getPose();

  trans.setTransform(origin, pose);
  return trans;
}

/**
    @return an ArTransform which can be used for transforming a position
    in global coordinates to one in local coordinates
**/
AREXPORT ArTransform ArRobot::getToLocalTransform(void) const
{
  ArTransform trans;
  ArPose origin(0, 0, 0);
  ArPose pose = getPose();

  trans.setTransform(pose, origin);
  return trans;
}

/**
    Applies a transform to the range devices and sonar...
    this is mostly useful for
    translating to/from local/global coords, but may have other uses
    @param trans the transform to apply
    @param doCumulative whether to transform the cumulative buffers or not
**/
AREXPORT void ArRobot::applyTransform(ArTransform trans, bool doCumulative)
{
  std::list<ArRangeDevice *>::iterator it;
  ArSensorReading *son;
  int i;

  for (it = myRangeDeviceList.begin(); it != myRangeDeviceList.end(); it++)
  {
      (*it)->lockDevice();
      (*it)->applyTransform(trans, doCumulative);
      (*it)->unlockDevice();
  }

  for (i = 0; i < getNumSonar(); i++)
  {
    son = getSonarReading(i);
    if (son != NULL)
      son->applyTransform(trans);
  }
}

AREXPORT const char *ArRobot::getName(void) const
{
  return myName.c_str();
}

AREXPORT void ArRobot::setName(const char * name)
{
	std::list<ArRobot *> *robotList;
    std::list<ArRobot *>::iterator it;
    int i;
    char buf[1024];

 if (name != NULL)
  {
    myName = name;
  }
  else
  {



    robotList = Aria::getRobotList();
    for (i = 1, it = robotList->begin(); it != robotList->end(); it++, i++)
    {
      if (this == (*it))
      {
	    if (i == 1)
			myName = "robot";
		else
		{
			sprintf(buf, "robot%d", i);
			myName = buf;
		}
	    return;
      }
    }
    sprintf(buf, "robot%d", (int)robotList->size());
    myName = buf;
   }
}

/**
   This sets the encoderCorrectionCB, this callback returns the robots
   change in heading, it takes in the change in heading, x, and y,
   between the previous and current readings.

   @param functor an ArRetFunctor1 created as an ArRetFunctor1C, that
   will be the callback... call this function NULL to clear the
   callback @see getEncoderCorrectionCallback
**/

AREXPORT void ArRobot::setEncoderCorrectionCallback(
	ArRetFunctor1<double, ArPoseWithTime> *functor)
{
  myEncoderCorrectionCB = functor;
}
/**
   This gets the encoderCorrectionCB, see setEncoderCorrectionCallback
   for details.

   @return the callback, or NULL if there isn't one
**/
AREXPORT ArRetFunctor1<double, ArPoseWithTime> *
ArRobot::getEncoderCorrectionCallback(void) const
{
  return myEncoderCorrectionCB;
}

/**
   The direct motion precedence time determines how long actions will be
   ignored after a direct motion command is given.  If the direct motion
   precedence time is 0, then direct motion will take precedence over actions
   until a clearDirectMotion command is issued.  This value defaults to 0.
   @param the number of milliseconds direct movement should trump actions,
   if a negative number is given, then the value will be 0
   @see setDirectMotionPrecedenceTime
   @see clearDirectMotion
**/
AREXPORT void ArRobot::setDirectMotionPrecedenceTime(int mSec)
{
  if (mSec < 0)
    myDirectPrecedenceTime = 0;
  else
    myDirectPrecedenceTime = mSec;
}

/**
   The direct motion precedence time determines how long actions will be
   ignored after a direct motion command is given.  If the direct motion
   precedence time is 0, then direct motion will take precedence over actions
   until a clearDirectMotion command is issued.  This value defaults to 0.
   @return the number of milliseconds direct movement will trump actions
   @see setDirectMotionPrecedenceTime
   @see clearDirectMotion
**/
AREXPORT unsigned int ArRobot::getDirectMotionPrecedenceTime(void) const
{
  return myDirectPrecedenceTime;
}


/**
   This clears the direct motion commands so that actions will be allowed to
   control the robot again.
   @see setDirectMotionPrecedenceTime
   @see getDirectMotionPrecedenceTime
**/
AREXPORT void ArRobot::clearDirectMotion(void)
{
  myTransType = TRANS_NONE;
  myLastTransType = TRANS_NONE;
  myRotType = ROT_NONE;
  myLastRotType = ROT_NONE;
  myLastActionTransVal = 0;
  myLastActionRotStopped = true;
  myLastActionRotHeading = false;
}

/**
   This stops the state reflection of motion commands from sending any
   commands to the robot.  If you use direct motion commands or
   clearDirectMotion state reflection will become active again (for
   motion commands it becomes active for the type you give it, ie just
   rot or vel).
 **/
AREXPORT void ArRobot::stopStateReflection(void)
{
  myTransType = TRANS_IGNORE;
  myLastTransType = TRANS_IGNORE;
  myRotType = ROT_IGNORE;
  myLastRotType = ROT_IGNORE;
}

/**
   Returns the state of direct motion commands: whether actions are allowed or not
   @see clearDirectMotion
**/
AREXPORT bool ArRobot::isDirectMotion(void) const
{
  if (myTransType ==  TRANS_NONE && myLastTransType == TRANS_NONE &&
      myRotType == ROT_NONE && myLastRotType == ROT_NONE)
    return false;
  else
    return true;
}


/**
   This command enables the motors on the robot, if it is connected.
**/
AREXPORT void ArRobot::enableMotors()
{
  comInt(ArCommands::ENABLE, 1);
}

/**
   This command disables the motors on the robot, if it is connected.
**/
AREXPORT void ArRobot::disableMotors()
{
  comInt(ArCommands::ENABLE, 0);
}

/**
   This command enables the sonars on the robot, if it is connected.
**/
AREXPORT void ArRobot::enableSonar()
{
  comInt(ArCommands::SONAR, 1);
}

/**
   This command disables the sonars on the robot, if it is connected.
**/
AREXPORT void ArRobot::disableSonar()
{
  comInt(ArCommands::SONAR, 0);
}


/**
   The state reflection refresh time is the number of milliseconds between
   when the state reflector will refresh the robot, if the command hasn't
   changed.  The default is 500 milliseconds.  If this number is less than
   the cyle time, it'll simply happen every cycle.
   @param mSec the refresh time, in milliseconds, non-negative, if negative is
   given, then the value will be 0
**/
AREXPORT void ArRobot::setStateReflectionRefreshTime(int mSec)
{
  if (mSec < 0)
    myStateReflectionRefreshTime = 0;
  else
    myStateReflectionRefreshTime = mSec;
}

/**
   The state reflection refresh time is the number of milliseconds between
   when the state reflector will refresh the robot, if the command hasn't
   changed.  The default is 500 milliseconds.  If this number is less than
   the cyle time, it'll simply happen every cycle.
   @return the state reflection refresh time
**/
AREXPORT int ArRobot::getStateReflectionRefreshTime(void) const
{
  return myStateReflectionRefreshTime;
}

/**
   This will attach a key handler to a robot, by putting it into the
   robots sensor interp task list (a keyboards a sensor of users will,
   right?).  By default exitOnEscape is true, which will cause this
   function to add an escape key handler to the key handler, this will
   make the program exit when escape is pressed... if you don't like
   this you can pass exitOnEscape in as false.

   @param keyHandler the key handler to attach

   @param exitOnEscape whether to exit when escape is pressed or not

   @param useExitNotShutdown if true then Aria::exit will be called
   instead of Aria::shutdown if it tries to exit
**/
AREXPORT void ArRobot::attachKeyHandler(ArKeyHandler *keyHandler,
					bool exitOnEscape,
					bool useExitNotShutdown)
{
  if (myKeyHandlerCB != NULL)
    delete myKeyHandlerCB;
  myKeyHandlerCB = new ArFunctorC<ArKeyHandler>(keyHandler,
					    &ArKeyHandler::checkKeys);
  addSensorInterpTask("Key Handler", 50, myKeyHandlerCB);

  myKeyHandler = keyHandler;
  myKeyHandlerUseExitNotShutdown = useExitNotShutdown;
  if (exitOnEscape)
    keyHandler->addKeyHandler(ArKeyHandler::ESCAPE, &myKeyHandlerExitCB);
}

AREXPORT ArKeyHandler *ArRobot::getKeyHandler(void) const
{
  return myKeyHandler;
}

AREXPORT void ArRobot::keyHandlerExit(void)
{
  ArLog::log(ArLog::Terse, "Escape was pressed, program is exiting.");
  // if we're using exit not the keyhandler then call Aria::exit
  // instead of shutdown, this call never returns
  if (myKeyHandlerUseExitNotShutdown)
    Aria::exit();
  stopRunning();
  unlock();
  Aria::shutdown();
}

AREXPORT void ArRobot::setPacketsReceivedTracking(bool packetsReceivedTracking)
{
  myPacketsReceivedTracking = packetsReceivedTracking;
  myPacketsReceivedTrackingCount = 0;
  myPacketsReceivedTrackingStarted.setToNow();
}

AREXPORT void ArRobot::ariaExitCallback(void)
{
  mySyncLoop.stopRunIfNotConnected(false);
  disconnect();
}

/**
   Note that this is only available on robots with an firwmare
   operating system (ARCOS or uARCS) thats at least newer than July of
   2005.  It'll just be CHARGING_UNKNOWN on everything else always.
 **/
AREXPORT ArRobot::ChargeState ArRobot::getChargeState(void) const
{
  return myChargeState;
}

AREXPORT void ArRobot::resetOdometer(void)
{
  myOdometerDistance = 0;
  myOdometerDegrees = 0;
  myOdometerStart.setToNow();
}
