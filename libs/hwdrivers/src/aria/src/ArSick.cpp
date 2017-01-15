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
#include "ArSick.h"
#include "ArRobot.h"
#include "ArSerialConnection.h"
#include "ariaInternal.h"
#include <time.h>

AREXPORT ArSick::ArSick(size_t currentBufferSize, size_t cumulativeBufferSize,
			const char *name, bool addAriaExitCB) :
  ArRangeDeviceThreaded(currentBufferSize, cumulativeBufferSize, name, 32500),
  myRobotConnectCB(this, &ArSick::robotConnectCallback),
  mySimPacketHandler(this, &ArSick::simPacketHandler),
  mySensorInterpCB(this, &ArSick::sensorInterpCallback),
  mySickPacketReceiver(0, true),
  myAriaExitCB(this, &ArSick::disconnect, true)
{
  myAriaExitCB.setName("ArSickExit");
  if (addAriaExitCB)
    Aria::addExitCallback(&myAriaExitCB, 10);
  mySimPacketHandler.setName("ArSick");
  configure();
  setRangeInformation();
  setSensorPosition(0, 0, 0);
  myAssembleReadings = new std::list<ArSensorReading *>;
  myCurrentReadings = new std::list<ArSensorReading *>;
  myRawReadings = myCurrentReadings;
  myIter = myAssembleReadings->begin();
  myConn = NULL;
  myRobot = NULL;
  myStartConnect = false;
  myRunningOnRobot = false;
  switchState(STATE_NONE);
  myProcessImmediately = false;
  myInterpolation = true;
  myTimeoutTime = 8;
  myRealConfigured = false;

  // default filter params
  setMinRange(125);
  setFilterNearDist(50);
  setFilterCumulativeMaxDist(6000);
  setFilterCumulativeInsertMaxDist(3000);
  setFilterCumulativeNearDist(200);
  setFilterCumulativeCleanDist(75);
  setFilterCumulativeMaxAge(30);
  setFilterCleanCumulativeInterval(1000);

  myLastCleanedCumulative.setToNow();

  setCurrentDrawingData(new ArDrawingData("polyDots",
										  ArColor(0, 0, 255),
										  80,  // mm diameter of dots
										  75), // layer above sonar
						true);

  setCumulativeDrawingData(new ArDrawingData("polyDots",
											 ArColor(125, 125, 125),
											 100, // mm diameter of dots
											 60), // layer below current range devices
						   true);
}

AREXPORT ArSick::~ArSick()
{
  if (myRobot != NULL)
  {
    myRobot->remRangeDevice(this);
    myRobot->remPacketHandler(&mySimPacketHandler);
    myRobot->remSensorInterpTask(&mySensorInterpCB);
    myRobot->addConnectCB(&myRobotConnectCB, ArListPos::FIRST);
  }
  lockDevice();
  if (isConnected())
  {
    disconnect();
  }
  unlockDevice();
}

/** @internal */
AREXPORT void ArSick::robotConnectCallback(void)
{
  const ArRobotParams *params;
  if (myRealConfigured || !myRobot->isConnected())
    return;
  params = myRobot->getRobotParams();
  myLaserFlipped = params->getLaserFlipped();
  myPowerControl = params->getLaserPowerControlled();

  if (myDegrees == DEGREES180)
    myOffsetAmount = 90;
  else if (myDegrees == DEGREES100)
    myOffsetAmount = 50;
  else
  {
    myOffsetAmount = 0;
    ArLog::log(ArLog::Terse,"ArSick::robotConnectCallback: bad degrees configured.\n");
  }


  if (myLaserFlipped)
    myOffsetAmount *= -1;

  if (myIncrement == INCREMENT_ONE)
    myIncrementAmount = 1.0;
  else if (myIncrement == INCREMENT_HALF)
    myIncrementAmount = 0.5;
  else
  {
    myIncrementAmount = 0;
    ArLog::log(ArLog::Terse,"ArSick::robotConnectCallback: bad increment configured.\n");
  }
  if (myLaserFlipped)
    myIncrementAmount *= -1;

  clearIgnoreReadings();
  ArArgumentBuilder builder;
  builder.add(myRobot->getRobotParams()->getLaserIgnore());
  size_t i;
  for (i = 0; i < builder.getArgc(); i++)
  {
    if (!builder.isArgDouble(i))
    {
      ArLog::log(ArLog::Normal, "ArRobotConfig::setIgnoreReadings: argument is not a double");
    }
    addIgnoreReading(builder.getArgDouble(i));
  }

  setSensorPosition(params->getLaserX(), params->getLaserY(), params->getLaserTh());
}

AREXPORT bool ArSick::isUsingSim(void)
{
  return myUseSim;
}

AREXPORT bool ArSick::isControllingPower(void)
{
  return myPowerControl;
}

AREXPORT bool ArSick::isLaserFlipped(void)
{
  return myLaserFlipped;
}

AREXPORT ArSick::Degrees ArSick::getDegrees(void)
{
  return myDegrees;
}

AREXPORT ArSick::Increment ArSick::getIncrement(void)
{
  return myIncrement;
}

AREXPORT ArSick::Bits ArSick::getBits(void) { return myBits; }
AREXPORT ArSick::Units ArSick::getUnits(void) { return myUnits; }

AREXPORT void ArSick::setIsUsingSim(bool usingSim)
{
  myUseSim = usingSim;
}

AREXPORT void ArSick::setIsControllingPower(bool controlPower)
{
  myPowerControl = controlPower;
}

AREXPORT void ArSick::setIsLaserFlipped(bool laserFlipped)
{
  myLaserFlipped = laserFlipped;
}


AREXPORT void ArSick::setSensorPosition(double x, double y, double th)
{
  mySensorPose.setPose(x, y, th);
}

AREXPORT void ArSick::setSensorPosition(ArPose pose)
{
  mySensorPose = pose;
}

AREXPORT ArPose ArSick::getSensorPosition(void)
{
  return mySensorPose;
}

AREXPORT double ArSick::getSensorPositionX(void)
{
  return mySensorPose.getX();
}

AREXPORT double ArSick::getSensorPositionY(void)
{
  return mySensorPose.getY();
}

AREXPORT double ArSick::getSensorPositionTh(void)
{
  return mySensorPose.getTh();
}


AREXPORT void ArSick::setDeviceConnection(ArDeviceConnection *conn)
{
  myConnLock.lock();
  myConn = conn;
  mySickPacketReceiver.setDeviceConnection(conn);
  myConnLock.unlock();
}

AREXPORT ArDeviceConnection *ArSick::getDeviceConnection(void)
{
  return myConn;
}

/**
   Sets the time to go without a response from the laser
   until it is assumed that the connection with the laser has been
   broken and the disconnect on error events will happen.

   @param mSecs if 0 then the connection timeout feature
   will be disabled, otherwise disconnect on error will be triggered
   after this number of miliseconds...
**/
AREXPORT void ArSick::setConnectionTimeoutTime(int mSecs)
{
  if (mSecs > 0)
    myTimeoutTime = mSecs;
  else
    myTimeoutTime = 0;
}

/**
   Gets the time (miliseconds) to go without response from the laser
   until it is assumed that the connection with the laser has been
   broken and the disconnect on error events will happen.
   If 0, then the timeout is disabled.
**/
AREXPORT int ArSick::getConnectionTimeoutTime(void)
{
  return myTimeoutTime;
}

AREXPORT ArTime ArSick::getLastReadingTime(void)
{
  return myLastReading;
}

AREXPORT void ArSick::setRobot(ArRobot *robot)
{
  myRobot = robot;
  if (myRobot != NULL)
  {
    myRobot->addPacketHandler(&mySimPacketHandler, ArListPos::LAST);
    myRobot->addSensorInterpTask("sick", 90, &mySensorInterpCB);
    myRobot->addConnectCB(&myRobotConnectCB, ArListPos::FIRST);
    if (myRobot->isConnected())
      robotConnectCallback();
  }
  ArRangeDevice::setRobot(robot);
}

/**
 * Manually set laser configuration options for connection. This must be called
 * only before connecting to the laser (not while the laser is connected).
 * This configuration is automatically performed if you are using
 * ArSimpleConnector to connect to the laser based on command line parameters,
 * so calling this function is only neccesary if you are not using
 * ArSimpleConnector, or you wish to always override ArSimpleConnector's
 * configuration.
 *
 * (Don't forget, you must lock ArSick with lockDevice() if multiple threads
 * are accessing the ArSick, e.g. if you used runAsync().)
**/
AREXPORT void ArSick::configure(bool useSim, bool powerControl,
				bool laserFlipped, BaudRate baud,
				Degrees deg, Increment incr)
{

  myUseSim = useSim;
  myPowerControl = powerControl;
  myLaserFlipped = laserFlipped;
  myBaud = baud;
  myDegrees = deg;
  myIncrement = incr;

  if (myDegrees == DEGREES180)
    myOffsetAmount = 90;
  else if (myDegrees == DEGREES100)
    myOffsetAmount = 50;
  else
  {
    myOffsetAmount = 0;
    ArLog::log(ArLog::Terse,"ArSick::configure: bad degrees configured.\n");
  }

  if (myLaserFlipped)
    myOffsetAmount *= -1;

  if (myIncrement == INCREMENT_ONE)
    myIncrementAmount = 1.0;
  else if (myIncrement == INCREMENT_HALF)
    myIncrementAmount = 0.5;
  else
  {
    myIncrementAmount = 0;
    ArLog::log(ArLog::Terse,"ArSick::configure: bad increment configured.\n");
  }

  if (myLaserFlipped)
    myIncrementAmount *= -1;

  myRealConfigured = true;
}

/**
 * @copydoc configure()
**/
AREXPORT void ArSick::configureShort(bool useSim, BaudRate baud,
				Degrees deg, Increment incr)
{

  myUseSim = useSim;
  myPowerControl = true;
  myLaserFlipped = false;
  myBaud = baud;
  myDegrees = deg;
  myIncrement = incr;

  if (myDegrees == DEGREES180)
    myOffsetAmount = 90;
  else if (myDegrees == DEGREES100)
    myOffsetAmount = 50;
  else
  {
    myOffsetAmount = 0;
    ArLog::log(ArLog::Terse,"ArSick::configureShort: bad degrees configured.\n");
  }

  if (myLaserFlipped)
    myOffsetAmount *= -1;

  if (myIncrement == INCREMENT_ONE)
    myIncrementAmount = 1.0;
  else if (myIncrement == INCREMENT_HALF)
    myIncrementAmount = 0.5;
  else
  {
    myIncrementAmount = 0;
    ArLog::log(ArLog::Terse,"ArSick::configureShort: bad increment configured.\n");
  }
  if (myLaserFlipped)
    myIncrementAmount *= -1;

  myRealConfigured = false;
  // if we're connected, just have this set things
  if (myRobot != NULL && myRobot->isConnected())
    robotConnectCallback();
}

/**
   Sets the range/bit information.  The old immutable combination is
   (in effect) the same as the new default.  If you look at the enums
   for these units you can see the effect this has on range.
**/
AREXPORT void ArSick::setRangeInformation(Bits bits, Units units)
{
  myUnits = units;
  myBits = bits;
}


/** @internal */
AREXPORT bool ArSick::simPacketHandler(ArRobotPacket *packet)
{
  std::list<ArFunctor *>::iterator it;

  unsigned int totalNumReadings;
  unsigned int readingNumber;
  double atDeg;
  unsigned int i;
  ArSensorReading *reading;
  std::list<ArSensorReading *>::iterator tempIt;
  unsigned int newReadings;
  int range;
  int refl = 0;
  ArPose encoderPose;
  std::list<double>::iterator ignoreIt;
  bool ignore;

  if (packet->getID() != 0x60 && packet->getID() != 0x61)
    return false;

  bool isExtendedPacket = (packet->getID() == 0x61);

  // if we got here, its the right type of packet

  //printf("Got in a packet from the simulator\n");
  lockDevice();
  //printf("1\n");
  if (!myUseSim)
  {
    ArLog::log(ArLog::Terse, "ArSick: Got a packet from the simulator with laser information, but the laser is not being simulated, major trouble.");
    unlockDevice();
    return true;
  }
  if(!isExtendedPacket)
  {
    // ignore the positional information
    packet->bufToByte2();
    packet->bufToByte2();
    packet->bufToByte2();
  }
  totalNumReadings = packet->bufToByte2(); // total for this reading
  readingNumber = packet->bufToByte2(); // which one we're on in this packet
  newReadings = packet->bufToUByte(); // how many are in this packet
  if (readingNumber == 0)
  {
    mySimPacketStart = myRobot->getPose();
    mySimPacketTrans = myRobot->getToGlobalTransform();
    mySimPacketEncoderTrans = myRobot->getEncoderTransform();
    mySimPacketCounter = myRobot->getCounter();
  }
  //printf("ArSick::simPacketHandler: On reading number %d out of %d, new %d\n", readingNumber, totalNumReadings, newReadings);
  // if we have too many readings in our list of raw readings, pop the extras
  while (myAssembleReadings->size() > totalNumReadings)
  {
    ArLog::log(ArLog::Verbose, "ArSick::simPacketHandler, too many readings, popping one.\n");
    tempIt = myAssembleReadings->begin();
    if (tempIt != myAssembleReadings->end())
      delete (*tempIt);
    myAssembleReadings->pop_front();
  }

  // If we don't have any sensor readings created at all, make 'em all now
  if (myAssembleReadings->size() == 0)
    for (i = 0; i < totalNumReadings; i++)
      myAssembleReadings->push_back(new ArSensorReading);

  // Okay, we know where we're at, so get an iterator to the right spot, or
  // make sure the one we keep around is in the right spot... if neither of
  // these trigger, then the iter should be in the right spot
  if ((readingNumber != myWhichReading + 1) ||
      totalNumReadings != myTotalNumReadings)
  {
    //printf("2\n");
    myWhichReading = readingNumber;
    myTotalNumReadings = totalNumReadings;
    for (i = 0, myIter = myAssembleReadings->begin(); i < readingNumber; i++)
    {
      tempIt = myIter;
      tempIt++;
      if (tempIt == myAssembleReadings->end() && (i + 1 != myTotalNumReadings))
	myAssembleReadings->push_back(new ArSensorReading);
      myIter++;
    }
  }
  else
  {
    //printf("3\n");
    myWhichReading = readingNumber;
  }

  atDeg = (mySensorPose.getTh() - myOffsetAmount +
	   readingNumber * myIncrementAmount);
  //printf("4\n");
  encoderPose = mySimPacketEncoderTrans.doInvTransform(mySimPacketStart);
  // while we have in the readings and have stuff left we can read
  for (i = 0;
       //	 (myWhichReading < myTotalNumReadings &&
       //	  packet->getReadLength() < packet->getLength() - 4);
       i < newReadings;
       i++, myWhichReading++, atDeg += myIncrementAmount)
  {
    reading = (*myIter);
    range = packet->bufToUByte2();
    if(isExtendedPacket)
    {
      refl = packet->bufToUByte();
      packet->bufToUByte(); // don't need this byte for anything yet
      packet->bufToUByte(); // don't need this byte for anything yet
    }
    ignore = false;
    for (ignoreIt = myIgnoreReadings.begin();
	 ignoreIt != myIgnoreReadings.end();
	 ignoreIt++)
    {
      //if (atDeg == 0)
      //printf("Ignoring %.0f\n", (*ignoreIt));
      if (ArMath::fabs(ArMath::subAngle(atDeg, *(ignoreIt))) < 1.0)
      {
	//printf("Ignoring %.0f\n", (*ignoreIt));
	ignore = true;
	break;
      }
    }
    if (myMaxRange != 0 && range < (int)myMinRange)
      ignore = true;
    if (myMaxRange != 0 && range > (int)myMaxRange)
      ignore = true;

    reading->resetSensorPosition(ArMath::roundInt(mySensorPose.getX()),
				 ArMath::roundInt(mySensorPose.getY()),
				 atDeg);
    //      printf("dist %d\n", dist);
    reading->newData(range, mySimPacketStart,
		     encoderPose,
		     mySimPacketTrans,
		     mySimPacketCounter, packet->getTimeReceived(), ignore, refl);

    //addReading(reading->getX(), reading->getY());
    tempIt = myIter;
    tempIt++;
    if (tempIt == myAssembleReadings->end() &&
	myWhichReading + 1 != myTotalNumReadings)
    {
      myAssembleReadings->push_back(new ArSensorReading);
    }
    myIter++;
  }

  // check if the sensor set is complete
  //printf("%d %d %d\n", newReadings, readingNumber, totalNumReadings);
  if (newReadings + readingNumber >= totalNumReadings)
  {
    // set ArRangeDevice buffer
    myRawReadings = myAssembleReadings;
    // switch internal buffers
    myAssembleReadings = myCurrentReadings;
    myCurrentReadings = myRawReadings;
    // We have in all the readings, now sort 'em and update the current ones
    filterReadings();

    if (myTimeLastSickPacket != time(NULL))
      {
	myTimeLastSickPacket = time(NULL);
	mySickPacCount = mySickPacCurrentCount;
	mySickPacCurrentCount = 0;
      }
    mySickPacCurrentCount++;
    myLastReading.setToNow();

    for (it = myDataCBList.begin(); it != myDataCBList.end(); it++)
      (*it)->invoke();
  }

  unlockDevice();
  return true;
}

/**
 @internal

 Filter readings, moving them from the raw current buffer to filtered current
 buffer (see ArRangeDevice), and then also to the cumulative
 buffer. This is called automatically when new data is received from
 the Sick.

 Current buffer filtering eliminates max (null) range readings, and
 compresses close readings.

 Cumulative buffer filtering replaces readings within the scope of the
 current sensor set.
**/

void ArSick::filterReadings()
{
  std::list<ArSensorReading *>::iterator sensIt;
  ArSensorReading *sReading;
  double x, y;
  double squaredDist;
  double lastX = 0.0, lastY = 0.0;
  unsigned int i;
  double squaredNearDist = myFilterNearDist * myFilterNearDist;
  ArTime len;
  len.setToNow();

  bool clean;
  if (myFilterCleanCumulativeInterval == 0 ||
      myLastCleanedCumulative.mSecSince() > myFilterCleanCumulativeInterval)
  {
    myLastCleanedCumulative.setToNow();
    clean = true;
  }
  else
  {
    clean = false;
  }

  sensIt = myRawReadings->begin();
  sReading = (*sensIt);
  myCurrentBuffer.setPoseTaken(sReading->getPoseTaken());
  myCurrentBuffer.setEncoderPoseTaken(sReading->getEncoderPoseTaken());
  //myCurrentBuffer.reset();

  // if we don't have any readings in the buffer yet, fill it up
  /* MPL commented this out since if we had no valid readings it'd
   * leak memory like a sieve

  if (myCurrentBuffer.getBuffer()->size() == 0)
    for (i = 0; i < myRawReadings->size(); i++)
      myCurrentBuffer.getBuffer()->push_back(new ArPoseWithTime);
  */

  i = 0;
  // walk the buffer of all the readings and see if we want to add them
  for (myCurrentBuffer.beginRedoBuffer();
       sensIt != myRawReadings->end();
       ++sensIt)
  {
    sReading = (*sensIt);
    /*
    if (sReading->getIgnoreThisReading())
       printf("Ignoring %.0f %d\n", sReading->getSensorTh(),sReading->getRange());
    */
    // see if the reading is in the valid range
    if (!sReading->getIgnoreThisReading())
    {
      // get our coords
      x = sReading->getX();
      y = sReading->getY();


      // see if we're checking on the filter near dist... if we are
      // and the reading is a good one we'll check the cumulative
      // buffer
      if (squaredNearDist > 0.0000001)
      {
	// see where the last reading was
	squaredDist = (x-lastX)*(x-lastX) + (y-lastY)*(y-lastY);
	// see if the reading is far enough from the last reading
	if (squaredDist > squaredNearDist)		// moved enough
	{
	  lastX = x;
	  lastY = y;
	  // since it was a good reading, see if we should toss it in
	  // the cumulative buffer...
	  filterAddAndCleanCumulative(x, y, clean);

	  /* we don't do this part anymore since it wound up leaving
	  // too many things not really tehre... if its outside of our
	  // sensor angle to use to filter then don't let this one
	  // clean  (ArMath::fabs(sReading->getSensorTh()) > 50)
	  // filterAddAndCleanCumulative(x, y, false); else*/
	}
	// it wasn't far enough, skip this one and go to the next one
	else
	{
	  continue;
	}
      }
      // we weren't filtering the readings, but see if it goes in the
      // cumulative buffer anyways
      else
      {
	filterAddAndCleanCumulative(x, y, clean);
      }
      // now drop the reading into the current buffer
      myCurrentBuffer.redoReading(x, y);
      i++;
    }
  }
  myCurrentBuffer.endRedoBuffer();
  /*  Put this in to see how long the cumulative filtering is taking
  if (clean)
    printf("### %ld %d\n", len.mSecSince(), myCumulativeBuffer.getBuffer()->size());
    */

}

/** @internal */
void ArSick::filterAddAndCleanCumulative(double x, double y, bool clean)
{
  if (myCumulativeBuffer.getSize() == 0)
    return;


  std::list<ArPoseWithTime *>::iterator cit;
  bool addReading = true;
  double squaredDist;

  ArLineSegment line;
  double xTaken = myCurrentBuffer.getPoseTaken().getX();
  double yTaken = myCurrentBuffer.getPoseTaken().getY();
  ArPose intersection;
  ArPoseWithTime reading(x, y);

  // make sure we really want to clean
  if (clean && myFilterSquaredCumulativeCleanDist < 1)
    clean = false;

  squaredDist = ArMath::squaredDistanceBetween(x, y, xTaken, yTaken);
  // if we're not cleaning and its further than we're keeping track of
  // readings ignore it
  if (!clean &&
      myFilterSquaredCumulativeInsertMaxDist > 1 &&
      squaredDist > myFilterSquaredCumulativeInsertMaxDist)
    return;

  // if we're cleaning we start our sweep
  if (clean)
    myCumulativeBuffer.beginInvalidationSweep();
  // run through all the readings
  for (cit = getCumulativeBuffer()->begin();
       cit != getCumulativeBuffer()->end();
       ++cit)
  {
    // if its closer to a reading than the filter near dist, just return
    if (myFilterSquaredCumulativeNearDist < .0000001 ||
	(ArMath::squaredDistanceBetween(x, y, (*cit)->getX(), (*cit)->getY()) <
	 myFilterSquaredCumulativeNearDist))
    {
      // if we're not cleaning it and its too close just return,
      // otherwise keep going (to clear out invalid readings)
      if (!clean)
	return;
      addReading = false;
    }
    // see if this reading invalidates some other readings by coming too close
    if (clean)
    {
      // set up our line
      line.newEndPoints(x, y, xTaken, yTaken);
      // see if the cumulative buffer reading perpindicular intersects
      // this line segment, and then see if its too close if it does
      if (line.getPerpPoint((*cit), &intersection) &&
	  (intersection.squaredFindDistanceTo(*(*cit)) <
        	                       myFilterSquaredCumulativeCleanDist) &&
	  (intersection.squaredFindDistanceTo(reading) > 50 * 50))
      {
	//printf("Found one too close to the line\n");
	myCumulativeBuffer.invalidateReading(cit);
      }
    }
  }
  // if we're cleaning finish the sweep
  if (clean)
    myCumulativeBuffer.endInvalidationSweep();
  // toss the reading in
  if (addReading)
    myCumulativeBuffer.addReading(x, y);

}

/** @internal */
AREXPORT void ArSick::switchState(State state)
{
  myStateMutex.lock();
  myState = state;
  myStateStart.setToNow();
  myStateMutex.unlock();
}

/**
   @internal
   @return 0 if its still trying to connect, 1 if it connected, 2 if it failed
**/
AREXPORT int ArSick::internalConnectHandler(void)
{
  ArSickPacket *packet;
  ArSerialConnection *conn;
  int value;

  switch (myState)
  {
  case STATE_INIT:
    if (myConn->getStatus() != ArDeviceConnection::STATUS_OPEN)
    {
      if ((conn = dynamic_cast<ArSerialConnection *>(myConn)) != NULL)
      {
	conn->setBaud(9600);
      }
      if (!myConn->openSimple())
      {
	ArLog::log(ArLog::Terse,
		   "ArSick: Failed to connect to laser, could not open port.");
	switchState(STATE_NONE);
	failedConnect();
	return 2;
      }
    }
    if (!myPowerControl)
    {
      /*
      myPacket.empty();
      myPacket.uByteToBuf(0x20);
      myPacket.uByteToBuf(0x25);
      myPacket.finalizePacket();
      myConn->write(myPacket.getBuf(), myPacket.getLength());
      */
      switchState(STATE_CHANGE_BAUD);
      return internalConnectHandler();
    }
    ArLog::log(ArLog::Terse, "ArSick: waiting for laser to power on.");
    myPacket.empty();
    myPacket.uByteToBuf(0x10);
    myPacket.finalizePacket();
    if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
    {
      switchState(STATE_WAIT_FOR_POWER_ON);
      return 0;
    }
    else
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, could not send init.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  case STATE_WAIT_FOR_POWER_ON:
    while ((packet = mySickPacketReceiver.receivePacket()) != NULL)
    {
      if (packet->getID() == 0x90)
      {
	switchState(STATE_CHANGE_BAUD);
	return 0;
      }
    }
    if (myStateStart.secSince() > 65)
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, no poweron received.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  case STATE_CHANGE_BAUD:
    myPacket.empty();
    myPacket.byteToBuf(0x20);
    if (myBaud == BAUD9600)
      myPacket.byteToBuf(0x42);
    else if (myBaud == BAUD19200)
      myPacket.byteToBuf(0x41);
    else if (myBaud == BAUD38400)
      myPacket.byteToBuf(0x40);
    myPacket.finalizePacket();
    if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
    {
      ArUtil::sleep(20);
      if ((conn = dynamic_cast<ArSerialConnection *>(myConn)))
      {
	if (myBaud == BAUD9600)
	  conn->setBaud(9600);
	else if (myBaud == BAUD19200)
	  conn->setBaud(19200);
	else if (myBaud == BAUD38400)
	  conn->setBaud(38400);
      }
      switchState(STATE_CONFIGURE);
      return 0;
    }
    else
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, could not send baud command.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  case STATE_CONFIGURE:
    // wait at least a 100 ms for the baud to change
    if (myStateStart.mSecSince() < 300)
      return 0;
    myPacket.empty();
    myPacket.byteToBuf(0x3b);
    myPacket.uByte2ToBuf(abs(ArMath::roundInt(myOffsetAmount * 2)));
    myPacket.uByte2ToBuf(abs(ArMath::roundInt(myIncrementAmount * 100)));
    myPacket.finalizePacket();
    if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
    {
      switchState(STATE_WAIT_FOR_CONFIGURE_ACK);
      return 0;
    }
    else
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, could not send configure command.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  case STATE_WAIT_FOR_CONFIGURE_ACK:
    while ((packet = mySickPacketReceiver.receivePacket()) != NULL)
    {
      if (packet->getID() == 0xbb)
      {
	value = packet->bufToByte();
	if (value == 0)
	{
	  ArLog::log(ArLog::Terse,
		     "ArSick: Could not configure laser, failed connect.");
	  switchState(STATE_NONE);
	  failedConnect();
	  return 2;
	}
	else if (value == 1)
	{
	  // here
	  //switchState(STATE_START_READINGS);
	  switchState(STATE_INSTALL_MODE);
	  return 0;
	}
	else
	{
	  ArLog::log(ArLog::Terse,
		     "ArSick: Could not configure laser, failed connect.");
	  switchState(STATE_NONE);
	  failedConnect();
	  return 2;
	}
      }
      else if (packet->getID() == 0xb0)
      {

	ArLog::log(ArLog::Terse, "ArSick: extra data packet while waiting for configure ack");
	myPacket.empty();
	myPacket.uByteToBuf(0x20);
	myPacket.uByteToBuf(0x25);
	myPacket.finalizePacket();
	if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
	{
	  switchState(STATE_CONFIGURE);
	  return 0;
	}
      }
      else
	ArLog::log(ArLog::Terse, "ArSick: Got a 0x%x\n", packet->getID());
    }
    if (myStateStart.mSecSince() > 10000)
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, no configure acknowledgement received.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  case STATE_INSTALL_MODE:
    if (myStateStart.mSecSince() < 200)
      return 0;
    myPacket.empty();
    myPacket.byteToBuf(0x20);
    myPacket.byteToBuf(0x00);
    myPacket.strNToBuf("SICK_LMS", strlen("SICK_LMS"));
    myPacket.finalizePacket();
    if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
    {
      switchState(STATE_WAIT_FOR_INSTALL_MODE_ACK);
      return 0;
    }
    else
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, could not send start command.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  case STATE_WAIT_FOR_INSTALL_MODE_ACK:
    while ((packet = mySickPacketReceiver.receivePacket()) != NULL)
    {
      if (packet->getID() == 0xa0)
      {
	value = packet->bufToByte();
	if (value == 0)
	{
	  //printf("Um, should set mode?\n");
	  switchState(STATE_SET_MODE);
	  return 0;
	}
	else if (value == 1)
	{
	  ArLog::log(ArLog::Terse,
		     "ArSick: Could not start laser, incorrect password.");
	  switchState(STATE_NONE);
	  failedConnect();
	  return 2;
	}
	else if (value == 2)
	{
	  ArLog::log(ArLog::Terse,
		     "ArSick: Could not start laser, LMI fault.");
	  switchState(STATE_NONE);
	  failedConnect();
	  return 2;
	}
	else
	{
	  ArLog::log(ArLog::Terse,
		     "ArSick: Could not start laser, unknown problem.");
	  switchState(STATE_NONE);
	  failedConnect();
	  return 2;
	}
      }
      else if (packet->getID() == 0xb0)
      {

	ArLog::log(ArLog::Terse, "ArSick: extra data packet\n");
	myPacket.empty();
	myPacket.uByteToBuf(0x20);
	myPacket.uByteToBuf(0x25);
	myPacket.finalizePacket();
	if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
	{
	  switchState(STATE_INSTALL_MODE);
	  return 0;
	}
      }
      else
	ArLog::log(ArLog::Terse, "ArSick: bad packet 0x%x\n", packet->getID());
    }
    if (myStateStart.mSecSince() > 10000)
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, no install mode ack received.");
      switchState(STATE_NONE);
      return 2;
    }
    break;
  case STATE_SET_MODE:
    if (myStateStart.mSecSince() < 200)
      return 0;
    myPacket.empty();
    // type of packet
    myPacket.byteToBuf(0x77);
    // blanking
    myPacket.uByte2ToBuf(0);
    // peak threshhold, stop threshold
    myPacket.uByte2ToBuf(70);
    // the old peak threshhold thats probably broken
    // myPacket.uByte2ToBuf(0);
    // fog correction
    myPacket.uByteToBuf(0);
    // measurement Mode (fun one) (we can switch this now)
    // next line was the previous permanent one
    // myPacket.uByteToBuf(6);
    int maxRange;
    maxRange = 8;
    if (myBits == BITS_1REFLECTOR)
    {
      myPacket.uByteToBuf(5);
      maxRange *= 4;
    }
    else if (myBits == BITS_2REFLECTOR)
    {
      myPacket.uByteToBuf(3);
      maxRange *= 2;
    }
    else if (myBits == BITS_3REFLECTOR)
    {
      myPacket.uByteToBuf(1);
      maxRange *= 1;
    }
    else
    {
      ArLog::log(ArLog::Terse, "ArSick: Bits set to unknown value");
      myPacket.uByteToBuf(5);
      maxRange *= 4;
    }
    // unit value (fun one), we can swithc this now
    // next line was the previous permanent one
    //myPacket.uByteToBuf(1);
    if (myUnits == UNITS_1MM)
    {
      maxRange *= 1000;
      myPacket.uByteToBuf(1);
    }
    else if (myUnits == UNITS_10CM)
    {
      maxRange = 150000;
      myPacket.uByteToBuf(2);
    }
    else if (myUnits == UNITS_1CM)
    {
      maxRange *= 10000;
      myPacket.uByteToBuf(0);
    }
    else
    {
      ArLog::log(ArLog::Terse, "ArSick: Units set to unknown value");
      maxRange *= 1000;
      myPacket.uByteToBuf(1);
    }
    setMaxRange(maxRange);
    // temporary field set
    myPacket.uByteToBuf(0);
    // fields A & B as subtractive
    myPacket.uByteToBuf(0);
    // multiple evaluation
    myPacket.uByteToBuf(2);
    // restart
    myPacket.uByteToBuf(2);
    // restart time
    myPacket.uByteToBuf(0);
    // contour A as reference object size
    myPacket.uByteToBuf(0);
    // contour A positive range of tolerance
    myPacket.uByteToBuf(0);
    // contour A negative range of tolerance
    myPacket.uByteToBuf(0);
    // contour A starting angle
    myPacket.uByteToBuf(0);
    // contour A stopping angle
    myPacket.uByteToBuf(0);
    // contour B as reference object size
    myPacket.uByteToBuf(0);
    // contour B positive range of tolerance
    myPacket.uByteToBuf(0);
    // contour B negative range of tolerance
    myPacket.uByteToBuf(0);
    // contour B starting angle
    myPacket.uByteToBuf(0);
    // contour B stopping angle
    myPacket.uByteToBuf(0);
    // contour C as reference object size
    myPacket.uByteToBuf(0);
    // contour C positive range of tolerance
    myPacket.uByteToBuf(0);
    // contour C negative range of tolerance
    myPacket.uByteToBuf(0);
    // contour C starting angle
    myPacket.uByteToBuf(0);
    // contour C stopping angle
    myPacket.uByteToBuf(0);
    // pixel oriented evaluation
    myPacket.uByteToBuf(0);
    // mode for single meas value eval
    myPacket.uByteToBuf(0);
    // restart times for field b and field c
    myPacket.byte2ToBuf(0);
    // um, an extra one (sick quickstart manual says its 21 not 20 long)
    myPacket.uByteToBuf(0);

    myPacket.finalizePacket();
    //myPacket.log();
    //printf("Sending mode!\n");
    if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
    {
      //printf("Set mode!\n");
      switchState(STATE_WAIT_FOR_SET_MODE_ACK);
      return 0;
    }
    else
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, could not send set mode command.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  case STATE_WAIT_FOR_SET_MODE_ACK:
    while ((packet = mySickPacketReceiver.receivePacket()) != NULL)
    {
      if (packet->getID() == 0xF7)
      {
	//value = packet->bufToByte();
	//printf("YAY %d\n", value);
	//packet->log();
	switchState(STATE_START_READINGS);
	return 0;
      }
      else if (packet->getID() == 0xb0)
      {

	ArLog::log(ArLog::Terse, "ArSick: extra data packet\n");
	myPacket.empty();
	myPacket.uByteToBuf(0x20);
	myPacket.uByteToBuf(0x25);
	myPacket.finalizePacket();
	if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
	{
	  switchState(STATE_INSTALL_MODE);
	  return 0;
	}
      }
      else if (packet->getID() == 0x92)
      {
	switchState(STATE_INSTALL_MODE);
	return 0;
      }
      else
	ArLog::log(ArLog::Terse, "ArSick: Got a 0x%x\n", packet->getID());
    }
    if (myStateStart.mSecSince() > 14000)
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, no set mode acknowledgement received.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
    case STATE_START_READINGS:
      if (myStateStart.mSecSince() < 200)
      return 0;
    myPacket.empty();
    myPacket.byteToBuf(0x20);
    myPacket.byteToBuf(0x24);
    myPacket.finalizePacket();
    if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
    {
      switchState(STATE_WAIT_FOR_START_ACK);
      return 0;
    }
    else
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, could not send start command.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  case STATE_WAIT_FOR_START_ACK:
    while ((packet = mySickPacketReceiver.receivePacket()) != NULL)
    {
      if (packet->getID() == 0xa0)
      {
	value = packet->bufToByte();
	if (value == 0)
	{
	  ArLog::log(ArLog::Terse, "ArSick: Connected to the laser.");
	  switchState(STATE_CONNECTED);
	  madeConnection();
	  return 1;
	}
	else if (value == 1)
	{
	  ArLog::log(ArLog::Terse,
	     "ArSick: Could not start laser laser, incorrect password.");
	  switchState(STATE_NONE);
	  failedConnect();
	  return 2;
	}
	else if (value == 2)
	{
	  ArLog::log(ArLog::Terse,
		     "ArSick: Could not start laser laser, LMI fault.");
	  switchState(STATE_NONE);
	  failedConnect();
	  return 2;
	}
	else
	{
	  ArLog::log(ArLog::Terse,
		     "ArSick: Could not start laser laser, unknown problem.");
	  switchState(STATE_NONE);
	  failedConnect();
	  return 2;
	}
      }
    }
    if (myStateStart.mSecSince() > 1000)
    {
      ArLog::log(ArLog::Terse,
		 "ArSick: Failed to connect to laser, no start acknowledgement received.");
      switchState(STATE_NONE);
      failedConnect();
      return 2;
    }
    break;
  default:
    ArLog::log(ArLog::Verbose, "ArSick: In bad connection state\n");
    break;
  }
  return 0;
}

/**
   Sends the commands to the sim to start up the connection

   @return true if the commands were sent, false otherwise
**/
AREXPORT bool ArSick::internalConnectSim(void)
{
  lockDevice();
  double offset = myOffsetAmount;
  double increment = myIncrementAmount;
  unlockDevice();

  myRobot->lock();
  // return true if we could send all the commands
  if (myRobot->comInt(36, -ArMath::roundInt(offset)) &&   // Start angle
      myRobot->comInt(37, ArMath::roundInt(offset)) &&    // End angle
      myRobot->comInt(38, ArMath::roundInt(increment * 100.0)) && // increment
      myRobot->comInt(35, 2)) // Enable sending data, with extended info
    ///@todo only choose extended info if reflector bits desired, also shorten range.
  {
    myRobot->unlock();
    switchState(STATE_CONNECTED);
    madeConnection();
    ArLog::log(ArLog::Terse, "ArSick: Connected to simulated laser.");
    return true;
  }
  else
  {
    switchState(STATE_NONE);
    failedConnect();
    ArLog::log(ArLog::Terse,
	       "ArSick: Failed to connect to simulated laser.");
    return false;
  }
}

/**
   Adds a connect callback, any ArFunctor, (e.g. created as an
   ArFunctorC subclass).  Each connect callback is called when a
   connection is made with the laser.  (Note, if you have some sort of module
   that adds a callback, that module must remove the callback when the
   module is removed or destroyed.)

   @param functor functor to add
   @param position whether to place the functor first or last
   @see remConnectCB()
**/
AREXPORT void ArSick::addConnectCB(ArFunctor *functor,
				    ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myConnectCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myConnectCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArSick::myConnectCallbackList: Invalid position.");
}

/**
    @param functor the functor to remove from the list of connect callbacks
    @see addConnectCB()
**/
AREXPORT void ArSick::remConnectCB(ArFunctor *functor)
{
  myConnectCBList.remove(functor);
}


/** Adds a failed connect callback ArFunctor.
    Each failed connect callbacks is
    invoked when an attempt is made to connect to the laser, but fails.
    The usual reason for this failure is either that there is no
    laser/sim where the connection was tried to be made.  Note, if you have
    some sort of module that adds a callback, that module must remove
    the callback when the module is removed or destroyed.

    @param functor functor to add
    @param position whether to place the functor first or last
    @see remFailedConnectCB()
**/
AREXPORT void ArSick::addFailedConnectCB(ArFunctor *functor,
					  ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myFailedConnectCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myFailedConnectCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArSick::myConnectCallbackList: Invalid position.");
}

/**
    @param functor the functor to remove from the list of connect-failed callbacks
    @see addFailedConnectCB()
**/
AREXPORT void ArSick::remFailedConnectCB(ArFunctor *functor)
{
  myFailedConnectCBList.remove(functor);
}

/** Adds a disconnect normally callback,which is an ArFunctor, created
    as an ArFunctorC.  This whole list of disconnect normally
    callbacks is called when something calls disconnect if the
    instance isConnected.  If there is no connection and disconnect is
    called nothing is done.  If you have some sort of module that adds
    a callback, that module must remove the callback when the module
    is removed.

    @param functor functor created from ArFunctorC which refers to the
    function to call.
    @param position whether to place the functor first or last
    @see remFailedConnectCB
**/
AREXPORT void ArSick::addDisconnectNormallyCB(ArFunctor *functor,
					       ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myDisconnectNormallyCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myDisconnectNormallyCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArSick::myConnectCallbackList: Invalid position.");
}

/**
    @param functor the functor to remove from the list of connect callbacks
    @see addDisconnectNormallyCB
**/
AREXPORT void ArSick::remDisconnectNormallyCB(ArFunctor *functor)
{
  myDisconnectNormallyCBList.remove(functor);
}

/** Adds a disconnect on error callback, which is an ArFunctor,
    created as an ArFunctorC.  This whole list of disconnect on error
    callbacks is called when ARIA loses connection to a laser because
    of an error.  This can occur if the physical connection (ie serial
    cable) between the laser and the computer is severed/disconnected,
    or if the laser is turned off.  Note that if the link between the
    two is lost the ARIA assumes it is temporary until it reaches a
    timeout value set with setConnectionTimeoutTime.  If you have some
    sort of module that adds a callback, that module must remove the
    callback when the module removed.

    @param functor functor created from ArFunctorC which refers to the
    function to call.
    @param position whether to place the functor first or last
    @see remFailedConnectCB
**/
AREXPORT void ArSick::addDisconnectOnErrorCB(ArFunctor *functor,
					      ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myDisconnectOnErrorCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myDisconnectOnErrorCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArSick::myConnectCallbackList: Invalid position");
}

/**
   @param functor the functor to remove from the list of connect callbacks
   @see addDisconnectOnErrorCB
**/
AREXPORT void ArSick::remDisconnectOnErrorCB(ArFunctor *functor)
{
  myDisconnectOnErrorCBList.remove(functor);
}

/**
   Adds a data callback, which is an ArFunctor, created as an
   ArFunctorC.  Whenever a new reading is processed this callback is
   called.  You can then get the raw readings with getRawReadings.

   @param functorfunctor created from ArFunctorC which refers to the
   function to call.
   @param position whether to place the functor first or last
   @see remConnectCB
**/
AREXPORT void ArSick::addDataCB(ArFunctor *functor,
				    ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myDataCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myDataCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse,
	       "ArSick::addDataCB: Invalid position.");
}

/**
    @param functor the functor to remove from the list of data callbacks
    @see addDataCB
**/
AREXPORT void ArSick::remDataCB(ArFunctor *functor)
{
  myDataCBList.remove(functor);
}

/** @internal */
AREXPORT void ArSick::dropConnection(void)
{
  std::list<ArFunctor *>::iterator it;

  if (myState != STATE_CONNECTED)
    return;

  myCurrentBuffer.reset();
  myCumulativeBuffer.reset();
  ArLog::log(ArLog::Terse,
	     "ArSick:  Lost connection to the laser because of error.");
  switchState(STATE_NONE);
  for (it = myDisconnectOnErrorCBList.begin();
       it != myDisconnectOnErrorCBList.end();
       it++)
    (*it)->invoke();
  if (myConn != NULL)
    myConn->close();
}

/** @internal */
AREXPORT void ArSick::failedConnect(void)
{
  std::list<ArFunctor *>::iterator it;

  switchState(STATE_NONE);
  for (it = myFailedConnectCBList.begin();
       it != myFailedConnectCBList.end();
       it++)
    (*it)->invoke();
  if (myConn != NULL)
    myConn->close();
}

/** @internal */
AREXPORT void ArSick::madeConnection(void)
{
  std::list<ArFunctor *>::iterator it;

  myLastReading.setToNow();
  for (it = myConnectCBList.begin(); it != myConnectCBList.end(); it++)
    (*it)->invoke();
}

/**
   Disconnects from the laser.  You should lockDevice the laser before
   calling this function.  Also if you are using the simulator it will
   lock the robot so it can send the command to the simulator, so you
   should make sure the robot is unlocked.

   @param doNotLockRobotForSim if this is true, this will not lock the
   robot if its trying to send a command to the sim... ONLY do this if
   you are calling this from within the robots sync loop (ie from a
   sync task, sensor interp task, or user task)

@return true if it could disconnect from the laser cleanly
**/
AREXPORT bool ArSick::disconnect(bool doNotLockRobotForSim)
{
  std::list<ArFunctor *>::iterator it;
  bool ret;
  ArSerialConnection *conn;

  myStateMutex.lock();
  if (myState == STATE_NONE)
  {
    myStateMutex.unlock();
    return true;
  }

  if (myState != STATE_CONNECTED)
  {
    lockDevice();
    myConnLock.lock();
    myState = STATE_NONE;
    ret = myConn->close();
    myConnLock.unlock();
    unlockDevice();
    ArLog::log(ArLog::Terse, "ArSick: Disconnecting from laser that was not fully connected to...");
    ArLog::log(ArLog::Terse, "this may cause problems later.");
    myStateMutex.unlock();
    return ret;
  }

  myCurrentBuffer.reset();
  myCumulativeBuffer.reset();
  ArLog::log(ArLog::Terse, "ArSick: Disconnecting from laser.");
  myState = STATE_NONE;
  myStateMutex.unlock();
  for (it = myDisconnectNormallyCBList.begin();
       it != myDisconnectNormallyCBList.end();
       it++)
    (*it)->invoke();
  if (myUseSim)
  {
    if (myRobot == NULL)
      return false;
    if (!doNotLockRobotForSim)
      myRobot->lock();
    ret = myRobot->comInt(35, 2); // 2=extendend info request
    ///@todo only choose extended info if reflector bits desired, also shorten range.
    if (!doNotLockRobotForSim)
      myRobot->unlock();
    return ret;
  }
  else
  {
    myConnLock.lock();
    while (mySickPacketReceiver.receivePacket() != NULL);
    myPacket.empty();
    myPacket.uByteToBuf(0x20);
    myPacket.uByteToBuf(0x25);
    myPacket.finalizePacket();
    ret = myConn->write(myPacket.getBuf(), myPacket.getLength());
    // put the thing back to 9600 baud
    ArUtil::sleep(1000);
    myPacket.empty();
    myPacket.byteToBuf(0x20);
    myPacket.byteToBuf(0x42);
    myPacket.finalizePacket();
    if (myConn->write(myPacket.getBuf(), myPacket.getLength()))
    {
      ArUtil::sleep(20);
      if ((conn = dynamic_cast<ArSerialConnection *>(myConn)))
	  conn->setBaud(9600);
    } else
      ret = false;
    ret = ret && myConn->close();
    myConnLock.unlock();
    ArUtil::sleep(300);
    return ret;
  }
}

/**
   Locks this class (using lockDevice()), tries to make a connection, then
   unlocks.  If connecting to the simulator,
   then it will commands to the simulator instead of connecting
   over the configured serial port.

   @note If you have previously locked
   the laser with lockDevice(), then you must unlock with unlockDevice()
   before calling this function.

   @note Since the simulated laser uses the robot connection instead
   of a separate, new connection, this ArSick object @b must have been
   added to the robot using ArRobot::addRangeDevice(), and the robot
   connection @b must be connected and running (e.g. in a background thread
   via ArRobot::runAsync()) for blockingConnect() to be able to successfully
   connect to the simulator.

   @return true if a connection was successfully made, false otherwise
**/

AREXPORT bool ArSick::blockingConnect(void)
{
  int ret;
  // if we're using the sim
  if (myUseSim)
  {
    return internalConnectSim();
  }
  // if we're talking to a real laser
  else
  {
    if (myConn == NULL)
    {
      ArLog::log(ArLog::Terse, "ArSick: Invalid device connection, cannot connect.");
      return false; // Nobody ever set the device connection.
    }

    lockDevice();
    myConnLock.lock();
    switchState(STATE_INIT);
    unlockDevice();
    while (getRunningWithLock() && (ret = internalConnectHandler()) == 0)
      ArUtil::sleep(100);
    myConnLock.unlock();
    if (ret == 1)
      return true;
    else
      return false;
  }
  return false;
}

/**
  This does not lock the laser, but you should lock the
  laser before you try to connect.  Also note that if you are
  connecting to the simulator the laser MUST be unlocked so that this can
  lock the laser and send the commands to the sim.  To be connected
  successfully, either the useSim must be set from configure (and the
  laser must be connected to a simulator, or this will return true but
  connection will fail), the device must have been run() or runAsync(), or
  runOnRobot() used.

  @return true if a connection will be able to be tried, false
  otherwise

  @see configure(), ArRangeDeviceThreaded::run(),
  ArRangeDeviceThreaded::runAsync(), runOnRobot()
**/
AREXPORT bool ArSick::asyncConnect(void)
{
  if (myState == STATE_CONNECTED)
  {
    ArLog::log(ArLog::Terse,"ArSick: already connected to laser.");
    return false;
  }
  if (myUseSim || getRunning() || myRunningOnRobot)
  {
    myStartConnect = true;
    return true;
  }
  else
  {
    ArLog::log(ArLog::Terse, "ArSick: Could not connect, to make an async connection either the sim needs to be used, the device needs to be run or runAsync, or the device needs to be runOnRobot.");
    return false;
  }
}

/**
   This alternate method of operation sets up a sensor interpretation task
   on the robot object, instead of in a background thread.
   Note that the device must have been added to
   the robot already so that the device has a pointer to the robot.
   You should lock the robot and lockDevice() this laser before calling this if
   other things are running already.
**/
AREXPORT bool ArSick::runOnRobot(void)
{
  if (myRobot == NULL)
    return false;
  else
  {
    myRunningOnRobot = true;
    if (getRunning())
      stopRunning();
    return true;
  }
}

/** @internal */
AREXPORT void ArSick::processPacket(ArSickPacket *packet, ArPose pose,
				    ArPose encoderPose,
				    unsigned int counter,
				    bool deinterlace,
				    ArPose deinterlaceDelta)
{
  std::list<ArFunctor *>::iterator it;
  unsigned int rawValue;
  unsigned int value;
  unsigned int reflector = 0;
  unsigned int numReadings;
  unsigned int i;
  double atDeg;
  unsigned int onReading;
  ArSensorReading *reading;
  int dist;
  std::list<ArSensorReading *>::iterator tempIt;
  int multiplier;
  ArTransform transform;
  std::list<double>::iterator ignoreIt;
  bool ignore;

  ArTime arTime;
  arTime = packet->getTimeReceived();
  arTime.addMSec(-13);

  ArTime deinterlaceTime;
  deinterlaceTime = packet->getTimeReceived();
  deinterlaceTime.addMSec(-27);

  //if (packet->getID() != 0xb0)
  //printf("Got in packet of type 0x%x\n", packet->getID());
  if (packet->getID() == 0xb0)
  {
    value = packet->bufToUByte2();
    numReadings = value & 0x3ff;
    //printf("numreadings %d\n", numReadings);
    if (!(value & ArUtil::BIT14) && !(value & ArUtil::BIT15))
      multiplier = 10;
    else if ((value & ArUtil::BIT14) && !(value & ArUtil::BIT15))
      multiplier = 1;
    else if (!(value & ArUtil::BIT14) && (value & ArUtil::BIT15))
      multiplier = 100;
    else
    {
      ArLog::log(ArLog::Terse,
		 "ArSick::processPacket: bad distance configuration in packet\n");
      multiplier = 0;
    }
    //printf("%ld ms after last reading.\n", myLastReading.mSecSince());
    /*printf("Reading number %d, complete %d, unit: %d %d:\n", numReadings,
      !(bool)(value & ArUtil::BIT13), (bool)(value & ArUtil::BIT14),
      (bool)(value & ArUtil::BIT15));*/
    while (myAssembleReadings->size() > numReadings)
    {
      ArLog::log(ArLog::Verbose, "ArSick::processPacket, too many readings, popping one.\n");
      tempIt = myAssembleReadings->begin();
      if (tempIt != myAssembleReadings->end())
	delete (*tempIt);
      myAssembleReadings->pop_front();
    }

    // If we don't have any sensor readings created at all, make 'em all
    if (myAssembleReadings->size() == 0)
      for (i = 0; i < numReadings; i++)
	myAssembleReadings->push_back(new ArSensorReading);

    transform.setTransform(pose);
    //deinterlaceDelta = transform.doInvTransform(deinterlacePose);
    // printf("usePose2 %d, th1 %.0f th2 %.0f\n",  usePose2, pose.getTh(), pose2.getTh());
    for (atDeg = mySensorPose.getTh() - myOffsetAmount, onReading = 0,
	 myIter = myAssembleReadings->begin();
	 (onReading < numReadings &&
	  packet->getReadLength() < packet->getLength() - 4);
	 myWhichReading++, atDeg += myIncrementAmount, myIter++, onReading++)
    {
      reading = (*myIter);
      //reading->resetSensorPosition(0, 0, 0);

      //value = packet->bufToUByte2() & 0x1fff;
      //dist = (value & 0x1fff) * multiplier ;

      rawValue = packet->bufToUByte2();
      if (myBits == BITS_1REFLECTOR)
      {
	dist = (rawValue & 0x7fff) * multiplier;
	reflector = ((rawValue & 0x8000) >> 15) << 2;
      }
      else if (myBits == BITS_2REFLECTOR)
      {
	dist = (rawValue & 0x3fff) * multiplier;
	reflector = ((rawValue & 0xc000) >> 14) << 1 ;
      }
      else if (myBits == BITS_3REFLECTOR)
      {
	dist = (rawValue & 0x1fff) * multiplier;
	reflector = ((rawValue & 0xe000) >> 13);
      }
      // just trap for if we don't know what it is, this shouldn't
      // happen though
      else
      {
	dist = (rawValue & 0x7fff) * multiplier;
	reflector = 0;
      }
      // there are 3 reflector bits (its already been normalized above
      // to that range) so now we need to shift it another 5 so we get
      // 0-255.
      reflector = reflector << 5;

      ignore = false;
      for (ignoreIt = myIgnoreReadings.begin();
	   ignoreIt != myIgnoreReadings.end();
	   ignoreIt++)
      {
	if (ArMath::fabs(ArMath::subAngle(atDeg, *(ignoreIt))) < 1.0)
	{
	  ignore = true;
	  break;
	}
      }
      if (myMinRange != 0 && dist < (int)myMinRange)
	ignore = true;
      if (myMaxRange != 0 && dist > (int)myMaxRange)
	ignore = true;
      if (deinterlace && (onReading % 2) == 0)
      {
	reading->resetSensorPosition(
	       ArMath::roundInt(mySensorPose.getX() + deinterlaceDelta.getX()),
	       ArMath::roundInt(mySensorPose.getY() + deinterlaceDelta.getY()),
	       ArMath::addAngle(atDeg, deinterlaceDelta.getTh()));
	reading->newData(dist, pose, encoderPose, transform, counter,
			 deinterlaceTime, ignore, reflector);
      }
      else
      {
	reading->resetSensorPosition(ArMath::roundInt(mySensorPose.getX()),
				     ArMath::roundInt(mySensorPose.getY()),
				     atDeg);
	reading->newData(dist, pose, encoderPose, transform, counter,
			 arTime, ignore, reflector);
      }
      /*
      reading->newData(onReading, 0, 0, 0,
		       ArTransform(), counter,
		       packet->getTimeReceived());
      */
      tempIt = myIter;
      tempIt++;
      if (tempIt == myAssembleReadings->end() &&
	  onReading + 1 != numReadings)
      {
	myAssembleReadings->push_back(new ArSensorReading);
      }
    }
    // set ArRangeDevice buffer, switch internal buffers
    myRawReadings = myAssembleReadings;
    //printf("Readings? 0x%x\n", myRawReadings);
    myAssembleReadings = myCurrentReadings;
    myCurrentReadings = myRawReadings;
    //printf("\n");
    myLastReading.setToNow();
    filterReadings();

    if (myTimeLastSickPacket != time(NULL))
    {
      myTimeLastSickPacket = time(NULL);
      mySickPacCount = mySickPacCurrentCount;
      mySickPacCurrentCount = 0;
    }
    mySickPacCurrentCount++;
    for (it = myDataCBList.begin(); it != myDataCBList.end(); it++)
      (*it)->invoke();
  }
}

/** @internal */
AREXPORT void ArSick::runOnce(bool lockRobot)
{
  ArSickPacket *packet;
  unsigned int counter=0;
  int ret;
  ArTime time;
  ArTime time2;
  ArPose pose;
  ArPose pose2;
  ArPose encoderPose;

  if (myProcessImmediately && myRobot != NULL)
  {
    if (lockRobot)
      myRobot->lock();
    pose = myRobot->getPose();
    counter = myRobot->getCounter();
    if (lockRobot)
      myRobot->unlock();
  }

  lockDevice();
  if (myState == STATE_CONNECTED && myTimeoutTime > 0 &&
      myLastReading.mSecSince() > myTimeoutTime * 1000)
  {
    dropConnection();
    unlockDevice();
    return;
  }
  if (myUseSim)
  {
    unlockDevice();
    return;
  }
  if (myState == STATE_CONNECTED)
  {
    unlockDevice();
    myConnLock.lock();
    packet = mySickPacketReceiver.receivePacket();
    myConnLock.unlock();
    lockDevice();
    // if we're attached to a robot and have a packet
    if (myRobot != NULL && packet != NULL && !myProcessImmediately)
    {
      myPackets.push_back(packet);
    }
    else if (myRobot != NULL && packet != NULL && myProcessImmediately)
    {
      unlockDevice();
      if (lockRobot && myInterpolation)
	myRobot->lock();
      // try to get the interpolated position, if we can't, just use
      // the robot's pose
      if (myInterpolation && (ret = myRobot->getPoseInterpPosition(
	      packet->getTimeReceived(), &pose)) < 0)
	pose = myRobot->getPose();
      // try to get the interpolated encoder position, if we can't,
      // just fake it from the robot's pose and the encoder transform
      if (myInterpolation && (ret = myRobot->getEncoderPoseInterpPosition(
	      packet->getTimeReceived(), &encoderPose)) < 0)
	encoderPose = myRobot->getEncoderTransform().doInvTransform(pose);
      if (lockRobot && myInterpolation)
	myRobot->unlock();
      lockDevice();
      processPacket(packet, pose, encoderPose, counter, false, ArPose());
    }
    else if (packet != NULL) // if there's no robot
    {
      processPacket(packet, pose, encoderPose, 0, false, ArPose());
      delete packet;
    }
  }
  unlockDevice();
  return;

}

AREXPORT int ArSick::getSickPacCount()
{
  if (myTimeLastSickPacket == time(NULL))
    return mySickPacCount;
  if (myTimeLastSickPacket == time(NULL) - 1)
    return mySickPacCurrentCount;
  return 0;
}

/**
   When readings are put into the current buffer they are compared
   against the last reading and must be at least this distance away
   from the last reading.  If this value is 0 then there is no
   filtering of this kind.
**/
AREXPORT void ArSick::setFilterNearDist(double dist)
{
  if (dist >= 0)
    myFilterNearDist = dist;
  else
    ArLog::log(ArLog::Terse, "ArSick::setFilterNearDist given a distance less than 0.\n");

}

/**
   When readings are put into the current buffer they are compared
   against the last reading and must be at least this distance away
   from the last reading.  If this value is 0 then there is no
   filtering of this kind.
**/
AREXPORT double ArSick::getFilterNearDist(void)
{
  return myFilterNearDist;
}


/**
   When readings are put into the cumulative buffer they must be
   within this distance of the robot.  If this value is 0 then there
   is no filtering of this kind.
 **/
AREXPORT void ArSick::setFilterCumulativeInsertMaxDist(double dist)
{
  if (dist >= 0)
  {
    myFilterCumulativeInsertMaxDist = dist;
    myFilterSquaredCumulativeInsertMaxDist = dist * dist;
  }
  else
    ArLog::log(ArLog::Terse, "ArSick::setFilterCumulativeMaxDistDist given a distance less than 0.\n");
}

/**
   When readings are put into the cumulative buffer they must be
   within this distance of the robot.  If this value is 0 then there
   is no filtering of this kind.
 **/
AREXPORT double ArSick::getFilterCumulativeInsertMaxDist(void)
{
  return myFilterCumulativeInsertMaxDist;
}

/**
   When readings are put into the cumulative buffer they must be this
   far from all the other cumulative readings or they aren't put in.
   If this value is 0 then there is no filtering of this kind.
 **/
AREXPORT void ArSick::setFilterCumulativeNearDist(double dist)
{
  if (dist >= 0)
  {
    myFilterCumulativeNearDist = dist;
    myFilterSquaredCumulativeNearDist = dist * dist;
  }
  else
    ArLog::log(ArLog::Terse, "ArSick::setFilterCumulativeNearDistDist given a distance less than 0.\n");
}

/**
   When readings are put into the cumulative buffer they must be this
   far from all the other cumulative readings or they aren't put in.
   If this value is 0 then there is no filtering of this kind.
 **/
AREXPORT double ArSick::getFilterCumulativeNearDist(void)
{
  return myFilterCumulativeNearDist;
}

/**
   When the readings are cleaned the current readings are compared
   against the cumulative readings... if a beam of a current reading
   comes within this distance of a cumulative reading that cumulative
   reading is removed.  If this value is 0 then there is no filtering
   of this kind.
 **/
AREXPORT void ArSick::setFilterCumulativeCleanDist(double dist)
{
  if (dist >= 0)
  {
    myFilterCumulativeCleanDist = dist;
    myFilterSquaredCumulativeCleanDist = dist * dist;
  }
  else
    ArLog::log(ArLog::Terse, "ArSick::setFilterCumulativeCleanDistDist given a distance less than 0.\n");
}

/**
   When the readings are cleaned the current readings are compared
   against the cumulative readings... if a beam of a current reading
   comes within this distance of a cumulative reading that cumulative
   reading is removed.  If this value is 0 then there is no filtering
   of this kind.
 **/
AREXPORT double ArSick::getFilterCumulativeCleanDist(void)
{
  return myFilterCumulativeCleanDist;
}

/**
   The cumulative readings are check against the current buffer every
   this number of milliseconds... if 0 its cleaned every time there
   are readings.
 **/
AREXPORT void ArSick::setFilterCleanCumulativeInterval(int milliSeconds)
{
  if (milliSeconds >= 0)
  {
    myFilterCleanCumulativeInterval = milliSeconds;
  }
  else
    ArLog::log(ArLog::Terse, "ArSick::setFilterCleanCumulativeInterval given a time less than 0.\n");
}

/**
   The cumulative readings are check against the current buffer every
   this number of milliseconds... if 0 its cleaned every time there
   are readings.
 **/
AREXPORT int ArSick::getFilterCleanCumulativeInterval(void)
{
  return myFilterCleanCumulativeInterval;
}



/** @internal */
AREXPORT void ArSick::sensorInterpCallback(void)
{
  std::list<ArSickPacket *>::iterator it;
  std::list<ArSickPacket *> processed;
  ArSickPacket *packet;
  ArTime time;
  ArPose pose;
  int ret;
  int retEncoder=0;
  ArPose encoderPose;
  ArPose deinterlaceEncoderPose;
  bool deinterlace;
  ArTime deinterlaceTime;
  ArPose deinterlaceDelta;

  if (myRunningOnRobot)
    runOnce(false);

  lockDevice();

  if (myIncrement == INCREMENT_HALF)
    adjustRawReadings(true);
  else
    adjustRawReadings(false);

  for (it = myPackets.begin(); it != myPackets.end(); it++)
  {
    packet = (*it);
    time = packet->getTimeReceived();
    time.addMSec(-13);
    if ((ret = myRobot->getPoseInterpPosition(time, &pose)) == 1 &&
	(retEncoder =
	 myRobot->getEncoderPoseInterpPosition(time, &encoderPose)) == 1)
    {
      deinterlaceTime = packet->getTimeReceived();
      deinterlaceTime.addMSec(-27);

      if (myIncrement == INCREMENT_HALF &&
	  (myRobot->getEncoderPoseInterpPosition(
		  deinterlaceTime, &deinterlaceEncoderPose)) == 1)
	deinterlace = true;
      else
	deinterlace = false;

      ArTransform deltaTransform;
      deltaTransform.setTransform(encoderPose);
      deinterlaceDelta = deltaTransform.doInvTransform(deinterlaceEncoderPose);

      processPacket(packet, pose, encoderPose, myRobot->getCounter(),
		    deinterlace, deinterlaceDelta);
      processed.push_back(packet);
    }
    else if (ret < -1 || retEncoder < -1)
    {
      if (myRobot->isConnected())
	ArLog::log(ArLog::Normal, "ArSick::processPacket: too old to process\n");
      else
      {
	processPacket(packet, pose, encoderPose, myRobot->getCounter(), false,
		      ArPose());
      }
      processed.push_back(packet);
    }
    else
    {
      //ArLog::log(ArLog::Terse, "ArSick::processPacket: error %d from interpolation\n", ret);
      //printf("$$$ ret = %d\n", ret);
    }
  }
  while ((it = processed.begin()) != processed.end())
  {
    packet = (*it);
    myPackets.remove(packet);
    processed.pop_front();
    delete packet;
  }
  unlockDevice();
}

/** @internal */
AREXPORT void *ArSick::runThread(void *arg)
{
  while (getRunningWithLock())
  {
    lockDevice();
    if (myStartConnect)
    {
      myStartConnect = false;
      switchState(STATE_INIT);
      if (myUseSim)
      {
	unlockDevice();
	internalConnectSim();
      }
      else
      {
	unlockDevice();
	while (getRunningWithLock())
	{
	  lockDevice();
	  myConnLock.lock();
	  if (internalConnectHandler() != 0)
	  {
	    myConnLock.unlock();
	    unlockDevice();
	    break;
	  }
	  myConnLock.unlock();
	  unlockDevice();
	  ArUtil::sleep(1);
	}
      }
    } else
      unlockDevice();
    runOnce(true);
    ArUtil::sleep(1);
  }
  lockDevice();
  if (isConnected())
  {
    disconnect();
  }
  unlockDevice();

  return NULL;
}

/**
    Applies a transform to the buffers. this is mostly useful for translating
    to/from local/global coordinates, but may have other uses.
    This is different from
    the class because it also transforms the raw readings.
    @param trans the transform to apply to the data
    @param doCumulative whether to transform the cumulative buffer or not
*/
AREXPORT void ArSick::applyTransform(ArTransform trans,
                                                bool doCumulative)
{
  myCurrentBuffer.applyTransform(trans);
  std::list<ArSensorReading *>::iterator it;

  for (it = myRawReadings->begin(); it != myRawReadings->end(); ++it)
    (*it)->applyTransform(trans);

  if (doCumulative)
    myCumulativeBuffer.applyTransform(trans);
}
