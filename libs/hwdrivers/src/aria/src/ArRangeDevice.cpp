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
#include "ArRangeDevice.h"
#include "ArRobot.h"

/**
   @param currentBufferSize number of readings to store in the current
   buffer

   @param cumulativeBufferSize number of readings to store in the
   cumulative buffer

   @param name the name of this device

   @param maxRange the maximum range of this device. If the device
   can't find a reading in a specified section, it returns this
   maxRange

   @param maxSecondsToKeepCurrent this is the number of seconds to
   keep current readings in the current buffer. If less than 0, then
   readings are not automatically removed based on time (but can be
   replaced or removed for other reasons).

   @param maxSecondsToKeepCumulative this is the number of seconds to
   keep cumulative readings in the cumulative buffer. If less than 0
   then readings are not automatically based on time (but can be
   replaced or removed for other reasons).
   
   @param maxDistToKeepCumulative if cumulative readings are further
   than this distance from the current robot pose, then they are
   removed. If this is less than 0 they are not removed because of
   this

   @param locationDependent if the data in this range device is
   dependent on the robot's location or not...  For instance, a laser
   would not be dependent on location, because it'll be correct in a
   relative manner, whereas forbidden lines are dependent on location,
   because if the robot isn't where it thinks it is then the forbidden
   lines will be avoided in the wrong spots

**/
AREXPORT ArRangeDevice::ArRangeDevice(size_t currentBufferSize,
				      size_t cumulativeBufferSize, 
				      const char *name, 
				      unsigned int maxRange,
				      int maxSecondsToKeepCurrent, 
				      int maxSecondsToKeepCumulative,
				      double maxDistToKeepCumulative,
				      bool locationDependent) :
  myCurrentBuffer(currentBufferSize),
  myCumulativeBuffer(cumulativeBufferSize),
  myFilterCB(this, &ArRangeDevice::filterCallback)
{
  myRobot = NULL;
  myName = name;
  myMaxRange = maxRange;
  myRawReadings = NULL;
  myAdjustedRawReadings = NULL;
  setMaxSecondsToKeepCurrent(maxSecondsToKeepCurrent);
  setMaxSecondsToKeepCumulative(maxSecondsToKeepCumulative);
  setMaxDistToKeepCumulative(maxDistToKeepCumulative);
  myCurrentDrawingData = NULL;
  myOwnCurrentDrawingData = false;
  myCumulativeDrawingData = NULL;
  myOwnCumulativeDrawingData = false;
  myIsLocationDependent = locationDependent;
}

AREXPORT ArRangeDevice::~ArRangeDevice()
{
  if (myRobot != NULL)
    myRobot->remSensorInterpTask(&myFilterCB);
  if (myCurrentDrawingData != NULL && myOwnCurrentDrawingData)
  {
    delete myCurrentDrawingData;
    myCurrentDrawingData = NULL;
    myOwnCurrentDrawingData = false;
  }
  if (myCumulativeDrawingData != NULL && myOwnCumulativeDrawingData)
  {
    delete myCumulativeDrawingData;
    myCumulativeDrawingData = NULL;
    myOwnCumulativeDrawingData = false;
  }
}


AREXPORT const char * ArRangeDevice::getName(void) const
{ 
  return myName.c_str(); 
}

AREXPORT void ArRangeDevice::setRobot(ArRobot *robot) 
{ 
  char buf[512];
  sprintf(buf, "filter %s", getName());

  if (myRobot != NULL)
    myRobot->remSensorInterpTask(&myFilterCB);

  myRobot = robot;

  if (myRobot != NULL)
    myRobot->addSensorInterpTask(buf, 100, &myFilterCB);
}

AREXPORT ArRobot *ArRangeDevice::getRobot(void) 
{
  return myRobot; 
}

AREXPORT void ArRangeDevice::filterCallback(void)
{
  std::list<ArPoseWithTime *>::iterator it;

  lockDevice();
  // first filter the current readings based on time
  if (myMaxSecondsToKeepCurrent > 0 && 
      myCurrentBuffer.getSize() > 0)
  {
    // just walk through and make sure nothings too far away
    myCurrentBuffer.beginInvalidationSweep();
    for (it = getCurrentBuffer()->begin(); 
	 it != getCurrentBuffer()->end(); 
	 ++it)
    {
      if ((*it)->getTime().secSince() >= myMaxSecondsToKeepCurrent)
	myCurrentBuffer.invalidateReading(it);
    }
    myCurrentBuffer.endInvalidationSweep();
  }
  
  // okay done with current, now do the cumulative
  bool doingDist = true;
  bool doingAge = true;

  if (myCumulativeBuffer.getSize() == 0)
  {
    unlockDevice();
    return;
  }

  double squaredFarDist = (myMaxDistToKeepCumulative * 
			   myMaxDistToKeepCumulative);

  if (squaredFarDist < 1)
    doingDist = false;
  if (myMaxSecondsToKeepCumulative <= 0)
    doingAge = false;
		    
  if (!doingDist && !doingAge)
  {
    unlockDevice();
    return;
  }

  // just walk through and make sure nothings too far away
  myCumulativeBuffer.beginInvalidationSweep();
  for (it = getCumulativeBuffer()->begin(); 
       it != getCumulativeBuffer()->end(); 
       ++it)
  {
    // if its closer to a reading than the filter near dist, just return
    if (doingDist && 
	myRobot->getPose().squaredFindDistanceTo(*(*it)) > squaredFarDist)
      myCumulativeBuffer.invalidateReading(it);
    else if (doingAge && 
	     (*it)->getTime().secSince() >= myMaxSecondsToKeepCumulative)
      myCumulativeBuffer.invalidateReading(it);
  }
  myCumulativeBuffer.endInvalidationSweep();
  unlockDevice();
}

/**
   If the @a size is smaller than the current buffer size, then 
   the oldest readings are discarded, leaving only @a size
   newest readings. If @a size is larger than the current size,
   then the buffer size will be allowed to grow to that size as new readings
   are added.
   @param size number of readings to set the buffer's maximum size to
*/
AREXPORT void ArRangeDevice::setCurrentBufferSize(size_t size)
{
  myCurrentBuffer.setSize(size);
}

/**
   If the @a size is smaller than the cumulative buffer size, then 
   the oldest readings are discarded, leaving only @a size
   newest readings. If @a size is larger than the cumulative buffer size,
   then the buffer size will be allowed to grow to that size as new readings
   are added.
   @param size number of readings to set the buffer to
*/
AREXPORT void ArRangeDevice::setCumulativeBufferSize(size_t size)
{
  myCumulativeBuffer.setSize(size);
}

AREXPORT void ArRangeDevice::addReading(double x, double y)
{
  myCurrentBuffer.addReading(x, y);
  myCumulativeBuffer.addReading(x, y);
}

/**
 * The closest reading within a polar region or "slice" defined by the given
 * angle range is returned.  Optionally, the specific angle of the found may be
 * placed in @a angle, if not NULL.
 * The region searched is the region between @a startAngle, sweeping
 * counter-clockwise to @a endAngle (0 is straight ahead of the device,
 * -90 to the right, 90 to the left).  Note that therefore there is a difference between
 *  e.g. the regions (0, 10) and (10, 0).  (0, 10) is a 10-degree span near the front 
 *  of the device, while (10, 0) is a 350 degree span covering the sides and
 *  rear.  Similarly, (-60, -30) covers 30 degrees on the right hand side, while
 *  (-30, -60) covers 330 degrees.   (-90, 90) is 180 degrees in front. (-180,
 *  180) covers all sides of the robot.
 *  In other words, if you want the smallest
 *  section between the two angles, ensure that @a startAngle < @a endAngle.
 *
   @param startAngle where to start the slice
   @param endAngle where to end the slice, going counterclockwise from startAngle
   @param angle if given, a pointer to a value in which to put the specific angle to the found reading
   @return the range to the obstacle (a value >= the maximum range indicates that no reading was detected in the specified region)

  Example:
   @image html figures/ArRangeDevice_currentReadingPolar.png This figure illustrates an example range device and the meanings of arguments and return value.
*/
AREXPORT double ArRangeDevice::currentReadingPolar(double startAngle,
						   double endAngle,
						   double *angle) const
{
  ArPose pose;
  if (myRobot != NULL)
    pose = myRobot->getPose();
  else
    {
      ArLog::log(ArLog::Normal, "ArRangeDevice %s: NULL robot, won't get polar reading correctly", getName());
      pose.setPose(0, 0);
    }
  return myCurrentBuffer.getClosestPolar(startAngle, endAngle, 
					 pose,
					 myMaxRange,
					 angle);
}

/**
 * The closest reading in this range device's cumulative buffer
 * within a polar region or "slice" defined by the given
 * angle range is returned.  Optionally return the specific angle of the found reading as
 * well. The region searched is the region between a starting angle, sweeping
 * counter-clockwise to the ending angle (0 is straight ahead of the device,
 * -90 to the right, 90 to the left).  Note that there is a difference between
 *  the region (0, 10) and (10, 0).  (0, 10) is a 10-degree span near the front 
 *  of the device, while (10, 0) is a 350 degree span covering the sides and
 *  rear.  Similarly, (-60, -30) covers 30 degrees on the right hand side, while
 *  (-30, -60) covers 330 degrees.
 *  In other words, if you want the smallest
 *  section between the two angles, ensure than startAngle < endAngle.
   @param startAngle where to start the slice
   @param endAngle where to end the slice, going counterclockwise from startAngle
   @param angle if given, a pointer to a value in which to put the specific angle to the found reading
   @return the range to the obstacle (a value >= the maximum range indicates that no reading was detected in the specified region)

  Example:
   @image html figures/ArRangeDevice_currentReadingPolar.png This figure illustrates an example range device and the meanings of arguments and return value.
*/
AREXPORT double ArRangeDevice::cumulativeReadingPolar(double startAngle,
						      double endAngle,
						      double *angle) const
{
  ArPose pose;
  if (myRobot != NULL)
    pose = myRobot->getPose();
  else
    {
      ArLog::log(ArLog::Normal, "ArRangeDevice %s: NULL robot, won't get polar reading correctly", getName());
      pose.setPose(0, 0);
    }
  return myCumulativeBuffer.getClosestPolar(startAngle, endAngle, 
					    pose,
					    myMaxRange,
					    angle);
}

/**
   Get the closest reading in the current buffer within a rectangular region
   defined by two points (opposite corners of the rectangle).
   @param x1 the x coordinate of one of the rectangle points
   @param y1 the y coordinate of one of the rectangle points
   @param x2 the x coordinate of the other rectangle point
   @param y2 the y coordinate of the other rectangle point
   @param pose a pointer to an ArPose object in which to store the location of
   the closest position
   @return The range to the reading from the device, or a value >= maxRange if
   no reading was found in the box.
*/
AREXPORT double ArRangeDevice::currentReadingBox(double x1, double y1, 
						 double x2, double y2,
						 ArPose *pose) const
{
  ArPose robotPose;
  if (myRobot != NULL)
      robotPose = myRobot->getPose();
  else
    {
      ArLog::log(ArLog::Normal, "ArRangeDevice %s: NULL robot, won't get reading box correctly", getName());
      robotPose.setPose(0, 0);
    }
  return myCurrentBuffer.getClosestBox(x1, y1, x2, y2, robotPose,
				       myMaxRange, pose);
}

/**
   Get the closest reading in the cumulative buffer within a rectangular region 
   around the range device, defined by two points (opposeite points
   of a rectangle).
   @param x1 the x coordinate of one of the rectangle points
   @param y1 the y coordinate of one of the rectangle points
   @param x2 the x coordinate of the other rectangle point
   @param y2 the y coordinate of the other rectangle point
   @param pose a pointer to an ArPose object in which to store the location of
   the closest position
   @return The range to the reading from the device, or a value >= maxRange if
   no reading was found in the box.
*/
AREXPORT double ArRangeDevice::cumulativeReadingBox(double x1, double y1, 
						 double x2, double y2,
						 ArPose *pose) const
{
  ArPose robotPose;
  if (myRobot != NULL)
    robotPose = myRobot->getPose();
  else
    {
      ArLog::log(ArLog::Normal, "ArRangeDevice %s: NULL robot, won't get reading box correctly", getName());
      robotPose.setPose(0, 0);
    }
  return myCumulativeBuffer.getClosestBox(x1, y1, x2, y2, 
					  robotPose,
					  myMaxRange, pose);
}

/** 
    Applies a coordinate transformation to some or all buffers. 
    This is mostly useful for translating
    to/from local/global coordinate systems, but may have other uses.
    @param trans the transform to apply to the data
    @param doCumulative whether to transform the cumulative buffer or not
*/    
AREXPORT void ArRangeDevice::applyTransform(ArTransform trans, 
					    bool doCumulative)
{
  myCurrentBuffer.applyTransform(trans);
  if (doCumulative)
    myCumulativeBuffer.applyTransform(trans);
}

/** Copies the list into a vector.
 *  @swignote The return type will be named ArSensorReadingVector instead
 *    of the std::vector template type.
 */
AREXPORT std::vector<ArSensorReading> *ArRangeDevice::getRawReadingsAsVector(void)
{
  
  std::list<ArSensorReading *>::const_iterator it;
  myRawReadingsVector.clear();
  // if we don't have any return an empty list
  if (myRawReadings == NULL)
    return &myRawReadingsVector;
  myRawReadingsVector.reserve(myRawReadings->size());
  for (it = myRawReadings->begin(); it != myRawReadings->end(); it++)
    myRawReadingsVector.insert(myRawReadingsVector.begin(), *(*it));
  return &myRawReadingsVector;
}

/** Copies the list into a vector.
 *  @swignote The return type will be named ArSensorReadingVector instead
 *    of the std::vector template type.
 */
AREXPORT std::vector<ArSensorReading> *ArRangeDevice::getAdjustedRawReadingsAsVector(void)
{
  
  std::list<ArSensorReading *>::const_iterator it;
  myAdjustedRawReadingsVector.clear();
  // if we don't have any return an empty list
  if (myAdjustedRawReadings == NULL)
    return &myRawReadingsVector;
  myAdjustedRawReadingsVector.reserve(myRawReadings->size());
  for (it = myAdjustedRawReadings->begin(); 
       it != myAdjustedRawReadings->end(); 
       it++)
    myAdjustedRawReadingsVector.insert(myAdjustedRawReadingsVector.begin(), 
				       *(*it));
  return &myAdjustedRawReadingsVector;
}


AREXPORT void ArRangeDevice::setCurrentDrawingData(ArDrawingData *data, 
						   bool takeOwnershipOfData)
{
  if (myCurrentDrawingData != NULL && myOwnCurrentDrawingData)
  {
    delete myCurrentDrawingData;
    myCurrentDrawingData = NULL;
    myOwnCurrentDrawingData = false;
  }
  myCurrentDrawingData = data; 
  myOwnCurrentDrawingData = takeOwnershipOfData; 
}

AREXPORT  void ArRangeDevice::setCumulativeDrawingData(ArDrawingData *data, 
						      bool takeOwnershipOfData)
{
  if (myCumulativeDrawingData != NULL && myOwnCumulativeDrawingData)
  {
    delete myCumulativeDrawingData;
    myCumulativeDrawingData = NULL;
    myOwnCumulativeDrawingData = false;
  }
  myCumulativeDrawingData = data; 
  myOwnCumulativeDrawingData = takeOwnershipOfData; 
}

AREXPORT void ArRangeDevice::adjustRawReadings(bool interlaced)
{
  std::list<ArSensorReading *>::iterator rawIt;

  // make sure we have raw readings and a robot, and a delay to
  // correct for (note that if we don't have a delay to correct for
  // but have already been adjusting (ie someone changed the delay)
  // we'll just keep adjusting)
  if (myRawReadings == NULL || myRobot == NULL || 
      (myAdjustedRawReadings == NULL && myRobot->getOdometryDelay() == 0))
    return;
  

  // if we don't already have a list then make one
  if (myAdjustedRawReadings == NULL)
    myAdjustedRawReadings = new std::list<ArSensorReading *>;
  
  // if we've already adjusted these readings then don't do it again
  if (myRawReadings->begin() != myRawReadings->end() &&
      myRawReadings->front()->getAdjusted())
    return;

  std::list<ArSensorReading *>::iterator adjIt;
  ArSensorReading *adjReading;
  ArSensorReading *rawReading;

  ArTransform trans;
  ArTransform encTrans;
  ArTransform interlacedTrans;
  ArTransform interlacedEncTrans;

  bool first = true;
  bool second = true;

  int onReading;
  for (rawIt = myRawReadings->begin(), adjIt = myAdjustedRawReadings->begin(), 
       onReading = 0; 
       rawIt != myRawReadings->end(); 
       rawIt++, onReading++)
  {
    rawReading = (*rawIt);
    if (adjIt != myAdjustedRawReadings->end())
    {
      adjReading = (*adjIt);
      adjIt++;
    }
    else
    {
      adjReading = new ArSensorReading;
      myAdjustedRawReadings->push_back(adjReading);
    }
    (*adjReading) = (*rawReading);
    if (first || (interlaced && second))
    {
      ArPose origPose;
      ArPose corPose;
      ArPose origEncPose;
      ArPose corEncPose;
      ArTime corTime;


      corTime = rawReading->getTimeTaken();
      corTime.addMSec(-myRobot->getOdometryDelay());
      if (myRobot->getPoseInterpPosition(corTime, 
					 &corPose) == 1 && 
	  myRobot->getEncoderPoseInterpPosition(corTime, 
						&corEncPose) == 1)
      {
	origPose = rawReading->getPoseTaken();
	origEncPose = rawReading->getEncoderPoseTaken();
	/*
	printf("Difference was %g %g %g (rotVel %.0f, rotvel/40 %g)\n", 
	       origEncPose.getX() - corEncPose.getX(),
	       origEncPose.getY() - corEncPose.getY(),
	       origEncPose.getTh() - corEncPose.getTh(),
	       myRobot->getRotVel(), myRobot->getRotVel() / 40);
	*/
	if (first)
	{
	  trans.setTransform(origPose, corPose);
	  encTrans.setTransform(origEncPose, corEncPose);
	}
	else if (interlaced && second)
	{
	  interlacedTrans.setTransform(origPose, corPose);
	  interlacedEncTrans.setTransform(origEncPose, corEncPose);
	}
      }
      else
      {
	//printf("Couldn't correct\n");
      }

      if (first)
	first = false;
      else if (interlaced && second)
	second = false;

    }
    if (!interlaced && (onReading % 2) == 0)
    {
      adjReading->applyTransform(trans);
      adjReading->applyEncoderTransform(encTrans);
    }
    else
    {
      adjReading->applyTransform(interlacedTrans);
      adjReading->applyEncoderTransform(interlacedEncTrans);
    }
    /*
    if (fabs(adjReading->getEncoderPoseTaken().getX() - 
	     corEncPose.getX()) > 1 ||
	fabs(adjReading->getEncoderPoseTaken().getY() - 
	     corEncPose.getY()) > 1 || 
	fabs(ArMath::subAngle(adjReading->getEncoderPoseTaken().getTh(), 
			      corEncPose.getTh())) > .2)
      printf("(%.0f %.0f %.0f) should be (%.0f %.0f %.0f)\n", 
	     adjReading->getEncoderPoseTaken().getX(),
	     adjReading->getEncoderPoseTaken().getY(),
	     adjReading->getEncoderPoseTaken().getTh(),
	     corEncPose.getX(), corEncPose.getY(),  corEncPose.getTh());
    */
    adjReading->setAdjusted(true);
    rawReading->setAdjusted(true);
  }  
}
