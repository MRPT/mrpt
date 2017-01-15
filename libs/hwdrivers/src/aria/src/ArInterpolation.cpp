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
#include "ArInterpolation.h"

AREXPORT ArInterpolation::ArInterpolation(size_t numberOfReadings)
{
  mySize = numberOfReadings;

}

AREXPORT ArInterpolation::~ArInterpolation()
{

}

AREXPORT bool ArInterpolation::addReading(ArTime timeOfReading, 
					  ArPose position)
{
  if (myTimes.size() >= mySize)
  {
    myTimes.pop_back();
    myPoses.pop_back();
  }
  myTimes.push_front(timeOfReading);
  myPoses.push_front(position);
  return true;
}

/**
   @param timeStamp the time we are interested in
   @param position the pose to set to the given position
   @return 1 its good interpolation, 0 its predicting, -1 its too far to 
   predict, -2 its too old, -3 there's not enough data to predict
   
**/

AREXPORT int ArInterpolation::getPose(ArTime timeStamp,
					  ArPose *position)
{
  std::list<ArTime>::iterator tit;
  std::list<ArPose>::iterator pit;
  
  ArPose thisPose;
  ArTime thisTime;
  ArPose lastPose;
  ArTime lastTime;

  ArTime nowTime;
  long total;
  long toStamp;
  double percentage;
  ArPose retPose;
  
  // find the time we want
  for (tit = myTimes.begin(), pit = myPoses.begin();
       tit != myTimes.end() && pit != myPoses.end(); 
       ++tit, ++pit)
  {
    lastTime = thisTime;
    lastPose = thisPose;

    thisTime = (*tit);
    thisPose = (*pit);

    //printf("## %d %d %d b %d at %d after %d\n", timeStamp.getMSec(), thisTime.getMSec(), timeStamp.mSecSince(thisTime), timeStamp.isBefore(thisTime), timeStamp.isAt(thisTime), timeStamp.isAfter(thisTime));
    //if (timeStamp.isBefore(thisTime) || timeStamp.isAt(thisTime))
    if (!timeStamp.isAfter(thisTime))
    {
      //printf("Found one!\n");
      break;
    } 

  }
  // if we're at the end then it was too long ago
  if (tit == myTimes.end() || pit == myPoses.end())
  {
    //printf("Too old\n");
    return -2;
  }
  
  // this is for forecasting (for the brave)
  if ((tit == myTimes.begin() || pit == myPoses.begin()) && 
      !timeStamp.isAt((*tit)))
  {
    //printf("Too new %d %d\n", tit == myTimes.begin(), pit == myPoses.begin());
  
    thisTime = (*tit);
    thisPose = (*pit);
    tit++;
    pit++;  
    if (tit == myTimes.end() || pit == myPoses.end())
    {
      //printf("Not enough data\n");
      return -3;
    }
    lastTime = (*tit);
    lastPose = (*pit);
    nowTime.setToNow();
    total = thisTime.mSecSince(lastTime);
    if (total == 0)
      total = 100;
    toStamp = nowTime.mSecSince(thisTime);
    percentage = (double)toStamp/(double)total;
    //printf("Total time %d, to stamp %d, percentage %.2f\n", total, toStamp, percentage);
    if (percentage > 3)
      return -1;

    retPose.setX(thisPose.getX() + 
		 (thisPose.getX() - lastPose.getX()) * percentage);
    retPose.setY(thisPose.getY() + 
		 (thisPose.getY() - lastPose.getY()) * percentage);
    retPose.setTh(ArMath::addAngle(thisPose.getTh(),
				   ArMath::subAngle(thisPose.getTh(),
						    lastPose.getTh())
				   * percentage));
    *position = retPose;
    return 0;
  }

  // this is the actual interpolation

  //printf("Woo hoo!\n");

  total = thisTime.mSecSince(lastTime);
  toStamp = thisTime.mSecSince(timeStamp);
  percentage = (double)toStamp/(double)total;
  //printf("Total time %d, to stamp %d, percentage %.2f\n", 	 total, toStamp, percentage);
  retPose.setX(thisPose.getX() + 
	      (lastPose.getX() - thisPose.getX()) * percentage); 
  retPose.setY(thisPose.getY() + 
	      (lastPose.getY() - thisPose.getY()) * percentage); 
  retPose.setTh(ArMath::addAngle(thisPose.getTh(),
				ArMath::subAngle(lastPose.getTh(), 
						 thisPose.getTh())
				* percentage));
/*
  printf("original:");
  thisPose.log();
  printf("After:");
  lastPose.log();
  printf("ret:");
  retPose.log();
*/
  *position = retPose;
  return 1;
  
}

AREXPORT size_t ArInterpolation::getNumberOfReadings(void) const
{
  return mySize;
}

AREXPORT void ArInterpolation::setNumberOfReadings(size_t numberOfReadings)
{
  while (myTimes.size() > numberOfReadings)
  {
    myTimes.pop_back();
    myPoses.pop_back();
  }
  mySize = numberOfReadings;  
}

AREXPORT void ArInterpolation::reset(void)
{
  while (myTimes.size() > 0)
    myTimes.pop_back();
  while (myPoses.size() > 0)
    myPoses.pop_back();
}


