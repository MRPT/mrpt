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
#include "ArSonarDevice.h"
#include "ArSensorReading.h"
#include "ArRobot.h"

AREXPORT ArSonarDevice::ArSonarDevice(size_t currentBufferSize,
			     size_t cumulativeBufferSize, const char *name) :
  ArRangeDevice(currentBufferSize, cumulativeBufferSize, name, 5000), 
  myProcessCB(this, &ArSonarDevice::processReadings)
{
  setMaxDistToKeepCumulative(3000); 
  myFilterNearDist = 50;	// 50 mm between cumulative readings, at least
  myFilterFarDist = 3000;       // throw out cumulative readings this far
                                // from robot
  setMaxSecondsToKeepCurrent(5);
  setMaxSecondsToKeepCumulative(15);
  setCurrentDrawingData(new ArDrawingData("polyArrows", 
                                          ArColor(0x33, 0xCC, 0xFF), 
                                          200,  // mm length of arrow
                                          70),  // first sensor layer
                        true);
}

AREXPORT ArSonarDevice::~ArSonarDevice()
{
  if (myRobot != NULL)
  {
    myRobot->remSensorInterpTask(&myProcessCB);
    myRobot->remRangeDevice(this);
  }
}

AREXPORT void ArSonarDevice::setRobot(ArRobot *robot)
{
  myRobot = robot;
  if (myRobot != NULL)
    myRobot->addSensorInterpTask(myName.c_str(), 10, &myProcessCB);
  ArRangeDevice::setRobot(robot);
}

AREXPORT void ArSonarDevice::processReadings(void)
{
  int i;
  ArSensorReading *reading;
  lockDevice();

  for (i = 0; i < myRobot->getNumSonar(); i++)
  {
    reading = myRobot->getSonarReading(i);
    if (reading == NULL || !reading->isNew(myRobot->getCounter()))
      continue;
    addReading(reading->getX(), reading->getY());
  }

  // delete too-far readings
  std::list<ArPoseWithTime *> *readingList;
  std::list<ArPoseWithTime *>::iterator it;
  double dx, dy, rx, ry;
    
  myCumulativeBuffer.beginInvalidationSweep();
  readingList = myCumulativeBuffer.getBuffer();
  rx = myRobot->getX();
  ry = myRobot->getY();
  // walk through the list and see if this makes any old readings bad
  if (readingList != NULL)
    {
      for (it = readingList->begin(); it != readingList->end(); ++it)
	{
	  dx = (*it)->getX() - rx;
	  dy = (*it)->getY() - ry;
	  if ((dx*dx + dy*dy) > (myFilterFarDist * myFilterFarDist)) 
	    myCumulativeBuffer.invalidateReading(it);
	}
    }
  myCumulativeBuffer.endInvalidationSweep();
  // leave this unlock here or the world WILL end
  unlockDevice();
}

/**
   Adds a sonar reading with the global coordinates x,y.  Makes sure the
   reading is within the proper distance to the robot, for
   both current and cumulative buffers.  Filters buffer points 
   Note: please lock the device using lockDevice() / unlockDevice() if
   calling this from outside process().
   @param x the global x coordinate of the reading
   @param y the global y coordinate of the reading
*/
AREXPORT void ArSonarDevice::addReading(double x, double y)
{
  double rx = myRobot->getX();
  double ry = myRobot->getY();
  double dx = x - rx;		
  double dy = y - ry;
  double dist2 = dx*dx + dy*dy;
  
  if (dist2 < myMaxRange*myMaxRange)
    myCurrentBuffer.addReading(x,y);
  
  if (dist2 < myMaxDistToKeepCumulative * myMaxDistToKeepCumulative)
    {
      std::list<ArPoseWithTime *> *readingList;
      std::list<ArPoseWithTime *>::iterator it;

      myCumulativeBuffer.beginInvalidationSweep();

      readingList = myCumulativeBuffer.getBuffer();
      // walk through the list and see if this makes any old readings bad
      if (readingList != NULL)
	{
	  for (it = readingList->begin(); it != readingList->end(); ++it)
	    {
	      dx = (*it)->getX() - x;
	      dy = (*it)->getY() - y;
	      if ((dx*dx + dy*dy) < (myFilterNearDist * myFilterNearDist)) 
		myCumulativeBuffer.invalidateReading(it);
	    }
	}
      myCumulativeBuffer.endInvalidationSweep();

      myCumulativeBuffer.addReading(x,y);
    }
}
