/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact
MobileRobots for information about a commercial version of ARIA at
robots@mobilerobots.com or
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArRobot.h"
#include "ariaUtil.h"
#include "ArIRs.h"

 /**
   @param currentBufferSize The number of readings to store in the current Buffer
   @param cumulativeBufferSize The number of readings in the cumulative buffer (This currently is not being used)
   @param name The name of this range device
   @param maxSecondsToKeepCurrent How long to keep readings in the current buffer
*/

AREXPORT ArIRs::ArIRs(size_t currentBufferSize, size_t cumulativeBufferSize,
		      const char *name, int maxSecondsToKeepCurrent) :
  ArRangeDevice(currentBufferSize, cumulativeBufferSize, name, 5000, maxSecondsToKeepCurrent),
  myProcessCB(this, &ArIRs::processReadings)
{
  setCurrentDrawingData(new ArDrawingData("polyArrows", ArColor(255, 255, 0),
					  120, // mm diameter of dots
					  80), // layer above sick and sonar below bumpers
			true);
}

AREXPORT ArIRs::~ArIRs()
{
  if (myRobot != NULL)
    {
      myRobot->remSensorInterpTask(&myProcessCB);
      myRobot->remRangeDevice(this);
    }
}

AREXPORT void ArIRs::setRobot(ArRobot *robot)
{
  myRobot = robot;
  if (myRobot != NULL)
    myRobot->addSensorInterpTask(myName.c_str(), 10, &myProcessCB);
  ArRangeDevice::setRobot(robot);

  const ArRobotParams *params;
  params = myRobot->getRobotParams();
  myParams = *params;

  for(int i = 0; i < myParams.getNumIR(); i++)
    cycleCounters.push_back(1);
}

/**
   This function is called every 100 milliseconds.
*/
AREXPORT void ArIRs::processReadings(void)
{
  ArUtil::BITS bit = ArUtil::BIT0;
  if(myParams.haveTableSensingIR())
    {
      for (int i = 0; i < myParams.getNumIR(); ++i)
	{
	  switch(i)
	    {
	    case 0:
	      bit = ArUtil::BIT0;
	      break;
	    case 1:
	      bit = ArUtil::BIT1;
	      break;
	    case 2:
	      bit = ArUtil::BIT2;
	      break;
	    case 3:
	      bit = ArUtil::BIT3;
	      break;
	    case 4:
	      bit = ArUtil::BIT4;
	      break;
	    case 5:
	      bit = ArUtil::BIT5;
	      break;
	    case 6:
	      bit = ArUtil::BIT6;
	      break;
	    case 7:
	      bit = ArUtil::BIT7;
	      break;
	    }

	  if(myParams.haveNewTableSensingIR() && myRobot->getIODigInSize() > 3)
	    {
	      if((myParams.getIRType(i) && !(myRobot->getIODigIn(3) & bit)) ||
		  (!myParams.getIRType(i) && (myRobot->getIODigIn(3) & bit)))
		{
		  if(cycleCounters[i] < myParams.getIRCycles(i))
		    {
		      cycleCounters[i] = cycleCounters[i] + 1;
		    }
		  else
		    {
		      cycleCounters[i] = 1;
		      ArPose pose;
		      pose.setX(myParams.getIRX(i));
		      pose.setY(myParams.getIRY(i));

		      ArTransform global = myRobot->getToGlobalTransform();
		      pose = global.doTransform(pose);

		      myCurrentBuffer.addReading(pose.getX(), pose.getY());
		    }
		}
	      else
		{
		  cycleCounters[i] = 1;
		}
	    }
	  else
	    {
	      if(!(myRobot->getDigIn() & bit))
		{
		  if(cycleCounters[i] < myParams.getIRCycles(i))
		    {
		      cycleCounters[i] = cycleCounters[i] + 1;
		    }
		  else
		    {
		      cycleCounters[i] = 1;

		      ArPose pose;
		      pose.setX(myParams.getIRX(i));
		      pose.setY(myParams.getIRY(i));

		      ArTransform global = myRobot->getToGlobalTransform();
		      pose = global.doTransform(pose);

		      myCurrentBuffer.addReading(pose.getX(), pose.getY());
		    }
		}
	      else
		{
		  cycleCounters[i] = 1;
		}
	    }
	}
    }
}

