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

#ifndef ARBUMPERS_H
#define ARBUMPERS_H

#include "ariaTypedefs.h"
#include "ArRangeDevice.h"


/// A class that treats the robot's bumpers as a range device.
/**
   The class treats bumpers like a range device.  When a bumper
   is bumped, it reports the approximate position of the bump
   in a buffer.  The positions are kept current for a specified 
   length of time.
*/
class ArBumpers : public ArRangeDevice
{
public:
  AREXPORT ArBumpers(size_t currentBufferSize = 30, 
		     size_t cumulativeBufferSize = 30,
		     const char *name = "bumpers",
		     int maxSecondsToKeepCurrent = 15,
		     double angleRange = 135);
  AREXPORT virtual ~ArBumpers(void);

  AREXPORT virtual void setRobot(ArRobot *robot);
  AREXPORT void processReadings(void);
  AREXPORT void addBumpToBuffer(int bumpValue, int whichBumper);

protected:
  ArFunctorC<ArBumpers> myProcessCB;
  ArRobot *myRobot;
  int myBumpMask;
  double myAngleRange;
};


#endif // ARBUMPERS_H
