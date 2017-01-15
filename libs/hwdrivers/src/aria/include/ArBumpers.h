/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
