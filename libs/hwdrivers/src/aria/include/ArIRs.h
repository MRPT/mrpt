/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARIRS_H
#define ARIRS_H

#include "ariaTypedefs.h"
#include "ArRangeDevice.h"


/// A class that treats the robot's Infareds as a range device.
/**
   
*/
class ArIRs : public ArRangeDevice
{
public:
  AREXPORT ArIRs(size_t currentBufferSize = 10, 
		     size_t cumulativeBufferSize = 10,
		     const char *name = "irs",
		     int maxSecondsToKeepCurrent = 15);
  AREXPORT virtual ~ArIRs(void);

  AREXPORT virtual void setRobot(ArRobot *robot);
  AREXPORT void processReadings(void);

protected:
  ArFunctorC<ArIRs> myProcessCB;
  ArRobotParams myParams;
  std::vector<int> cycleCounters;
};


#endif // ARIRS_H
