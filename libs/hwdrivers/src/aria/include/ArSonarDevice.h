/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSONARDEVICE_H
#define ARSONARDEVICE_H

#include "ariaTypedefs.h"
#include "ArRangeDevice.h"
#include "ArFunctor.h"

#include "ArRobot.h"

/// Keep track of recent sonar readings from a robot as an ArRangeDevice
/**
    This class is for keeping a sonar history, which you may use for obstacle
    avoidance, display, etc.
    Simply use ArRobot::addRangeDevice()  (or ArSonarDevice::setRobot())
    to attach an ArSonarDevice object to an ArRobot
    robot object; ArSonarDevice will add a Sensor Interpretation task to the
    ArRobot which will read new sonar readings each robot cycle and add
    them to its sonar history.

    (Note that sonar range readings are from the surface of the sonar transducer disc,
    not from the center of the robot.)
*/
class ArSonarDevice : public ArRangeDevice
{
public:
  /// Constructor
  AREXPORT ArSonarDevice(size_t currentBufferSize = 24,
			 size_t cumulativeBufferSize = 64,
			 const char * name = "sonar");
  /// Destructor
  AREXPORT virtual ~ArSonarDevice();
  /// Grabs the new readings from the robot and adds them to the buffers
  /// (Primarily for internal use.)
  AREXPORT void processReadings(void);

  /// Sets the robot pointer, also attaches its process function to the
  /// robot as a Sensor Interpretation task.
  AREXPORT virtual void setRobot(ArRobot *robot);

  /// Adds sonar readings to the current and cumulative buffers
  /// Overrides the ArRangeDevice default action.
  /// (This method is primarily for internal use.)
  AREXPORT virtual void addReading(double x, double y);

  /** @deprecated
   *  @sa ArRangeDevice::setMaxDistToKeepCumulative()
   */
  /*AREXPORT*/void setCumulativeMaxRange(double range)
    { setMaxDistToKeepCumulative(range); }
protected:
  ArFunctorC<ArSonarDevice> myProcessCB;
  double myFilterNearDist;	// we throw out cumulative readings this close to current one
  double myFilterFarDist;	// throw out cumulative readings beyond this far from robot
};


#endif // ARSONARDEVICE_H
