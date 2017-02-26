/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARRATIOINPUTROBOTJOYDRIVE_H
#define ARRATIOINPUTROBOTJOYDRIVE_H

#include "ariaTypedefs.h"
#include "ArActionRatioInput.h"

class ArRobotPacket;
class ArRobot;
class ArRobotJoyHandler;

/// This action will use the joystick on the robot to drive
/**

**/
class ArRatioInputRobotJoydrive 
{
public:
  /// Constructor
  AREXPORT ArRatioInputRobotJoydrive(ArRobot *robot, 
				     ArActionRatioInput *input,
				     int priority = 75,
				     bool requireDeadmanPushed = true);
  /// Destructor
  AREXPORT virtual ~ArRatioInputRobotJoydrive();
protected:
  AREXPORT void fireCallback(void);

  ArRobot *myRobot;
  ArActionRatioInput *myInput;
  bool myRequireDeadmanPushed;
  bool myDeadZoneLast;

  ArRobotJoyHandler *myRobotJoyHandler;
  ArFunctorC<ArRatioInputRobotJoydrive> myFireCB;
};

#endif //ARRATIOINPUTROBOTJOYDRIVE_H
