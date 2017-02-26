/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARROBOTCONFIG_H
#define ARROBOTCONFIG_H

#include "ArFunctor.h"

class ArRobot;
class ArAnalogGyro;

/// Class for controlling robot movement parameters from config
class ArRobotConfig
{
public:
  /// Constructor
  AREXPORT ArRobotConfig(ArRobot *robot);
  /// Destructor
  AREXPORT virtual ~ArRobotConfig();
  /// Adds a gyro to turn on and off
  AREXPORT void addAnalogGyro(ArAnalogGyro *gyro);
  /// Called when we process the config
  AREXPORT bool processFile(void);
  /// Called when we connect to the robot
  AREXPORT void connectCallback(void);
protected:
  ArRobot *myRobot;
  ArAnalogGyro *myAnalogGyro;

  bool mySavedOriginalMovementParameters;
  int myOriginalTransVelMax;
  int myOriginalTransAccel;
  int myOriginalTransDecel;
  int myOriginalRotVelMax;
  int myOriginalRotAccel;
  int myOriginalRotDecel;

  bool myAddedMovementParams;
  int myTransVelMax;
  int myTransAccel;
  int myTransDecel;
  int myRotVelMax;
  int myRotAccel;
  int myRotDecel;

  bool myAddedGyro;
  bool myUseGyro;

  ArFunctorC<ArRobotConfig> myConnectCB;  
  ArRetFunctorC<bool, ArRobotConfig> myProcessFileCB;
};

#endif
