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
