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

#ifndef ARLASERREFLECTORDEVICE_H
#define ARLASERREFLECTORDEVICE_H

#include "ariaTypedefs.h"
#include "ArRangeDevice.h"
#include "ArFunctor.h"
#include "ArSick.h"

#include "ArRobot.h"

/// A class for keeping track of laser reflectors that we see right now
/** 
    This class is for showing the laser reflectors in MobileEyes.
    This requires that the range device you pass in uses the
    'extraInt' in the rawReadings ArSensorReading to note reflector
    value and that anything greater than 0 is a reflector.
*/
class ArLaserReflectorDevice : public ArRangeDevice
{
public:
  /// Constructor
  AREXPORT ArLaserReflectorDevice(ArRangeDevice *laser, ArRobot *robot,
				  const char * name = "reflector");
  /// Destructor
  AREXPORT virtual ~ArLaserReflectorDevice();
  /// Grabs the new readings from the robot and adds them to the buffers
  AREXPORT void processReadings(void);
  /// Specifically does nothing since it was done in the constructor
  AREXPORT virtual void setRobot(ArRobot *robot);
protected:
  ArRangeDevice *myLaser;
  ArFunctorC<ArLaserReflectorDevice> myProcessCB;
};


#endif // ARLASERREFLECTORDEVICE_H
