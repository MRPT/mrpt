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


#ifndef ARIRRFDEVICE_H
#define ARIRRFDEVICE_H

#include "ariaTypedefs.h"
#include "ArRangeDevice.h"
#include "ArFunctor.h"

#include "ArRobot.h"

/// A class for connecting to a PB-9 and managing the resulting data
/**
   This class is for use with a PB9 IR rangefinder.  It has the packethandler
   necessary to process the packets, and will put the data into ArRangeBuffers
   for use with obstacle avoidance, etc.

   The PB9 is still under development, and only works on an H8 controller
   running AROS.
*/

class ArIrrfDevice : public ArRangeDevice
{
public:
  /// Constructor
  AREXPORT ArIrrfDevice(size_t currentBufferSize = 91,
                        size_t cumulativeBufferSize = 273,
                        const char * name = "irrf");
  /// Destructor
  AREXPORT virtual ~ArIrrfDevice();

  /// The packet handler for use when connecting to an H8 micro-controller
  AREXPORT bool packetHandler(ArRobotPacket *packet);

  /// Maximum range for a reading to be added to the cumulative buffer (mm)
  /*AREXPORT*/ void setCumulativeMaxRange(double r) { myCumulativeMaxRange = r; }
  AREXPORT virtual void setRobot(ArRobot *);

protected:
  ArRetFunctor1C<bool, ArIrrfDevice, ArRobotPacket *> myPacketHandler;
  ArTime myLastReading;
  AREXPORT void processReadings(void);
  double myCumulativeMaxRange;
  double myFilterNearDist;
  double myFilterFarDist;
  std::map<int, ArSensorReading *> myIrrfReadings;
};


#endif // ARIRRFDEVICE_H
