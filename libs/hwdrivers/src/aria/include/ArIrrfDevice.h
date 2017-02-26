/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


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
