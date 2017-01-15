/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARFORBIDDENRANGEDEVICE_H
#define ARFORBIDDENRANGEDEVICE_H

#include "ariaTypedefs.h"
#include "ArRangeDevice.h"
#include "ArMap.h"

/// Class that takes forbidden lines and turns them into range readings
class ArForbiddenRangeDevice : public ArRangeDevice
{
public:
  /// Constructor
  AREXPORT ArForbiddenRangeDevice(ArMap *armap, double distanceIncrement = 100,
				  unsigned int maxRange = 4000,
				  const char *name = "forbidden");
  /// Destructor
  AREXPORT virtual ~ArForbiddenRangeDevice();
  /// Saves the forbidden lines from the map
  AREXPORT void processMap(void);
  /// Remakes the readings 
  AREXPORT void processReadings(void);
  /// Sets the robot pointer and attachs its process function
  AREXPORT virtual void setRobot(ArRobot *robot);

  /// Enable readings 
  AREXPORT void enable(void);
  /// Disables readings until reenabled
  AREXPORT void disable(void);
  /// Sees if this device is active or not
  /*AREXPORT*/ bool isEnabled(void) const { return myIsEnabled;; }
  /// Gets a callback to enable the device
  /*AREXPORT*/ ArFunctor *getEnableCB(void) { return &myEnableCB; } 
  /// Gets a callback to disable the device
  /*AREXPORT*/ ArFunctor *getDisableCB(void) { return &myDisableCB; } 
protected:
  ArMutex myDataMutex;
  ArMap *myMap;
  double myDistanceIncrement;
  std::list<ArLineSegment *> mySegments;
  ArFunctorC<ArForbiddenRangeDevice> myProcessCB;
  ArFunctorC<ArForbiddenRangeDevice> myMapChangedCB;
  bool myIsEnabled;
  ArFunctorC<ArForbiddenRangeDevice> myEnableCB;
  ArFunctorC<ArForbiddenRangeDevice> myDisableCB;
};

#endif // ARFORBIDDENRANGEDEVICE_H
