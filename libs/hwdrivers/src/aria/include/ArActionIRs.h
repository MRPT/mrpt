/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONIRS_H
#define ARACTIONIRS_H

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArRobotParams.h"
#include <vector>

/// Action to back up if short-range IR sensors trigger
/**
 * If the robot has front-mounted binary (triggered/not triggered) IR sensors, 
 * this action will respond to a sensor trigger by backing up and perhaps
 * turning, similar to bumpers.  This action assumes that if an IR triggers, the
 * robot caused it by moving forward into or under an obstacle, and backing up
 * is a good reaction.
 */

class ArActionIRs : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionIRs(const char *name = "IRs", 
		       double backOffSpeed = 100, int backOffTime = 5000,
		       int turnTime = 3000, bool setMaximums = false);
  /// Destructor
  AREXPORT virtual ~ArActionIRs();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual void setRobot(ArRobot *robot);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  ArActionDesired myDesired;
  bool mySetMaximums;
  double myBackOffSpeed;
  int myBackOffTime;
  int myTurnTime;
  int myStopTime;
  bool myFiring;
  double mySpeed;
  double myHeading;
  ArTime myStartBack;
  ArTime stoppedSince;
  ArRobotParams myParams;
  std::vector<int> cycleCounters;
};

#endif // ARACTIONIRS
