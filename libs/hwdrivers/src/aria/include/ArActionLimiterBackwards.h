/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONBACKWARDSSPEEDLIMITER_H
#define ARACTIONBACKWARDSSPEEDLIMITER_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to limit the backwards motion of the robot based on range sensor readings
/**
   This class limits the backwards motion of the robot according to range sensor
   readings (e.g. sonar, laser), and the parameters given. When the range
   sensor (e.g. sonar or laser) detects rearward obstacles closer than the given parameters,
   this action requests that the robot decelerate or stop any current backwards movement.
*/
class ArActionLimiterBackwards : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionLimiterBackwards(const char *name = "speed limiter", 
				    double stopDistance = -250,
				    double slowDistance = -600,
				    double maxBackwardsSpeed = -250,
				    double widthRatio = 1.5);
  /// Destructor
  AREXPORT virtual ~ArActionLimiterBackwards();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  double myStopDist;
  double mySlowDist;
  double myMaxBackwardsSpeed;
  double myWidthRatio;
  ArActionDesired myDesired;
};

#endif // ARACTIONBACKWARDSSPEEDLIMITER_H

