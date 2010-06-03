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

