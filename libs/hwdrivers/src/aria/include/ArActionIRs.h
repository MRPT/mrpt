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
