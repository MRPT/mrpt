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

#ifndef ARACTIONINPUT_H
#define ARACTIONINPUT_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action for taking input from outside to control the robot
/**
   This action sets up how we want to drive
*/
class ArActionInput : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionInput(const char *name = "Input");
  /// Destructor
  AREXPORT virtual ~ArActionInput();
  /// Set velocity (cancels deltaVel)
  AREXPORT void setVel(double vel);
  /// Increment/decrement the heading from current
  AREXPORT void deltaHeadingFromCurrent(double delta);
  /// Sets a rotational velocity
  AREXPORT void setRotVel(double rotVel);
  /// Sets a heading
  AREXPORT void setHeading(double heading);
  /// Clears it so its not using vel or heading
  AREXPORT void clear(void);
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  enum RotRegime { NONE, ROTVEL, DELTAHEADING, SETHEADING };
  RotRegime myRotRegime;
  double myRotVal;
  bool myUsingVel;
  double myVelSet;
  ArActionDesired myDesired;
};

#endif // ARACTIONSTOP_H
