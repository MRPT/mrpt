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

#ifndef ARACTIONAVOIDSIDE_H
#define ARACTIONAVOIDSIDE_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to avoid impacts by firening into walls at a shallow angle
/**
   This action watches the sensors to see if it is close to firening into a wall
   at a shallow enough angle that other avoidance may not avoid.
*/
class ArActionAvoidSide : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionAvoidSide(const char *name = "Avoid side", 
		    double obstacleDistance = 300,
		    double turnAmount = 5);
  /// Destructor
  AREXPORT virtual ~ArActionAvoidSide();
  AREXPORT virtual ArActionDesired * fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  double myObsDist;
  double myTurnAmount;
  bool myTurning;
  ArActionDesired myDesired;

};

#endif // ARACTIONAVOIDSIDE_H
