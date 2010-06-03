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

#ifndef ARACTIONAVOIDFRONT_H
#define ARACTIONAVOIDFRONT_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"
#include "ArAction.h"

/// This action does obstacle avoidance, controlling both trans and rot
/**
   This action uses whatever available range device have been added to
   the robot to avoid obstacles.  See the ArActionAvoidFront
   constructor documentation to see the parameters it takes.

   Also note that this action does something most others don't, which
   is to check for a specific piece of hardware.  This is the
   tableSensingIR.  If this is set up in the parameters for the robot,
   it will use DigIn0 and DigIn1, where the tableSensingIRs are
   connected.  Note that if you make useTableIRIfAvail false in the
   constructor it'll ignore these. Whether the action thinks the robot
   has them or not depends on the value of tableSensingIR in the
   parameter file for that robot. 
*/
class ArActionAvoidFront : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionAvoidFront(const char *name = "avoid front obstacles", 
		     double obstacleDistance = 450, double avoidVelocity = 200,
		     double turnAmount = 15, bool useTableIRIfAvail = true);
  /// Destructor
  AREXPORT virtual ~ArActionAvoidFront();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  double myTurnAmount;
  double myObsDist;
  double myAvoidVel;
  double myTurnAmountParam;
  bool myUseTableIRIfAvail;
  int myTurning; // 1 for turning left, 0 for not turning, -1 for turning right
  ArActionDesired myDesired;
  ArSectors myQuadrants;
  ArFunctorC<ArActionAvoidFront> myConnectCB;
};

#endif // ARACTIONAVOIDFRONT_H
