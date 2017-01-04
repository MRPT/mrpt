/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
