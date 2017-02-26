/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
