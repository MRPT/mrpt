/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONTURN
#define ARACTIONTURN

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to turn when the behaviors with more priority have limited the speed
/**
   This action is basically made so that you can just have a ton of
   limiters of different kinds and types to keep speed under control,
   then throw this into the mix to have the robot wander.  Note that
   the turn amount ramps up to turnAmount starting at 0 at
   speedStartTurn and hitting the full amount at speedFullTurn.
**/
class ArActionTurn : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionTurn(const char *name = "turn",
			double speedStartTurn = 200,
			double speedFullTurn = 100,
			double turnAmount = 15);
  /// Destructor
  AREXPORT virtual ~ArActionTurn();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  double mySpeedStart;
  double mySpeedFull;
  double myTurnAmount;
  double myTurning;

  ArActionDesired myDesired;

};

#endif // ARACTIONTURN
