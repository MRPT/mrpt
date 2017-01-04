/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef ARACTIONBUMPERS_H
#define ARACTIONBUMPERS_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to deal with if the bumpers trigger
/**
   This class basically responds to the bumpers the robot has, what
   the activity things the robot has is decided by the param file.  If
   the robot is going forwards and bumps into something with the front
   bumpers, it will back up and turn.  If the robot is going backwards
   and bumps into something with the rear bumpers then the robot will
   move forward and turn.  
*/

class ArActionBumpers : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionBumpers(const char *name = "bumpers", 
			   double backOffSpeed = 100, int backOffTime = 3000,
			   int turnTime = 3000, bool setMaximums = false);
  /// Destructor
  AREXPORT virtual ~ArActionBumpers();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  AREXPORT double findDegreesToTurn(int bumpValue, int whichBumper);
protected:
  ArActionDesired myDesired;
  bool mySetMaximums;
  double myBackOffSpeed;
  int myBackOffTime;
  int myTurnTime;
  //int myStopTime;
  bool myFiring;
  double mySpeed;
  double myHeading;
  int myBumpMask;
  ArTime myStartBack;
  //ArTime myStoppedSince;
};

#endif // ARACTIONBUMPERS
