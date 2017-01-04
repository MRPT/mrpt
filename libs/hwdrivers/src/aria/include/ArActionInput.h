/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
