/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONSTOP_H
#define ARACTIONSTOP_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action for stopping the robot
/**
   This action simply sets the robot to a 0 velocity and a deltaHeading of 0.
*/
class ArActionStop : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionStop(const char *name = "stop");
  /// Destructor
  AREXPORT virtual ~ArActionStop();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  ArActionDesired myDesired;
};

#endif // ARACTIONSTOP_H
