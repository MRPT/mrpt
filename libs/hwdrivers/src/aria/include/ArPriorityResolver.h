/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARPRIORITYRESOLVER_H
#define ARPRIORITYRESOLVER_H

#include "ArResolver.h"

/// (Default resolver), takes the action list and uses the priority to resolve
/** 
    This is the default resolver for ArRobot, meaning if you don't do a 
    non-normal init on the robot, or a setResolver, you'll have one these.
*/
class ArPriorityResolver : public ArResolver
{
public:
  /// Constructor
  AREXPORT ArPriorityResolver();
  /// Destructor
  AREXPORT virtual ~ArPriorityResolver();
  AREXPORT virtual ArActionDesired *resolve(ActionMap *actions,
					    ArRobot *robot,
					    bool logActions = false);
protected:
  ArActionDesired myActionDesired;
};

#endif // ARPRIORITYRESOLVER_H
