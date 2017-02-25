/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARRESOLVER_H
#define ARRESOLVER_H

#include "ariaTypedefs.h"
#include "ArActionDesired.h"
#include <string>

class ArAction;
class ArRobot;

/// Resolves a list of actions and returns what to do
/**
  ArResolver::resolve() is the function that ArRobot
  calls with the action list in order
  to produce a combined ArActionDesired object from them, according to
  the subclass's particular algorithm or policy.
*/
class ArResolver
{
public:
  /// Constructor
  typedef std::multimap<int, ArAction *> ActionMap;
  ArResolver(const char *name, const char * description = "")
    { myName = name; myDescription = description; }
  /// Desturctor
  virtual ~ArResolver() {};
  /// Figure out a single ArActionDesired from a list of ArAction s
  virtual ArActionDesired *resolve(ActionMap *actions, ArRobot *robot,
				   bool logActions = false) = 0;
  /// Gets the name of the resolver
  virtual const char *getName(void) const { return myName.c_str(); }
  /// Gets the long description fo the resolver
  virtual const char *getDescription(void) const { return myDescription.c_str(); }
  
protected:
  std::string myName;
  std::string myDescription;
};

#endif
