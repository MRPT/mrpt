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
