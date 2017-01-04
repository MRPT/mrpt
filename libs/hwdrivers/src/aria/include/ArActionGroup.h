/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONGROUP_H
#define ARACTIONGROUP_H

#include "ariaTypedefs.h"

#include <list>

class ArRobot;
class ArAction;

/// Group a set of ArAction objects together 
/**
   This class is used to collect a group of related ArActions together, 
   and easily turn them on and off in aggregate. The group list may also
   be retrieved for performing any other operation you wish (e.g. to delete 
   or get information about them.)
   
   @see @ref actions overview
   @see ArAction
   @see @ref actionGroupExample.cpp
**/
class ArActionGroup
{
public:
  /// Constructor
  AREXPORT ArActionGroup(ArRobot * robot);
  /// Destructor, it also deletes the actions in its group
  AREXPORT virtual ~ArActionGroup();
  /// Adds an action to this group's robot, and associates the action with this group.
  AREXPORT virtual void addAction(ArAction *action, int priority);
  /// Removes the action from this group's robot and dissasociates it from this group.
  AREXPORT virtual void remAction(ArAction *action);
  /// Activates all the actions in this group
  AREXPORT virtual void activate(void);
  /// Activates all the actions in this group and deactivates all others
  AREXPORT virtual void activateExclusive(void);
  /// Deactivates all the actions in this group
  AREXPORT virtual void deactivate(void);
  /// Removes all the actions in this group from the robot
  AREXPORT virtual void removeActions(void);
  /// Delets all the actions in this group (doesn't delete them right now)
  AREXPORT virtual void deleteActions(void);
  /// Gets the action list (use this to delete actions after doing removeActions)
  AREXPORT virtual std::list<ArAction *> *getActionList(void);
protected:
  std::list<ArAction *> myActions;
  ArRobot *myRobot;
};

#endif // ARACTIONGROUP_H
