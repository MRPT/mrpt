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
