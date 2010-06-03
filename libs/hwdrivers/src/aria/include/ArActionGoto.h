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

#ifndef ARACTIONGOTO_H
#define ARACTIONGOTO_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArAction.h"

/// This action goes to a given ArPose very naively

/**
   This action naively drives straight towards a given ArPose. the
   action stops when it gets to be a certain distance (closeDist) 
   from the goal pose.  It travels at the given speed (mm/sec). 

   You can give it a new goal with setGoal(), cancel its movement
   with cancelGoal(), and see if it got there with haveAchievedGoal().
   Once the goal is reached, this action stops requesting any action.

   This doesn't avoid obstacles or anything, you could have an avoid
   routine at a higher priority to avoid on the way there... but for
   real and intelligent looking navigation you should use something
   like ARNL, or build on these actions.
**/


class ArActionGoto : public ArAction
{
public:
  AREXPORT ArActionGoto(const char *name = "goto", 
			ArPose goal = ArPose(0.0, 0.0, 0.0), 
			double closeDist = 100, double speed = 400,
			double speedToTurnAt = 150, double turnAmount = 7);
  AREXPORT virtual ~ArActionGoto();

  /** Sees if the goal has been achieved. The goal is achieved when
   *  the robot's repordet position is within a certain distance
   *  (given in the constructor or in setCloseDist) from the goal pose. */
  AREXPORT bool haveAchievedGoal(void);

  /// Cancels the goal; this action will stop requesting movement.
  AREXPORT void cancelGoal(void);

  /// Sets a new goal and sets the action to go there
  AREXPORT void setGoal(ArPose goal);

  /// Gets the goal the action has
  /*AREXPORT*/ ArPose getGoal(void) { return myGoal; }

  /// Set the distance which is close enough to the goal (mm);
  /*AREXPORT*/ void setCloseDist(double closeDist) { myCloseDist = closeDist; }
  /// Gets the distance which is close enough to the goal (mm)
  /*AREXPORT*/ double getCloseDist(void) { return myCloseDist; }
  /// Sets the speed the action will travel to the goal at (mm/sec)
  /*AREXPORT*/ void setSpeed(double speed) { mySpeed = speed; }
  /// Gets the speed the action will travel to the goal at (mm/sec)
  /*AREXPORT*/ double getSpeed(void) { return mySpeed; }

  /** Called by the action resover; request movement towards goal if we
   *  have one. 
   *  @param currentDesired Current desired action from the resolver
   */
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);

  /** Used by the action resolvel; return current desired action. */
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  ArPose myGoal;
  double myCloseDist;
  double mySpeed;
  double mySpeedToTurnAt;
  double myDirectionToTurn;
  double myCurTurnDir;
  double myTurnAmount;
  ArActionDesired myDesired;
  bool myTurnedBack;
  bool myPrinting;
  ArPose myOldGoal;
  
  enum State
  {
    STATE_NO_GOAL, 
    STATE_ACHIEVED_GOAL,
    STATE_GOING_TO_GOAL
  };
  State myState;
};

#endif // ARACTIONGOTO
