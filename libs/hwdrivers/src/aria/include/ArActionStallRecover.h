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

#ifndef ARACTIONSTALLRECOVER_H
#define ARACTIONSTALLRECOVER_H

#include "ariaTypedefs.h"
#include "ArAction.h"

class ArResolver;

/// Action to recover from a stall
/**
   This action tries to recover if one of the wheels has stalled, it has a
   series of actions it tries in order to get out of the stall.
*/
class ArActionStallRecover : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionStallRecover(const char * name = "stall recover",
		       double obstacleDistance = 225, int cyclesToMove = 50,
		       double speed = 150, double degreesToTurn = 45);
  /// Destructor
  AREXPORT virtual ~ArActionStallRecover();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/virtual ArActionDesired *getDesired(void)
    { return &myActionDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const
                                                   { return &myActionDesired; }
#endif
  AREXPORT void addToConfig(ArConfig* config, const char* sectionName, int priority = ArPriority::NORMAL);
protected:
  // these are internal things, don't touch unless you know what you are doing
  void doit(void); // does whatever should be done
  void addSequence(int sequence);
  int myDoing; // what we're doing, uses the stuff from the enum What
  int myState; // holds the state
  int myCount; // count down variable, -1 if first time in this state
  int mySideStalled; // 1 for left, 2 for right, 3 for both

  enum State
  {
    STATE_NOTHING = 0, // waiting
    STATE_GOING // do something
  };

  enum What
  {
    BACK=0x1, // back up
    FORWARD=0x2, // go forward
    TURN=0x4, // turn away from obstacles
    TURN_LEFT=0x8, // turn left
    TURN_RIGHT=0x10, // turn right
    MOVEMASK = BACK | FORWARD,
    TURNMASK = TURN | TURN_LEFT | TURN_RIGHT
  };

  std::map<int, int> mySequence; // list of things to do as stall continues
  int mySequenceNum;
  int mySequencePos;
  time_t myLastFired;
  double myObstacleDistance;
  int myCyclesToMove;
  double mySpeed;
  int myCyclesToTurn;
  double myDegreesToTurn;
  double myDesiredHeading;
  ArActionDesired myActionDesired;

  ArResolver *myResolver;
};

#endif //ARACTIONSTALLRECOVER_H
