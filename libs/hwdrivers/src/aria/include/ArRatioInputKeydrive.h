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

#ifndef ARRATIOINPUTKEYDRIVE_H
#define ARRATIOINPUTKEYDRIVE_H

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArFunctor.h"
#include "ArActionRatioInput.h"


/// This will use the keyboard arrow keys and the ArActionRatioInput to drive the robot
/**
   You have to make an ArActionRatioInput and add it to the robot like
   a normal action for this to work.
**/
class ArRatioInputKeydrive 
{
public:
  /// Constructor
  AREXPORT ArRatioInputKeydrive(ArRobot *robot, ArActionRatioInput *input, 
				int priority = 25, double velIncrement = 5);
  /// Destructor
  AREXPORT virtual ~ArRatioInputKeydrive();
  /// Takes the keys this action wants to use to drive
  AREXPORT void takeKeys(void);
  /// Gives up the keys this action wants to use to drive
  AREXPORT void giveUpKeys(void);
  /// Internal, callback for up arrow
  AREXPORT void up(void);
  /// Internal, callback for down arrow
  AREXPORT void down(void);
  /// Internal, callback for left arrow
  AREXPORT void left(void);
  /// Internal, callback for right arrow
  AREXPORT void right(void);
  /// Internal, callback for space key
  AREXPORT void space(void);
  /// Internal, gets our firecb
  /*AREXPORT*/ ArFunctor *getFireCB(void) { return &myFireCB; }
protected:
  AREXPORT void activate(void);
  AREXPORT void deactivate(void);
  AREXPORT void fireCallback(void);
  ArFunctorC<ArRatioInputKeydrive> myUpCB;
  ArFunctorC<ArRatioInputKeydrive> myDownCB;
  ArFunctorC<ArRatioInputKeydrive> myLeftCB;
  ArFunctorC<ArRatioInputKeydrive> myRightCB;
  ArFunctorC<ArRatioInputKeydrive> mySpaceCB;

  double myPrinting;
  double myTransRatio;
  double myRotRatio;
  double myThrottle;
  ArRobot *myRobot;
  bool myHaveKeys;
  double myVelIncrement;
  ArActionRatioInput *myInput;
  ArFunctorC<ArRatioInputKeydrive> myFireCB;
  ArFunctorC<ArRatioInputKeydrive> myActivateCB;
  ArFunctorC<ArRatioInputKeydrive> myDeactivateCB;
};


#endif // ARRATIOINPUTKEYDRIVE_H
