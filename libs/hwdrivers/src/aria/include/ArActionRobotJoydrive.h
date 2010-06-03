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

#ifndef ARACTIONROBOTJOYDRIVE_H
#define ARACTIONROBOTJOYDRIVE_H

#include "ariaTypedefs.h"
#include "ArAction.h"

class ArRobotPacket;

/// This action will use the joystick for input to drive the robot
/**
   This class creates its own ArJoyHandler to get input from the
   joystick.  Then it will scale the speed between 0 and the given max
   for velocity and turning, up and down on the joystick go
   forwards/backwards while right and left go right and left.  You
   must press in one of the two joystick buttons for the class to pay
   attention to the joystick.
   
   NOTE: The joystick does not save calibration information, so you
   must calibrate the joystick before each time you use it.  To do
   this, press the button for at least a half a second while the
   joystick is in the middle.  Then let go of the button and hold the
   joystick in the upper left for at least a half second and then in
   the lower right corner for at least a half second.
**/
class ArActionRobotJoydrive : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionRobotJoydrive(const char * name = "robotJoyDrive", 
				 bool requireDeadmanPushed = true);
  /// Destructor
  AREXPORT virtual ~ArActionRobotJoydrive();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  AREXPORT virtual void setRobot(ArRobot *robot);
protected:
  AREXPORT bool handleJoystickPacket(ArRobotPacket *packet);
  AREXPORT void connectCallback(void);
  // whether we require the deadman to be pushed to drive
  bool myRequireDeadmanPushed;

  bool myDeadZoneLast;
  int myButton1;
  int myButton2;
  int myJoyX;
  int myJoyY;
  int myThrottle;
  ArTime myPacketReceivedTime;
  // action desired
  ArActionDesired myDesired;
  ArRetFunctor1C<bool, ArActionRobotJoydrive, 
      ArRobotPacket *> myHandleJoystickPacketCB;
  ArFunctorC<ArActionRobotJoydrive> myConnectCB;
};

#endif //ARACTIONROBOTJOYDRIVE_H
