/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
