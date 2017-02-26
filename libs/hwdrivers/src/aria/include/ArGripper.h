/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARGRIPPER_H
#define ARGRIPPER_H

#include "ariaTypedefs.h"
#include "ArRobot.h"

/// Constains gripper command numbers
/**
   A class with an enum of the commands for the gripper, see the Pioneer operations
   manual and the gripper guide for more detailed descriptions.  The enum 
   values which start with GRIP are for the gripper paddles, the ones which
   start with LIFT are the for the lift, and the ones which start with GRIPPER
   are for the entire unit.
*/
class ArGripperCommands
{
public:
  enum Commands {
    GRIP_OPEN = 1, ///< open the gripper paddles fully
    GRIP_CLOSE = 2, ///< close the gripper paddles all the way
    GRIP_STOP = 3, ///< stop the gripper paddles where they are
    LIFT_UP = 4, ///< raises the lift to the top of its range
    LIFT_DOWN = 5, ///< lowers the lift to the bottom of its range
    LIFT_STOP = 6, ///< stops the lift where it is
    GRIPPER_STORE = 7, /**< 
		       closes the paddles and raises the lift simultaneously, 
		       this is for storage not for grasping/carrying an object
		    */
    GRIPPER_DEPLOY = 8, /**< 
			opens the paddles and lowers the lieft simultaneously,
			this is for getting ready to grasp an object, not for
			object drops
		     */
    GRIPPER_HALT = 15, ///< stops the gripper paddles and lift from moving
    GRIP_PRESSURE = 16, /**< 
			sets the time delay in 20 msec increments after the
			gripper paddles first grasp an object before they stop
			moving, regulates grasp pressure
		     */
    LIFT_CARRY = 17 /**<
		       raises or lowers the lieft, the argument is the number
		       of 20 msec increments to raise or lower the lift,
		       poseitive arguments for raise, negative for lower
		    */
  };
};

/// Provides an interface to the Pioneer gripper device
class ArGripper
{
public:
  /// Constructor
  AREXPORT ArGripper(ArRobot *robot, int gripperType = QUERYTYPE);
  /// Destructor
  AREXPORT virtual ~ArGripper();
  /// Opens the gripper paddles
  AREXPORT bool gripOpen(void);
  /// Closes the gripper paddles
  AREXPORT bool gripClose(void);
  /// Stops the gripper paddles
  AREXPORT bool gripStop(void);
  /// Raises the lift to the top
  AREXPORT bool liftUp(void);
  /// Lowers the lift to the bottom
  AREXPORT bool liftDown(void);
  /// Stops the lift
  AREXPORT bool liftStop(void);
  /// Puts the gripper in a storage position
  AREXPORT bool gripperStore(void);
  /// Puts the gripper in a deployed position, ready for use
  AREXPORT bool gripperDeploy(void);
  /// Halts the lift and the gripper paddles
  AREXPORT bool gripperHalt(void);
  /// Sets the amount of pressure the gripper applies
  AREXPORT bool gripPressure(int mSecIntervals);
  /// Raises the lift by a given amount of time
  AREXPORT bool liftCarry(int mSecIntervals);
  /// Returns true if the gripper paddles are moving
  AREXPORT bool isGripMoving(void) const;
  /// Returns true if the lift is moving
  AREXPORT bool isLiftMoving(void) const;
  /// Returns the state of the gripper paddles
  AREXPORT int getGripState(void) const;
  /// Returns the state of each gripper paddle
  AREXPORT int getPaddleState(void) const;
  /// Returns the state of the gripper's breakbeams
  AREXPORT int getBreakBeamState(void) const;
  /// Returns the state of the lift
  AREXPORT bool isLiftMaxed(void) const;
  /// Gets the type of the gripper
  AREXPORT int getType(void) const;
  /// Sets the type of the gripper
  AREXPORT void setType(int type);
  /// Gets the number of mSec since the last gripper packet
  AREXPORT long getMSecSinceLastPacket(void) const;
  /// Gets the grasp time
  AREXPORT int getGraspTime(void) const;
  /// logs the gripper state
  AREXPORT void logState(void) const;
  /// Parses the gripper packet
  AREXPORT bool packetHandler(ArRobotPacket *packet);
  /// The handler for when the robot connects
  AREXPORT void connectHandler(void);
  /// These are the types for the gripper
  enum Type {
    QUERYTYPE, ///< Finds out what type from the robot, default
    GENIO, ///< Uses general IO
    USERIO, ///< Uses the user IO
    GRIPPAC, ///< Uses a packet requested from the robot
    NOGRIPPER ///< There isn't a gripper
  }; 
protected:
  ArRobot *myRobot;
  int myType;
  unsigned char myState;
  unsigned char mySwitches;
  unsigned char myGraspTime;
  ArTime myLastDataTime;
  ArFunctorC<ArGripper> myConnectCB;
  ArRetFunctor1C<bool, ArGripper, ArRobotPacket *> myPacketHandlerCB;
};


#endif // ARGRIPPER_H
