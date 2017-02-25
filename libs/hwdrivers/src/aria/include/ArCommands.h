/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARCOMMANDS_H
#define ARCOMMANDS_H

/// A class containing names for most robot microcontroller system commands
/**
   A class with an enum of the commands that can be sent to the robot, see the 
   robot operations manual for more detailed descriptions.
*/
class ArCommands
{
public:
  enum Commands { 
  PULSE = 0, ///< none, keep alive command, so watchdog doesn't trigger
  OPEN = 1, ///< none, sent after connection to initiate connection
  CLOSE = 2, ///< none, sent to close the connection to the robot
  POLLING = 3, ///< string, string that sets sonar polling sequence
  ENABLE = 4, ///< int, enable (1) or disable (0) motors
  SETA = 5, ///< int, sets translational accel (+) or decel (-) (mm/sec/sec)
  SETV = 6, ///< int, sets maximum velocity (mm/sec)
  SETO = 7, ///< int, resets robots origin back to 0, 0, 0
  MOVE = 8, ///< int, translational move (mm)
  ROTATE = 9, ///< int, set rotational velocity, duplicate of RVEL (deg/sec)
  SETRV = 10, ///< int, sets the maximum rotational velocity (deg/sec)
  VEL = 11, ///< int, set the translational velocity (mm/sec)
  HEAD = 12, ///< int, turn to absolute heading 0-359 (degrees)
  DHEAD = 13, ///< int, turn relative to current heading (degrees)
  //DROTATE = 14, does not really exist
  SAY = 15, /**< string, makes the robot beep.
	     up to 20 pairs of duration (20 ms incrs) and tones (halfcycle) */
  JOYINFO = 17, // int, requests joystick packet, 0 to stop, 1 for 1, 2 for continuous
  CONFIG = 18, ///< int, request configuration packet
  ENCODER = 19, ///< int, > 0 to request continous stream of packets, 0 to stop
  RVEL = 21, ///< int, set rotational velocity (deg/sec)
  DCHEAD = 22, ///< int, colbert relative heading setpoint (degrees)
  SETRA = 23, ///< int, sets rotational accel(+) or decel(-) (deg/sec)
  SONAR = 28, ///< int, enable (1) or disable (0) sonar 
  STOP = 29, ///< int, stops the robot
  DIGOUT = 30, ///< int, sets the digout lines
  //TIMER = 31, ... no clue about this one
  VEL2 = 32, /**< int, independent wheel velocities,
		first 8 bits = right, second 8 bits = left, multiplied by Vel2 divisor. See manual.  */
  GRIPPER = 33, ///< int, gripper server command, see gripper manual for detail
  //KICK = 34, um...
  ADSEL = 35, ///< int, select the port given as argument
  GRIPPERVAL = 36, ///< p2 gripper server value, see gripper manual for details
  GRIPPERPACREQUEST = 37, ///< p2 gripper packet request
  IOREQUEST = 40, ///< request iopackets from p2os
  PTUPOS = 41, ///< most-sig byte is port number, least-sig byte is pulse width
  TTY2 = 42, ///< string, send string argument to serial dev connected to aux1 
  GETAUX = 43, ///< int, requests 1-200 bytes from aux1 serial channel, 0 flush
  BUMPSTALL = 44, /**< int, stop and register a stall if front (1), rear (2),
		   or both (3) bump rings are triggered, Off (default) is 0 */
  TCM2 = 45, ///< TCM2 module commands, see tcm2 manual for details 
  JOYDRIVE = 47, /**< Command to tell p2os to drive with the joystick 
		    plugged into the robot */
  MOVINGBLINK = 49, ///< int, 1 to blink lamp quickly before moving, 0 not to (for patrolbot)
  HOSTBAUD = 50, ///< int, set baud rate for host port - 0=9600, 1=19200, 2=38400, 3=57600, 4=115200
  AUX1BAUD = 51, ///< int, set baud rate for Aux1 - 0=9600, 1=19200, 2=38400, 3=57600, 4=115200
  AUX2BAUD = 52, ///< int, set baud rate for Aux2 - 0=9600, 1=19200, 2=38400, 3=57600, 4=115200
  ESTOP = 55, ///< none, emergency stop, overrides decel
  ESTALL = 56, // ?
  GYRO = 58, ///< int, set to 1 to enable gyro packets, 0 to disable
  TTY3 = 66,
  GETAUX2 = 67,

  // SRISIM specific:
  LOADPARAM = 61, ///< @deprecated  only supported by SRISim
  OLDSIM_LOADPARAM = 61, ///< @deprecated only supported by SRISim
  ENDSIM = 62, ///< @deprecated use SIM_EXIT 
  OLDSIM_EXIT = 62, ///< @deprecated use SIM_EXIT
  LOADWORLD = 63, ///< @deprecated only supported by SRISim
  OLDSIM_LOADWORLD = 63, ///< @deprecated only supported by SRISim
  STEP = 64, ///< @deprecated only supported by SRISim
  OLDSIM_STEP = 64, ///< @deprecated only supported by SRISim

  // for calibrating the compass:
  CALCOMP = 65, ///< int, commands for calibrating compass, see compass manual

  // SRISIM specific:
  // SETORIGINX and SETORIGINY overlap with TTY3 and GETAUX2 so they are disabled:
  //SETSIMORIGINX = 66, 
  //SETSIMORIGINY = 67, 
  //OLDSIM_SETORIGINX = 66,
  //OLDSIM_SETORIGINY = 67,
  SETSIMORIGINTH = 68, ///< @deprecated use SIM_SET_POSE
  OLDSIM_SETORIGINTH = 68, ///< @deprecated use SIM_SET_POSE
  RESETSIMTOORIGIN = 69, ///< @deprecated use SIM_RESET
  OLDSIM_RESETTOORIGIN = 69, ///< @deprecated use SIM_RESET

  // AmigoBot-H8 specific:
  SOUND = 90, ///< int, AmigoBot (old H8 model) specific, plays sound with given number
  PLAYLIST = 91, /**< int, AmigoBot (old H8 model) specific, requests name of sound, 
		    0 for all, otherwise for specific sound */
  SOUNDTOG = 92, ///< int, AmigoBot (old H8 model) specific, enable(1) or diable(0) sound

  // MobileSim specific:
  SIM_SET_POSE = 224,       ///< int4,int4,int4 Move robot to global pose in simulator (does not change odometry). Each value is a 4-byte integer.
  SIM_RESET= 225,         ///< none, Reset robot's state to original in simulator and reset odometry to 0,0,0.
  SIM_LRF_ENABLE = 230,   ///< int, 1 to begin sending packets of data from a simulated laser rangefinder (on the same socket connection), 2 to send extended-information laser packets (with reading flags), 0 to disable LRF
  SIM_LRF_SET_FOV_START = 231,  ///< int Set angle (degrees from center) at which the simulater laser takes its first reading (normally -90).
  SIM_LRF_SET_FOV_END = 232,  ///< int Set angle (degrees from center) at which the simulated laser takes its last reading (normally 90).
  SIM_LRF_SET_RES = 233,  ///< int Set the number of degrees between laser readings (in combination with FOV, determines the number of readings per sweep) (normally 1)
  SIM_CTRL = 236,         ///< int,..., Send a simulator meta-command (an operation on the simulator itself). The initial 2-byte integer argument selects the operation. See simulator documentation.
  SIM_STAT = 237,         ///< none, Request that the simulator reply with a SIMSTAT (0x62) packet. You must have a packet handler registered with ArRobot to receive its output. See simulator documentation.
  SIM_MESSAGE = 238,      ///< string, Display a log message in the simulator. Argument is a length-prefixed ASCII byte string.
  SIM_EXIT = 239          ///< int, Exit the simulator. Argument is the exit code (use 0 for a "normal" exit).
  };
  
};

#endif // ARCOMMANDS_H


