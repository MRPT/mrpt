/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARDPPTU_H
#define ARDPPTU_H

#include "ariaTypedefs.h"
#include "ArRobot.h"
#include "ArPTZ.h"

/** This class is for controlling the Directed Perceptions Pan-Tilt Unit

Note that there are far too many functions enabled in here, most of which are extraneous.  The important ones are defined in the ArPTZ class and include the basic pan, tilt commands.

The DPPTU's pan and tilt commands work on a number of units equal to (degrees / 0.514).  The panTilt function always rounds the conversion closer to zero, so that a magnitude greater than the allowable range of movement is not sent to the camera.

If the DPPTU is connected to the microcontroller, make sure that the baud rate of the microcontroller-DPPTU connection is at least as fast, if not faster than the connection of the computer to the microcontroller.  If it's slower then the commands sent to the DPPTU may get backed up in the AUX port buffer and cause the DPPTU to behave erratically.  So, if the computer-microcontroller connection is autobauding up to 38400bps, then make sure that the microcontroller aux port is set to 38400bps, as well, and consult the DPPTU manual for directions on changing its baud rate.

*/

/// A class with the commands for the DPPTU
class ArDPPTUCommands
{
public:
  enum {
    DELIM = 0x20, ///<Space - Carriage return delimeter
    INIT = 0x40, ///<Init character
    ACCEL = 0x61, ///<Acceleration, Await position-command completion
    BASE = 0x62, ///<Base speed
    CONTROL = 0x63, ///<Speed control
    DISABLE = 0x64, ///<Disable character, Delta, Default
    ENABLE = 0x65, ///<Enable character, Echoing
    FACTORY = 0x66, ///<Restore factory defaults
    HALT = 0x68, ///<Halt, Hold, High
    IMMED = 0x69, ///<Immediate position-command execution mode, Independent control mode
    LIMIT = 0x6C, ///<Position limit character, Low
    MONITOR = 0x6D, ///<Monitor, In-motion power mode
    OFFSET = 0x6F, ///<Offset position, Off
    PAN = 0x70, ///<Pan
    RESET = 0x72, ///<Reset calibration, Restore stored defaults, Regular
    SPEED = 0x73, ///<Speed, Slave
    TILT = 0x74, ///<Tilt
    UPPER = 0x75, ///<Upper speed limit
    VELOCITY = 0x76 ///<Velocity control mode
  };

};

/// A class for for making commands to send to the DPPTU
/**
    There are only a few functioning ways to put things into this packet, you
    MUST use these, if you use anything else your commands won't work.  You
    must use byteToBuf and byte2ToBuf.
*/
class ArDPPTUPacket: public ArBasePacket
{
public:
  /// Constructor
  AREXPORT ArDPPTUPacket(ArTypes::UByte2 bufferSize = 30);
  /// Destructor
  AREXPORT virtual ~ArDPPTUPacket();

  AREXPORT virtual void byte2ToBuf(int val);

  AREXPORT virtual void finalizePacket(void);

protected:
};

/// Driver for the DPPTU
class ArDPPTU : public ArPTZ
{
public:
  /// Constructor
  AREXPORT ArDPPTU(ArRobot *robot);
  /// Destructor
  AREXPORT virtual ~ArDPPTU();

  AREXPORT bool init(void);

  /*AREXPORT*/ bool canZoom(void) const { return 0; }

  /// Sends a delimiter only
  AREXPORT bool blank(void);

  /// Perform reset calibration
  AREXPORT bool resetCalib(void);
  /// Disable power-on reset
  AREXPORT bool disableReset(void);
  /// Reset tilt axis
  AREXPORT bool resetTilt(void);
  /// Reset pan axis only
  AREXPORT bool resetPan(void);
  /// Reset pan and tilt axes on power-on
  AREXPORT bool resetAll(void);

  /// Save current settings as defaults
  AREXPORT bool saveSet(void);
  /// Restore stored defaults
  AREXPORT bool restoreSet(void);
  /// Restore factory defaults
  AREXPORT bool factorySet(void);

  AREXPORT bool panTilt(double pdeg, double tdeg);

  /*AREXPORT*/ bool pan(double deg) { return panTilt(deg, myTilt); }
  /*AREXPORT*/ bool panRel(double deg) { return panTilt(myPan+deg, myTilt); }
  /*AREXPORT*/ bool tilt(double deg) { return panTilt(myPan, deg); }
  /*AREXPORT*/ bool tiltRel(double deg) { return panTilt(myPan, myTilt+deg); }
  /*AREXPORT*/ bool panTiltRel(double pdeg, double tdeg) { return panTilt(myPan+pdeg, myTilt+tdeg); }

  /// Enables or disables the position limit enforcement
  AREXPORT bool limitEnforce(bool val);
  /// Sets unit to immediate-execution mode for positional commands
  AREXPORT bool immedExec(void);
  /// Sets unit to slaved-execution mode for positional commands
  AREXPORT bool slaveExec(void);
  /// Instructs unit to await completion of the last issued command
  AREXPORT bool awaitExec(void);
  /// Halts all pan-tilt movement
  AREXPORT bool haltAll(void);
  /// Halts pan axis movement
  AREXPORT bool haltPan(void);
  /// Halts tilt axis movement
  AREXPORT bool haltTilt(void);

  /*AREXPORT*/ virtual double getMaxPosPan(void) const { return MAX_PAN; }
  /*AREXPORT*/ virtual double getMaxNegPan(void) const { return MIN_PAN; }
  /*AREXPORT*/ virtual double getMaxPosTilt(void) const { return MAX_TILT; }
  /*AREXPORT*/ virtual double getMaxNegTilt(void) const { return MIN_TILT; }
  /*AREXPORT*/ double getMaxPanSlew(void) { return MAX_PAN_SLEW; }
  /*AREXPORT*/ double getMinPanSlew(void) { return MIN_PAN_SLEW; }
  /*AREXPORT*/ double getMaxTiltSlew(void) { return MAX_TILT_SLEW; }
  /*AREXPORT*/ double getMinTiltSlew(void) { return MIN_TILT_SLEW; }
  /*AREXPORT*/ double getMaxPanAccel(void) { return MAX_PAN_ACCEL; }
  /*AREXPORT*/ double getMinPanAccel(void) { return MIN_PAN_ACCEL; }
  /*AREXPORT*/ double getMaxTiltAccel(void) { return MAX_TILT_ACCEL; }
  /*AREXPORT*/ double getMinTiltAccel(void) { return MIN_TILT_ACCEL; }

  /// Sets monitor mode - pan pos1/pos2, tilt pos1/pos2
  AREXPORT bool initMon(double deg1, double deg2, double deg3, double deg4);
  /// Enables monitor mode at power up
  AREXPORT bool enMon(void);
  /// Disables monitor mode at power up
  AREXPORT bool disMon(void);

  /// Sets stationary power mode to off
  AREXPORT bool offStatPower(void);
  /// Sets regular stationary power mode
  AREXPORT bool regStatPower(void);
  /// Sets low stationary power mode
  AREXPORT bool lowStatPower(void);
  /// Sets high in-motion power mode
  AREXPORT bool highMotPower(void);
  /// Sets regular in-motion power mode
  AREXPORT bool regMotPower(void);
  /// Sets low in-motion power mode
  AREXPORT bool lowMotPower(void);

  /// Sets acceleration for pan axis
  AREXPORT bool panAccel(double deg);
  /// Sets acceleration for tilt axis
  AREXPORT bool tiltAccel(double deg);

  /// Sets the start-up pan slew
  AREXPORT bool basePanSlew(double deg);
  /// Sets the start-up tilt slew
  AREXPORT bool baseTiltSlew(double deg);

  /// Sets the upper pan slew
  AREXPORT bool upperPanSlew(double deg);
  /// Sets the lower pan slew
  AREXPORT bool lowerPanSlew(double deg);
  /// Sets the upper tilt slew
  AREXPORT bool upperTiltSlew(double deg);
  /// Sets the lower pan slew
  AREXPORT bool lowerTiltSlew(double deg);

  /// Sets motion to indenpendent control mode
  AREXPORT bool indepMove(void);
  /// Sets motion to pure velocity control mode
  AREXPORT bool velMove(void);

  /// Sets the rate that the unit pans at
  AREXPORT bool panSlew(double deg);
  /// Sets the rate the unit tilts at
  AREXPORT bool tiltSlew(double deg);

  /// Sets the rate that the unit pans at, relative to current slew
  /*AREXPORT*/ bool panSlewRel(double deg) { return panSlew(myPanSlew+deg); }
  /// Sets the rate the unit tilts at, relative to current slew
  /*AREXPORT*/ bool tiltSlewRel(double deg) { return tiltSlew(myTiltSlew+deg); }

  virtual double getPan(void) const { return myPan; }
  virtual double getTilt(void) const { return myTilt; }

  /// Gets the current pan slew
  double getPanSlew(void) { return myPanSlew; }
  /// Gets the current tilt slew
  double getTiltSlew(void) { return myTiltSlew; }
  /// Gets the base pan slew
  double getBasePanSlew(void) { return myBasePanSlew; }
  /// Gets the base tilt slew
  double getBaseTiltSlew(void) { return myBaseTiltSlew; }
  /// Gets the current pan acceleration rate
  double getPanAccel(void) { return myPanAccel; }
  /// Gets the current tilt acceleration rate
  double getTiltAccel(void) { return myTiltAccel; }

  enum {
    MAX_PAN = 158, /// Maximum pan range of 3090 positions
    MIN_PAN = -158, /// Minimum pan range of -3090 positions
    MAX_TILT = 30, /// Maximum tilt range of 600 positions
    MIN_TILT = -46, /// Minimum tilt range of -900 positions
    MAX_PAN_SLEW = 149, /// Maximum pan slew of 2902 positions/sec
    MIN_PAN_SLEW = 2, /// Minimum tilt slew of 31 positions/sec
    MAX_TILT_SLEW = 149, /// Maximum tilt slew of 2902 positions/sec
    MIN_TILT_SLEW = 2, /// Minimum tilt slew of 31 positions/sec
// These aren't correct, because the manual doesn't specify
// the min/max acceleration
    MAX_PAN_ACCEL = 102, /// Maximum pan acceleration of 2000 positions/sec^2
    MIN_PAN_ACCEL = 2, /// Minimum pan acceleration of 0 positions/sec^2
    MAX_TILT_ACCEL = 102, /// Maximum tilt acceleration of 2000 positions/sec^2
    MIN_TILT_ACCEL = 2 /// Minimum tilt acceleration of 0 positions/sec^2
  };
protected:
  ArRobot *myRobot;
  ArDPPTUPacket myPacket;
  void preparePacket(void); /// adds on extra delim in front to work on H8
  double myPan;
  double myTilt;
  double myPanSlew;
  double myTiltSlew;
  double myBasePanSlew;
  double myBaseTiltSlew;
  double myPanAccel;
  double myTiltAccel;
  static const double CONVERT; /// 0.0514 degrees per position
};

#endif // ARDPPTU_H

