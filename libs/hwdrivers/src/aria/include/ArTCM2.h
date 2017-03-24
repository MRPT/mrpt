/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARTCM2_H
#define ARTCM2_H

#include "ariaUtil.h"
#include "ArFunctor.h"
#include "ArRobot.h"

/// This class will get extra information that the tcm2 supplies
/**
   This class will get back the tcm2 packets and send commands to the
   robot related to the TCM2, you should also look at the information
   in our TCM2 manual and the TCM2 user manual for a real
   understanding of what these all do.
 **/
class ArTCM2
{
public:

  /// Constructor
  AREXPORT ArTCM2(ArRobot *robot);
  /// Destructor
  AREXPORT virtual ~ArTCM2();
  /// Gets the compass heading (-180, 180] degrees
  double getCompass(void) const { return myCompass; }
  /// Gets the pitch
  double getPitch(void) const { return myPitch; }
  /// Gets the roll
  double getRoll(void) const { return myRoll; }
  /// Gets the magnetic field X component
  double getXMagnetic(void) const { return myXMag; }
  /// Gets the magnetic field Y component
  double getYMagnetic(void) const { return myYMag; }
  /// Gets the magnetic field Z component
  double getZMagnetic(void) const { return myZMag; }
  /// Gets the temperature
  double getTemperature(void) const { return myTemperature; }
  // Gets the error (see manual)
  int getError(void) const { return myError; }
  /// Gets the calibration H score
  double getCalibrationH(void) const { return myCalibrationH; }
  /// Gets the calibration V score
  double getCalibrationV(void) const { return myCalibrationV; }
  /// Gets the calibration M score
  double getCalibrationM(void) const { return myCalibrationM; }
  /// Turns the compass off (software only, not low power standby)
  void commandOff(void) { myRobot->comInt(ArCommands::TCM2, 0); }
  /// Puts it to just compass mode (into ArRobot::getCompass)
  void commandJustCompass(void) { myRobot->comInt(ArCommands::TCM2, 1); }
  /// Gets a single packet back
  void commandOnePacket(void) { myRobot->comInt(ArCommands::TCM2, 2); }
  /// Gets packets back continuously
  void commandContinuousPackets(void) { myRobot->comInt(ArCommands::TCM2, 3); }
  /// Enables user calibration
  void commandUserCalibration(void) { myRobot->comInt(ArCommands::TCM2, 4); }
  /// Enable auto calibration
  void commandAutoCalibration(void) { myRobot->comInt(ArCommands::TCM2, 5); }
  /// Stops calibration and gets one packet, then resets to just compass mode
  void commandStopCalibration(void) { myRobot->comInt(ArCommands::TCM2, 6); }
  /// Command to do a soft reset of the compass
  void commandSoftReset(void) { myRobot->comInt(ArCommands::TCM2, 7); }
  /// Gets the number of readings in the last second
  AREXPORT int getPacCount(void);
  /// Packet handler for the tcm2 packet
  AREXPORT bool packetHandler(ArRobotPacket *packet);
protected:  
  ArRobot *myRobot;

  double myCompass;
  double myPitch;
  double myRoll;
  double myXMag;
  double myYMag;
  double myZMag;
  double myTemperature;
  int myError;
  double myCalibrationH;
  double myCalibrationV;
  double myCalibrationM;

  // packet count
  time_t myTimeLastPacket;
  int myPacCurrentCount;
  int myPacCount;		
  ArRetFunctor1C<bool, ArTCM2, ArRobotPacket *> myPacketHandlerCB;
  
};

#endif // ARTCM2_H
