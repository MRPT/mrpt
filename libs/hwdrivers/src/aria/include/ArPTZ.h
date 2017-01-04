/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARPTZ_H
#define ARPTZ_H

#include "ariaTypedefs.h"
#include "ArFunctor.h"
#include "ArCommands.h"
#include <mrpt/utils/mrpt_macros.h>

class ArRobot;
class ArBasePacket;
class ArRobotPacket;
class ArDeviceConnection;


/// Base class which handles the PTZ cameras
/**
    This class is mainly concerned with making all the cameras look
    the same for outgoing data, it is also set up to facilitate the
    acquisition of incoming data but that is described in the
    following paragraphs.  There are two ways this can be used.  The
    first is the simplest and default behavior and should be used by
    those whose cameras are attached to their robot's microcontroller,
    a ArRobot pointer is passed in to the contructor, this is where
    the commands will be sent to the robot via the robot's connection
    which will then send it along over the second serial port.  The
    second way is to pass an ArDeviceConnection to
    setDeviceConnection, if this is done commands will be sent along
    the given serial port, this should ONLY be done if the camera is
    attached straight to a serial port on the computer this program is
    running on.

    The next two paragraphs describe how to get data back from the
    cameras, but this base class is set up so that by default it won't
    try to get data back and assumes you're not trying to do that.  If
    you are trying to get data back the important functions are
    packetHandler, robotPacketHandler and readPacket and you should
    read the docs on those.

    If the camera is attached to the robot (and you are thus using the
    first method described in the first paragraph) then the only way
    to get data back is to send an ArCommands::GETAUX, then set up a
    robotPacketHandler for the AUX id and have it call the
    packetHandler you set up in in the class.

    If the camera is attached to the serial port on the computer (and
    thus the second method described in the first paragraph was used)
    then its more complicated... the default way is to just pass in an
    ArDeviceConnection to setDeviceConnection and implement the
    readPacket method (which MUST not block), and every time through
    the robot loop readPacket (with the sensorInterpHandler) will be
    called and any packets will be given to the packetHandler (which
    you need to implement in your class) to be processed.  The other
    way to do this method is to pass both an ArDefaultConnection and
    false to setDeviceConnection, this means the camera will not be
    read at all by default, and you're on your own for reading the
    data in (ie like your own thread).
**/
class ArPTZ
{
public:
  AREXPORT ArPTZ(ArRobot *robot);
  /// Destructor
  AREXPORT virtual ~ArPTZ();

  /// Initializes the camera
  AREXPORT virtual bool init(void) = 0;

  /// Resets the camera
  /**
     This function will reset the camera to 0 0 pan tilt, and 0 zoom,
     on some cameras that can get out of sync it may need to do more,
     such as call init on it again.
   **/
  /*AREXPORT*/virtual void reset(void)
    { panTilt(0, 0); if (canZoom()) zoom(getMinZoom()); }
  /// Pans to the given degrees
  AREXPORT virtual bool pan(double degrees) = 0;
  /// Pans relative to current position by given degrees
  AREXPORT virtual bool panRel(double degrees) = 0;

  /// Tilts to the given degrees
  AREXPORT virtual bool tilt(double degrees) = 0;
  /// Tilts relative to the current position by given degrees
  AREXPORT virtual bool tiltRel(double degrees) = 0;

  /// Pans and tilts to the given degrees
  AREXPORT virtual bool panTilt(double degreesPan, double degreesTilt) = 0;
  /// Pans and tilts relatives to the current position by the given degrees
  AREXPORT virtual bool panTiltRel(double degreesPan, double degreesTilt) = 0;

  /// Returns true if camera can zoom (or rather, if it is controlled by this)
  AREXPORT virtual bool canZoom(void) const = 0;

  /// Zooms to the given value
  /*AREXPORT*/ virtual bool zoom(int zoomValue) {
	  MRPT_UNUSED_PARAM(zoomValue);
	  return false;
  }
  /// Zooms relative to the current value, by the given value
  /*AREXPORT*/ virtual bool zoomRel(int zoomValue) {
	  MRPT_UNUSED_PARAM(zoomValue);
	  return false;
  }

  /// The angle the camera was last told to pan to
  AREXPORT virtual double getPan(void) const = 0;
  /// The angle the camera was last told to tilt to
  AREXPORT virtual double getTilt(void) const = 0;
  /// The value the camera was last told to zoom to
  /*AREXPORT*/ virtual int getZoom(void) const { return 0; }

  /// If this driver can tell the real pan/tilt angle
  /*AREXPORT*/ virtual bool canGetRealPanTilt(void) const { return false; }
  // The angle the camera says its at (not needed and was confusing,
  //                                   just use getPan)
  // /*AREXPORT*/ virtual double getRealPan(void) const { return 0; }
  // The angle the camera says its at (not needed and was confusing,
  //                                   just use getTilt)
  ///*AREXPORT*/ virtual double getRealTilt(void) const { return 0; }

  /// If this driver can tell the real zoom
  /*AREXPORT*/ virtual bool canGetRealZoom(void) const { return false; }
  // The zoom the camera says its at (not needed and was confusing,
  //                                   just use getZoom)
  // /*AREXPORT*/ virtual double getRealZoom(void) const { return 0; }

  /// Gets the highest positive degree the camera can pan to
  AREXPORT virtual double getMaxPosPan(void) const = 0;
  /// Gets the lowest negative degree the camera can pan to
  AREXPORT virtual double getMaxNegPan(void) const = 0;
  /// Gets the highest positive degree the camera can tilt to
  AREXPORT virtual double getMaxPosTilt(void) const = 0;
  /// Gets the lowest negative degree the camera can tilt to
  AREXPORT virtual double getMaxNegTilt(void) const = 0;
  /// Gets the maximum value for the zoom on this camera
  /*AREXPORT*/ virtual int getMaxZoom(void) const { return 0; }
  /// Gets the lowest value for the zoom on this camera
  /*AREXPORT*/ virtual int getMinZoom(void) const { return 0; }
  /// Whether we can get the FOV (field of view) or not
  /*AREXPORT*/ virtual bool canGetFOV(void) { return false; }
  /// Gets the field of view at maximum zoom
  /*AREXPORT*/ virtual double getFOVAtMaxZoom(void) { return 0; }
  /// Gets the field of view at minimum zoom
  /*AREXPORT*/ virtual double getFOVAtMinZoom(void) { return 0; }

  /// Set gain on camera, range of 1-100.  Returns false if out of range
  /// or if you can't set the gain on the camera
  /*AREXPORT*/ virtual bool setGain(double gain) const {
	  MRPT_UNUSED_PARAM(gain);
	  return false;
  }
  /// Get the gain the camera is set to.  0 if not supported
  /*AREXPORT*/ virtual double getGain(double gain) const {
	  MRPT_UNUSED_PARAM(gain);
	  return 0;
  }
  /// If the driver can set gain on the camera, or not
  /*AREXPORT*/ virtual bool canSetGain(void) const { return false; }

  /// Set focus on camera, range of 1-100.  Returns false if out of range
  /// or if you can't set the focus on the camera
  /*AREXPORT*/ virtual bool setFocus(double focus) const {
	  MRPT_UNUSED_PARAM(focus);
	  return false;
  }

  /// Get the focus the camera is set to.  0 if not supported
  /*AREXPORT*/ virtual double getFocus(double focus) const {
	  MRPT_UNUSED_PARAM(focus);
	  return 0;
  }

  /// If the driver can set the focus on the camera, or not
  /*AREXPORT*/ virtual bool canSetFocus(void) const { return false; }

  /// Sets the device connection to be used by this PTZ camera, if set
  /// this camera will send commands via this connection, otherwise
  /// its via robot
  AREXPORT virtual bool setDeviceConnection(ArDeviceConnection *connection,
					    bool driveFromRobotLoop = true);
  /// Gets the device connection used by this PTZ camera
  AREXPORT virtual ArDeviceConnection *getDeviceConnection(void);
  /// Sets the aux port on the robot to be used to communicate with this device
  AREXPORT virtual bool setAuxPort(int auxPort);
  /// Gets the port the device is set to communicate on
  /*AREXPORT*/ virtual int getAuxPort(void) { return myAuxPort; }
  /// Reads a packet from the device connection, MUST NOT BLOCK
  /**
      This should read in a packet from the myConn connection and
      return a pointer to a packet if there was on to read in, or NULL
      if there wasn't one... this MUST not block if it is used with
      the default mode of being driven from the sensorInterpHandler,
      since that is on the robot loop.
      @return packet read in, or NULL if there was no packet read
   **/
  /*AREXPORT*/ virtual ArBasePacket *readPacket(void) { return NULL; }

  /// Sends a given packet to the camera (via robot or serial port, depending)
  AREXPORT virtual bool sendPacket(ArBasePacket *packet);
  /// Handles a packet that was read from the device
  /**
     This should work for the robot packet handler or for packets read
     in from readPacket (the joys of OO), but it can't deal with the
     need to check the id on robot packets, so you should check the id
     from robotPacketHandler and then call this one so that your stuff
     can be used by both robot and serial port connections.
     @param packet the packet to handle
     @return true if this packet was handled (ie this knows what it
     is), false otherwise
  **/
  /*AREXPORT*/ virtual bool packetHandler(ArBasePacket *packet) {
	  MRPT_UNUSED_PARAM(packet);
	  return false;
  }

  /// Handles a packet that was read by the robot
  /**
     This handles packets read in from the robot, this function should
     just check the ID of the robot packet and then return what
     packetHandler thinks of the packet.
     @param packet the packet to handle
     @return true if the packet was handled (ie this konws what it is),
     false otherwise
  **/
  AREXPORT virtual bool robotPacketHandler(ArRobotPacket *packet);

  /// Internal, attached to robot, inits the camera when robot connects
  AREXPORT virtual void connectHandler(void);
  /// Internal, for attaching to the robots sensor interp to read serial port
  AREXPORT virtual void sensorInterpHandler(void);
protected:
  ArRobot *myRobot;
  ArDeviceConnection *myConn;
  ArFunctorC<ArPTZ> myConnectCB;
  ArFunctorC<ArPTZ> mySensorInterpCB;
  int myAuxPort;
  ArCommands::Commands myAuxTxCmd;
  ArCommands::Commands myAuxRxCmd;
  ArRetFunctor1C<bool, ArPTZ, ArRobotPacket *> myRobotPacketHandlerCB;
};

#endif // ARPTZ_H
