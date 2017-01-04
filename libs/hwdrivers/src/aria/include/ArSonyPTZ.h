/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARSONYPTZ_H
#define ARSONYPTZ_H

#include "ariaTypedefs.h"
#include "ArBasePacket.h"
#include "ArPTZ.h"

/// A class for for making commands to send to the sony
/** There are only two functioning ways to put things into this packet, you
    MUST use thse, if you use anything else your commands won't work.  You 
    must use uByteToBuf and byte2ToBuf.  
*/
class ArSonyPacket: public ArBasePacket
{
public:
  /// Constructor
  AREXPORT ArSonyPacket(ArTypes::UByte2 bufferSize = 15);
  AREXPORT virtual ~ArSonyPacket();
  
  AREXPORT virtual void uByteToBuf(ArTypes::UByte val);
  AREXPORT virtual void byte2ToBuf(ArTypes::Byte2 val);
  /// This is a new function, read the details before you try to use it
  AREXPORT void byte2ToBufAtPos(ArTypes::Byte2 val, ArTypes::UByte2 pose);
};

class ArRobot;

/// A class to use the sony pan tilt zoom unit

class ArSonyPTZ : public ArPTZ
{
public:
  AREXPORT ArSonyPTZ(ArRobot *robot);
  AREXPORT virtual ~ArSonyPTZ();
  
  AREXPORT virtual bool init(void);
  AREXPORT virtual bool pan(double degrees);
  AREXPORT virtual bool panRel(double degrees);
  AREXPORT virtual bool tilt(double degrees);
  AREXPORT virtual bool tiltRel(double degrees);
  AREXPORT virtual bool panTilt(double degreesPan, double degreesTilt);
  AREXPORT virtual bool panTiltRel(double degreesPan, double degreesTilt);
  /*AREXPORT*/ virtual bool canZoom(void) const { return true; }
  AREXPORT virtual bool zoom(int zoomValue);
  AREXPORT virtual bool zoomRel(int zoomValue);
  /*AREXPORT*/ virtual double getPan(void) const { return myPan; }
  /*AREXPORT*/ virtual double getTilt(void) const { return myTilt; }
  /*AREXPORT*/ virtual int getZoom(void) const { return myZoom; }
  /*AREXPORT*/ virtual double getMaxPosPan(void) const { return 90; }
  /*AREXPORT*/ virtual double getMaxNegPan(void) const { return -90; }
  /*AREXPORT*/ virtual double getMaxPosTilt(void) const { return 30; }
  /*AREXPORT*/ virtual double getMaxNegTilt(void) const { return -30; }
  /*AREXPORT*/ virtual int getMaxZoom(void) const { return 1024; }
  /*AREXPORT*/ virtual int getMinZoom(void) const { return 0; }

  /*AREXPORT*/ virtual bool canGetFOV(void) { return true; }
  /// Gets the field of view at maximum zoom
  /*AREXPORT*/ virtual double getFOVAtMaxZoom(void) { return 4.4; }
  /// Gets the field of view at minimum zoom
  /*AREXPORT*/ virtual double getFOVAtMinZoom(void) { return 48.8; }

  AREXPORT bool backLightingOn(void);
  AREXPORT bool backLightingOff(void);
  //AREXPORT bool packetHandler(ArRobotPacket *packet);
  enum {
    MAX_PAN = 95, ///< maximum degrees the unit can pan (either direction)
    MAX_TILT = 25, ///< maximum degrees the unit can tilt (either direction)
    MIN_ZOOM = 0, ///< minimum value for zoom
    MAX_ZOOM = 1023 ///< maximum value for zoom
  };
protected:
  void initializePackets(void);
  ArRobot *myRobot;
  double myPan;
  double myTilt;
  int myZoom;
  double myDegToTilt;
  double myDegToPan;
  ArSonyPacket myPacket;
  ArSonyPacket myZoomPacket; 
  ArSonyPacket myPanTiltPacket;
};

#endif // ARSONYPTZ_H
