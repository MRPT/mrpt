/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArRobotTypes.h"
#include "ArLog.h"

/** @cond INCLUDE_INTERNAL_ROBOT_PARAM_CLASSES */

// Generic robot class

AREXPORT ArRobotGeneric::ArRobotGeneric(const char *dir)
{
}

// AmigoBot robot class

AREXPORT ArRobotAmigo::ArRobotAmigo(const char *dir)
{
  sprintf(mySubClass, "amigo");
  myRobotRadius = 180;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1000;
  myDistConvFactor = 0.5083;
  myVelConvFactor = 0.6154;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.011;
  myVel2Divisor = 20;
  myRobotWidth = 279;
  myRobotLength = 330;
  myRobotLengthFront = 160;
  myRobotLengthRear = 170;

  myNumSonar = 8;
  internalSetSonar(0, 76, 100, 90);
  internalSetSonar(1, 125, 75, 41);
  internalSetSonar(2, 150, 30, 15);
  internalSetSonar(3, 150, -30, -15);
  internalSetSonar(4, 125, -75, -41);
  internalSetSonar(5, 76, -100, -90);
  internalSetSonar(6, -140, -58, -145);
  internalSetSonar(7, -140, 58, 145);

  myLaserPort[0] = '\0';
}

// AmigoBot robot class

AREXPORT ArRobotAmigoSh::ArRobotAmigoSh(const char *dir)
{
  sprintf(mySubClass, "amigo-sh");
  myRobotRadius = 180;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1000;
  myDistConvFactor = 1;
  myVelConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.011;
  myVel2Divisor = 20;
  myRobotWidth = 279;
  myRobotLength = 330;
  myRobotLengthFront = 160;
  myRobotLengthRear = 170;

  myNumSonar = 8;
  internalSetSonar(0, 70, 100, 90);
  internalSetSonar(1, 125, 75, 41);
  internalSetSonar(2, 144, 30, 15);
  internalSetSonar(3, 144, -30, -15);
  internalSetSonar(4, 120, -75, -41);
  internalSetSonar(5, 70, -100, -90);
  internalSetSonar(6, -146, -58, -145);
  internalSetSonar(7, -146, 58, 145);

  myLaserPort[0] = '\0';
}

// P2AT robot class

AREXPORT ArRobotP2AT::ArRobotP2AT(const char *dir)
{
  sprintf(mySubClass, "p2at");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300; 
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.32;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  myLaserX = 160;
  myLaserY = 7;
}

// P2AT8 robot class

AREXPORT ArRobotP2AT8::ArRobotP2AT8(const char *dir)
{
  sprintf(mySubClass, "p2at8");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.32;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  myLaserX = 160;
  myLaserY = 7;
}


// P2IT robot class

AREXPORT ArRobotP2IT::ArRobotP2IT(const char *dir)
{
  sprintf(mySubClass, "p2it");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.136;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0032;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);
  myLaserX = 160;
  myLaserY = 7;
}


// P2DX robot class

AREXPORT ArRobotP2DX::ArRobotP2DX(const char *dir)
{
  sprintf(mySubClass, "p2dx");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.84;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  myLaserX = 17;
  myLaserY = 8;
}

// P2DXe robot class

AREXPORT ArRobotP2DXe::ArRobotP2DXe(const char *dir)
{
  sprintf(mySubClass, "p2de");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.969;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  myLaserX = 17;
  myLaserY = 8;
}

// P2DF robot class

AREXPORT ArRobotP2DF::ArRobotP2DF(const char *dir)
{
  sprintf(mySubClass, "p2df");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = .0060;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  myLaserX = 17;
  myLaserY = 8;
}

// P2D8 robot class

AREXPORT ArRobotP2D8::ArRobotP2D8(const char *dir)
{
  sprintf(mySubClass, "p2d8");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  myLaserX = 18;
  myLaserY = 0;
}



// P2CE robot class

AREXPORT ArRobotP2CE::ArRobotP2CE(const char *dir)
{
  sprintf(mySubClass, "p2ce");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.826;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0057;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  myLaserPort[0] = '\0';
}


// P2PP robot class

AREXPORT ArRobotP2PP::ArRobotP2PP(const char *dir)
{
  sprintf(mySubClass, "p2pp");
  myRobotRadius = 300;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0060;
  myRobotWidth = 425;
  myRobotLength = 513;

  myTableSensingIR = true;
  myNewTableSensingIR = false;
  myFrontBumpers = true;
  myRearBumpers = true;

  myNumSonar = 24;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -20, 136, 90);
  internalSetSonar(9, 24, 119, 50);
  internalSetSonar(10, 58, 78, 30);
  internalSetSonar(11, 77, 27, 10);
  internalSetSonar(12, 77, -27, -10);
  internalSetSonar(13, 58, -78, -30);
  internalSetSonar(14, 24, -119, -50);
  internalSetSonar(15, -20, -136, -90);

  internalSetSonar(16, -157, -136, -90);
  internalSetSonar(17, -203, -119, -130);
  internalSetSonar(18, -237, -78, -150);
  internalSetSonar(19, -255, -27, -170);
  internalSetSonar(20, -255, 27, 170);
  internalSetSonar(21, -237, 78, 150);
  internalSetSonar(22, -203, 119, 130);
  internalSetSonar(23, -157, 136, 90);

  myLaserX = 16;
  myLaserY = 1;
}

// P2PB robot class

AREXPORT ArRobotP2PB::ArRobotP2PB(const char *dir)
{
  sprintf(mySubClass, "p2pb");
  myRobotRadius = 300;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.424;
  myRangeConvFactor = 0.268;
  myDiffConvFactor = 0.0056;
  myFrontBumpers = true;
  myRearBumpers = true;
  myRobotWidth = 425;
  myRobotLength = 513;
  
  myNumSonar = 24;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -20, 136, 90);
  internalSetSonar(9, 24, 119, 50);
  internalSetSonar(10, 58, 78, 30);
  internalSetSonar(11, 77, 27, 10);
  internalSetSonar(12, 77, -27, -10);
  internalSetSonar(13, 58, -78, -30);
  internalSetSonar(14, 24, -119, -50);
  internalSetSonar(15, -20, -136, -90);

  internalSetSonar(16, -157, -136, -90);
  internalSetSonar(17, -203, -119, -130);
  internalSetSonar(18, -237, -78, -150);
  internalSetSonar(19, -255, -27, -170);
  internalSetSonar(20, -255, 27, 170);
  internalSetSonar(21, -237, 78, 150);
  internalSetSonar(22, -203, 119, 130);
  internalSetSonar(23, -157, 136, 90);

  myLaserX = 17;
  myLaserY = 8;
}

// PerfPB robot class

AREXPORT ArRobotPerfPB::ArRobotPerfPB(const char *dir)
{
  sprintf(mySubClass, "perfpb");
  myRobotRadius = 340;
  myRobotDiagonal = 120; 
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.006;

  myRequestIOPackets = true;
  myTableSensingIR = true;
  myNewTableSensingIR = true;
  myFrontBumpers = true;
  myRearBumpers = true;
  myRobotWidth = 425;
  myRobotLength = 513;

  myNumIR = 4;
  internalSetIR(0, 1, 2, 333, -233);
  internalSetIR(1, 1, 2, 333, 233);
  internalSetIR(2, 1, 2, -2, -116);
  internalSetIR(3, 1, 2, -2, 116);

  myNumSonar = 32;

  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -20, 136, 90);
  internalSetSonar(9, 24, 119, 50);
  internalSetSonar(10, 58, 78, 30);
  internalSetSonar(11, 77, 27, 10);
  internalSetSonar(12, 77, -27, -10);
  internalSetSonar(13, 58, -78, -30);
  internalSetSonar(14, 24, -119, -50);
  internalSetSonar(15, -20, -136, -90);

  internalSetSonar(16, -157, -136, -90);
  internalSetSonar(17, -203, -119, -130);
  internalSetSonar(18, -237, -78, -150);
  internalSetSonar(19, -255, -27, -170);
  internalSetSonar(20, -255, 27, 170);
  internalSetSonar(21, -237, 78, 150);
  internalSetSonar(22, -203, 119, 130);
  internalSetSonar(23, -157, 136, 90);

  internalSetSonar(24, -191, -136, -90);
  internalSetSonar(25, -237, -119, -130);
  internalSetSonar(26, -271, -78, -150);
  internalSetSonar(27, -290, -27, -170);
  internalSetSonar(28, -290, 27, 170);
  internalSetSonar(29, -271, 78, 150);
  internalSetSonar(30, -237, 119, 130);
  internalSetSonar(31, -191, 136, 90);

  myLaserX = 21;
  myLaserY = 0;
}



AREXPORT ArRobotPion1M::ArRobotPion1M(const char *dir)
{
  sprintf(mySubClass, "pion1m");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myLaserPort[0] = '\0';

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}

AREXPORT ArRobotPsos1M::ArRobotPsos1M(const char *dir)
{
  sprintf(mySubClass, "psos1m");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myLaserPort[0] = '\0';

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}

AREXPORT ArRobotPsos43M::ArRobotPsos43M(const char *dir)
{
  sprintf(mySubClass, "psos43m");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myLaserPort[0] = '\0';

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}


// PionAT robot class

AREXPORT ArRobotPionAT::ArRobotPionAT(const char *dir)
{
  sprintf(mySubClass, "pionat");
  myRobotRadius = 330;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 500;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.07;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myLaserPort[0] = '\0';

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}


AREXPORT ArRobotPion1X::ArRobotPion1X(const char *dir)
{
  sprintf(mySubClass, "pion1x");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myLaserPort[0] = '\0';

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}

AREXPORT ArRobotPsos1X::ArRobotPsos1X(const char *dir)
{
  sprintf(mySubClass, "psos1x");
  myRobotRadius = 220;
  myRobotDiagonal = 90;
  myAbsoluteMaxRVelocity = 100;
  myAbsoluteMaxVelocity = 400;
  myHaveMoveCommand = 0;
  mySwitchToBaudRate = 0;

  myAngleConvFactor = 0.0061359;
  myDistConvFactor = 0.05066;
  myVelConvFactor = 2.5332;
  myRangeConvFactor = 0.1734;
  myDiffConvFactor = 1.0/300.0;
  myVel2Divisor = 4;

  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  
  myNumSonar = 7;
  internalSetSonar(0, 100, 100, 90);
  internalSetSonar(1, 120, 80, 30);
  internalSetSonar(2, 130, 40, 15);
  internalSetSonar(3, 130, 0, 0);
  internalSetSonar(4, 130, -40, -15);
  internalSetSonar(5, 120, -80, -30);
  internalSetSonar(6, 100, -100, -90);

  myLaserPort[0] = '\0';

  myTransVelMax = 400;
  myRotVelMax = 100;
  mySettableAccsDecs = false;
}


AREXPORT ArRobotMapper::ArRobotMapper(const char *dir)
{
  sprintf(mySubClass, "mappr");
  myRobotRadius = 180;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 0;
  myAbsoluteMaxVelocity = 0;
  myHaveMoveCommand = false;
  myDistConvFactor = 1.00;
  //myDistConvFactor = 6.77; // solid foam tires
  // myDistConvFactor = 7.39; // pneumatic tires at 40 psi
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .011;
  myGyroScaler = 1.626; // the default used on Pioneers
  myVelConvFactor = 0.615400;
  mySwitchToBaudRate = 0;
  mySettableAccsDecs = false;
  mySettableVelMaxes = false;
  
  myLaserPossessed = true;
  myLaserFlipped = false; // the normal configuration
  //myLaserFlipped = true; // for low-inverted and high configurations
  myLaserPowerControlled = false;
  myNumFrontBumpers = 0;
  myNumRearBumpers = 0;
  myLaserX = 312; // the normal laser low position
  //myLaserX = 237; // the high laser position
  myLaserY = 0;
  myLaserTh = 0;
}

// PowerBot robot class

AREXPORT ArRobotPowerBot::ArRobotPowerBot(const char *dir)
{

  sprintf(mySubClass, "powerbot");
  myRobotRadius = 550;
  myRobotDiagonal = 240; 
  myAbsoluteMaxRVelocity = 360;
  myAbsoluteMaxVelocity = 2000;
  myDistConvFactor = 0.5813;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .00373;
  myRobotWidth = 680;
  myRobotLength = 911;
  myRobotLengthFront = 369;
  myRobotLengthRear = 542;
  
  myFrontBumpers = true;
  myNumFrontBumpers = 7;
  myRearBumpers = true;
  myNumRearBumpers = 5;
  myNumSonar = 32;
  internalSetSonar(0, 152, 278, 90);
  internalSetSonar(1, 200, 267, 65);
  internalSetSonar(2, 241, 238, 45);
  internalSetSonar(3, 274, 200, 35);
  internalSetSonar(4, 300, 153, 25);
  internalSetSonar(5, 320, 96, 15);
  internalSetSonar(6, 332, 33, 5);
  internalSetSonar(7, 0, 0, -180);

  internalSetSonar(8, 332, -33, -5);
  internalSetSonar(9, 320, -96, -15);
  internalSetSonar(10, 300, -153, -25);
  internalSetSonar(11, 274, -200, -35);
  internalSetSonar(12, 241, -238, -45);
  internalSetSonar(13, 200, -267, -65);
  internalSetSonar(14, 152, -278, -90);
  internalSetSonar(15, 0, 0, -180);

  internalSetSonar(16, -298, -278, -90);
  internalSetSonar(17, -347, -267, -115);
  internalSetSonar(18, -388, -238, -135);
  internalSetSonar(19, -420, -200, -145);
  internalSetSonar(20, -447, -153, -155);
  internalSetSonar(21, -467, -96, -165);
  internalSetSonar(22, -478, -33, -175);
  internalSetSonar(23, 0, 0, -180);

  internalSetSonar(24, -478, 33, 175);
  internalSetSonar(25, -467, 96, 165);
  internalSetSonar(26, -447, 153, 155);
  internalSetSonar(27, -420, 200, 145);
  internalSetSonar(28, -388, 238, 135);
  internalSetSonar(29, -347, 267, 115);
  internalSetSonar(30, -298, 278, 90);

  sprintf(myLaserPort, "COM2");
  myLaserFlipped = true;
  myLaserX = 251;
  myLaserY = 0;
}

AREXPORT ArRobotP2D8Plus::ArRobotP2D8Plus(const char *dir)
{
  sprintf(mySubClass, "p2d8+");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  myLaserX = 18;
  myLaserY = 0;
}


AREXPORT ArRobotP2AT8Plus::ArRobotP2AT8Plus(const char *dir)
{
  sprintf(mySubClass, "p2at8+");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 0.465;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  myLaserX = 160;
  myLaserY = 7;
}

AREXPORT ArRobotP3AT::ArRobotP3AT(const char *dir)
{
  sprintf(mySubClass, "p3at");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 0.465;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  myLaserX = 160;
  myLaserY = 0;
}


AREXPORT ArRobotP3DX::ArRobotP3DX(const char *dir)
{
  sprintf(mySubClass, "p3dx");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);


  myLaserX = 18;
  myLaserY = 0;
}


AREXPORT ArRobotPerfPBPlus::ArRobotPerfPBPlus(const char *dir)
{
  sprintf(mySubClass, "perfpb+");
  myRobotRadius = 340;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myRequestIOPackets = true;
  myDistConvFactor = 0.485;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .006;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;
  
  myRequestIOPackets = true;
  myTableSensingIR = true;
  myNewTableSensingIR = true;
  myFrontBumpers = true;
  myRearBumpers = true;

  myNumIR = 4;
  internalSetIR(0, 1, 2, 333, -233);
  internalSetIR(1, 1, 2, 333, 233);
  internalSetIR(2, 1, 2, -2, -116);
  internalSetIR(3, 1, 2, -2, 116);

  myNumSonar = 32;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -20, 136, 90);
  internalSetSonar(9, 24, 119, 50);
  internalSetSonar(10, 58, 78, 30);
  internalSetSonar(11, 77, 27, 10);
  internalSetSonar(12, 77, -27, -10);
  internalSetSonar(13, 58, -78, -30);
  internalSetSonar(14, 24, -119, -50);
  internalSetSonar(15, -20, -136, -90);

  internalSetSonar(16, -157, -136, -90);
  internalSetSonar(17, -203, -119, -130);
  internalSetSonar(18, -237, -78, -150);
  internalSetSonar(19, -255, -27, -170);
  internalSetSonar(20, -255, 27, 170);
  internalSetSonar(21, -237, 78, 150);
  internalSetSonar(22, -203, 119, 130);
  internalSetSonar(23, -157, 136, 90);

  internalSetSonar(24, -191, -136, -90);
  internalSetSonar(25, -237, -119, -130);
  internalSetSonar(26, -271, -78, -150);
  internalSetSonar(27, -290, -27, -170);
  internalSetSonar(28, -290, 27, 170);
  internalSetSonar(29, -271, 78, 150);
  internalSetSonar(30, -237, 119, 130);
  internalSetSonar(31, -191, 136, 90);


  myLaserX = 21;
  myLaserY = 0;
}


AREXPORT ArRobotP3DXSH::ArRobotP3DXSH(const char *dir)
{
  sprintf(mySubClass, "p3dx-sh");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 511;
  myRobotLengthFront = 210;
  myRobotLengthRear = 301;

  myNumSonar = 16;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  myLaserX = 18;
  myLaserY = 0;
}


AREXPORT ArRobotP3ATSH::ArRobotP3ATSH(const char *dir)
{
  sprintf(mySubClass, "p3at-sh");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0034;
  myRobotWidth = 505;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  myLaserX = 125;
  myLaserY = 0;
}


AREXPORT ArRobotP3ATIWSH::ArRobotP3ATIWSH(const char *dir)
{
  sprintf(mySubClass, "p3atiw-sh");
  myRobotRadius = 500;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 300;
  myAbsoluteMaxVelocity = 1200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .0034;
  myRobotWidth = 490;
  myRobotLength = 626;
  myRobotLengthFront = 313;
  myRobotLengthRear = 313;

  myNumSonar = 16;
  internalSetSonar(0, 147, 136, 90);
  internalSetSonar(1, 193, 119, 50);
  internalSetSonar(2, 227, 79, 30);
  internalSetSonar(3, 245, 27, 10);
  internalSetSonar(4, 245, -27, -10);
  internalSetSonar(5, 227, -79, -30);
  internalSetSonar(6, 193, -119, -50);
  internalSetSonar(7, 147, -136, -90);

  internalSetSonar(8, -144, -136, -90);
  internalSetSonar(9, -189, -119, -130);
  internalSetSonar(10, -223, -79, -150);
  internalSetSonar(11, -241, -27, -170);
  internalSetSonar(12, -241, 27, 170);
  internalSetSonar(13, -223, 79, 150);
  internalSetSonar(14, -189, 119, 130);
  internalSetSonar(15, -144, 136, 90);

  myLaserX = 125;
  myLaserY = 0;
}


AREXPORT ArRobotPatrolBotSH::ArRobotPatrolBotSH(const char *dir)
{
  sprintf(mySubClass, "patrolbot-sh");
  myRobotRadius = 250;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = 0.0056;
  myRobotWidth = 425;
  myRobotLength = 510;
  myRobotLengthFront = 255;
  myRobotLengthRear = 255;

  myFrontBumpers = true;
  myNumFrontBumpers = 6;
  myRearBumpers = true;
  myNumRearBumpers = 6;

  myNumSonar = 16;
  internalSetSonar(0, 83, 229, 90);
  internalSetSonar(1, 169, 202, 55);
  internalSetSonar(2, 232, 134, 30);
  internalSetSonar(3, 263, 46, 10);
  internalSetSonar(4, 263, -46, -10);
  internalSetSonar(5, 232, -134, -30);
  internalSetSonar(6, 169, -202, -55);
  internalSetSonar(7, 83, -229, -90);

  internalSetSonar(8, -83, -229, -90);
  internalSetSonar(9, -169, -202, -125);
  internalSetSonar(10, -232, -134, -150);
  internalSetSonar(11, -263, -46, -170);
  internalSetSonar(12, -263, 46, 170);
  internalSetSonar(13, -232, 134, 150);
  internalSetSonar(14, -169, 202, 125);
  internalSetSonar(15, -83, 229, 90);

  myLaserFlipped = true;
  myLaserPowerControlled = true;
  myLaserX = 37;
  myLaserY = 0;
  myLaserTh = 0.0;
  strcpy(myLaserIgnore, "73 74 75 -73 -74 -75");

  myRequestIOPackets = true;
}


AREXPORT ArRobotPeopleBotSH::ArRobotPeopleBotSH(const char *dir)
{
  sprintf(mySubClass, "peoplebot-sh");
  myRobotRadius = 340;
  myRobotDiagonal = 120;
  myAbsoluteMaxRVelocity = 500;
  myAbsoluteMaxVelocity = 2200;
  myRequestIOPackets = true;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .006;
  myRobotWidth = 425;
  myRobotLength = 513;
  
  myRequestIOPackets = true;
  myTableSensingIR = true;
  myNewTableSensingIR = true;
  myFrontBumpers = true;
  myRearBumpers = true;

  myNumIR = 4;
  internalSetIR(0, 1, 2, 333, -233);
  internalSetIR(1, 1, 2, 333, 233);
  internalSetIR(2, 1, 2, -2, -116);
  internalSetIR(3, 1, 2, -2, 116);

  myNumSonar = 32;
  internalSetSonar(0, 69, 136, 90);
  internalSetSonar(1, 114, 119, 50);
  internalSetSonar(2, 148, 78, 30);
  internalSetSonar(3, 166, 27, 10);
  internalSetSonar(4, 166, -27, -10);
  internalSetSonar(5, 148, -78, -30);
  internalSetSonar(6, 114, -119, -50);
  internalSetSonar(7, 69, -136, -90);

  internalSetSonar(8, -157, -136, -90);
  internalSetSonar(9, -203, -119, -130);
  internalSetSonar(10, -237, -78, -150);
  internalSetSonar(11, -255, -27, -170);
  internalSetSonar(12, -255, 27, 170);
  internalSetSonar(13, -237, 78, 150);
  internalSetSonar(14, -203, 119, 130);
  internalSetSonar(15, -157, 136, 90);

  internalSetSonar(16, -20, 136, 90);
  internalSetSonar(17, 24, 119, 50);
  internalSetSonar(18, 58, 78, 30);
  internalSetSonar(19, 77, 27, 10);
  internalSetSonar(20, 77, -27, -10);
  internalSetSonar(21, 58, -78, -30);
  internalSetSonar(22, 24, -119, -50);
  internalSetSonar(23, -20, -136, -90);

  internalSetSonar(24, -191, -136, -90);
  internalSetSonar(25, -237, -119, -130);
  internalSetSonar(26, -271, -78, -150);
  internalSetSonar(27, -290, -27, -170);
  internalSetSonar(28, -290, 27, 170);
  internalSetSonar(29, -271, 78, 150);
  internalSetSonar(30, -237, 119, 130);
  internalSetSonar(31, -191, 136, 90);

  myLaserX = 21;
  myLaserY = 0;
}


AREXPORT ArRobotPowerBotSH::ArRobotPowerBotSH(const char *dir)
{

  sprintf(mySubClass, "powerbot-sh");
  myRobotRadius = 550;
  myRobotDiagonal = 240; 
  myAbsoluteMaxRVelocity = 360;
  myAbsoluteMaxVelocity = 2000;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .00373;
  myRobotWidth = 680;
  myRobotLength = 911;
  myRobotLengthFront = 369;
  myRobotLengthRear = 542;

  myFrontBumpers = true;
  myNumFrontBumpers = 7;
  myRearBumpers = true;
  myNumRearBumpers = 5;
  myNumSonar = 32;
  internalSetSonar(0, 152, 278, 90);
  internalSetSonar(1, 200, 267, 65);
  internalSetSonar(2, 241, 238, 45);
  internalSetSonar(3, 274, 200, 35);
  internalSetSonar(4, 300, 153, 25);
  internalSetSonar(5, 320, 96, 15);
  internalSetSonar(6, 332, 33, 5);
  internalSetSonar(7, 0, 0, -180);

  internalSetSonar(8, 332, -33, -5);
  internalSetSonar(9, 320, -96, -15);
  internalSetSonar(10, 300, -153, -25);
  internalSetSonar(11, 274, -200, -35);
  internalSetSonar(12, 241, -238, -45);
  internalSetSonar(13, 200, -267, -65);
  internalSetSonar(14, 152, -278, -90);
  internalSetSonar(15, 0, 0, -180);

  internalSetSonar(16, -298, -278, -90);
  internalSetSonar(17, -347, -267, -115);
  internalSetSonar(18, -388, -238, -135);
  internalSetSonar(19, -420, -200, -145);
  internalSetSonar(20, -447, -153, -155);
  internalSetSonar(21, -467, -96, -165);
  internalSetSonar(22, -478, -33, -175);
  internalSetSonar(23, 0, 0, -180);

  internalSetSonar(24, -478, 33, 175);
  internalSetSonar(25, -467, 96, 165);
  internalSetSonar(26, -447, 153, 155);
  internalSetSonar(27, -420, 200, 145);
  internalSetSonar(28, -388, 238, 135);
  internalSetSonar(29, -347, 267, 115);
  internalSetSonar(30, -298, 278, 90);
  internalSetSonar(31, 0, 0, -180);

  sprintf(myLaserPort, "COM2");
  myLaserFlipped = true;
  myLaserX = 251;
  myLaserY = 0;
}


AREXPORT ArRobotWheelchairSH::ArRobotWheelchairSH(const char *dir)
{
  sprintf(mySubClass, "wheelchair-sh");
  myRobotRadius = 550;
  myRobotDiagonal = 300; 
  myAbsoluteMaxRVelocity = 360;
  myAbsoluteMaxVelocity = 2000;
  myDistConvFactor = 1.0;
  myRangeConvFactor = 1.0;
  myDiffConvFactor = .00373;
  myRobotWidth = 680;
  myRobotLength = 1340;
  
  myFrontBumpers = true;
  myNumFrontBumpers = 4;
  myRearBumpers = true;
  myNumRearBumpers = 3;
  myNumSonar = 0;

  sprintf(myLaserPort, "COM2");
  myLaserPossessed = true;
  myLaserFlipped = true;
  myLaserPowerControlled = true;
  myLaserX = -418;
  myLaserY = 0;

  mySettableAccsDecs = true;
  mySettableVelMaxes = false;
  myTransVelMax = 0;
  myRotVelMax = 0;
  myTransAccel = 0;
  myTransDecel = 0;
  myRotAccel = 0;
  myRotDecel = 0;
}

/** @endcond INCLUDE_INTERNAL_ROBOT_PARAM_CLASSES */
