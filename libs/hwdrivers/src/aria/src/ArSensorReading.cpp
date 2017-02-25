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
#include "ArSensorReading.h"
#include "ariaUtil.h"

/**
   @param xPos the x position of the sensor on the robot (mm)
   @param yPos the y position of the sensor on the robot (mm)
   @param thPos the heading of the sensor on the robot (deg)
*/
ArSensorReading::ArSensorReading(double xPos, double yPos, double thPos)
{
  myRange = 5000;
  myCounterTaken = 0;
  myReading.setPose(-1, -1);
  myReadingTaken.setPose(-1, -1, -1);
  resetSensorPosition(xPos, yPos, thPos, true);
  myExtraInt = 0;
  myAdjusted = false;
}

AREXPORT ArSensorReading::ArSensorReading(const ArSensorReading & reading)
{
  myCounterTaken = reading.myCounterTaken;
  myReading = reading.myReading;
  myLocalReading = reading.myLocalReading;  
  myReadingTaken = reading.myReadingTaken;
  myEncoderPoseTaken = reading.myEncoderPoseTaken;
  mySensorPos = reading.mySensorPos;
  mySensorCos = reading.mySensorCos;
  mySensorSin = reading.mySensorSin;
  myDistToCenter = reading.myDistToCenter;
  myAngleToCenter = reading.myAngleToCenter;
  myRange = reading.myRange;
  myTimeTaken = reading.myTimeTaken;
  myIgnoreThisReading = reading.myIgnoreThisReading;
  myExtraInt = reading.myExtraInt;  
  myAdjusted = reading.myAdjusted;
}

AREXPORT ArSensorReading &ArSensorReading::operator=(
        const ArSensorReading &reading)
{
  if (this != &reading)
  {
    myCounterTaken = reading.myCounterTaken;
    myReading = reading.myReading;
    myLocalReading = reading.myLocalReading;  
    myReadingTaken = reading.myReadingTaken;
    myEncoderPoseTaken = reading.myEncoderPoseTaken;
    mySensorPos = reading.mySensorPos;
    mySensorCos = reading.mySensorCos;
    mySensorSin = reading.mySensorSin;
    myDistToCenter = reading.myDistToCenter;
    myAngleToCenter = reading.myAngleToCenter;
    myRange = reading.myRange;
    myTimeTaken = reading.myTimeTaken;
    myIgnoreThisReading = reading.myIgnoreThisReading;
    myExtraInt = reading.myExtraInt;
    myAdjusted = reading.myAdjusted;
  }
  return *this;
}



ArSensorReading::~ArSensorReading()
{
}


/**
   @param range the distance from the sensor to the sensor return (mm)
   @param robotPose the robot's pose when the reading was taken
   @param encoderPose the robot's encoder pose when the reading was taken
   @param trans the transform from local coords to global coords
   @param counter the counter from the robot when the sensor reading was taken
   @param timeTaken the time the reading was taken
   @param ignoreThisReading if this reading should be ignored or not
*/
AREXPORT void ArSensorReading::newData(int range, ArPose robotPose,
				       ArPose encoderPose, ArTransform trans, 
				       unsigned int counter,
				       ArTime timeTaken,
				       bool ignoreThisReading, int extraInt)
{
  // TODO calculate the x and y position of the sensor
  double rx, ry;
  myRange = range;
  myCounterTaken = counter;
  myReadingTaken = robotPose;
  myEncoderPoseTaken = encoderPose;
  rx = getSensorX() + myRange * mySensorCos;
  ry = getSensorY() + myRange * mySensorSin;
  myLocalReading.setPose(rx, ry);
  myReading = trans.doTransform(myLocalReading);
  myTimeTaken = timeTaken;
  myIgnoreThisReading = ignoreThisReading;
  myExtraInt = extraInt;
  myAdjusted = false;
}

/**
   @param sx the coords of the sensor return relative to sensor (mm)
   @param sy the coords of the sensor return relative to sensor (mm)
   @param robotPose the robot's pose when the reading was taken
   @param encoderPose the robot's encoder pose when the reading was taken
   @param th the heading of the robot when the sensor reading was taken (deg)
   @param counter the counter from the robot when the sensor reading was taken
   @param timeTaken the time the reading was taken
   @param ignoreThisReading if this reading should be ignored or not
*/
AREXPORT void ArSensorReading::newData(int sx, int sy, ArPose robotPose,
				       ArPose encoderPose, ArTransform trans, 
				       unsigned int counter, ArTime timeTaken,
				       bool ignoreThisReading, int extraInt)
{
  // TODO calculate the x and y position of the sensor
  double rx, ry;
  myRange = (int)sqrt((double)(sx*sx + sy*sy));
  myCounterTaken = counter;
  myReadingTaken = robotPose;
  myEncoderPoseTaken = encoderPose;
  rx = getSensorX() + sx;
  ry = getSensorY() + sy;
  myLocalReading.setPose(rx, ry);
  myReading = trans.doTransform(myLocalReading);
  myTimeTaken = timeTaken;
  myIgnoreThisReading = ignoreThisReading;
  myExtraInt = extraInt;
  myAdjusted = false;
}


/**
   @param xPos the x position of the sensor on the robot (mm)
   @param yPos the y position of the sensor on the robot (mm)
   @param thPos the heading of the sensor on the robot (deg)
*/
AREXPORT void ArSensorReading::resetSensorPosition(double xPos, double yPos, 
						   double thPos, 
						   bool forceComputation)
{
  // if its the same position and we're not forcing, just bail
  if (!forceComputation && fabs(thPos - mySensorPos.getTh()) < .00001 &&
      xPos == mySensorPos.getX() && yPos == mySensorPos.getY())
    return;
      
  mySensorPos.setPose(xPos, yPos, thPos);
  myDistToCenter = sqrt(xPos * xPos + yPos * yPos);
  myAngleToCenter = ArMath::atan2(yPos, xPos);
  mySensorCos = ArMath::cos(thPos);
  mySensorSin = ArMath::sin(thPos);
  //printf("xpose %d ypose %d thpose %d disttoC %.1f angletoC %.1f\n",
  //xPos, yPos, thPos, myDistToCenter, myAngleToCenter);
}

/**
   @param trans the transform to apply to the reading and where the reading was taken
*/
AREXPORT void ArSensorReading::applyTransform(ArTransform trans)
{
  myReading = trans.doTransform(myReading);
  myReadingTaken = trans.doTransform(myReadingTaken);
}

/**
   @param trans the transform to apply to the encoder pose taken
*/
AREXPORT void ArSensorReading::applyEncoderTransform(ArTransform trans)
{
  myEncoderPoseTaken = trans.doTransform(myEncoderPoseTaken);
}
