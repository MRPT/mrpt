/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <stdarg.h>

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArSickLogger.h"
#include "ArRobot.h"
#include "ArSick.h"
#include "ArJoyHandler.h"
#include "ArRobotJoyHandler.h"
#include "ariaInternal.h"

/**
   Make sure you have called ArSick::configure or
   ArSick::configureShort on your laser before you make this class

   @param robot The robot to attach to

   @param sick the laser to log from

   @param distDiff the distance traveled at which to take a new reading

   @param degDiff the degrees turned at which to take a new reading

   @param fileName the file name in which to put the log

   @param addGoals whether to add goals automatically or... if true
   then the sick logger puts hooks into places it needs this to
   happen, into any keyhandler thats around (for a keypress of G), it
   pays attention to the flag bit of the robot, and it puts in a
   button press callback for the joyhandler passed in (if any)
**/
AREXPORT ArSickLogger::ArSickLogger(ArRobot *robot, ArSick *sick,
				    double distDiff, double degDiff,
				    const char *fileName, bool addGoals,
				    ArJoyHandler *joyHandler,
				    const char *baseDirectory,
				    bool useReflectorValues,
				    ArRobotJoyHandler *robotJoyHandler) :
  mySectors(18),
  myTaskCB(this, &ArSickLogger::robotTask),
  myGoalKeyCB(this, &ArSickLogger::goalKeyCallback),
  myLoopPacketHandlerCB(this, &ArSickLogger::loopPacketHandler)
{
  ArKeyHandler *keyHandler;

  ArSick::Degrees degrees;
  ArSick::Increment increment;
  double deg, incr;

  myOldReadings = false;
  myNewReadings = true;
  myUseReflectorValues = useReflectorValues;
  myWrote = false;
  myRobot = robot;
  mySick = sick;
  if (baseDirectory != NULL && strlen(baseDirectory) > 0)
    myBaseDirectory = baseDirectory;
  else
    myBaseDirectory = "";
  std::string realFileName;
  if (fileName[0] == '/' || fileName[0] == '\\')
  {
    realFileName = fileName;
  }
  else
  {
    realFileName = myBaseDirectory;
    realFileName += fileName;
  }
  myFileName = realFileName;

  myFile = fopen(realFileName.c_str(), "w+");
  degrees = mySick->getDegrees();
  increment = mySick->getIncrement();
  if (degrees == ArSick::DEGREES180)
    deg = 180;
  else
    deg = 100;
  if (increment == ArSick::INCREMENT_ONE)
    incr = 1;
  else
    incr = .5;
  if (myFile != NULL)
  {
    const ArRobotParams *params;
    params = robot->getRobotParams();
    fprintf(myFile, "LaserOdometryLog\n");
    fprintf(myFile, "#Created by ARIA's ArSickLogger\n");
    fprintf(myFile, "version: 2\n");
    fprintf(myFile, "sick1pose: %d %d %.2f\n", params->getLaserX(),
	    params->getLaserY(), params->getLaserTh());
    fprintf(myFile, "sick1conf: %d %d %d\n",
	    ArMath::roundInt(0.0 - deg / 2.0),
	    ArMath::roundInt(deg / 2.0), ArMath::roundInt(deg / incr + 1.0));
  }
  else
    ArLog::log(ArLog::Terse, "ArSickLogger cannot write to file %s",
	       myFileName.c_str());

  myDistDiff = distDiff;
  myDegDiff = degDiff;
  myFirstTaken = false;
  myScanNumber = 0;
  myLastVel = 0;
  myStartTime.setToNow();
  myRobot->addUserTask("Sick Logger", 1, &myTaskCB);

  char uCFileName[15];
  strncpy(uCFileName, fileName, 14);
  uCFileName[14] = '\0';
  myRobot->comStr(94, uCFileName);

  myLoopPacketHandlerCB.setName("ArSickLogger");
  myRobot->addPacketHandler(&myLoopPacketHandlerCB);

  myAddGoals = addGoals;
  myJoyHandler = joyHandler;
  myRobotJoyHandler = robotJoyHandler;
  myTakeReadingExplicit = false;
  myAddGoalExplicit = false;
  myAddGoalKeyboard = false;
  myLastAddGoalKeyboard = false;
  myLastJoyButton = false;
  myLastRobotJoyButton = false;
  myFirstGoalTaken = false;
  myNumGoal = 1;
  myLastLoops = 0;
  // only add goals from the keyboard if there's already a keyboard handler
  if (myAddGoals && (keyHandler = Aria::getKeyHandler()) != NULL)
  {
    // now that we have a key handler, add our keys as callbacks, print out big
    // warning messages if they fail
    if (!keyHandler->addKeyHandler('g', &myGoalKeyCB))
      ArLog::log(ArLog::Terse, "The key handler already has a key for g, sick logger goal handling will not work correctly.");
    if (!keyHandler->addKeyHandler('G', &myGoalKeyCB))
      ArLog::log(ArLog::Terse, "The key handler already has a key for g, sick logger goal handling will not work correctly.");
  }
}

AREXPORT ArSickLogger::~ArSickLogger()
{
  myRobot->remUserTask(&myTaskCB);
  myRobot->remPacketHandler(&myLoopPacketHandlerCB);
  myRobot->comStr(94, "");
  if (myFile != NULL)
  {
    fprintf(myFile, "# End of log\n");
    fclose(myFile);
  }
}

AREXPORT bool ArSickLogger::loopPacketHandler(ArRobotPacket *packet)
{
  unsigned char loops;
  if (packet->getID() != 0x96)
    return false;
  loops = packet->bufToUByte();
  unsigned char bit;
  int num;
  if (loops != myLastLoops)
  {
    for (bit = 1, num = 1; num <= 8; bit *= 2, num++)
    {
      if ((loops & bit) && !(myLastLoops & bit))
      {
	addTagToLog("loop: start %d", num);
	ArLog::log(ArLog::Normal, "Starting loop %d", num);
      }
      else if (!(loops & bit) && (myLastLoops & bit))
      {
	addTagToLog("loop: end %d", num);
	ArLog::log(ArLog::Normal, "Ending loop %d", num);
      }
    }
  }
  myLastLoops = loops;
  return true;
}

/**
   The robot MUST be locked before you call this function, so that
   this function is not adding to a list as the robotTask is using it.

   This function takes the given tag and puts it into the log file
   along with a tag as to where the robot was and when in the mapping
   it was
**/
AREXPORT void ArSickLogger::addTagToLogPlain(const char *str)
{
  myTags.push_back(str);
}

/**
   The robot MUST be locked before you call this function, so that
   this function is not adding to a list as the robotTask is using it.

   This function takes the given tag and puts it into the log file
   along with a tag as to where the robot was and when in the mapping
   it was
**/
AREXPORT void ArSickLogger::addTagToLog(const char *str, ...)
{
  char buf[2048];
  va_list ptr;
  va_start(ptr, str);
  vsprintf(buf, str, ptr);
  addTagToLogPlain(buf);
  va_end(ptr);
}


/**
   The robot MUST be locked before you call this function, so that
   this function is not adding to a list as the robotTask is using it.

   This function takes the given tag and puts it into the log file by
   itself
**/

AREXPORT void ArSickLogger::addInfoToLogPlain(const char *str)
{
  myInfos.push_back(str);
}

/**
   The robot MUST be locked before you call this function, so that
   this function is not adding to a list as the robotTask is using it.

   This function takes the given tag and puts it into the log file by
   itself
**/
AREXPORT void ArSickLogger::addInfoToLog(const char *str, ...)
{
  char buf[2048];
  va_list ptr;
  va_start(ptr, str);
  vsprintf(buf, str, ptr);
  addInfoToLogPlain(buf);
  va_end(ptr);
}

void ArSickLogger::goalKeyCallback(void)
{
  myAddGoalKeyboard = true;
}

void ArSickLogger::internalAddGoal(void)
{
  bool joyButton;
  bool robotJoyButton;

  // this check is for if we're not adding goals return... but if
  // we're not adding goals and one was requested explicitly then add
  // that one
  if (!myAddGoals && !myAddGoalExplicit)
    return;

  if (myJoyHandler != NULL)
    joyButton = (myJoyHandler->getButton(2) ||
		   myJoyHandler->getButton(3) ||
		   myJoyHandler->getButton(4));
  else
    joyButton = (myRobot->getFlags() & ArUtil::BIT9);

  if (myRobotJoyHandler != NULL)
    robotJoyButton = myRobotJoyHandler->getButton2();
  else
    robotJoyButton = false;

  // see if we want to add a goal... note that if the button is pushed
  // it must have been unpushed at one point to cause the goal to
  // trigger
  if (myRobot->isConnected() &&
      (myAddGoalExplicit ||
       (myAddGoalKeyboard && !myLastAddGoalKeyboard) ||
       (joyButton && !myLastJoyButton) ||
       (robotJoyButton && !myLastRobotJoyButton)))
  {
    myFirstGoalTaken = true;
    myAddGoalExplicit = false;
    myLastGoalTakenTime.setToNow();
    myLastGoalTakenPose = myRobot->getEncoderPose();
    // call addTagToLog not do it directly so we get additional info
    // needed
    addTagToLog("cairn: GoalWithHeading \"\" ICON_GOALWITHHEADING \"goal%d\"", myNumGoal);
    ArLog::log(ArLog::Normal, "Goal %d taken", myNumGoal);
    myNumGoal++;
  }
  myLastAddGoalKeyboard = myAddGoalKeyboard;
  myLastJoyButton = joyButton;
  myLastRobotJoyButton = robotJoyButton;

  // reset this here for if they held the key down a little, so it
  // gets reset and doesn't hit multiple goals
  myAddGoalKeyboard = false;
}

void ArSickLogger::internalWriteTags(void)
{
  time_t msec;

  // now put the tags into the file
  while (myInfos.size() > 0)
  {
    if (myFile != NULL)
    {
      myWrote = true;
      fprintf(myFile, "%s\n", (*myInfos.begin()).c_str());
    }
    myInfos.pop_front();
  }


  // now put the tags into the file
  while (myTags.size() > 0)
  {
    if (myFile != NULL)
    {
      myWrote = true;
      msec = myStartTime.mSecSince();
      fprintf(myFile, "time: %ld.%ld\n", msec / 1000, msec % 1000);
      internalPrintPos(myRobot->getEncoderPose());
      fprintf(myFile, "%s\n", (*myTags.begin()).c_str());
    }
    myTags.pop_front();
  }
}

void ArSickLogger::internalTakeReading(void)
{
  const std::list<ArSensorReading *> *readings;
  std::list<ArSensorReading *>::const_iterator it;
  std::list<ArSensorReading *>::const_reverse_iterator rit;
  ArPose poseTaken;
  time_t msec;
  ArSensorReading *reading;
  bool usingAdjustedReadings;

  // we take readings in any of the following cases if we haven't
  // taken one yet or if we've been explicitly told to take one or if
  // we've gone further than myDistDiff if we've turned more than
  // myDegDiff if we've switched sign on velocity and gone more than
  // 50 mm (so it doesn't oscilate and cause us to trigger)

  if (myRobot->isConnected() && (!myFirstTaken || myTakeReadingExplicit ||
      myLast.findDistanceTo(myRobot->getEncoderPose()) > myDistDiff ||
      fabs(ArMath::subAngle(myLast.getTh(),
			    myRobot->getEncoderPose().getTh())) > myDegDiff ||
      (( (myLastVel < 0 && myRobot->getVel() > 0) ||
	(myLastVel > 0 && myRobot->getVel() < 0)) &&
       myLast.findDistanceTo(myRobot->getEncoderPose()) > 50)))
  {
    myWrote = true;
    mySick->lockDevice();
    /// use the adjusted raw readings if we can, otherwise just use
    /// the raw readings like before
    if ((readings = mySick->getAdjustedRawReadings()) != NULL)
    {
      usingAdjustedReadings = true;
    }
    else
    {
      usingAdjustedReadings = false;
      readings = mySick->getRawReadings();
    }
    if (readings == NULL || (it = readings->begin()) == readings->end() ||
	myFile == NULL)
    {
      mySick->unlockDevice();
      return;
    }
    myTakeReadingExplicit = false;
    myScanNumber++;
    if (usingAdjustedReadings)
      ArLog::log(ArLog::Normal,
		 "Taking adjusted readings from the %d laser values",
		 readings->size());
    else
      ArLog::log(ArLog::Normal,
		 "Taking readings from the %d laser values",
		 readings->size());
    myFirstTaken = true;
    myLast = myRobot->getEncoderPose();
    poseTaken = (*readings->begin())->getEncoderPoseTaken();
    myLastVel = myRobot->getVel();
    msec = myStartTime.mSecSince();
    fprintf(myFile, "scan1Id: %d\n", myScanNumber);
    fprintf(myFile, "time: %ld.%ld\n", msec / 1000, msec % 1000);
    /* ScanStudio isn't using these yet so don't print them
      fprintf(myFile, "velocities: %.2f %.2f\n", myRobot->getRotVel(),
      myRobot->getVel());*/
    internalPrintPos(poseTaken);

    if (myUseReflectorValues)
    {
      fprintf(myFile, "reflector1: ");

      if (!mySick->isLaserFlipped())
      {
	// make sure that the list is in increasing order
	for (it = readings->begin(); it != readings->end(); it++)
	{
	  reading = (*it);
	  if (!reading->getIgnoreThisReading())
	    fprintf(myFile, "%d ", reading->getExtraInt());
	  else
	    fprintf(myFile, "0 ");
	}
      }
      else
      {
	for (rit = readings->rbegin(); rit != readings->rend(); rit++)
	{
	  reading = (*rit);
	  if (!reading->getIgnoreThisReading())
	    fprintf(myFile, "%d ", reading->getExtraInt());
	  else
	    fprintf(myFile, "0 ");
	}
      }
      fprintf(myFile, "\n");
    }
    /**
       Note that the the sick1: or scan1: must be the last thing in
       that timestamp, ie that you should put any other data before
       it.
     **/
    if (myOldReadings)
    {
      fprintf(myFile, "sick1: ");

      if (!mySick->isLaserFlipped())
      {
	// make sure that the list is in increasing order
	for (it = readings->begin(); it != readings->end(); it++)
	{
	  reading = (*it);
	  fprintf(myFile, "%d ", reading->getRange());
	}
      }
      else
      {
	for (rit = readings->rbegin(); rit != readings->rend(); rit++)
	{
	  reading = (*rit);
	  fprintf(myFile, "%d ", reading->getRange());
	}
      }
      fprintf(myFile, "\n");
    }
    if (myNewReadings)
    {
      fprintf(myFile, "scan1: ");

      if (!mySick->isLaserFlipped())
      {
	// make sure that the list is in increasing order
	for (it = readings->begin(); it != readings->end(); it++)
	{
	  reading = (*it);
	  if (!reading->getIgnoreThisReading())
	    fprintf(myFile, "%.0f %.0f  ",
		    reading->getLocalX() - mySick->getSensorPositionX(),
		    reading->getLocalY() - mySick->getSensorPositionY());
	  else
	    fprintf(myFile, "0 0  ");
	}
      }
      else
      {
	for (rit = readings->rbegin(); rit != readings->rend(); rit++)
	{
	  reading = (*rit);
	  if (!reading->getIgnoreThisReading())
	    fprintf(myFile, "%.0f %.0f  ",
		    reading->getLocalX() - mySick->getSensorPositionX(),
		    reading->getLocalY() - mySick->getSensorPositionY());
	  else
	    fprintf(myFile, "0 0  ");
	}
      }
      fprintf(myFile, "\n");
    }
    mySick->unlockDevice();
  }
}

void ArSickLogger::internalPrintPos(ArPose poseTaken)
{
  if (myFile == NULL)
    return;
  ArPose encoderPose = myRobot->getEncoderPose();
  ArPose rawEncoderPose = myRobot->getRawEncoderPose();
  ArTransform normalToRaw(rawEncoderPose, encoderPose);

  ArPose rawPose;
  rawPose = normalToRaw.doInvTransform(poseTaken);
  fprintf(myFile, "#rawRobot: %.0f %.0f %.2f %.0f %.0f\n",
	  rawPose.getX(),
	  rawPose.getY(),
	  rawPose.getTh(),
	  myRobot->getVel(),
	  myRobot->getRotVel());
  fprintf(myFile, "robot: %.0f %.0f %.2f %.0f %.0f\n",
	  poseTaken.getX(),
	  poseTaken.getY(),
	  poseTaken.getTh(),
	  myRobot->getVel(),
	  myRobot->getRotVel());
}

AREXPORT void ArSickLogger::robotTask(void)
{

  // call our function to check goals
  internalAddGoal();

  // call our function to dump tags
  internalWriteTags();

  // call our function to take a reading
  internalTakeReading();

  // now make sure the files all out to disk
  if (myWrote)
  {
    fflush(myFile);
#ifndef WIN32
    fsync(fileno(myFile));
#endif
  }
  myWrote = false;
}




