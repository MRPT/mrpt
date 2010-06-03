/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/

#ifndef ARDATALOGGER_H
#define ARDATALOGGER_H

#include "ariaUtil.h"
#include "ArMutex.h"
#include "ArFunctor.h"
#include <vector>

class ArRobot;
class ArConfig;


/// This class will log data, but you have to use it through an ArConfig right now
/**
   This class doesn't log anything by default, but can be set up to
   log all sorts of data.  Note that if you do an addString after you
   do an addToConfig it'll automatically be enabled (since right now
   we don't want to change config after loading since the values would
   wind up wierd).
 **/
class ArDataLogger
{
public:
  /// Constructor
  AREXPORT ArDataLogger(ArRobot *robot, const char *fileName = NULL);
  /// Destructor
  AREXPORT ~ArDataLogger();
  /// Adds the data logger information to the config
  AREXPORT void addToConfig(ArConfig *config);
  /// Adds a string to the list of options in the raw format
  AREXPORT void addString(const char *name, ArTypes::UByte2 maxLen, 
			  ArFunctor2<char *, ArTypes::UByte2> *functor);

  /// Gets the functor for adding a string (for ArStringInfoGroup)
  ArFunctor3<const char *, ArTypes::UByte2,
				    ArFunctor2<char *, ArTypes::UByte2> *> *
                     getAddStringFunctor(void) { return &myAddStringFunctor; }

protected:
  AREXPORT void connectCallback(void);
  AREXPORT bool processFile(char *errorBuffer, size_t errorBufferLen);
  AREXPORT void userTask(void);
  ArRobot *myRobot;
  ArTime myLastLogged;
  ArConfig *myConfig;
  bool myAddToConfigAtConnect;
  bool myAddedToConfig;

  FILE *myFile;
  bool myConfigLogging;
  int myConfigLogInterval;
  char myOpenedFileName[512];
  char myConfigFileName[512];
  std::string myPermanentFileName;

  // for what we're logging
  bool myLogVoltage;
  bool myLogLeftVel;
  bool myLogRightVel;
  bool myLogTransVel;
  bool myLogRotVel;
  bool myLogLeftStalled;
  bool myLogRightStalled;
  bool myLogStallBits;
  bool myLogFlags;
  int myAnalogCount;
  bool *myAnalogEnabled;
  int myAnalogVoltageCount;
  bool *myAnalogVoltageEnabled;
  int myDigInCount;
  bool *myDigInEnabled;
  int myDigOutCount;
  bool *myDigOutEnabled;
  bool myLogPose;
  bool myLogEncoderPose;
  bool myLogCorrectedEncoderPose;
  bool myLogEncoders;
  int myStringsCount;
  std::vector<bool *> myStringsEnabled;
  ArMutex myMutex;

  std::vector<ArStringInfoHolder *> myStrings;
  ArTypes::UByte2 myMaxMaxLength;
  ArFunctor3C<ArDataLogger, const char *, ArTypes::UByte2,
		    ArFunctor2<char *, ArTypes::UByte2> *> myAddStringFunctor;
  

  ArFunctorC<ArDataLogger> myConnectCB;  
  ArRetFunctor2C<bool, ArDataLogger, char *, size_t> myProcessFileCB;
  ArFunctorC<ArDataLogger> myUserTaskCB;
};

#endif // ARDATALOGGER_H
