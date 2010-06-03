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

#ifndef ARLOGFILECONNECTION_H
#define ARLOGFILECONNECTION_H

#include "ArDeviceConnection.h"
#include <string>
#include <stdio.h>

#include "ariaTypedefs.h"

/// For connecting through a log file
class ArLogFileConnection: public ArDeviceConnection
{
 public:
  /// Constructor
  AREXPORT ArLogFileConnection();
  /// Destructor also closes connection
  AREXPORT virtual ~ArLogFileConnection();

  /// Opens a connection to the given host and port
  AREXPORT int open(const char * fname = NULL);

  AREXPORT void setLogFile(const char *fname = NULL);
  AREXPORT virtual bool openSimple(void);  
  AREXPORT virtual int getStatus(void);
  AREXPORT virtual bool close(void);
  AREXPORT virtual int read(const char *data, unsigned int size, 
			    unsigned int msWait = 0);
  AREXPORT virtual int write(const char *data, unsigned int size);
  AREXPORT virtual const char * getOpenMessage(int messageNumber);
  AREXPORT virtual ArTime getTimeRead(int index);
  AREXPORT virtual bool isTimeStamping(void);

  /// Gets the name of the host connected to
  AREXPORT const char *getLogFile(void);

  /* This doens't exist in the C++ file so I'm commenting it out
  /// Gets the initial pose of the robot
  AREXPORT ArPose getLogPose(void);
  */

  /// Internal function used by open and openSimple
  AREXPORT int internalOpen(void);

  enum Open { 
      OPEN_FILE_NOT_FOUND = 1,  ///< Can't find the file
      OPEN_NOT_A_LOG_FILE       ///< Doesn't look like a log file
  };

  // robot parameters
  ArPose myPose;     
  bool havePose;
  char myName[100];
  char myType[20];
  char mySubtype[20];

protected:
  void buildStrMap(void);
  ArStrMap myStrMap;

  int myStatus;
  int stopAfter;                // temp here for returning one packet at a time
  
  const char *myLogFile;
  FILE *myFD;                   // file descriptor

};

#endif //ARLOGFILECONNECTION_H
