/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
