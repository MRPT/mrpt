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

#include <ctype.h>

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArLogFileConnection.h"
#include "ArLog.h"
#include "ariaUtil.h"

AREXPORT ArLogFileConnection::ArLogFileConnection()
{
  myStatus = STATUS_NEVER_OPENED;
  myLogFile = NULL;
  myFD = NULL;
  stopAfter = 1;
  strcpy(myName, "random");
  strcpy(myType, "amigo");
  strcpy(mySubtype, "amigo");
}

AREXPORT ArLogFileConnection::~ArLogFileConnection()
{
  if (myFD != NULL)
    fclose(myFD);
}


AREXPORT void ArLogFileConnection::setLogFile(const char *fname)
{
  if (fname == NULL)
    myLogFile = "robot.log";
  else
    myLogFile = fname;
}

AREXPORT bool ArLogFileConnection::openSimple(void)
{
  if (internalOpen() == 0)
    return true;
  else
    return false;
}

/**
   @param fname the file to connect to, if NULL (default) then robot.log
   @return 0 for success, otherwise one of the open enums
   @see getOpenMessage
*/
AREXPORT int ArLogFileConnection::open(const char *fname)
{
  setLogFile(fname);
  return internalOpen();
}

AREXPORT int ArLogFileConnection::internalOpen(void)
{
  havePose = false;
  myFD = fopen(myLogFile, "r");
  if (myFD == NULL)
    {
      myStatus = STATUS_OPEN_FAILED;
      return OPEN_FILE_NOT_FOUND;
    }

  char buf[100];
  if (fgets(buf,100,myFD) != NULL)
    {
      if (strncmp(buf, "// Saphira log file", 19) != 0)
        {
          myStatus = STATUS_OPEN_FAILED;
          fclose(myFD);
          myFD = NULL;
          return OPEN_NOT_A_LOG_FILE;
        }
    }
  else
    {
      myStatus = STATUS_OPEN_FAILED;
      fclose(myFD);
      myFD = NULL;
      return OPEN_NOT_A_LOG_FILE;
    }

  // Set the robot pose
  if (fgets(buf,100,myFD) != NULL)
    {
      if (strncmp(buf, "// Robot position", 17) == 0)  // have a position!
        {
          int x,y,th;
          char * dumm_ret = fgets(buf,100,myFD);
          if (!dumm_ret) return OPEN_NOT_A_LOG_FILE;
          sscanf(buf, "%d %d %d", &x, &y, &th);
          myPose.setX(x);
          myPose.setY(y);
          myPose.setTh(th);
          havePose = true;
        }
      if (strncmp(buf, "// Robot name", 13) == 0)  // have a name!
        {
          char * dumm_ret =fgets(buf,100,myFD);
          if (!dumm_ret) return OPEN_NOT_A_LOG_FILE;
          sscanf(buf, "%s %s %s", myName, myType, mySubtype);
        }
    }

  myStatus = STATUS_OPEN;
  return 0;
}

void ArLogFileConnection::buildStrMap(void)
{
  myStrMap[OPEN_FILE_NOT_FOUND] = "File not found.";
  myStrMap[OPEN_NOT_A_LOG_FILE] = "File is not a log file.";
}

AREXPORT const char * ArLogFileConnection::getOpenMessage(int messageNumber)
{
  return myStrMap[messageNumber].c_str();
}

AREXPORT bool ArLogFileConnection::close(void)
{
  myStatus = STATUS_CLOSED_NORMALLY;
  if (myFD != NULL)
    fclose(myFD);
  myFD = NULL;
  return true;
}

AREXPORT int ArLogFileConnection::read(const char *data, unsigned int size,
				   unsigned int msWait)
{
  ArTime timeDone;
  unsigned int bytesRead = 0;
  int n=0;

  if (getStatus() != STATUS_OPEN)
  {
    ArLog::log(ArLog::Terse,
	       "ArLogFileConnection::read: Attempt to use port that is not open.");
    return -1;
  }

  timeDone.setToNow();
  timeDone.addMSec(msWait);

  if (stopAfter-- <= 0)
    {
      stopAfter= 1;
      return 0;
    }

  if (myFD != NULL)
    {
      char line[1000];
      if (fgets(line, 1000, myFD) == NULL) // done with file, close
        {
          close();
          return -1;
        }
      // parse the line
      int i=0;
      n = 0;
      while (line[i] != 0)
        {
          if (isdigit(line[i]))
            {
              if (isdigit(line[i+1]))
                {
                  if (isdigit(line[i+2]))
                    {
                      const_cast<char *>(data)[n++] =
                        100 * (line[i]-'0') + 10*(line[i+1]-'0') + line[i+2]-'0';
                      i++;
                    }
                  else
                      const_cast<char *>(data)[n++] = 10*(line[i]-'0') + line[i+1]-'0';
                  i++;
                }
              else
                const_cast<char *>(data)[n++] = line[i]-'0';
            }
          i++;
        }
    }

#if 0
  if (n > 0)                    // add in checksum
    {
      int i;
      unsigned char nn;
      int c = 0;

      i = 3;
      nn = data[2] - 2;
      while (nn > 1)
        {
          c += ((unsigned char)data[i]<<8) | (unsigned char)data[i+1];
          c = c & 0xffff;
          nn -= 2;
          i += 2;
        }
      if (nn > 0)
        c = c ^ (int)((unsigned char) data[i]);

      const_cast<char *>(data)[n++] = (c << 8) & 0xff;
      const_cast<char *>(data)[n++] = c & 0xff;
    }
#endif

  bytesRead = n;
  return bytesRead;
}



AREXPORT int ArLogFileConnection::write(const char *data, unsigned int size)
{
  return size;                  // always write
}


/**
    @return the name of the log file
*/
AREXPORT const char *ArLogFileConnection::getLogFile(void)
{
  return myLogFile;
}

AREXPORT int ArLogFileConnection::getStatus(void)
{
  return myStatus;
}

AREXPORT bool ArLogFileConnection::isTimeStamping(void)
{
  return false;
}

AREXPORT ArTime ArLogFileConnection::getTimeRead(int index)
{
  ArTime now;
  now.setToNow();
  return now;
}
