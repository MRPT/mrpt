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

#ifndef ARMODULELOADER_H
#define ARMODULELOADER_H


#include <map>
#include <string>
#include "ariaTypedefs.h"
#include "ArRobot.h"


/// Dynamic ArModule loader
/**
   The ArModuleLoader is used to load ArModules into a program and invoke
   them. 

   See also ArModule to see how to define an ArModule.

   See also the example programs advanced/simpleMod.cpp and advanced/simpleModule.cpp. 
*/
class ArModuleLoader
{
public:

#ifdef WIN32
  typedef HINSTANCE DllRef;
#else
  typedef void * DllRef;
#endif

  typedef enum {
    STATUS_SUCCESS=0, ///< Load succeded
    STATUS_ALREADY_LOADED, ///< Module already loaded
    STATUS_FAILED_OPEN, ///< Could not find or open the module
    STATUS_INVALID, ///< Invalid module file format
    STATUS_INIT_FAILED, ///< The module failed its init stage
    STATUS_EXIT_FAILED, ///< The module failed its exit stage
    STATUS_NOT_FOUND ///< The module was not found
  } Status;

  /// Load an ArModule
  AREXPORT static Status load(const char *modName, ArRobot *robot,
			      void *modArgument = NULL, bool quiet = false);
  /// Close and then reload an ArModule
  AREXPORT static Status reload(const char *modName, ArRobot *robot,
				void * modArgument = NULL, bool quiet = false);
  /// Close an ArModule
  AREXPORT static Status close(const char *modName, bool quiet = false);
  /// Close all open ArModule
  AREXPORT static void closeAll();

protected:

  static std::map<std::string, DllRef> ourModMap;
};


#endif // ARMODULELOADER_H
