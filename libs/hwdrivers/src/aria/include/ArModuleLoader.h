/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
