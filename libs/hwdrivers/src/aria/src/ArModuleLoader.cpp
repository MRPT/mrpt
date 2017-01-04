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
#ifdef WIN32
#else
#include <dlfcn.h>
#endif
#include "ArModuleLoader.h"
#include "ArModule.h"
#include "ArLog.h"


std::map<std::string, ArModuleLoader::DllRef> ArModuleLoader::ourModMap;


#ifdef WIN32

#define RTLD_NOW 0
#define RTLD_GLOBAL 0

HINSTANCE dlopen(const char *fileName, int flag)
{
  return(LoadLibraryA(fileName));
}

int dlclose(HINSTANCE handle)
{
  FreeLibrary(handle);
  return(0);
}

void *dlsym(HINSTANCE handle, char *symbol)
{
  return (void*)(GetProcAddress(handle, symbol));
}

const char *dlerror(void)
{
  return(0);
}
#endif // WIN32


/**
   THIS ONLY LOADS one init on the module right now, if its called
   again it'll load the same init over.  I'll fix it later... read the
   more verbose description in ArModule.h.

   Takes a string name of the module which is just the file name of
   the module without the extension (.dll or .so). It will figure out
   the correct extension based on wheter its a Linux or Windows
   build. It will also uses the standard operating systems ability to
   find the library.  So the library must be located within the PATH
   variable for Windows and the LD_LIBRARY_PATH for Linux. You can
   also just give the absolute path to the library, or the relative
   path from the directory the program was started in (ie
   ./simpleMod).  The ArModule will be passed the ArRobot reference
   that load() takes. This is the ArRobot that the ArModule will use
   for its processing.

   @param modName fileName of the module without the extension (.dll
   or .so)

   @param robot ArRobot reference which the module is to use, this can
   be NULL

   @param modArgument A void pointer argument to pass to the module,
   if its a const value you'll need to cast it to a non-const value to
   get it to work (for example if you were using a constant string).
   This value defaults to NULL.

   @param quiet whether to print out a message if this fails or not,
   defaults to false
**/
AREXPORT ArModuleLoader::Status ArModuleLoader::load(const char *modName,
						     ArRobot *robot,
						     void *modArgument,
						     bool quiet)
{
  std::string name;
  std::map<std::string, DllRef>::iterator iter;
  DllRef handle;
  typedef bool(*TFunc)(ArRobot*,void*);  // Added by JLBC for MRPT
  TFunc func;
  bool ret;

  name=modName;
#ifdef WIN32
  if (strstr(modName, ".dll") == 0)
    name+=".dll";
#else
  if (strstr(modName, ".so") == 0)
    name+=".so";
#endif

  iter=ourModMap.find(name);
  if (iter != ourModMap.end())
    return(STATUS_ALREADY_LOADED);

  handle=dlopen(name.c_str(), RTLD_NOW | RTLD_GLOBAL);

  if (!handle || dlerror() != NULL)
  {
    if (!quiet)
      ArLog::log(ArLog::Terse, "Failure to load module '%s': %s",
		 name.c_str(), dlerror());
    return(STATUS_FAILED_OPEN);
  }

#ifndef _MSC_VER
  #pragma GCC diagnostic push
//  #pragma GCC diagnostic ignored "-fstrict-aliasing"
#endif
  //func=(bool(*)(ArRobot*,void*))dlsym(handle, "ariaInitModule");
  func=reinterpret_cast<TFunc>( dlsym(handle, "ariaInitModule") );
#ifndef _MSC_VER
  #pragma GCC diagnostic pop
#endif
  if (!func || dlerror() != NULL)
  {
    if (!quiet)
      ArLog::log(ArLog::Terse, "No module initializer for %s.", modName);
    ourModMap.insert(std::map<std::string, DllRef>::value_type(name,
							       handle));
    return(STATUS_SUCCESS);
    //dlclose(handle);
    //return(STATUS_INVALID);
  }
  ret=(*func)(robot, modArgument);

  if (ret)
  {
    ourModMap.insert(std::map<std::string, DllRef>::value_type(name,
							       handle));
    return(STATUS_SUCCESS);
  }
  else
  {
    if (!quiet)
      ArLog::log(ArLog::Terse, "Module '%s' failed its init sequence",
		 name.c_str());
    dlclose(handle);
    return(STATUS_INIT_FAILED);
  }
}


/**
   reload() is similiar to load(), except that it will call close() on the
   module and then call load().
   @param modName fileName of the module without the extension (.dll or .so)
*/
AREXPORT ArModuleLoader::Status ArModuleLoader::reload(const char *modName,
						       ArRobot *robot,
						       void *modArgument,
						       bool quiet)
{
  close(modName, quiet);
  return(load(modName, robot, modArgument, quiet));
}

/**
   Calls ArModule::exit() on the module, then closes the library.
   @param modName fileName of the module without the extension (.dll or .so)
   @param quiet whether to print out a message if this fails or not,
   defaults to false
*/
AREXPORT ArModuleLoader::Status ArModuleLoader::close(const char *modName,
						      bool quiet)
{
  std::string name;
  std::map<std::string, DllRef>::iterator iter;
  bool (*func)();
  bool funcRet;
  DllRef handle;
  Status ret=STATUS_SUCCESS;

  name=modName;
#ifdef WIN32
  if (strstr(modName, ".dll") == 0)
    name+=".dll";
#else
  if (strstr(modName, ".so") == 0)
    name+=".so";
#endif

  iter=ourModMap.find(name.c_str());
  if (iter == ourModMap.end())
  {
    ArLog::log(ArLog::Terse, "Module '%s' could not be found to be closed.",
	       modName);
    return(STATUS_NOT_FOUND);
  }
  else
  {
    handle=reinterpret_cast<DllRef>((*iter).second);
    func=(bool(*)())dlsym(handle, "ariaExitModule");
    if (!func)
    {
      if (!quiet)
	ArLog::log(ArLog::Verbose,
		   "Failure to find module exit function for '%s'", (*iter).first.c_str());
      //ArLog::log(ArLog::Terse, "Failure to find module exit function: '%s'",
      //dlerror());
      //ret=STATUS_INVALID;
      ourModMap.erase(name);
      return STATUS_SUCCESS;
    }
    funcRet=(*func)();
    if (funcRet)
      ret=STATUS_SUCCESS;
    else
    {
      if (!quiet)
	ArLog::log(ArLog::Terse, "Module '%s' failed its exit sequence",
		   modName);
      ret=STATUS_INIT_FAILED;
    }
    dlclose(handle);
    ourModMap.erase(name);
  }

  return(ret);
}

AREXPORT void ArModuleLoader::closeAll()
{
  std::map<std::string, DllRef>::iterator iter;

  while ((iter = ourModMap.begin()) != ourModMap.end())
    close((*iter).first.c_str());
  //for (iter=ourModMap.begin(); iter != ourModMap.end(); iter=ourModMap.begin())

}
