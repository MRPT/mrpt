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
#include "ArConfigGroup.h"
#include "ArArgumentBuilder.h"
#include "ArLog.h"

AREXPORT ArConfigGroup::ArConfigGroup(const char *baseDirectory)
{
  if (baseDirectory != NULL)
    myBaseDirectory = baseDirectory;
  else
    myBaseDirectory = "";
}

AREXPORT ArConfigGroup::~ArConfigGroup(void)
{
  
}

AREXPORT void ArConfigGroup::addConfig(ArConfig *config)
{
  myConfigs.push_back(config);
}

AREXPORT void ArConfigGroup::remConfig(ArConfig *config)
{
  myConfigs.remove(config);
}

AREXPORT bool ArConfigGroup::parseFile(const char *fileName, 
				       bool continueOnError)
{
  std::list<ArConfig *>::iterator it;
  bool ret = true;

  myLastFile = fileName;
  // go through all the configs and set the base directory (we don't
  // do it when we're parsing just so that whether it suceeds or fails
  // its the same behavior in this base directory regard)
  for (it = myConfigs.begin(); it != myConfigs.end(); it++)
  {
    (*it)->setBaseDirectory(myBaseDirectory.c_str());
  }
  // now we go through and parse files... if we get an error we stop
  // if we're supposed to
  for (it = myConfigs.begin(); it != myConfigs.end(); it++)
  {
    if (!(*it)->parseFile(fileName, continueOnError))
    {
      // if we are continuing on errors we still want to tell them we
      // had an error
      ret = false;
      // if we aren't continuing on error then just return
      if (!continueOnError)
	return false;
    }
  }
  return ret;
}

AREXPORT bool ArConfigGroup::reloadFile(bool continueOnError)
{
  return parseFile(myLastFile.c_str(), continueOnError);
}

AREXPORT bool ArConfigGroup::writeFile(const char *fileName)
{
  std::set<std::string> alreadyWritten;
  std::list<ArConfig *>::iterator it;
  bool ret = true;
  bool append = false;
  
  // go through all the configs and set the base directory (we don't
  // do it when we're parsing just so that whether it suceeds or fails
  // its the same behavior in this base directory regard)
  for (it = myConfigs.begin(); it != myConfigs.end(); it++)
  {
    (*it)->setBaseDirectory(myBaseDirectory.c_str());
  }
  // now we go through and parse files... if we get an error we stop
  // if we're supposed to
  for (it = myConfigs.begin(); it != myConfigs.end(); it++)
  {
    ArLog::log(ArLog::Verbose, "Writing config file");
    if (!(*it)->writeFile(fileName, append, &alreadyWritten))
    {
      // if we are continuing on errors we still want to tell them we
      // had an error
      ret = false;
    }
    append = true;
  }
  return ret;
  
}

AREXPORT void ArConfigGroup::setBaseDirectory(const char *baseDirectory)
{
  myBaseDirectory = baseDirectory;
}

AREXPORT const char *ArConfigGroup::getBaseDirectory(void) const
{
  return myBaseDirectory.c_str();
}
