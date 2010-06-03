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
