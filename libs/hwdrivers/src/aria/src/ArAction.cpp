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
#include "ArResolver.h"
#include "ArAction.h"
#include "ArLog.h"
#include "ArRobot.h"

AREXPORT ArAction::ArAction(const char *name, const char *description)
{
  myRobot = NULL;
  myNumArgs = 0;
  myName = name;
  myDescription = description;
  myIsActive = true;
}

AREXPORT ArAction::~ArAction()
{
  if (myRobot != NULL)
    myRobot->remAction(this);
}

AREXPORT const char *ArAction::getName(void) const
{
  return myName.c_str();
}

AREXPORT const char *ArAction::getDescription(void) const
{
  return myDescription.c_str();
}

AREXPORT int ArAction::getNumArgs(void) const
{
  return myNumArgs;
}

AREXPORT void ArAction::setNextArgument(ArArg const &arg)
{
  myArgumentMap[myNumArgs] = arg;
  myNumArgs++;
}

AREXPORT ArArg *ArAction::getArg(int number) 
{
  std::map<int, ArArg>::iterator it;
  
  it = myArgumentMap.find(number);
  if (it != myArgumentMap.end())
    return &(*it).second;
  else
    return NULL;
}

AREXPORT const ArArg *ArAction::getArg(int number) const
{
  std::map<int, ArArg>::const_iterator it;
  
  it = myArgumentMap.find(number);
  if (it != myArgumentMap.end())
    return &(*it).second;
  else
    return NULL;
}

/**
 *  @swignote If you override this method in a Java or Python subclass, use
 *  setActionRobotObj(ArRobot) instead of trying to call super.setRobot() or 
 *  ArAction.setRobot(). (SWIG's subclassing "directors" feature cannot properly
 *  direct the call to the parent class, an infinite recursion results instead.)
 */
AREXPORT void ArAction::setRobot(ArRobot *robot)
{
  myRobot = robot;
}

AREXPORT bool ArAction::isActive(void) const
{
  return myIsActive;
}

AREXPORT void ArAction::activate(void)
{
  myIsActive = true;
}

AREXPORT void ArAction::deactivate(void)
{
  myIsActive = false;
}

AREXPORT void ArAction::log(bool verbose) const
{
  int i;
  const ArArg *arg;
  const ArActionDesired *desired;

  ArLog::log(ArLog::Terse, "Action %s isActive %d", getName(), myIsActive);
  if (myIsActive && (desired = getDesired()) != NULL)
    desired->log();
  if (!verbose)
    return;
  if (strlen(getDescription()) != 0)
    ArLog::log(ArLog::Terse, "Action %s is described as: %s", 
	       getName(), getDescription());
  else
    ArLog::log(ArLog::Terse, "Action %s has no description.", 
	       getName());
  if (getNumArgs() == 0)
    ArLog::log(ArLog::Terse, "Action %s has no arguments.\n", 
	       getName());
  else
  {
    ArLog::log(ArLog::Terse, "Action %s has %d arguments, of type(s):", 
	       (getName()), getNumArgs());

    for (i = 0; i < getNumArgs(); i++) 
    {
      arg = getArg(i);
      if (arg == NULL)
	continue;
      arg->log();
    }
    ArLog::log(ArLog::Terse, "");
  }
}



