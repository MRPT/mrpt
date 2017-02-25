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
#include "ArPriorityResolver.h"
#include "ArAction.h"
#include "ArRobot.h"

AREXPORT ArPriorityResolver::ArPriorityResolver() :
  ArResolver("ArPriorityResolver", "Resolves strictly by using priority, the highest priority action to act is the one that gets to go.  Does no mixing of any variety.")
{


}


AREXPORT ArPriorityResolver::~ArPriorityResolver()
{
}

AREXPORT ArActionDesired *ArPriorityResolver::resolve(
	ArResolver::ActionMap *actions, ArRobot *robot, bool logActions)
{
  ArResolver::ActionMap::reverse_iterator it;
  ArAction *action;
  ArActionDesired *act;
  ArActionDesired averaging;
  bool first = true;
  int lastPriority=0;
  bool printedFirst = true;
  int printedLast=0;

  if (actions == NULL)
    return NULL;

  myActionDesired.reset();
  averaging.reset();
  averaging.startAverage();
  for (it = actions->rbegin(); it != actions->rend(); ++it)
  {
    action = (*it).second;
    if (action != NULL && action->isActive())
    {
      act = action->fire(myActionDesired);
      if (robot != NULL && act != NULL)
	act->accountForRobotHeading(robot->getTh());
      if (first || (*it).first != lastPriority)
      {
	averaging.endAverage();
	myActionDesired.merge(&averaging);

	averaging.reset();
	averaging.startAverage();
	first = false;
	lastPriority = (*it).first;
      }
      averaging.addAverage(act);
      if (logActions && act != NULL && act->isAnythingDesired())
      {
	if (printedFirst || printedLast != (*it).first)
	{
	  ArLog::log(ArLog::Terse, "Priority %d:", (*it).first);
	  printedLast = (*it).first;
	  printedFirst = false;
	}
	ArLog::log(ArLog::Terse, "Action: %s", action->getName());
	act->log();
      }


    }
  }
  averaging.endAverage();
  myActionDesired.merge(&averaging);
  /*
  printf(
      "desired delta %.0f strength %.3f, desired speed %.0f strength %.3f\n",
      myActionDesired.getDeltaHeading(), myActionDesired.getHeadingStrength(),
      myActionDesired.getVel(), myActionDesired.getVelStrength());
  */
  return &myActionDesired;
}


