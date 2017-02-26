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
#include "ArActionDesired.h"

const double ArActionDesired::NO_STRENGTH = 0.0;
const double ArActionDesired::MIN_STRENGTH = .000001;
const double ArActionDesired::MAX_STRENGTH = 1.0;

const double ArActionDesiredChannel::NO_STRENGTH =
                                                 ArActionDesired::NO_STRENGTH;
const double ArActionDesiredChannel::MIN_STRENGTH = 
                                                ArActionDesired::MIN_STRENGTH;
const double ArActionDesiredChannel::MAX_STRENGTH = 
                                                ArActionDesired::MAX_STRENGTH;


AREXPORT void ArActionDesired::log(void) const
{
  // all those maxes and movement parameters
  if (getMaxVelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "\tMaxTransVel %.0f", getMaxVel());
  if (getMaxNegVelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "\tMaxTransNegVel %.0f", 
	       getMaxNegVel());
  if (getTransAccelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "\tTransAccel %.0f", getTransAccel());
  if (getTransDecelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "\tTransDecel %.0f", getTransDecel());
  if (getMaxRotVelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "%25s\tMaxRotVel %.0f", "",
	       getMaxRotVel());
  if (getRotAccelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "%25s\tRotAccel %.0f", "",
	       getRotAccel());
  if (getRotDecelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "%25s\tRotDecel %.0f", "",
	       getRotDecel());
  
  // the actual movement part
  if (getVelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "\tVel %.0f", getVel());
  if (getHeadingStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "%25s\tHeading %.0f", "", 
	       getHeading());
  if (getDeltaHeadingStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "%25s\tDeltaHeading %.0f", "", 
	       getDeltaHeading());
  if (getRotVelStrength() >= ArActionDesired::MIN_STRENGTH)
    ArLog::log(ArLog::Normal, "%25s\tRotVel %.0f", "", 
	       getRotVel());
}


AREXPORT bool ArActionDesired::isAnythingDesired(void) const
{
  if (getMaxVelStrength() >= ArActionDesired::MIN_STRENGTH ||
      getMaxNegVelStrength() >= ArActionDesired::MIN_STRENGTH || 
      getTransAccelStrength() >= ArActionDesired::MIN_STRENGTH || 
      getTransDecelStrength() >= ArActionDesired::MIN_STRENGTH ||
      getMaxRotVelStrength() >= ArActionDesired::MIN_STRENGTH ||
      getRotAccelStrength() >= ArActionDesired::MIN_STRENGTH ||
      getRotDecelStrength() >= ArActionDesired::MIN_STRENGTH ||
      getVelStrength() >= ArActionDesired::MIN_STRENGTH ||
      getHeadingStrength() >= ArActionDesired::MIN_STRENGTH ||
      getDeltaHeadingStrength() >= ArActionDesired::MIN_STRENGTH ||
      getRotVelStrength() >= ArActionDesired::MIN_STRENGTH)
    return true;
  else
    return false;
}
