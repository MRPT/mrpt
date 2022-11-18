/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*
 * File: logging.h
 * (Was: utils.h in mrpt_bridge package, now obsolete since 2022)
 * Author: Vladislav Tananaev
 */

#pragma once

#include <log4cxx/logger.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <ros/console.h>

namespace mrpt::ros1bridge
{
/**
 *@brief function that converts ROS verbosity level log4cxx::Level to MRPT
 * equivalent MRPT's VerbosityLevel
 */
static inline mrpt::system::VerbosityLevel rosLoggerLvlToMRPTLoggerLvl(
	log4cxx::LevelPtr lvl)
{
	using namespace log4cxx;

	// determine on the corresponding VerbosityLevel
	mrpt::system::VerbosityLevel mrpt_lvl;

	if (lvl == Level::getFatal() || lvl == Level::getError())
	{
		mrpt_lvl = mrpt::system::LVL_ERROR;
	}
	else if (lvl == Level::getWarn())
	{
		mrpt_lvl = mrpt::system::LVL_WARN;
	}
	else if (lvl == Level::getInfo())
	{
		mrpt_lvl = mrpt::system::LVL_INFO;
	}
	else if (lvl == Level::getDebug() || lvl == Level::getTrace())
	{
		mrpt_lvl = mrpt::system::LVL_DEBUG;
	}
	else
	{
		mrpt_lvl = mrpt::system::LVL_INFO;
		ROS_ERROR("Unknown log4cxx::Level is given.");
	}

	return mrpt_lvl;

}  // end of rosLoggerLvlToMRPTLoggerLvl

/**
 *@brief callback that is called by MRPT mrpt::system::COuputLogger to redirect
 * log messages to ROS logger.
 *	This function has to be inline, otherwise option
 * log4j.logger.ros.package_name will be taken from mrpt_bridge
 * instead of the package from which macro is actually called.
 */
static inline void mrptToROSLoggerCallback(
	const std::string& msg, const mrpt::system::VerbosityLevel level,
	[[maybe_unused]] const std::string& loggerName,
	[[maybe_unused]] const mrpt::system::TTimeStamp timestamp)
{
	// Remove trailing \n if present
	std::string tmsg = msg;
	if (!tmsg.empty() &&
		tmsg.compare(tmsg.length() - 1, tmsg.length(), "\n") == 0)
	{
		tmsg.erase(tmsg.end() - 1);
	}

	switch (level)
	{
		case mrpt::system::LVL_DEBUG: ROS_DEBUG_STREAM(msg); break;
		case mrpt::system::LVL_WARN: ROS_WARN_STREAM(tmsg); break;
		case mrpt::system::LVL_ERROR: ROS_ERROR_STREAM(tmsg); break;
		default:
		case mrpt::system::LVL_INFO: ROS_INFO_STREAM(tmsg); break;
	}
}

}  // namespace mrpt::ros1bridge