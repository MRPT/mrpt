/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CARMEN_LOG_TOOLS_H
#define CARMEN_LOG_TOOLS_H

#include <mrpt/system/datetime.h>
#include <mrpt/obs/CObservation.h>

namespace mrpt
{
	namespace obs
	{
		/** Parse one line from an text input stream and interpret it as a CARMEN log entry,
		  *  returning its MRPT observation representation.
		  *
		  * The first word in each line determines the type of that entry. Supported
		  *  line entries in this class are the following:
		  *		- "ROBOTLASER(.*)": A joint odometry-laser entry. This function will return both an mrpt::obs::CObservationOdometry and an mrpt::obs::CObservation2DRangeScan
		  *		- "FLASER": (Idem)
		  *		- "RLASER": (Idem)
		  *		- "PARAM": This case is special (read below).
		  *
		  *  Note that the sensor position on the robot will be always deduced from the corresponding CARMEN log line automatically.
		  *
		  *  The following entry types will be IGNORED, since with the tags above will be enough in most applications.
		  *		- "ODOM"
		  *		- "RAWLASER"
		  *
		  * About the PARAM entries: This functions has an internal static status consisting on some PARAM values
		  *  which have effects on the future read observations. When the function finds one of these, it stores the
		  *  parameter value, return an empty vector of observations, and use those values for future laser entries of types FLASER or RLASER.
		  * Currently, only these parameters are processed:
		  *		- "robot_front_laser_max"
		  *		- "laser_front_laser_resolution"
		  *		- "robot_rear_laser_max"
		  *		- "laser_rear_laser_resolution"
		  *
		  *  \param time_start_log The real timestamp that corresponds to a "0" in the CARMEN log (you can get a mrpt::system::now() once and pass that same value with each call).
		  *
		  * References: http://carmen.sourceforge.net/doc/binary__loggerplayback.html
		  *
		  * \return true on success, false on end-of-file (EOF).
		  * \exception std::runtime_error On any invalid line found.
	 	  * \ingroup mrpt_obs_grp
		  */
		  bool OBS_IMPEXP carmen_log_parse_line(
			std::istream &in_stream,
			std::vector<mrpt::obs::CObservationPtr> &out_imported_observations,
			const mrpt::system::TTimeStamp &time_start_log );


	} // end NS
} // end NS
#endif
