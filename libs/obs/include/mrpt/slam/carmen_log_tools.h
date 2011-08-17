/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef CARMEN_LOG_TOOLS_H
#define CARMEN_LOG_TOOLS_H

#include <mrpt/system/datetime.h>
#include <mrpt/slam/CObservation.h>

namespace mrpt
{
	namespace slam
	{
		/** Parse one line from an text input stream and interpret it as a CARMEN log entry,
		  *  returning its MRPT observation representation.
		  *
		  * The first word in each line determines the type of that entry. Supported
		  *  line entries in this class are the following:
		  *		- "ROBOTLASER(.*)": A joint odometry-laser entry. This function will return both an mrpt::slam::CObservationOdometry and an mrpt::slam::CObservation2DRangeScan
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
			std::vector<mrpt::slam::CObservationPtr> &out_imported_observations,
			const mrpt::system::TTimeStamp &time_start_log );


	} // end NS
} // end NS
#endif
