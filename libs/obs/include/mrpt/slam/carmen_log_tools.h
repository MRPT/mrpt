/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
