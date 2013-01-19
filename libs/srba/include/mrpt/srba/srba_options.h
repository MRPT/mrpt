/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#pragma once

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
namespace srba
{
	/** \defgroup mrpt_srba_options Struct traits used for setting different SRBA options at compile time
		* \ingroup mrpt_srba_grp */

	/** \addtogroup mrpt_srba_options
		* @{ */

		/** Usage: A possible type for RBA_OPTIONS::sensor_pose_on_robot_t. 
		  * Meaning: The robot pose and the sensor pose coincide, i.e. the sensor pose on the robot is the identitity transformation.  */
		struct sensor_pose_on_robot_none
		{
			/** In this case there are no needed parameters */
			struct parameters_t 
			{
			};
		};

		/** Usage: A possible type for RBA_OPTIONS::sensor_pose_on_robot_t. 
		  * Meaning: The sensor is located at an arbitrary SE(3) pose wrt the robot reference frame. */
		struct sensor_pose_on_robot_se3
		{
			/** In this case there are no needed parameters */
			struct parameters_t
			{
				mrpt::poses::CPose3D  relative_pose;
			};
		};


		/** @} */

} // End of namespace
} // end of namespace
