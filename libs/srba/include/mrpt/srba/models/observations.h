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

#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/TStereoCamera.h>

namespace mrpt { namespace srba {

	/** @name Observation types
	    @{ */

	/** Observation = one monocular camera feature (the coordinates of one pixel) */
	struct observation_traits_MonocularCamera
	{
		static const size_t  OBS_DIMS = 2; //!< Each observation is one pixel (px,py) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::utils::TPixelCoordf  px;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = px.x;
				obs[1] = px.y;
			}
		};

		/** The type "TObservationParams" must be declared in each "observation_traits_*" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			mrpt::utils::TCamera  camera_calib;
		};
	};

	// -------------------------------------------------------------------------------------------------

	/** Observation = one stereo camera feature (the coordinates of two pixels) */
	struct observation_traits_StereoCamera
	{
		static const size_t  OBS_DIMS = 4; //!< Each observation is a pair of pixels (px_l,py_l,px_r,py_r) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::utils::TPixelCoordf  left_px, right_px;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = left_px.x;  obs[1] = left_px.y;
				obs[2] = right_px.x; obs[3] = right_px.y;
			}
		};

		/** The type "TObservationParams" must be declared in each "observation_traits_*" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			mrpt::utils::TStereoCamera camera_calib;
		};
	};

	// -------------------------------------------------------------------------------------------------

	/** @} */

} } // end NS