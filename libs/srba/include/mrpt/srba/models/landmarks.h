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

namespace mrpt { namespace srba {
namespace landmarks {

	/** \defgroup mrpt_srba_landmarks Landmark parameterizations
		* \ingroup mrpt_srba_grp */

	/** \addtogroup mrpt_srba_landmarks
		* @{ */

	/** A parameterization of landmark positions in Euclidean coordinates (3D) */
	struct Euclidean3D
	{
		static const size_t  LM_DIMS = 3; //!< The number of parameters in each LM parameterization relative to its base KF: (x,y,z)
		static const size_t  LM_EUCLIDEAN_DIMS = 3; //!< Either 2 or 3, depending on the real minimum number of coordinates needed to parameterize the landmark.

		/** Converts the landmark parameterization into 3D Eucliden coordinates (used for OpenGL rendering, etc.) */
		template <class VECTOR> inline static void relativeEuclideanLocation(const VECTOR &posParams, mrpt::math::TPoint3D &posEuclidean) 
		{
			posEuclidean.x = posParams[0];
			posEuclidean.y = posParams[1];
			posEuclidean.z = posParams[2];
		}
	};

	/** A parameterization of landmark positions in Euclidean coordinates (2D) */
	struct Euclidean2D
	{
		static const size_t  LM_DIMS = 2; //!< The number of parameters in each LM parameterization relative to its base KF: (x,y)
		static const size_t  LM_EUCLIDEAN_DIMS = 2; //!< Either 2 or 3, depending on the real minimum number of coordinates needed to parameterize the landmark.

		/** Converts the landmark parameterization into 3D Eucliden coordinates (used for OpenGL rendering, etc.) */
		template <class VECTOR> inline static void relativeEuclideanLocation(const VECTOR &posParams, mrpt::math::TPoint2D &posEuclidean) 
		{
			posEuclidean.x = posParams[0];
			posEuclidean.y = posParams[1];
		}
	};

	/** @} */

}
} } // end NS