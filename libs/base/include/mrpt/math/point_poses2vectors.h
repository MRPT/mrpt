/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/math_frwds.h>  // forward declarations

namespace mrpt
{
	namespace math
	{
		/** \name Container initializer from pose classes
		  * @{
		  */

		/** Conversion of poses to MRPT containers (vector/matrix) */
		template <class CONTAINER,class POINT_OR_POSE>
		CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const POINT_OR_POSE &p) {
			const size_t DIMS = POINT_OR_POSE::static_size;
			C.resize(DIMS,1);
			for (size_t i=0;i<DIMS;i++)  C.coeffRef(i,0)= static_cast<typename CONTAINER::Scalar>(p[i]);
			return C;
		}

		/** @} */

	} // End of math namespace
} // End of mrpt namespace
