/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
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
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPoint2D &p) {
			C.resize(2,1);
			for (size_t i=0;i<2;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPoint3D &p) {
			C.resize(3,1);
			for (size_t i=0;i<3;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose2D &p) {
			C.resize(3,1);
			for (size_t i=0;i<3;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose3D &p) {
			C.resize(6,1);
			for (size_t i=0;i<6;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}
		template <class CONTAINER> CONTAINER & containerFromPoseOrPoint(CONTAINER &C, const TPose3DQuat &p) {
			C.resize(7,1);
			for (size_t i=0;i<7;i++)  C.coeffRef(i,0)=p[i];
			return C;
		}

		/** @} */



	} // End of math namespace
} // End of mrpt namespace
