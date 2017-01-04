/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_math_slerp_H
#define  mrpt_math_slerp_H

#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/poses_frwds.h>

namespace mrpt
{
	namespace math
	{
		/** \addtogroup geometry_grp
		  *  @{ */

		/** @name SLERP (Spherical Linear Interpolation) functions
		    @{ */

		/** SLERP interpolation between two quaternions
		  * \param[in] q0 The quaternion for t=0
		  * \param[in] q1 The quaternion for t=1
		  * \param[in] t  A "time" parameter, in the range [0,1].
		  * \param[out] q The output, interpolated quaternion.
		  * \tparam T  The type of the quaternion (e.g. float, double).
		  * \exception std::exception Only in Debug, if t is not in the valid range.
		  * \sa http://en.wikipedia.org/wiki/Slerp
		  */
		template <typename T>
		void slerp(
			const CQuaternion<T>  & q0,
			const CQuaternion<T>  & q1,
			const double            t,
			CQuaternion<T>        & q)
		{
			ASSERTDEB_(t>=0 && t<=1)
			// See: http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
			// Angle between q0-q1:
			double cosHalfTheta = q0[0]*q1[0]+q0[1]*q1[1]+q0[2]*q1[2]+q0[3]*q1[3];
			// if qa=qb or qa=-qb then theta = 0 and we can return qa
			if (std::abs(cosHalfTheta) >= 1.0)
			{
				q = q0;
				return;
			}
			bool reverse_q1 = false;
			if (cosHalfTheta < 0) // Always follow the shortest path
			{
				reverse_q1 = true;
				cosHalfTheta = -cosHalfTheta;
			}
			// Calculate temporary values.
			const double halfTheta = acos(cosHalfTheta);
			const double sinHalfTheta = std::sqrt(1.0 - mrpt::utils::square(cosHalfTheta));
			// if theta = 180 degrees then result is not fully defined
			// we could rotate around any axis normal to qa or qb
			if (std::abs(sinHalfTheta) < 0.001)
			{
				if (!reverse_q1)
				     for (int i=0;i<4;i++) q[i] = (1-t)*q0[i] + t*q1[i];
				else for (int i=0;i<4;i++) q[i] = (1-t)*q0[i] - t*q1[i];
				return;
			}
			const double A = sin((1-t) * halfTheta)/sinHalfTheta;
			const double B = sin(t*halfTheta)/sinHalfTheta;
			if (!reverse_q1)
			     for (int i=0;i<4;i++) q[i] = A*q0[i] + B*q1[i];
			else for (int i=0;i<4;i++) q[i] = A*q0[i] - B*q1[i];
		}

		/** SLERP interpolation between two 6D poses - like mrpt::math::slerp for quaternions, but interpolates the [X,Y,Z] coordinates as well.
		  * \param[in] p0 The pose for t=0
		  * \param[in] p1 The pose for t=1
		  * \param[in] t  A "time" parameter, in the range [0,1].
		  * \param[out] p The output, interpolated pose.
		  * \exception std::exception Only in Debug, if t is not in the valid range.
		  */
		void BASE_IMPEXP slerp(
			const mrpt::poses::CPose3D  & q0,
			const mrpt::poses::CPose3D  & q1,
			const double     t,
			mrpt::poses::CPose3D        & p);

		//! \overload
		void BASE_IMPEXP slerp(
			const mrpt::poses::CPose3DQuat & q0,
			const mrpt::poses::CPose3DQuat & q1,
			const double        t,
			mrpt::poses::CPose3DQuat       & p);

		/** @} */

		/** @} */  // grouping
	}
}
#endif
