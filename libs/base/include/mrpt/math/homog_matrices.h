/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_HOMOG_MATRICES_H
#define  MRPT_HOMOG_MATRICES_H

#include <mrpt/config.h>
#include <mrpt/utils/compiler_fixes.h>
#include <mrpt/utils/boost_join.h>
#include <mrpt/utils/mrpt_macros.h>

namespace mrpt
{
	namespace math
	{

		/** Efficiently compute the inverse of a 4x4 homogeneous matrix by only transposing the rotation 3x3 part and solving the translation with dot products.
		  *  This is a generic template which works with:
		  *    MATRIXLIKE: CMatrixTemplateNumeric, CMatrixFixedNumeric
		  */
		template <class MATRIXLIKE1,class MATRIXLIKE2>
		void homogeneousMatrixInverse(const MATRIXLIKE1 &M, MATRIXLIKE2 &out_inverse_M)
		{
			MRPT_START
			ASSERT_( M.isSquare() && size(M,1)==4);

			/* Instead of performing a generic 4x4 matrix inversion, we only need to
			  transpose the rotation part, then replace the translation part by
			  three dot products. See, for example:
			 https://graphics.stanford.edu/courses/cs248-98-fall/Final/q4.html

				[ux vx wx tx] -1   [ux uy uz -dot(u,t)]
				[uy vy wy ty]      [vx vy vz -dot(v,t)]
				[uz vz wz tz]    = [wx wy wz -dot(w,t)]
				[ 0  0  0  1]      [ 0  0  0     1    ]
			*/

			out_inverse_M.setSize(4,4);

			// 3x3 rotation part:
			out_inverse_M.set_unsafe(0,0, M.get_unsafe(0,0));
			out_inverse_M.set_unsafe(0,1, M.get_unsafe(1,0));
			out_inverse_M.set_unsafe(0,2, M.get_unsafe(2,0));

			out_inverse_M.set_unsafe(1,0, M.get_unsafe(0,1));
			out_inverse_M.set_unsafe(1,1, M.get_unsafe(1,1));
			out_inverse_M.set_unsafe(1,2, M.get_unsafe(2,1));

			out_inverse_M.set_unsafe(2,0, M.get_unsafe(0,2));
			out_inverse_M.set_unsafe(2,1, M.get_unsafe(1,2));
			out_inverse_M.set_unsafe(2,2, M.get_unsafe(2,2));

			const double tx = -M.get_unsafe(0,3);
			const double ty = -M.get_unsafe(1,3);
			const double tz = -M.get_unsafe(2,3);

			const double tx_ = tx*M.get_unsafe(0,0)+ty*M.get_unsafe(1,0)+tz*M.get_unsafe(2,0);
			const double ty_ = tx*M.get_unsafe(0,1)+ty*M.get_unsafe(1,1)+tz*M.get_unsafe(2,1);
			const double tz_ = tx*M.get_unsafe(0,2)+ty*M.get_unsafe(1,2)+tz*M.get_unsafe(2,2);

			out_inverse_M.set_unsafe(0,3, tx_ );
			out_inverse_M.set_unsafe(1,3, ty_ );
			out_inverse_M.set_unsafe(2,3, tz_ );

			out_inverse_M.set_unsafe(3,0,  0);
			out_inverse_M.set_unsafe(3,1,  0);
			out_inverse_M.set_unsafe(3,2,  0);
			out_inverse_M.set_unsafe(3,3,  1);

			MRPT_END
		}
		//! \overload
		template <class IN_ROTMATRIX,class IN_XYZ, class OUT_ROTMATRIX, class OUT_XYZ>
		void homogeneousMatrixInverse(
			const IN_ROTMATRIX  & in_R,
			const IN_XYZ        & in_xyz,
			OUT_ROTMATRIX & out_R,
			OUT_XYZ       & out_xyz
			)
		{
			MRPT_START
			ASSERT_( in_R.isSquare() && size(in_R,1)==3 && in_xyz.size()==3)
			out_R.setSize(3,3);
			out_xyz.resize(3);

			// translation part:
			typedef typename IN_ROTMATRIX::Scalar T;
			const T tx = -in_xyz[0];
			const T ty = -in_xyz[1];
			const T tz = -in_xyz[2];

			out_xyz[0] = tx*in_R.get_unsafe(0,0)+ty*in_R.get_unsafe(1,0)+tz*in_R.get_unsafe(2,0);
			out_xyz[1] = tx*in_R.get_unsafe(0,1)+ty*in_R.get_unsafe(1,1)+tz*in_R.get_unsafe(2,1);
			out_xyz[2] = tx*in_R.get_unsafe(0,2)+ty*in_R.get_unsafe(1,2)+tz*in_R.get_unsafe(2,2);

			// 3x3 rotation part: transpose
			out_R = in_R.adjoint();

			MRPT_END
		}
		//! \overload
		template <class MATRIXLIKE>
		inline void homogeneousMatrixInverse(MATRIXLIKE &M)
		{
			ASSERTDEB_( M.isSquare() && size(M,1)==4);
			// translation:
			const double tx = -M(0,3);
			const double ty = -M(1,3);
			const double tz = -M(2,3);
			M(0,3) = tx*M(0,0)+ty*M(1,0)+tz*M(2,0);
			M(1,3) = tx*M(0,1)+ty*M(1,1)+tz*M(2,1);
			M(2,3) = tx*M(0,2)+ty*M(1,2)+tz*M(2,2);
			// 3x3 rotation part:
			double t; // avoid std::swap() to avoid <algorithm> only for that.
			t=M(1,0); M(1,0) = M(0,1); M(0,1)=t;
			t=M(2,0); M(2,0) = M(0,2); M(0,2)=t;
			t=M(1,2); M(1,2) = M(2,1); M(2,1)=t;
		}

	}
}
#endif
