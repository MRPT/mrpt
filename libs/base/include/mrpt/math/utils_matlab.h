/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_math_utils_matlab_H
#define  mrpt_math_utils_matlab_H

/** \file Provide helper functions for MEX/MATLAB. 
  * This file can be safely included without checking MRPT_HAS_MATLAB
  */

#include <mrpt/config.h>
#if MRPT_HAS_MATLAB
#	include <mexplus.h>
#endif
#include <mrpt/math/math_frwds.h>

namespace mrpt
{
	namespace math
	{
#if MRPT_HAS_MATLAB
		/** \addtogroup matlab_grp Helper functions for MEX & MATLAB 
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Convert vectors, arrays and matrices into Matlab vectors/matrices. 
		  * Supported input classes:
		  *  - Eigen::Matrix<T,N,1>
		  *  - mrpt::math::CArrayNumeric<T,N>
		  *  - mrpt::math::CMatrix{*}
		  */
		template <typename Derived>
		mxArray* convertToMatlab(const Eigen::EigenBase<Derived>& mat)
		{
			const size_t m = mat.rows(), n = mat.cols();
			mxArray * mxa = mxCreateDoubleMatrix(m,n,mxREAL);
			double *mxa_data = mxGetPr(mxa);  // *IMPORTANT* Matlab stores matrices in *column-major* order!
			for (size_t j=0;j<n;j++) // column
				for	(size_t i=0;i<m;i++)  // rows
					*mxa_data++  = mat.derived().coeff(i,j);
			return mxa;
		}

		/** Convert std::vector<> or std::deque<> of numeric types into Matlab vectors */
		template <typename CONTAINER>
		mxArray* convertVectorToMatlab(const CONTAINER& vec)
		{
			const size_t m = vec.size(), n = 1;
			mxArray * mxa = mxCreateDoubleMatrix(m,n,mxREAL);
			double *mxa_data = mxGetPr(mxa);  // *IMPORTANT* Matlab stores matrices in *column-major* order!
			for	(size_t i=0;i<m;i++)  // rows
				*mxa_data++  = vec[i];
			return mxa;
		}

		/** @} */
#endif
	}
}

#endif
