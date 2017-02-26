/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_MATH_FIT_INTERP_H
#define  MRPT_MATH_FIT_INTERP_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/wrap2pi.h>

namespace mrpt
{
	namespace math
	{

		/** @addtogroup interpolation_grp Interpolation, least-squares fit, splines
		  * \ingroup mrpt_base_grp
		  *  @{ */

		/** Interpolate a data sequence "ys" ranging from "x0" to "x1" (equally spaced), to obtain the approximation of the sequence at the point "x".
		  *  If the point "x" is out of the range [x0,x1], the closest extreme "ys" value is returned.
		  * \sa spline, interpolate2points
		  */
		template <class T,class VECTOR>
		T interpolate(
			const T			&x,
			const VECTOR	&ys,
			const T			&x0,
			const T			&x1 )
		{
			MRPT_START
			ASSERT_(x1>x0); ASSERT_(!ys.empty());
			const size_t N = ys.size();
			if (x<=x0)	return ys[0];
			if (x>=x1)	return ys[N-1];
			const T Ax = (x1-x0)/T(N);
			const size_t i = int( (x-x0)/Ax );
			if (i>=N-1) return ys[N-1];
			const T Ay = ys[i+1]-ys[i];
			return ys[i] + (x-(x0+i*Ax))*Ay/Ax;
			MRPT_END
		}

		/** Linear interpolation/extrapolation: evaluates at "x" the line (x0,y0)-(x1,y1).
		  *  If wrap2pi is true, output is wrapped to ]-pi,pi] (It is assumed that input "y" values already are in the correct range).
		  * \sa spline, interpolate, leastSquareLinearFit
		  */
		double BASE_IMPEXP interpolate2points(const double x, const double x0, const double y0, const double x1, const double y1, bool wrap2pi = false);

		/** Interpolates the value of a function in a point "t" given 4 SORTED points where "t" is between the two middle points
		  *  If wrap2pi is true, output "y" values are wrapped to ]-pi,pi] (It is assumed that input "y" values already are in the correct range).
		  * \sa leastSquareLinearFit
		  */
		double BASE_IMPEXP  spline(const double t, const CVectorDouble &x, const CVectorDouble &y, bool wrap2pi = false);

		/** Interpolates or extrapolates using a least-square linear fit of the set of values "x" and "y", evaluated at a single point "t".
		  *  The vectors x and y must have size >=2, and all values of "x" must be different.
		  *  If wrap2pi is true, output "y" values are wrapped to ]-pi,pi] (It is assumed that input "y" values already are in the correct range).
		  * \sa spline
		  * \sa getRegressionLine, getRegressionPlane
		  */
		template <typename NUMTYPE,class VECTORLIKE>
		NUMTYPE leastSquareLinearFit(const NUMTYPE t, const VECTORLIKE &x, const VECTORLIKE &y, bool wrap2pi = false)
		{
			MRPT_START

			// http://en.wikipedia.org/wiki/Linear_least_squares
			ASSERT_(x.size()==y.size());
			ASSERT_(x.size()>1);

			const size_t N = x.size();

			typedef typename VECTORLIKE::Scalar NUM;

			// X= [1 columns of ones, x' ]
			const NUM x_min = x.minimum();
			mrpt::math::CMatrixTemplateNumeric<NUM> Xt(2,N);
			for (size_t i=0;i<N;i++)
			{
				Xt.set_unsafe(0,i, 1);
				Xt.set_unsafe(1,i, x[i]-x_min);
			}

			mrpt::math::CMatrixTemplateNumeric<NUM> XtX;
			XtX.multiply_AAt(Xt);

			mrpt::math::CMatrixTemplateNumeric<NUM> XtXinv;
			XtX.inv_fast(XtXinv);

			mrpt::math::CMatrixTemplateNumeric<NUM> XtXinvXt;	// B = inv(X' * X)*X'  (pseudoinverse)
			XtXinvXt.multiply(XtXinv,Xt);

			VECTORLIKE B;
			XtXinvXt.multiply_Ab(y,B);

			ASSERT_(B.size()==2)

			NUM ret = B[0] + B[1]*(t-x_min);

			// wrap?
			if (!wrap2pi)
					return ret;
			else 	return mrpt::math::wrapToPi(ret);

			MRPT_END
		}

		/** Interpolates or extrapolates using a least-square linear fit of the set of values "x" and "y", evaluated at a sequence of points "ts" and returned at "outs".
		  *  If wrap2pi is true, output "y" values are wrapped to ]-pi,pi] (It is assumed that input "y" values already are in the correct range).
		  * \sa spline, getRegressionLine, getRegressionPlane
		  */
		template <class VECTORLIKE1,class VECTORLIKE2,class VECTORLIKE3>
		void leastSquareLinearFit(
			const VECTORLIKE1 &ts,
			VECTORLIKE2 &outs,
			const VECTORLIKE3 &x,
			const VECTORLIKE3 &y,
			bool wrap2pi = false)
		{
			MRPT_START

			// http://en.wikipedia.org/wiki/Linear_least_squares
			ASSERT_(x.size()==y.size());
			ASSERT_(x.size()>1);

			const size_t N = x.size();

			// X= [1 columns of ones, x' ]
			typedef typename VECTORLIKE3::Scalar NUM;
			const NUM x_min = x.minimum();
			mrpt::math::CMatrixTemplateNumeric<NUM> Xt(2,N);
			for (size_t i=0;i<N;i++)
			{
				Xt.set_unsafe(0,i, 1);
				Xt.set_unsafe(1,i, x[i]-x_min);
			}

			mrpt::math::CMatrixTemplateNumeric<NUM> XtX;
			XtX.multiply_AAt(Xt);

			mrpt::math::CMatrixTemplateNumeric<NUM> XtXinv;
			XtX.inv_fast(XtXinv);

			mrpt::math::CMatrixTemplateNumeric<NUM> XtXinvXt;	// B = inv(X' * X)*X' (pseudoinverse)
			XtXinvXt.multiply(XtXinv,Xt);

			VECTORLIKE3 B;
			XtXinvXt.multiply_Ab(y,B);

			ASSERT_(B.size()==2)

			const size_t tsN = size_t(ts.size());
			outs.resize(tsN);
			if (!wrap2pi)
				for (size_t k=0;k<tsN;k++)
					outs[k] = B[0] + B[1]*(ts[k]-x_min);
			else
				for (size_t k=0;k<tsN;k++)
					outs[k] = mrpt::math::wrapToPi( B[0] + B[1]*(ts[k]-x_min) );
			MRPT_END
		}

		/** @} */  // end grouping interpolation_grp


	} // End of MATH namespace
} // End of namespace

#endif
