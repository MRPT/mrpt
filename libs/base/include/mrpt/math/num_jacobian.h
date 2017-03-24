/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/core_defs.h>

namespace mrpt
{
	namespace math
	{
		/** Estimate the Jacobian of a multi-dimensional function around a point "x", using finite differences of a given size in each input dimension.
			*  The template argument USERPARAM is for the data can be passed to the functor.
			*   If it is not required, set to "int" or any other basic type.
			*
			*  This is a generic template which works with:
			*    VECTORLIKE: vector_float, CVectorDouble, CArrayNumeric<>, double [N], ...
			*    MATRIXLIKE: CMatrixTemplateNumeric, CMatrixFixedNumeric
			*/
		template <class VECTORLIKE,class VECTORLIKE2, class VECTORLIKE3, class MATRIXLIKE, class USERPARAM >
		void estimateJacobian(
			const VECTORLIKE 	&x,
			void 				(*functor) (const VECTORLIKE &x,const USERPARAM &y, VECTORLIKE3  &out),
			const VECTORLIKE2 	&increments,
			const USERPARAM		&userParam,
			MATRIXLIKE 			&out_Jacobian )
		{
			MRPT_START
			ASSERT_(x.size()>0 && increments.size() == x.size());

			size_t m = 0;		// will determine automatically on the first call to "f":
			const size_t n = x.size();

			for (size_t j=0;j<n;j++) { ASSERT_( increments[j]>0 ) }		// Who knows...

			VECTORLIKE3	f_minus, f_plus;
			VECTORLIKE	x_mod(x);

			// Evaluate the function "i" with increments in the "j" input x variable:
			for (size_t j=0;j<n;j++)
			{
				// Create the modified "x" vector:
				x_mod[j]=x[j]+increments[j];
				functor(x_mod,userParam,   f_plus);

				x_mod[j]=x[j]-increments[j];
				functor(x_mod,userParam,   f_minus);

				x_mod[j]=x[j]; // Leave as original
				const double Ax_2_inv = 0.5/increments[j];

				// The first time?
				if (j==0)
				{
					m = f_plus.size();
					out_Jacobian.setSize(m,n);
				}

				for (size_t i=0;i<m;i++)
					out_Jacobian.get_unsafe(i,j) = Ax_2_inv* (f_plus[i]-f_minus[i]);

			} // end for j

			MRPT_END
		}

	} // End of MATH namespace
} // End of namespace

