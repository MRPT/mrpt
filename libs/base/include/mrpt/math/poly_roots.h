/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <cstdlib>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
	namespace math
	{
		/** @addtogroup  polynomial_roots Find polynomial roots (`#include <mrpt/math/poly_roots.h>`)
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Solves cubic equation `x^3 + a*x^2 + b*x + c = 0`. Returns the number of real roots `N`<=3.
		  * The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1, `x[0]` and `x[1]` if N=2, etc.
		  * \param x array of size 3
		  * \note Based on `poly34.h`, by Khashin S.I. http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com */
		int BASE_IMPEXP solve_poly3(double *x,double a,double b,double c) MRPT_NO_THROWS;

		/** Solves quartic equation `x^4 + a*x^3 + b*x^2 + c*x + d = 0` by Dekart-Euler method. 
		  * Returns the number of real roots `N`<=4:
		  * - return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
		  * - return 2: 2 real roots x[0], x[1] and complex x[2]+-i*x[3],
		  * - return 0: two pair of complex roots: x[0]+-i*x[1],  x[2]+-i*x[3],
		  *
		  * The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1, `x[0]` and `x[1]` if N=2, etc.
		  * \param x array of size 4
		  * \note Based on `poly34.h`, by Khashin S.I. http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com */
		int BASE_IMPEXP solve_poly4(double *x,double a,double b,double c,double d) MRPT_NO_THROWS;

		/** Solves equation `x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0`.
		  * Returns the number of real roots `N`<=5.
		  * The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1, `x[0]` and `x[1]` if N=2, etc.
		  * \param x array of size 5
		  * \note Based on `poly34.h`, by Khashin S.I. http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com */
		int BASE_IMPEXP solve_poly5(double *x,double a,double b,double c,double d,double e) MRPT_NO_THROWS;

		int  BASE_IMPEXP  solve_poly4Bi(double *x, double b, double d) MRPT_NO_THROWS; //!< Solve equation x^4 + b*x^2 + d = 0
		int  BASE_IMPEXP  solve_poly4De(double *x, double b, double c, double d) MRPT_NO_THROWS;	//!< Solve equation x^4 + b*x^2 + c*x + d = 0

		/** Solves equation `a*x^2 + b*x + c = 0`.
		  * Returns the number of real roots: either 0 or 2; or 1 if a=0 (in this case the root is in r1).
		  * r1, r2 are the roots. (r1<=r2)
		  * \note Based on `poly34.h`, by Khashin S.I. http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html - khash2 (at) gmail.com */
		int BASE_IMPEXP solve_poly2( double a, double b, double c, double &r1, double &r2) MRPT_NO_THROWS;

		/** @} */

	} // End of MATH namespace

} // End of namespace

