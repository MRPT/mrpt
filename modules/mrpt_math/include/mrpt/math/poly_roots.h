/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#pragma once

#include <cstdlib>

namespace mrpt::math
{
/** @addtogroup  polynomial_roots Find polynomial roots (`#include
 * <mrpt/math/poly_roots.h>`)
 *  \ingroup mrpt_math_grp
 * @{ */

/** Solves cubic equation `x^3 + a*x^2 + b*x + c = 0`. Returns the number of
 * real roots `N`<=3.
 * The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1,
 * `x[0]` and `x[1]` if N=2, etc.
 * - return 3: 3 real roots x[0], x[1], x[2].
 * - return 2: x[0] and the double root x[1] (also copied into x[2]).
 * - return 1: real root x[0] and the complex pair x[1]+-i*x[2].
 * \param x array of size 3
 * \note Based on `poly34.h`, by Khashin S.I.
 * https://web.archive.org/web/20160402113335/http://math.ivanovo.ac.ru/dalgebra/Khashin/poly/index.html
 * (khash2 (at) gmail.com)
 */
int solve_poly3(double* x, double a, double b, double c) noexcept;

/** Solves quartic equation `x^4 + a*x^3 + b*x^2 + c*x + d = 0` by the
 * Descartes-Euler method.
 * Returns the number of real roots `N`<=4:
 * - return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
 * - return 2: 2 real roots x[0], x[1] and complex x[2]+-i*x[3],
 * - return 0: two pair of complex roots: x[0]+-i*x[1],  x[2]+-i*x[3],
 *
 * The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1,
 * `x[0]` and `x[1]` if N=2, etc.
 * \param x array of size 4
 * \note Based on `poly34.h`, by Khashin S.I.
 * https://web.archive.org/web/20160402113335/http://math.ivanovo.ac.ru/dalgebra/Khashin/poly/index.html
 * (khash2 (at) gmail.com)
 */
int solve_poly4(double* x, double a, double b, double c, double d) noexcept;

/** Solves equation `x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0`.
 * Returns the number of real roots `N`<=5.
 * The roots are returned in the first entries of `x`, i.e. `x[0]` if N=1,
 * `x[0]` and `x[1]` if N=2, etc.
 * \param x array of size 5
 * \note Based on `poly34.h`, by Khashin S.I.
 * https://web.archive.org/web/20160402113335/http://math.ivanovo.ac.ru/dalgebra/Khashin/poly/index.html
 * (khash2 (at) gmail.com)
 */
int solve_poly5(double* x, double a, double b, double c, double d, double e) noexcept;

/** Solves equation `a*x^2 + b*x + c = 0`.
 * Returns the number of real roots: either 0 or 2; or 1 if a=0 (in this case
 * the root is in r1); or 0 if a=b=0. "Zero" here means an absolute value
 * below 1e-14.
 * r1, r2 are the roots. (r1<=r2)
 * \note Based on `poly34.h`, by Khashin S.I.
 * https://web.archive.org/web/20160402113335/http://math.ivanovo.ac.ru/dalgebra/Khashin/poly/index.html
 * (khash2 (at) gmail.com)
 */
int solve_poly2(double a, double b, double c, double& r1, double& r2) noexcept;

/** @} */

}  // namespace mrpt::math
