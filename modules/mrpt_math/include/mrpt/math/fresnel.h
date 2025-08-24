/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#pragma once
#include <cstdlib>

namespace mrpt::math
{
/** @addtogroup  fresnel_integrals_grp Fresnel integrals (`#include
 * <mrpt/math/fresnel.h>`)
 *  \ingroup mrpt_math_grp
 * @{ */

/** Evaluates the integral from 0 to x of sqrt(2/pi) sin(t^2) dt. Equivalent to
 * MATLAB fresnels()
 * \sa https://en.wikipedia.org/wiki/Fresnel_integral
 * \note Code based on
 * http://www.mymathlib.com/functions/fresnel_sin_cos_integrals.html */
double fresnel_sin_integral(double x) noexcept;

/** Evaluates the integral from 0 to x of sqrt(2/pi) cos(t^2) dt. Equivalent to
 *MATLAB fresnelc()
 * \sa https://en.wikipedia.org/wiki/Fresnel_integral
 *\note Code based on
 *http://www.mymathlib.com/functions/fresnel_sin_cos_integrals.html */
double fresnel_cos_integral(double x) noexcept;

/** long double version of fresnel_sin_integral */
long double lfresnel_sin_integral(long double x) noexcept;

/** long double version of fresnel_cos_integral */
long double lfresnel_cos_integral(long double x) noexcept;

/** @} */

}  // namespace mrpt::math
