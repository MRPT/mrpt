/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cstdlib>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/base/link_pragmas.h>
#include <mrpt/config.h>

namespace mrpt
{
	namespace math
	{
		/** @addtogroup  fresnel_integrals_grp Fresnel integrals (`#include <mrpt/math/fresnel.h>`)
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Evaluates the integral from 0 to x of sqrt(2/pi) sin(t^2) dt. Equivalent to MATLAB fresnels()
		  * \sa https://en.wikipedia.org/wiki/Fresnel_integral  
		  * \note Code based on http://www.mymathlib.com/functions/fresnel_sin_cos_integrals.html */
		double BASE_IMPEXP fresnel_sin_integral(double x) noexcept;

		/** Evaluates the integral from 0 to x of sqrt(2/pi) cos(t^2) dt. Equivalent to MATLAB fresnelc()
		  * \sa https://en.wikipedia.org/wiki/Fresnel_integral  
		  *\note Code based on http://www.mymathlib.com/functions/fresnel_sin_cos_integrals.html */
		double BASE_IMPEXP fresnel_cos_integral(double x) noexcept;

#ifdef HAVE_LONG_DOUBLE
		/** long double version of fresnel_sin_integral */
		long double BASE_IMPEXP lfresnel_sin_integral(long double x) noexcept;

		/** long double version of fresnel_cos_integral */
		long double BASE_IMPEXP lfresnel_cos_integral(long double x) noexcept;
#endif


		/** @} */

	} // End of MATH namespace

} // End of namespace

