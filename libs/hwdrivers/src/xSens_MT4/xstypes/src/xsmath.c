/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsmath.h"
#include <math.h>
#include <float.h>

/*! \namespace XsMath
	\brief Namespace for mathematical constants and operations.
*/
/*! \addtogroup cinterface C Interface
	@{
*/

//! \brief The value e
const XsReal XsMath_e = 2.7182818284590452353602874713527;
//! \brief The value pi
const XsReal XsMath_pi = 3.1415926535897932384626433832795;
//! \brief A really small value
const XsReal XsMath_tinyValue = 1.0e-16;
//! \brief A convincingly large number
const XsReal XsMath_hugeValue = 1.0e+16;

//! \brief A value related to the precisson of floating point arithmetic (2.2204460492503131e-016)
const XsReal XsMath_epsilon = 2.2204460492503131e-016;
//! \brief Square root of XsMath_epsilon
const XsReal XsMath_sqrtEpsilon = 1.4901161193847656e-008;

#ifdef XSENS_SINGLE_PRECISION
	//! \brief Value that represents the subnormal number in floating point wizardry
	const XsReal XsMath_denormalized = 1e-37;
	//! \brief Square root of XsMath_denormalized
	const XsReal XsMath_sqrtDenormalized = 3.1622776601683793319988935444327e-19;
#else
	//! \brief Value that represents the subnormal number in floating point wizardry
	const XsReal XsMath_denormalized = 1e-307;
	//! \brief Square root of XsMath_denormalized
	const XsReal XsMath_sqrtDenormalized = 3.1622776601683793319988935444327e-154;
#endif

//! \brief Value to convert radians to degrees by multiplication
const XsReal XsMath_rad2degValue = 57.295779513082320876798154814105;	// (180.0/pi)
//! \brief Value to convert degrees to radians by multiplication
const XsReal XsMath_deg2radValue = 0.017453292519943295769236907684886;	// (pi/180.0)

//! \brief 0
const XsReal XsMath_zero = 0.0;
//! \brief 0.25
const XsReal XsMath_pt25 = 0.25;
//! \brief 0.5
const XsReal XsMath_pt5 = 0.5;
//! \brief -0.5
const XsReal XsMath_minusPt5 = -0.5;
//! \brief 1.0
const XsReal XsMath_one = 1.0;
//! \brief -1.0
const XsReal XsMath_minusOne = -1.0;
//! \brief 2
const XsReal XsMath_two = 2.0;
//! \brief 4
const XsReal XsMath_four = 4.0;
//! \brief -2
const XsReal XsMath_minusTwo = -2.0;

//! \brief -pi/2
const XsReal XsMath_minusHalfPi = -1.570796326794897;
//! \brief pi/2
const XsReal XsMath_halfPi = +1.570796326794897;
//! \brief sqrt(2)
const XsReal XsMath_sqrt2 = 1.4142135623730950488016887242097;

#ifdef XSENS_SINGLE_PRECISION
//! \brief infinity value
const XsReal XsMath_infinity = FLT_MAX;
#else
//! \brief infinity value
const XsReal XsMath_infinity = DBL_MAX;
#endif

/*! \brief Returns asin(\a x) for -1 < x < 1
*/
XsReal XsMath_asinClamped(XsReal x)
{
	if (x <= XsMath_minusOne)
		return XsMath_minusHalfPi;

	if (x >= XsMath_one)
		return XsMath_halfPi;

	return asin(x);
}

/*!	\brief Convert radians to degrees
*/
XsReal XsMath_rad2deg(XsReal radians)
{
	return XsMath_rad2degValue * radians;
}

/*!	\brief Convert degrees to radians
*/
XsReal XsMath_deg2rad(XsReal degrees)
{
	return XsMath_deg2radValue * degrees;
}

/*!	\brief Returns \a a to the power of 2
*/
XsReal XsMath_pow2(XsReal a)
{
	return a*a;
}

/*!	\brief Returns \a a to the power of 3
*/
XsReal XsMath_pow3(XsReal a)
{
	return a*a*a;
}

/*! \brief Returns non-zero if \a x is finite
*/
int XsMath_isFinite(XsReal x)
{
#ifdef _MSC_VER
	return _finite(x);
#elif defined(APPLE)
	return isfinite(x);
#elif defined(__GNUC__)
	return isfinite(x);
#else
	return 1;
#endif
}

/*! \brief Returns \a d integer converted from a double precision floating point value
*/
int32_t XsMath_doubleToLong(double d)
{
	if (d >= 0)
		return (int32_t) floor(d+0.5);
	else
		return (int32_t) ceil(d-0.5);
}

/*! \brief Returns \a d integer converted from a double precision floating point value
*/
int64_t XsMath_doubleToInt64(double d)
{
	if (d >= 0)
		return (int64_t) floor(d+0.5);
	else
		return (int64_t) ceil(d-0.5);
}

/*! @} */
