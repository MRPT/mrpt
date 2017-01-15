/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSMATH_H
#define XSMATH_H

#include "xstypesconfig.h"
#include "xstypedefs.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API extern const XsReal XsMath_e;
XSTYPES_DLL_API extern const XsReal XsMath_pi;
XSTYPES_DLL_API extern const XsReal XsMath_tinyValue;
XSTYPES_DLL_API extern const XsReal XsMath_hugeValue;
XSTYPES_DLL_API extern const XsReal XsMath_epsilon;
XSTYPES_DLL_API extern const XsReal XsMath_sqrtEpsilon;
XSTYPES_DLL_API extern const XsReal XsMath_denormalized;
XSTYPES_DLL_API extern const XsReal XsMath_sqrtDenormalized;
XSTYPES_DLL_API extern const XsReal XsMath_rad2degValue;
XSTYPES_DLL_API extern const XsReal XsMath_deg2radValue;
XSTYPES_DLL_API extern const XsReal XsMath_zero;
XSTYPES_DLL_API extern const XsReal XsMath_pt25;
XSTYPES_DLL_API extern const XsReal XsMath_pt5;
XSTYPES_DLL_API extern const XsReal XsMath_minusPt5;
XSTYPES_DLL_API extern const XsReal XsMath_one;
XSTYPES_DLL_API extern const XsReal XsMath_minusOne;
XSTYPES_DLL_API extern const XsReal XsMath_two;
XSTYPES_DLL_API extern const XsReal XsMath_four;
XSTYPES_DLL_API extern const XsReal XsMath_minusTwo;
XSTYPES_DLL_API extern const XsReal XsMath_minusHalfPi;
XSTYPES_DLL_API extern const XsReal XsMath_halfPi;
XSTYPES_DLL_API extern const XsReal XsMath_sqrt2;
XSTYPES_DLL_API extern const XsReal XsMath_infinity;

XSTYPES_DLL_API XsReal XsMath_rad2deg(XsReal radians);
XSTYPES_DLL_API XsReal XsMath_deg2rad(XsReal degrees);
XSTYPES_DLL_API XsReal XsMath_pow2(XsReal a);
XSTYPES_DLL_API XsReal XsMath_pow3(XsReal a);
XSTYPES_DLL_API XsReal XsMath_asinClamped(XsReal x);
XSTYPES_DLL_API int XsMath_isFinite(XsReal x);
XSTYPES_DLL_API int32_t XsMath_doubleToLong(double d);
#ifndef XSENS_NO_INT64
XSTYPES_DLL_API int64_t XsMath_doubleToInt64(double d);
#endif

#ifdef __cplusplus
} // extern "C"

namespace XsMath {
	//! \brief The value e
	static const XsReal& e = XsMath_e;
	//! \brief The value pi
	static const XsReal& pi = XsMath_pi;
	//! \brief A really small value
	static const XsReal& tinyValue = XsMath_tinyValue;
	//! \brief A convincingly large number
	static const XsReal& hugeValue = XsMath_hugeValue;
	//! \brief A value related to the precision of floating point arithmetic (2.2204460492503131e-016)
	static const XsReal& epsilon = XsMath_epsilon;
	/*! \brief Square root of epsilon
		\sa epsilon
	*/
	static const XsReal& sqrtEpsilon = XsMath_sqrtEpsilon;
	//! \brief Value that represents the subnormal number in floating point wizardry
	static const XsReal& denormalized = XsMath_denormalized;
	/*! \brief Square root of denormalized
		\sa denormalized
	*/
	static const XsReal& sqrtDenormalized = XsMath_sqrtDenormalized;
	//! \brief Value to convert radians to degrees by multiplication
	static const XsReal& rad2degValue = XsMath_rad2degValue;
	//! \brief Value to convert degrees to radians by multiplication
	static const XsReal& deg2radValue = XsMath_deg2radValue;
	//! \brief 0
	static const XsReal& zero = XsMath_zero;
	//! \brief 0.25
	static const XsReal& pt25 = XsMath_pt25;
	//! \brief 0.5
	static const XsReal& pt5 = XsMath_pt5;
	//! \brief -0.5
	static const XsReal& minusPt5 = XsMath_minusPt5;
	//! \brief 1
	static const XsReal& one = XsMath_one;
	//! \brief -1
	static const XsReal& minusOne = XsMath_minusOne;
	//! \brief 2
	static const XsReal& two = XsMath_two;
	//! \brief 4
	static const XsReal& four = XsMath_four;
	//! \brief -2
	static const XsReal& minusTwo = XsMath_minusTwo;
	//! \brief -pi/2
	static const XsReal& minusHalfPi = XsMath_minusHalfPi;
	//! \brief pi/2
	static const XsReal& halfPi = XsMath_halfPi;
	//! \brief sqrt(2)
	static const XsReal& sqrt2 = XsMath_sqrt2;
	//! \brief infinity
	static const XsReal& infinity = XsMath_infinity;


	//! \brief \copybrief XsMath_asinClamped
	inline XsReal asinClamped(XsReal x)
	{
		return XsMath_asinClamped(x);
	}
	//! \brief \copybrief XsMath_rad2deg
	inline XsReal rad2deg(XsReal radians)
	{
		return XsMath_rad2deg(radians);
	}
	//! \brief \copybrief XsMath_deg2rad
	inline XsReal deg2rad(XsReal degrees)
	{
		return XsMath_deg2rad(degrees);
	}
	//! \brief \copybrief XsMath_pow2
	inline XsReal pow2(XsReal a)
	{
		return XsMath_pow2(a);
	}
	//! \brief \copybrief XsMath_pow3
	inline XsReal pow3(XsReal a)
	{
		return XsMath_pow3(a);
	}
	//! \brief \copybrief XsMath_doubleToLong
	inline int32_t doubleToLong(double d)
	{
		return XsMath_doubleToLong(d);
	}
#ifndef XSENS_NO_INT64
	//! \brief \copybrief XsMath_doubleToInt64
	inline int64_t doubleToInt64(double d)
	{
		return XsMath_doubleToInt64(d);
	}
#endif
}	// namespace
#endif

#endif // file guard
