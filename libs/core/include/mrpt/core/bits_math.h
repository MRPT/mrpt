/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <algorithm>  // max()
#include <cmath>  // floor(),isnan(),...
#include <stdexcept>

namespace mrpt
{
/** \addtogroup mrpt_bits_math Funtions in #include <mrpt/core/bits_math.h>
 *  \ingroup mrpt_core_grp
 * @{ */

/** Inline function for the square of a number. */
template <typename num_t, typename return_t = num_t>
inline return_t square(const num_t x)
{
	return static_cast<return_t>(x * x);
}

/** Faster version of std::hypot(), to use when overflow is not an issue and we
 * prefer fast code. */
template <class T>
inline T hypot_fast(const T x, const T y)
{
	return std::sqrt(x * x + y * y);
}

#ifdef DEG2RAD  // functions are preferred over macros
#undef DEG2RAD
#endif
#ifdef RAD2DEG
#undef RAD2DEG
#endif
#if !defined(M_PI)
#define M_PI 3.14159265358979323846
#endif

/** Degrees to radians  */
constexpr inline double DEG2RAD(const double x) { return x * M_PI / 180.0; }
/** Degrees to radians */
constexpr inline float DEG2RAD(const float x)
{
	return x * float(M_PI) / 180.0f;
}
/** Degrees to radians */
constexpr inline double DEG2RAD(const int x) { return x * M_PI / 180.0; }
/** Radians to degrees */
constexpr inline double RAD2DEG(const double x) { return x * 180.0 / M_PI; }
/** Radians to degrees */
constexpr inline float RAD2DEG(const float x)
{
	return x * 180.0f / float(M_PI);
}
#if !defined(M_PIl)
#define M_PIl 3.14159265358979323846264338327950288L
#define M_2PIl (2.0L * 3.14159265358979323846264338327950288L)
#endif
/** Degrees to radians */
constexpr inline long double DEG2RAD(const long double x)
{
	return x * M_PIl / 180.0;
}
/** Radians to degrees */
constexpr inline long double RAD2DEG(const long double x)
{
	return x * 180.0 / M_PIl;
}

// This is required to avoid other libs (like PCL) to #define their own macros
// after including this header
#define DEG2RAD DEG2RAD
#define RAD2DEG RAD2DEG

/** degrees to radian literal operator (e.g. `x=90.0_deg;`) */
constexpr inline double operator"" _deg(long double v)
{
	return static_cast<double>(mrpt::DEG2RAD(v));
}

/** Returns the sign of X as "1" or "-1" */
template <typename T>
inline int sign(T x)
{
	return x < 0 ? -1 : 1;
}

/** Returns the sign of X as "0", "1" or "-1" */
template <typename T>
inline int signWithZero(T x)
{
	return (x == 0 || x == -0) ? 0 : sign(x);
}

/** Returns the smallest positive number among a and b */
template <typename T>
T lowestPositive(const T a, const T b)
{
	if (a > 0 && a <= b)
		return a;  // a positive and smaller than b
	else if (b > 0)
		return b;  // b is positive and either smaller than a or a is negative
	else
		return a;  // at least b is negative, we might not have an answer
}

/** Efficient and portable evaluation of the absolute difference of two unsigned
 * integer values
 * (but will also work for signed and floating point types) */
template <typename T>
inline T abs_diff(const T a, const T b)
{
	return std::max(a, b) - std::min(a, b);
}

template <typename T>
inline const T min3(const T& A, const T& B, const T& C)
{
	return std::min<T>(A, std::min<T>(B, C));
}
template <typename T>
inline const T max3(const T& A, const T& B, const T& C)
{
	return std::max<T>(A, std::max<T>(B, C));
}

/** Rounds toward zero  */
template <typename T>
inline int fix(T x)
{
	return x > 0 ? static_cast<int>(floor(static_cast<double>(x)))
				 : static_cast<int>(ceil(static_cast<double>(x)));
}

/** If the second argument is below the first one, set the first argument to
 * this lower value. */
template <typename T, typename K>
inline void keep_min(T& var, const K test_val)
{
	if (test_val < var) var = static_cast<T>(test_val);
}
/** If the second argument is above the first one, set the first argument to
 * this higher value. */
template <typename T, typename K>
inline void keep_max(T& var, const K test_val)
{
	if (test_val > var) var = static_cast<T>(test_val);
}
/** Saturate the value of var (the variable gets modified) so it does not get
 * out of [min,max]. */
template <typename T>
inline void saturate(T& var, const T sat_min, const T sat_max)
{
	if (var > sat_max) var = sat_max;
	if (var < sat_min) var = sat_min;
}
/** Like saturate() but it returns the value instead of modifying the variable
 */
template <typename T>
inline T saturate_val(const T& value, const T sat_min, const T sat_max)
{
	T var = value;
	if (var > sat_max) var = sat_max;
	if (var < sat_min) var = sat_min;
	return var;
}

/** Round up to the nearest power of two of a given number */
template <class T>
T round2up(T val)
{
	T n = 1;
	while (n < val)
	{
		n <<= 1;
		if (n <= 1) throw std::invalid_argument("round2up: Overflow!");
	}
	return n;
}

/** shortcut for static_cast<float>(double) */
inline float d2f(const double d) { return static_cast<float>(d); }

/** converts a float [0,1] into an uint8_t [0,255] (without checking for out of
 * bounds) \sa u8tof */
inline uint8_t f2u8(const float f) { return static_cast<uint8_t>(f * 255); }

/** converts a uint8_t [0,255] into a float [0,1] \sa f2u8 */
inline float u8tof(const uint8_t v) { return v / 255.0f; }

/** @} */

}  // namespace mrpt
