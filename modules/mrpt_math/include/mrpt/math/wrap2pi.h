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

#include <mrpt/core/bits_math.h>

#include <cmath>
#include <cstddef>  // size_t

namespace mrpt::math
{
/** \addtogroup container_ops_grp
 * @{ */

/** Modifies the given angle to translate it into the [0,2pi[ range.
 * \note Take care of not instancing this template for integer numbers, since
 * it only works for float, double and long double.
 * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
 */
template <class T>
void wrapTo2PiInPlace(T& a)
{
  a = static_cast<T>(fmod(a, static_cast<T>(2.0 * M_PI)));
  if (a < 0)
  {
    a += static_cast<T>(2.0 * M_PI);
  }
  // Tiny negative values round up to exactly 2pi:
  if (a >= static_cast<T>(2.0 * M_PI))
  {
    a = 0;
  }
}

/** Modifies the given angle to translate it into the [0,2pi[ range.
 * \note Take care of not instancing this template for integer numbers, since
 * it only works for float, double and long double.
 * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
 */
template <class T>
T wrapTo2Pi(T a)
{
  wrapTo2PiInPlace(a);
  return a;
}

/** Modifies the given angle to translate it into the [-pi,pi[ range.
 * Note that both pi and -pi are mapped to -pi.
 * \note Take care of not instancing this template for integer numbers, since
 * it only works for float, double and long double.
 * \sa wrapTo2Pi, wrapToPiInPlace, unwrap2PiSequence
 */
template <class T>
T wrapToPi(T a)
{
  return wrapTo2Pi(a + static_cast<T>(M_PI)) - static_cast<T>(M_PI);
}

/** Modifies the given angle to translate it into the [-pi,pi[ range.
 * \note Take care of not instancing this template for integer numbers, since
 * it only works for float, double and long double.
 * \sa wrapToPi,wrapTo2Pi, unwrap2PiSequence
 */
template <class T>
void wrapToPiInPlace(T& a)
{
  a = wrapToPi(a);
}

/** Computes the shortest angular increment (or distance) between two planar
 * orientations,
 * such that it is constrained to [-pi,pi] and is correct for any combination
 * of angles (e.g. near +-pi)
 * Examples: angDistance(0,pi) -> -pi; angDistance(pi,0) -> +pi (since pi is
 * wrapped to -pi); angDistance(-3.1,3.1) -> -0.08; angDistance(3.1,-3.1) -> +0.08;
 * \note Take care of not instancing this template for integer numbers, since
 * it only works for float, double and long double.
 * \sa wrapToPi, wrapTo2Pi
 */
template <class T>
T angDistance(T from, T to)
{
  wrapToPiInPlace(from);
  wrapToPiInPlace(to);
  T d = to - from;
  if (d > M_PI)
    d -= 2. * M_PI;
  else if (d < -M_PI)
    d += 2. * M_PI;
  return d;
}

/** Modify a sequence of angle values such as no consecutive values have a jump
 * larger than PI in absolute value. The first value is wrapped to [-pi,pi[,
 * the rest may grow beyond that range to follow multiple turns.
 * \sa wrapToPi
 */
template <class VECTOR>
void unwrap2PiSequence(VECTOR& x)
{
  const size_t N = x.size();
  if (N == 0)
  {
    return;
  }
  mrpt::math::wrapToPiInPlace(x[0]);
  for (size_t i = 1; i < N; i++)
  {
    x[i] = x[i - 1] + mrpt::math::angDistance(x[i - 1], x[i]);
  }
}

/** @} */

}  // namespace mrpt::math
