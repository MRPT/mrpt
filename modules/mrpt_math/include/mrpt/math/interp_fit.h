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

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/wrap2pi.h>

namespace mrpt::math
{
/** @addtogroup interpolation_grp Interpolation, least-squares fit, splines
 * \ingroup mrpt_math_grp
 *  @{ */

/** Interpolate a data sequence "ys" ranging from "x0" to "x1" (equally spaced),
 * to obtain the approximation of the sequence at the point "x".
 * The N samples are taken as N cells covering [x0,x1): ys[i] is the value at
 * x0+i*(x1-x0)/N, hence ys[N-1] is returned for x >= x1-(x1-x0)/N.
 *  If the point "x" is out of the range [x0,x1], the closest extreme "ys"
 * value is returned.
 * \note Implementation in `#include <mrpt/math/interp_fit.hpp>`
 * \sa spline, interpolate2points
 */
template <class T, class VECTOR>
T interpolate(const T& x, const VECTOR& ys, const T& x0, const T& x1);

/** Linear interpolation/extrapolation: evaluates at "x" the line
 * (x0,y0)-(x1,y1).
 *  If wrap2pi is true, y0 and y1 are angles: the interpolation follows the
 * shortest arc between them, and the output is wrapped to [-pi,pi[.
 * \sa spline, interpolate, leastSquareLinearFit
 */
double interpolate2points(
    const double x,
    const double x0,
    const double y0,
    const double x1,
    const double y1,
    bool wrap2pi = false);

/** Interpolates the value of a function in a point "t" given 4 SORTED points
 * where "t" is between the two middle points
 *  If wrap2pi is true, the "y" values are angles: consecutive values are
 * unwrapped (assuming jumps below pi) and the output is wrapped to [-pi,pi[.
 * \sa leastSquareLinearFit
 * \note Implementation in `#include <mrpt/math/interp_fit.hpp>`
 */
template <typename NUMTYPE, class VECTORLIKE>
NUMTYPE spline(const NUMTYPE t, const VECTORLIKE& x, const VECTORLIKE& y, bool wrap2pi = false);

/** Interpolates or extrapolates using a least-square linear fit of the set of
 * values "x" and "y", evaluated at a single point "t".
 *  The vectors x and y must have size >=2, and all values of "x" must be
 * different.
 *  If wrap2pi is true, the "y" values are angles: consecutive values (in the
 * given order) are unwrapped, assuming jumps below pi, and the output is
 * wrapped to [-pi,pi[.
 * \sa spline
 * \sa getRegressionLine, getRegressionPlane
 * \note Implementation in `#include <mrpt/math/interp_fit.hpp>`
 */
template <typename NUMTYPE, class VECTORLIKE, int NUM_POINTS = -1 /*Eigen::Dynamic*/>
NUMTYPE leastSquareLinearFit(
    const NUMTYPE t, const VECTORLIKE& x, const VECTORLIKE& y, bool wrap2pi = false);

/** Interpolates or extrapolates using a least-square linear fit of the set of
 * values "x" and "y", evaluated at a sequence of points "ts" and returned at
 * "outs".
 *  If wrap2pi is true, the "y" values are angles: consecutive values (in the
 * given order) are unwrapped, assuming jumps below pi, and the output is
 * wrapped to [-pi,pi[.
 * \sa spline, getRegressionLine, getRegressionPlane
 * \note Implementation in `#include <mrpt/math/interp_fit.hpp>`
 * \note Requires `#include <Eigen/Dense>`
 */
template <
    class VECTORLIKE1,
    class VECTORLIKE2,
    class VECTORLIKE3,
    int NUM_POINTS = -1 /*Eigen::Dynamic*/>
void leastSquareLinearFit(
    const VECTORLIKE1& ts,
    VECTORLIKE2& outs,
    const VECTORLIKE3& x,
    const VECTORLIKE3& y,
    bool wrap2pi = false);

/** @} */  // end grouping interpolation_grp

}  // namespace mrpt::math
