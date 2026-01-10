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

#include <cmath>
#include <cstdlib>

namespace mrpt::math
{
/** @addtogroup filtering_grp Filtering algorithms
 * \ingroup mrpt_math_grp
 *  @{ */

/** 1-order low-pass IIR filter.
 * Discrete time equation: `y[k]=alpha*y[k-1]+(1-alpha)*x[k]`.
 * With: x[k] input, y[k] output, alpha a parameter in [0,1]
 */
struct LowPassFilter_IIR1
{
  LowPassFilter_IIR1(double alpha = 0.5, double y_k_minus_1 = .0);
  /** Processes one input sample, updates the filter state and return the
   * filtered value. */
  double filter(double x);
  double getLastOutput() const;
  /** See equation in LowPassFilter_IIR1 */
  double alpha;

 private:
  double m_y_km1;
};

/** @} */  // end grouping filtering_grp
}  // namespace mrpt::math
