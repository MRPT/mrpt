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

#include <mrpt/math/filters.h>

using namespace mrpt::math;

LowPassFilter_IIR1::LowPassFilter_IIR1(double _alpha, double y_k_minus_1) :
    alpha(_alpha), m_y_km1(y_k_minus_1)
{
}

double LowPassFilter_IIR1::filter(double x)
{
  // y[k] = alpha*y[k - 1] + (1 - alpha)*x[k]
  const double y = alpha * m_y_km1 + (1 - alpha) * x;
  m_y_km1 = y;
  return y;
}

double LowPassFilter_IIR1::getLastOutput() const { return m_y_km1; }
