/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdlib>
#include <cmath>

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
