/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/math/filters.h>

using namespace mrpt::math;

LowPassFilter_IIR1::LowPassFilter_IIR1(double _alpha, double y_k_minus_1)
	: alpha(_alpha), m_y_km1(y_k_minus_1)
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
