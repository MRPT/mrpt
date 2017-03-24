/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/filters.h>
#include <cmath>

using namespace mrpt::math;

LowPassFilter_IIR1::LowPassFilter_IIR1(double _alpha, double y_k_minus_1) :
	alpha(_alpha),
	m_y_km1(y_k_minus_1)
{
}

double LowPassFilter_IIR1::filter(double x)
{
	// y[k] = alpha*y[k - 1] + (1 - alpha)*x[k]
	const double y = alpha*m_y_km1 + (1 - alpha)*x;
	m_y_km1 = y;
	return y;
}

double LowPassFilter_IIR1::getLastOutput() const
{
	return m_y_km1;
}
