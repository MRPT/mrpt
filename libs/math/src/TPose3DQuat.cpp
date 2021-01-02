/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/core/bits_math.h>  // square
#include <mrpt/math/TPose3DQuat.h>
#include <cmath>

using namespace mrpt::math;

static_assert(std::is_trivially_copyable_v<TPose3DQuat>);

void TPose3DQuat::fromString(const std::string& s)
{
	CMatrixDouble m;
	if (!m.fromMatlabStringFormat(s))
		THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(
		m.rows() == 1 && m.cols() == 7, "Wrong size of vector in ::fromString");
	for (int i = 0; i < m.cols(); i++) (*this)[i] = m(0, i);
}

double TPose3DQuat::norm() const
{
	return std::sqrt(mrpt::square(x) + mrpt::square(y) + mrpt::square(z));
}
