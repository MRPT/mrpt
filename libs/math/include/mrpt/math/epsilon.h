/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::math
{
/**
 * Changes the value of the geometric epsilon (default = 1e-5)
 * \sa getEpsilon
 * \ingroup mrpt_math_grp
 */
void setEpsilon(double nE);
/**
 * Gets the value of the geometric epsilon  (default = 1e-5)
 * \sa setEpsilon
 * \ingroup mrpt_math_grp
 */
double getEpsilon();

}  // namespace mrpt::math
