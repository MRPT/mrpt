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
