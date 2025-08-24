/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <algorithm>  // max()

namespace mrpt
{
/** Efficient and portable evaluation of the absolute difference of two
 * unsigned integer values; but also works for signed and floating point types
 * \ingroup mrpt_bits_math
 */
template <typename T>
inline T abs_diff(const T a, const T b)
{
  return std::max(a, b) - std::min(a, b);
}

}  // namespace mrpt