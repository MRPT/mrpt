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

namespace mrpt
{
/** Like calling a std::vector<>'s clear() method, but really forcing
 * deallocating the memory.
 * \ingroup mrpt_core_grp
 */
template <class VECTOR_T>
inline void vector_strong_clear(VECTOR_T& v)
{
  VECTOR_T dummy;
  dummy.swap(v);
}
}  // namespace mrpt
