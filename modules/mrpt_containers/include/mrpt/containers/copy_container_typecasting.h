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

namespace mrpt::containers
{
/** \addtogroup mrpt_containers_grp
 * @{ */

/** Copy all the elements in a container (vector, deque, list) into a different
 * one performing the appropriate typecasting.
 *  The target container is automatically resized to the appropriate size, and
 * previous contents are lost.
 *  This can be used to assign std::vector's of different types:
 * \code
 *   std::vector<int>    vi(10);
 *   std::vector<float>  vf;
 *   vf = vi;   // Compiler error
 *   mrpt::containers::copy_container_typecasting(v1,vf);  // Ok
 * \endcode
 */
template <typename src_container, typename dst_container>
inline void copy_container_typecasting(const src_container& src, dst_container& trg)
{
  trg.resize(src.size());
  auto i = src.begin();
  auto last = src.end();
  auto target = trg.begin();
  for (; i != last; ++i, ++target) *target = static_cast<typename dst_container::value_type>(*i);
}
/** @} */  // end of grouping
}  // namespace mrpt::containers
