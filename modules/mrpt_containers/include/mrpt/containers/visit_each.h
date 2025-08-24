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

#include <array>
#include <functional>  // invoke
#include <type_traits>

namespace mrpt
{
/** \addtogroup mrpt_containers_grp
 * @{ */

template <class Visitor, class... T>
void visit_each(const Visitor& vis, T&&... t)
{
  (std::invoke(vis, std::forward<T>(t)), ...);
}
/** @} */

}  // namespace mrpt
