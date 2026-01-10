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

#include <functional>  // reference_wrapper
#include <optional>    // optional

namespace mrpt
{
/** Shorter name for std::optional<std::reference_wrapper<T>>
 * \ingroup mrpt_core_grp
 */
template <class T>
using optional_ref = std::optional<std::reference_wrapper<T>>;

}  // namespace mrpt
