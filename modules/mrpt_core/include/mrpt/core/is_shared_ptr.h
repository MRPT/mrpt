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

#include <memory>
#include <type_traits>

namespace mrpt
{
/** This is useful for checking ::Ptr types.
 * I'm surprised it's not defined in <memory>
 */
template <class T>
struct is_shared_ptr : std::false_type
{
};
template <class T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type
{
};
}  // namespace mrpt