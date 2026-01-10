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

#include <type_traits>

namespace mrpt
{
template <typename T>
class CTraitsTest
{
	CTraitsTest()
	{
		static_assert(std::is_move_constructible<T>(), "Can't move construct");
		static_assert(std::is_copy_constructible<T>(), "Can't copy construct");
		static_assert(std::is_move_assignable<T>(), "Can't move assign");
		static_assert(std::is_copy_assignable<T>(), "Can't copy assign");
	}
};
}  // namespace mrpt