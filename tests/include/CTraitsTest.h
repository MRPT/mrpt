/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
}