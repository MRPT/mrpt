/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
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