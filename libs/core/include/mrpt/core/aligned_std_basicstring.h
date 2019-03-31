/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once
#include <mrpt/core/aligned_allocator.h>
#include <string>
namespace mrpt
{
template <class T>
using aligned_std_basicstring = std::basic_string<T, std::char_traits<T>, mrpt::aligned_allocator_cpp11<T>>;
}
