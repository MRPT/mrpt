/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once
#include <mrpt/core/aligned_allocator.h>
#include <map>
namespace mrpt
{
template <class KEY, class VALUE>
using aligned_std_map = std::map<
	KEY, VALUE, std::less<KEY>,
	mrpt::aligned_allocator_cpp11<std::pair<const KEY, VALUE>>>;

template <class KEY, class VALUE>
using aligned_std_multimap = std::multimap<
	KEY, VALUE, std::less<KEY>,
	mrpt::aligned_allocator_cpp11<std::pair<const KEY, VALUE>>>;
}  // namespace mrpt
