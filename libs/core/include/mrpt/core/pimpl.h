/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/3rdparty/spimpl.h>
namespace mrpt
{
template <typename T>
using pimpl = spimpl::impl_ptr<T>;

template <class T, class... Args>
inline pimpl<T> make_impl(Args&&... args)
{
	return spimpl::make_impl<T, Args...>(std::forward<Args>(args)...);
}
}  // namespace mrpt
