/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
