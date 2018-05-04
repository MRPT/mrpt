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
#include <array>
#include <functional>  // invoke

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
