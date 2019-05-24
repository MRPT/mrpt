/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <functional>  // reference_wrapper
#include <optional>  // optional

namespace mrpt
{
/** Shorter name for std::optional<std::reference_wrapper<T>>
 * \ingroup mrpt_core_grp
 */
template <class T>
using optional_ref = std::optional<std::reference_wrapper<T>>;

}  // namespace mrpt
