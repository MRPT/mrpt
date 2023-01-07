/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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