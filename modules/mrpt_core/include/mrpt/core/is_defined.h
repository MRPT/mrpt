/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <type_traits>  // enable_if_t

namespace mrpt
{
/** Checks if type is defined (fails for forward declarations).
 * \note Credits: WindyFields https://stackoverflow.com/a/45594334/1631514
 */
template <class T, class Enable = void>
struct is_defined
{
  static constexpr bool value = false;
};
template <class T>
struct is_defined<T, std::enable_if_t<(sizeof(T) > 0)>>
{
  static constexpr bool value = true;
};

template <class T>
inline constexpr bool is_defined_v = is_defined<T>::value;

}  // namespace mrpt
