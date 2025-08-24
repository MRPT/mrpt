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

#include <cstddef>  //size_t

namespace mrpt
{
namespace typemeta
{
namespace detail
{
template <unsigned... digits>
struct to_chars
{
  static const char value[sizeof...(digits) + 1];
};

template <unsigned... digits>
constexpr char to_chars<digits...>::value[sizeof...(digits) + 1] = {('0' + digits)..., 0};

template <unsigned rem, unsigned... digits>
struct explode : explode<rem / 10, rem % 10, digits...>
{
};

template <unsigned... digits>
struct explode<0, digits...> : to_chars<digits...>
{
};
}  // namespace detail

/** constexpr string representation of a number.
 * Use: `num_to_string<NUMBER>::value`.
 * \note Credits: https://stackoverflow.com/a/24000041/1631514 */
template <unsigned num>
struct num_to_string : detail::explode<num>
{
};
}  // namespace typemeta
}  // namespace mrpt