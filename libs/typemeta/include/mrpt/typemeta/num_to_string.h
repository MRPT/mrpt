/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
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
constexpr char to_chars<digits...>::value[sizeof...(digits) + 1] = {
	('0' + digits)..., 0};

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